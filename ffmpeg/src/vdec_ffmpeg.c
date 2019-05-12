/**
 * Copyright (c) 2017 Parrot Drones SAS
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the Parrot Drones SAS Company nor the
 *     names of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define _GNU_SOURCE
#include <stdio.h>

#define ULOG_TAG vdec_ffmpeg
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


#include "vdec_ffmpeg_priv.h"


#define NB_SUPPORTED_FORMATS 2
static struct vdef_coded_format supported_formats[NB_SUPPORTED_FORMATS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_h264_byte_stream;
	supported_formats[1] = vdef_h265_byte_stream;
}


static int av_to_ulog_level(int level)
{
	switch (level) {
	case AV_LOG_PANIC:
		return ULOG_CRIT;
	case AV_LOG_FATAL:
	case AV_LOG_ERROR:
		return ULOG_ERR;
	case AV_LOG_WARNING:
		return ULOG_WARN;
	case AV_LOG_INFO:
		return ULOG_INFO;
	case AV_LOG_VERBOSE:
	case AV_LOG_DEBUG:
	case AV_LOG_TRACE:
	default:
		return ULOG_DEBUG;
	}
}


static void av_log_cb(void *avcl, int level, const char *fmt, va_list vl)
{
	char *str = NULL;
	int ret = asprintf(&str, "av: %s", fmt);
	if (ret > 0 && str != NULL)
		ULOG_PRI_VA(av_to_ulog_level(level), str, vl);
	free(str);
}


static void flush_complete(struct vdec_ffmpeg *self)
{
	/* Call the flush callback if defined */
	if (self->base->cbs.flush)
		self->base->cbs.flush(self->base, self->base->userdata);
}


static void stop_complete(struct vdec_ffmpeg *self)
{
	/* Call the stop callback if defined */
	if (self->base->cbs.stop)
		self->base->cbs.stop(self->base, self->base->userdata);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct vdec_ffmpeg *self = userdata;
	int ret;
	char message;

	do {
		/* Read from the mailbox */
		ret = mbox_peek(self->mbox, &message);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("mbox_peek", -ret);
			break;
		}

		switch (message) {
		case VDEC_MSG_FLUSH:
			flush_complete(self);
			break;
		case VDEC_MSG_STOP:
			stop_complete(self);
			break;
		default:
			ULOGE("unknown message: %c", message);
			break;
		}
	} while (ret == 0);
}


static void out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct vdec_ffmpeg *self = userdata;
	struct mbuf_raw_video_frame *out_frame;
	int ret;

	do {
		ret = mbuf_raw_video_frame_queue_pop(self->out_queue,
						     &out_frame);
		if (ret == -EAGAIN) {
			return;
		} else if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop:output",
				   -ret);
			return;
		}
		self->base->cbs.frame_output(
			self->base, 0, out_frame, self->base->userdata);
		mbuf_raw_video_frame_unref(out_frame);
	} while (ret == 0);
}


static const struct vdef_raw_format *
format_from_av_pixel_format(enum AVPixelFormat format)
{
	switch (format) {
	case AV_PIX_FMT_YUV420P:
	case AV_PIX_FMT_YUVJ420P:
		return &vdef_i420;

	case AV_PIX_FMT_NV12:
		return &vdef_nv12;

	case AV_PIX_FMT_NV21:
		return &vdef_nv21;

	case AV_PIX_FMT_YUV420P10LE:
		return &vdef_i420_10_16le;

	case AV_PIX_FMT_P010LE:
		return &vdef_nv12_10_16le_high;

	case AV_PIX_FMT_NONE:
	default:
		ULOGW("unknown AVPixelFormat %d (%s)",
		      format,
		      av_get_pix_fmt_name(format));
		return NULL;
	}
}


static void
vdec_ffmpeg_avframe_mbuf_free(void *data, size_t len, void *userdata)
{
	AVFrame *frame = userdata;
	av_frame_free(&frame);
}


static int
vdec_ffmpeg_set_frame_metadata(struct vdec_ffmpeg *self,
			       struct mbuf_coded_video_frame *in_frame,
			       struct mbuf_raw_video_frame **out_frame,
			       AVFrame *frame)
{
	int ret = 0, err;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	uint8_t *base;
	size_t sz;
	size_t plane_offset;
	struct mbuf_mem *out_mem = NULL;
	struct vmeta_frame *metadata = NULL;
	struct vdef_raw_frame out_info;
	struct vdef_coded_frame in_info;
	const struct vdef_raw_format *fmt;

	if (self->hw_device_ctx != NULL) {
		/* HW decoding: transfer frame from GPU */
		AVFrame *sw_frame = av_frame_alloc();
		if (sw_frame == NULL) {
			ret = -ENOMEM;
			ULOG_ERRNO("av_frame_alloc", -ret);
			goto out;
		}

		/* Find a compatible pixel format */
		enum AVPixelFormat *formats = NULL;
		ret = av_hwframe_transfer_get_formats(
			frame->hw_frames_ctx,
			AV_HWFRAME_TRANSFER_DIRECTION_FROM,
			&formats,
			0);
		if (ret < 0) {
			ULOG_ERRNO("av_hwframe_transfer_get_formats", -ret);
			goto out;
		}
		/* Just use the first format available (there always seems to
		 * be only 1 format anyway) */
		sw_frame->format = *formats;

		ret = av_hwframe_transfer_data(sw_frame, frame, 0);
		if (ret < 0) {
			ULOG_ERRNO("av_hwframe_transfer_data", -ret);
			goto out;
		}

		av_frame_free(&frame);
		frame = sw_frame;
	}

	ret = mbuf_coded_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		goto out;
	}
	out_info.info = in_info.info;
	fmt = format_from_av_pixel_format(frame->format);
	if (!fmt) {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output chroma format", -ret);
		return ret;
	}
	out_info.format = *fmt;
	if (vdef_raw_format_cmp(fmt, &vdef_nv12) ||
	    vdef_raw_format_cmp(fmt, &vdef_nv12_10_16le_high) ||
	    vdef_raw_format_cmp(fmt, &vdef_nv21)) {
		base = (frame->data[0] < frame->data[1]) ? frame->data[0]
							 : frame->data[1];
		sz = frame->linesize[0] * frame->height +
		     frame->linesize[1] * frame->height / 2;
		out_info.plane_stride[0] = frame->linesize[0];
		out_info.plane_stride[1] = frame->linesize[1];
	} else if (vdef_raw_format_cmp(fmt, &vdef_i420) ||
		   vdef_raw_format_cmp(fmt, &vdef_i420_10_16le)) {
		base = (frame->data[0] < frame->data[1]) ? frame->data[0]
							 : frame->data[1];
		base = (base < frame->data[2]) ? base : frame->data[2];
		sz = frame->linesize[0] * frame->height +
		     frame->linesize[1] * frame->height / 2 +
		     frame->linesize[2] * frame->height / 2;
		out_info.plane_stride[0] = frame->linesize[0];
		out_info.plane_stride[1] = frame->linesize[1];
		out_info.plane_stride[2] = frame->linesize[2];
	} else {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output chroma format", -ret);
		return ret;
	}
	out_info.info.bit_depth = self->base->video_info.bit_depth;
	out_info.info.full_range = self->base->video_info.full_range;
	out_info.info.color_primaries = self->base->video_info.color_primaries;
	out_info.info.transfer_function =
		self->base->video_info.transfer_function;
	out_info.info.matrix_coefs = self->base->video_info.matrix_coefs;
	out_info.info.resolution.width = self->base->video_info.crop.width;
	out_info.info.resolution.height = self->base->video_info.crop.height;
	out_info.info.sar = self->base->video_info.sar;

	ret = mbuf_raw_video_frame_new(&out_info, out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_foreach_ancillary_data(
		in_frame,
		mbuf_raw_video_frame_ancillary_data_copier,
		*out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_foreach_ancillary_data",
			   -ret);
		goto out;
	}

	ret = mbuf_mem_generic_wrap(
		base, sz, vdec_ffmpeg_avframe_mbuf_free, frame, &out_mem);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_generic_wrap", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_get_metadata(in_frame, &metadata);
	if (ret == 0) {
		ret = mbuf_raw_video_frame_set_metadata(*out_frame, metadata);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_metadata", -ret);
			goto out;
		}
	} else if (ret == -ENOENT) {
		ret = 0;
		/* No metadata, nothing to do */
	} else {
		ULOG_ERRNO("mbuf_coded_video_frame_get_metadata", -ret);
		goto out;
	}

	switch (out_info.format.data_layout) {
	case VDEF_RAW_DATA_LAYOUT_SEMI_PLANAR:
		plane_offset = frame->data[0] - base;
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			0,
			out_mem,
			plane_offset,
			out_info.plane_stride[0] *
				out_info.info.resolution.height);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]", -ret);
			goto out;
		}
		plane_offset = frame->data[1] - base;
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			1,
			out_mem,
			plane_offset,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[1]", -ret);
			goto out;
		}
		break;
	case VDEF_RAW_DATA_LAYOUT_PLANAR:
		plane_offset = frame->data[0] - base;
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			0,
			out_mem,
			plane_offset,
			out_info.plane_stride[0] *
				out_info.info.resolution.height);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]", -ret);
			goto out;
		}
		plane_offset = frame->data[1] - base;
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			1,
			out_mem,
			plane_offset,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[1]", -ret);
			goto out;
		}
		plane_offset = frame->data[2] - base;
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			2,
			out_mem,
			plane_offset,
			out_info.plane_stride[2] *
				out_info.info.resolution.height / 2);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[2]", -ret);
			goto out;
		}
		break;
	default:
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output chroma format", -ret);
		return ret;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		*out_frame,
		VDEC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer", -ret);

	if (self->base->dbg.output_yuv != NULL) {
		int dbgret = vdec_dbg_write_yuv_frame(
			self->base->dbg.output_yuv, *out_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_yuv_frame", -dbgret);
	}

	ret = mbuf_raw_video_frame_finalize(*out_frame);
	if (ret < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);

out:
	err = mbuf_mem_unref(out_mem);
	if (err != 0)
		ULOG_ERRNO("mbuf_mem_unref", -err);
	if (ret < 0 && *out_frame) {
		mbuf_raw_video_frame_unref(*out_frame);
		*out_frame = NULL;
	}
	if (metadata)
		vmeta_frame_unref(metadata);
	return ret;
}


static void vdec_ffmpeg_complete_flush(struct vdec_ffmpeg *self)
{
	/* Flush the decoder queue (just in case) */
	int ret = mbuf_coded_video_frame_queue_flush(self->decoder_queue);
	if (ret < 0)
		ULOG_ERRNO("mbuf_coded_frame_queue_flush:decoder", -ret);

	avcodec_flush_buffers(self->avcodec);
	atomic_store(&self->flushing, 0);

	/* Call the flush callback on the loop */
	char message = VDEC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);
}


static int vdec_ffmpeg_start_flush(struct vdec_ffmpeg *self)
{
	int ret;

	if (atomic_load(&self->flush_discard)) {
		/* Flush the input queue */
		ret = mbuf_coded_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_flush:input",
				   -ret);
			return ret;
		}
		/* Flush the output queue */
		ret = mbuf_raw_video_frame_queue_flush(self->out_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_flush:output",
				   -ret);
			return ret;
		}
	}

	/* Push an empty frame to enter draining mode */
	self->avpacket.data = NULL;
	self->avpacket.size = 0;
	ret = avcodec_send_packet(self->avcodec, &self->avpacket);
	/*
	 * AVERROR_EOF and -EAGAIN are not treated as errors here.
	 * The decoder can return AVERROR_EOF if it is already flushed.
	 * The decoder can return -EAGAIN if its input queue is filled (i.e. it
	 * is waiting for an avcodec_receive_frame call), but it will still
	 * register the flush request.
	 */
	if (ret < 0 && ret != AVERROR_EOF && ret != -EAGAIN) {
		ULOG_ERRNO("avcodec_send_packet", -ret);
		return ret;
	}

	atomic_store(&self->flush, 0);
	atomic_store(&self->flushing, 1);

	/* Already flushed, just call flush done */
	if (ret == AVERROR_EOF)
		vdec_ffmpeg_complete_flush(self);

	return 0;
}


static int vdec_ffmpeg_insert_grey_idr(struct vdec_ffmpeg *self,
				       struct vdef_coded_frame *in_frame_info,
				       uint64_t *delta)
{
	int ret;
	struct mbuf_mem *idr_mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;
	uint64_t timestamp;
	size_t out_buf_size = self->base->video_info.resolution.width *
			      self->base->video_info.resolution.height * 3 / 4;
	const void *frame_data = NULL;
	size_t frame_len;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(in_frame_info == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);

	/* Create buffer */
	ret = mbuf_mem_generic_new(out_buf_size, &idr_mem);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -ret);
		goto out;
	}

	timestamp = in_frame_info->info.timestamp;

	ret = vdec_h264_write_grey_idr(self->base,
				       in_frame_info,
				       delta,
				       &timestamp,
				       idr_mem,
				       &idr_frame);
	if (ret < 0) {
		ULOG_ERRNO("vdec_h264_write_grey_idr", -ret);
		goto out;
	}

	/* Push the frame */
	ret = mbuf_coded_video_frame_get_packed_buffer(
		idr_frame, &frame_data, &frame_len);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer", -ret);
		goto out;
	}
	self->avpacket.data = (uint8_t *)frame_data;
	self->avpacket.size = frame_len;
	self->avpacket.pts = timestamp;
	ret = avcodec_send_packet(self->avcodec, &self->avpacket);
	if (ret < 0) {
		ULOG_ERRNO("avcodec_send_packet", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_queue_push(self->decoder_queue, idr_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push:decoder", -ret);
		goto out;
	}

	/* Debug files */
	if (self->base->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_frame(self->base->dbg.input_bs,
						  idr_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_frame", -dbgret);
	}
	if (self->base->dbg.analysis != NULL) {
		int dbgret = vdec_dbg_parse_frame(self->base, idr_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_frame", -dbgret);
	}

out:
	if (idr_mem)
		mbuf_mem_unref(idr_mem);
	if (ret != 0 && frame_data)
		mbuf_coded_video_frame_release_packed_buffer(idr_frame,
							     frame_data);
	if (idr_frame)
		mbuf_coded_video_frame_unref(idr_frame);

	return ret;
}


static int vdec_ffmpeg_buffer_push_one(struct vdec_ffmpeg *self,
				       struct mbuf_coded_video_frame *in_frame)
{
	int ret = 0, err;
	struct vdef_coded_frame in_info;
	struct timespec cur_ts = {0, 0};
	uint64_t delta = 0;
	uint64_t ts_us;
	const void *frame_data = NULL;
	size_t frame_len;

	ret = mbuf_coded_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		goto out;
	}

	if (!vdef_coded_format_intersect(
		    &in_info.format, supported_formats, NB_SUPPORTED_FORMATS)) {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported format: %s",
			   -ret,
			   vdef_coded_data_format_to_str(
				   in_info.format.data_format));
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	if (atomic_load(&self->need_sync)) {
		if (vdec_is_sync_frame(in_frame, &in_info)) {
			ULOGI("frame is a sync point");
			atomic_store(&self->need_sync, 0);
		} else {
			if (self->base->config.encoding == VDEF_ENCODING_H264 &&
			    self->base->config.gen_grey_idr) {
				ULOGI("frame is not an IDR, "
				      "generating grey IDR");
				ret = vdec_ffmpeg_insert_grey_idr(
					self, &in_info, &delta);
				if (ret < 0) {
					ULOG_ERRNO(
						"vdec_ffmpeg_insert_grey_idr",
						-ret);
					goto out;
				}
				atomic_store(&self->need_sync, 0);
			} else {
				ULOGI("frame is not a sync point, "
				      "discarding frame");
				goto out;
			}
		}
	}

	ret = mbuf_coded_video_frame_get_packed_buffer(
		in_frame, &frame_data, &frame_len);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer", -ret);
		goto out;
	}

	/* Push the frame */
	self->avpacket.data = (uint8_t *)frame_data;
	self->avpacket.size = frame_len;
	self->avpacket.pts = in_info.info.timestamp + delta;
	ret = avcodec_send_packet(self->avcodec, &self->avpacket);
	if (ret < 0) {
		if (ret != -EAGAIN)
			ULOG_ERRNO("avcodec_send_packet", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_queue_push(self->decoder_queue, in_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push:decoder", -ret);
		goto out;
	}

	err = mbuf_coded_video_frame_add_ancillary_buffer(
		in_frame,
		VDEC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (err != 0)
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -err);

	/* Debug files */
	if (self->base->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_frame(self->base->dbg.input_bs,
						  in_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_frame", -dbgret);
	}
	if (self->base->dbg.analysis != NULL) {
		int dbgret = vdec_dbg_parse_frame(self->base, in_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_frame", -dbgret);
	}

out:
	if (ret != 0 && frame_data)
		mbuf_coded_video_frame_release_packed_buffer(in_frame,
							     frame_data);
	mbuf_coded_video_frame_unref(in_frame);
	return ret;
}


static void release_decoder_frame(struct mbuf_coded_video_frame *frame)
{
	/* The frame has its packed buffer referenced from the push_one
	 * function, so we need to release it here. */
	const void *frame_data;
	size_t frame_len;
	if (mbuf_coded_video_frame_get_packed_buffer(
		    frame, &frame_data, &frame_len) == 0) {
		mbuf_coded_video_frame_release_packed_buffer(frame, frame_data);
		mbuf_coded_video_frame_release_packed_buffer(frame, frame_data);
	}
	mbuf_coded_video_frame_unref(frame);
}


static int vdec_ffmpeg_discard_frame(struct vdec_ffmpeg *self)
{
	int ret, avcodec_ret;
	struct mbuf_coded_video_frame *in_frame = NULL;
	struct vdef_coded_frame in_info;

	/* Dequeue the frame from the decoder and drop it */
	avcodec_ret = avcodec_receive_frame(self->avcodec, self->dummy_frame);
	if ((avcodec_ret == -EAGAIN) || (avcodec_ret == AVERROR_EOF)) {
		/* No frame available (all frames were output),
		 * do not dequeue the input buffer */
		if ((avcodec_ret == AVERROR_EOF) &&
		    atomic_load(&self->flushing))
			vdec_ffmpeg_complete_flush(self);
		return avcodec_ret;
	} else if (avcodec_ret < 0) {
		ULOG_ERRNO("avcodec_receive_frame", -avcodec_ret);
	}

	/* Get the input buffer (non-blocking);
	 * drop input buffers until the correct timestamp is met,
	 * in case ffmpeg has internally dropped some frames */
	do {
		if (in_frame != NULL)
			release_decoder_frame(in_frame);
		ret = mbuf_coded_video_frame_queue_pop(self->decoder_queue,
						       &in_frame);
		if ((ret < 0) || (in_frame == NULL)) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_pop:decoder",
				   -ret);
			break;
		}
		ret = mbuf_coded_video_frame_get_frame_info(in_frame, &in_info);
		if (ret < 0) {
			ULOG_ERRNO(
				"mbuf_coded_video_frame_get_frame_info:decoder",
				-ret);
			break;
		}
	} while (self->dummy_frame->pts > (int64_t)in_info.info.timestamp);

	if (in_frame != NULL)
		release_decoder_frame(in_frame);

	return 0;
}


static int vdec_ffmpeg_buffer_pop_all(struct vdec_ffmpeg *self)
{
	int ret = 0, err, avcodec_ret;
	AVFrame *frame;
	struct mbuf_coded_video_frame *in_frame = NULL;
	struct mbuf_raw_video_frame *out_frame = NULL;
	struct vdef_coded_frame in_info;
	bool need_av_free = false;

	do {
		/* Discard frames if flush is in progress */
		if (atomic_load(&self->flushing) &&
		    atomic_load(&self->flush_discard)) {
			err = vdec_ffmpeg_discard_frame(self);
			if (err == AVERROR_EOF)
				break;
			else
				continue;
		}

		frame = av_frame_alloc();
		if (frame == NULL) {
			ret = -ENOMEM;
			ULOG_ERRNO("av_frame_alloc", -ret);
			break;
		}
		need_av_free = true;

		/* Dequeue the frame from the decoder */
		avcodec_ret = avcodec_receive_frame(self->avcodec, frame);
		if ((avcodec_ret == -EAGAIN) || (avcodec_ret == AVERROR_EOF)) {
			/* No frame available (all frames were output),
			 * do not dequeue the input buffer */
			if ((avcodec_ret == AVERROR_EOF) &&
			    atomic_load(&self->flushing))
				vdec_ffmpeg_complete_flush(self);
			break;
		}

		/* Get the input buffer (non-blocking);
		 * drop input buffers until the correct timestamp is met,
		 * in case ffmpeg has internally dropped some frames */
		do {
			if (in_frame != NULL)
				release_decoder_frame(in_frame);
			ret = mbuf_coded_video_frame_queue_pop(
				self->decoder_queue, &in_frame);
			if ((ret < 0) || (in_frame == NULL)) {
				ULOG_ERRNO(
					"mbuf_coded_video_frame_queue_pop:decoder",
					-ret);
				break;
			}
			ret = mbuf_coded_video_frame_get_frame_info(in_frame,
								    &in_info);
			if (ret < 0) {
				ULOG_ERRNO(
					"mbuf_coded_video_frame_get_frame_info:decoder",
					-ret);
				break;
			}
		} while (frame->pts > (int64_t)in_info.info.timestamp);

		if (ret < 0 || in_frame == NULL)
			break;

		if (avcodec_ret < 0) {
			/* Decoding error,
			 * unref the input buffer */
			ret = avcodec_ret;
			ULOG_ERRNO("avcodec_receive_frame", -avcodec_ret);
			break;
		}

		/* Set the metadata */
		ret = vdec_ffmpeg_set_frame_metadata(
			self, in_frame, &out_frame, frame);
		if (ret < 0)
			break;
		need_av_free = false;

		/* Push the frame (if not silent) */
		if ((in_info.info.flags & VDEF_FRAME_FLAG_SILENT) &&
		    (!self->base->config.output_silent_frames)) {
			ULOGD("silent frame (ignored)");
		} else {
			ret = mbuf_raw_video_frame_queue_push(self->out_queue,
							      out_frame);
			if (ret < 0)
				ULOG_ERRNO(
					"mbuf_raw_video_frame_queue_push:output",
					-ret);
		}

		/* Unref the buffers */
		release_decoder_frame(in_frame);
		in_frame = NULL;
		mbuf_raw_video_frame_unref(out_frame);
		out_frame = NULL;
	} while ((!atomic_load(&self->flush) &&
		  !atomic_load(&self->should_stop)) ||
		 atomic_load(&self->flushing));

	if (need_av_free)
		av_frame_free(&frame);

	return ret;
}


static void check_input_queue(struct vdec_ffmpeg *self)
{
	int ret;
	struct mbuf_coded_video_frame *frame;

	ret = mbuf_coded_video_frame_queue_peek(self->in_queue, &frame);
	while (ret == 0) {
		/* Push the input frame */
		ret = vdec_ffmpeg_buffer_push_one(self, frame);
		if (ret < 0) {
			if (ret == -EAGAIN) {
				ret = -ENOSPC;
				break;
			}
			ULOG_ERRNO("vdec_ffmpeg_buffer_push_one", -ret);
		}
		/* Pop the frame for real */
		ret = mbuf_coded_video_frame_queue_pop(self->in_queue, &frame);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_pop", -ret);
			break;
		}
		mbuf_coded_video_frame_unref(frame);
		/* Once flush is requested, frames shall be pushed and popped
		 * one by one to avoid loosing the last one. */
		if (!atomic_load(&self->flush)) {
			/* Peek the next frame */
			ret = mbuf_coded_video_frame_queue_peek(self->in_queue,
								&frame);
		} else {
			/* Exit the loop to make sure only one frame
			 * is processed */
			ret = -ENOSPC;
		}
	}
	if (ret != -EAGAIN && ret != -ENOSPC)
		ULOG_ERRNO("mbuf_coded_video_frame_queue_peek", -ret);
	if (atomic_load(&self->flush) && ret == -EAGAIN) {
		/* Flush without discarding frames */
		ret = vdec_ffmpeg_start_flush(self);
		if (ret < 0 && ret)
			ULOG_ERRNO("vdec_ffmpeg_start_flush", -ret);
	}
	/* Pop output frames */
	ret = vdec_ffmpeg_buffer_pop_all(self);
	if (ret < 0 && ret != -EAGAIN)
		ULOG_ERRNO("vdec_ffmpeg_buffer_pop_all", -ret);
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct vdec_ffmpeg *self = userdata;
	check_input_queue(self);
}


static void *vdec_ffmpeg_decoder_thread(void *ptr)
{
	int ret;
	struct vdec_ffmpeg *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	int timeout;
	char message;

	loop = pomp_loop_new();
	if (!loop) {
		ULOG_ERRNO("pomp_loop_new", ENOMEM);
		goto exit;
	}
	ret = mbuf_coded_video_frame_queue_get_event(self->in_queue,
						     &in_queue_evt);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_get_event", -ret);
		goto exit;
	}
	ret = pomp_evt_attach_to_loop(in_queue_evt, loop, input_event_cb, self);
	if (ret != 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto exit;
	}

	while (!atomic_load(&self->should_stop) || atomic_load(&self->flush)) {
		/* Start flush, discarding all frames */
		if (atomic_load(&self->flush) &&
		    atomic_load(&self->flush_discard)) {
			ret = vdec_ffmpeg_start_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_ffmpeg_start_flush", -ret);
			goto exit;
		}

		timeout = (atomic_load(&self->flushing) &&
			   !atomic_load(&self->flush_discard))
				  ? 0
				  : 5;

		ret = pomp_loop_wait_and_process(loop, timeout);
		if (ret < 0 && ret != -ETIMEDOUT) {
			ULOG_ERRNO("pomp_loop_wait_and_process", -ret);
			if (!atomic_load(&self->should_stop)) {
				/* Avoid looping on errors */
				usleep(5000);
			}
			continue;
		} else if (ret == -ETIMEDOUT) {
			check_input_queue(self);
		}
	}

	/* Call the stop callback on the loop */
	message = VDEC_MSG_STOP;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);

exit:
	if (in_queue_evt != NULL) {
		ret = pomp_evt_detach_from_loop(in_queue_evt, loop);
		if (ret != 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -ret);
	}
	if (loop != NULL) {
		ret = pomp_loop_destroy(loop);
		if (ret != 0)
			ULOG_ERRNO("pomp_loop_destroy", -ret);
	}

	return NULL;
}


static int get_supported_input_formats(const struct vdef_coded_format **formats)
{
	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);
	*formats = supported_formats;
	return NB_SUPPORTED_FORMATS;
}


static int stop(struct vdec_decoder *base)
{
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the decoding thread */
	atomic_store(&self->should_stop, 1);
	self->base->configured = 0;

	return 0;
}


static int destroy(struct vdec_decoder *base)
{
	int err;
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the decoding thread */
	err = stop(base);
	if (err < 0)
		ULOG_ERRNO("vdec_ffmpeg_stop", -err);
	if (atomic_load(&self->thread_launched)) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->out_queue_evt != NULL) {
		err = pomp_evt_detach_from_loop(self->out_queue_evt,
						base->loop);
		if (err < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->out_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->out_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_queue_destroy", -err);
	}
	if (self->in_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
				   -err);
	}
	if (self->decoder_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->decoder_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
				   -err);
	}
	if (self->mbox != NULL) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			ULOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	if (self->avcodec != NULL)
		avcodec_free_context(&self->avcodec);
	if (self->hw_device_ctx != NULL)
		av_buffer_unref(&self->hw_device_ctx);
	if (self->dummy_frame != NULL)
		av_frame_free(&self->dummy_frame);
	free(self);

	return 0;
}


static bool input_filter(struct mbuf_coded_video_frame *frame, void *userdata)
{
	const void *tmp;
	size_t tmplen;
	struct vdef_coded_frame info;
	int ret;
	struct vdec_ffmpeg *self = userdata;

	ret = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (ret != 0)
		return false;

	/* Pass default filters first */
	if (!vdec_default_input_filter_internal(self->base,
						frame,
						&info,
						supported_formats,
						NB_SUPPORTED_FORMATS))
		return false;

	/* Input frame must be packed */
	ret = mbuf_coded_video_frame_get_packed_buffer(frame, &tmp, &tmplen);
	if (ret != 0)
		return false;

	mbuf_coded_video_frame_release_packed_buffer(frame, tmp);

	vdec_default_input_filter_internal_confirm_frame(
		self->base, frame, &info);

	return true;
}


static int create(struct vdec_decoder *base)
{
	int ret = 0;
	struct vdec_ffmpeg *self = NULL;
	unsigned int ver = avcodec_version();
	enum AVHWDeviceType hw_type = AV_HWDEVICE_TYPE_NONE;
	enum AVPixelFormat pix_fmt = AV_PIX_FMT_YUV420P;
	struct mbuf_coded_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;
	queue_args.filter_userdata = self;

	/* Find the decoder */
	av_log_set_level(AV_LOG_VERBOSE);
	av_log_set_callback(&av_log_cb);

	AVCodec *codec;
	switch (base->config.encoding) {
	case VDEF_ENCODING_H264:
		atomic_store(&self->need_sync, 1);
		codec = avcodec_find_decoder(AV_CODEC_ID_H264);
		break;
	case VDEF_ENCODING_H265:
		codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
		break;
	default:
		codec = NULL;
		break;
	}

	if (codec == NULL) {
		ret = -ENOENT;
		ULOG_ERRNO("codec not found", -ret);
		goto error;
	}

	/* Optional NVDEC HW decoding support (still called CUDA by ffmpeg) */
	hw_type = av_hwdevice_find_type_by_name("cuda");
	if (hw_type != AV_HWDEVICE_TYPE_NONE) {
		bool found = false;
		for (int i = 0;; i++) {
			const AVCodecHWConfig *config =
				avcodec_get_hw_config(codec, i);
			if (config == NULL)
				break;
			if (config->methods &
				    AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
			    config->device_type == hw_type) {
				found = true;
				pix_fmt = config->pix_fmt;
				break;
			}
		}
		if (!found) {
			ULOGI("no config found for device type %s",
			      av_hwdevice_get_type_name(hw_type));
			hw_type = AV_HWDEVICE_TYPE_NONE;
		}
	}

	ULOGI("libavcodec version=%u.%u.%u - using %s %s decoding",
	      (ver >> 16) & 0xFF,
	      (ver >> 8) & 0xFF,
	      ver & 0xFF,
	      vdef_encoding_to_str(base->config.encoding),
	      (hw_type != AV_HWDEVICE_TYPE_NONE) ? "NVDEC HW" : "CPU");

	/* Initialize the decoder */
	self->avcodec = avcodec_alloc_context3(codec);
	if (self->avcodec == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("avcodec_alloc_context3", -ret);
		goto error;
	}

	self->avcodec->pix_fmt = pix_fmt;
	self->avcodec->skip_frame = AVDISCARD_DEFAULT;
	self->avcodec->error_concealment =
		FF_EC_GUESS_MVS | FF_EC_DEBLOCK | FF_EC_FAVOR_INTER;
	self->avcodec->skip_loop_filter = AVDISCARD_DEFAULT;
	self->avcodec->workaround_bugs = FF_BUG_AUTODETECT;
	self->avcodec->codec_type = AVMEDIA_TYPE_VIDEO;
	self->avcodec->skip_idct = AVDISCARD_DEFAULT;
	self->avcodec->thread_type = FF_THREAD_SLICE;
	if (!base->config.low_delay)
		self->avcodec->thread_type |= FF_THREAD_FRAME;
	switch (base->config.preferred_thread_count) {
	case 1:
		self->avcodec->thread_count = 0;
		self->avcodec->thread_type &= ~FF_THREAD_FRAME;
		break;
	case 0:
		self->avcodec->thread_count = VDEC_FFMPEG_DEFAULT_THREAD_COUNT;
		break;
	default:
		self->avcodec->thread_count =
			base->config.preferred_thread_count;
		break;
	}

	if (hw_type != AV_HWDEVICE_TYPE_NONE) {
		ret = av_hwdevice_ctx_create(
			&self->hw_device_ctx, hw_type, NULL, NULL, 0);
		if (ret < 0) {
			ULOG_ERRNO("av_hwdevice_ctx_create", -ret);
			goto error;
		}
		self->avcodec->hw_device_ctx =
			av_buffer_ref(self->hw_device_ctx);
	}

	av_init_packet(&self->avpacket);
	self->dummy_frame = av_frame_alloc();
	if (self->dummy_frame == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("av_frame_alloc", -ret);
		goto error;
	}

	/* Initialize the mailbox for inter-thread messages  */
	self->mbox = mbox_new(1);
	if (self->mbox == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("mbox_new", -ret);
		goto error;
	}
	ret = pomp_loop_add(base->loop,
			    mbox_get_read_fd(self->mbox),
			    POMP_FD_EVENT_IN,
			    &mbox_cb,
			    self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_loop_add", -ret);
		goto error;
	}

	/* Create the ouput buffers queue */
	ret = mbuf_raw_video_frame_queue_new(&self->out_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_new:output", -ret);
		goto error;
	}
	ret = mbuf_raw_video_frame_queue_get_event(self->out_queue,
						   &self->out_queue_evt);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -ret);
		goto error;
	}
	ret = pomp_evt_attach_to_loop(
		self->out_queue_evt, base->loop, &out_queue_evt_cb, self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	/* Create the input buffers queue */
	ret = mbuf_coded_video_frame_queue_new_with_args(&queue_args,
							 &self->in_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new_with_args", -ret);
		goto error;
	}

	/* Create the decoder queue */
	ret = mbuf_coded_video_frame_queue_new(&self->decoder_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new", -ret);
		goto error;
	}

	return 0;

error:
	/* Cleanup on error */
	destroy(base);
	base->derived = NULL;
	return ret;
}


static int flush(struct vdec_decoder *base, int discard)
{
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flush, 1);
	atomic_store(&self->flush_discard, discard);

	return 0;
}


static int set_ps(struct vdec_decoder *base,
		  const uint8_t **ps_table,
		  const size_t *size_table,
		  size_t ps_count,
		  const struct vdef_coded_format *format)
{
	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ps_table == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size_table == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);

	size_t padding_len =
		(format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU) ? 4
									 : 0;
	struct vdec_ffmpeg *self = base->derived;

	size_t size = 0;
	for (size_t i = 0; i < ps_count; ++i)
		size += padding_len + size_table[i];

	uint8_t *data = av_mallocz(size + AV_INPUT_BUFFER_PADDING_SIZE);
	if (data == NULL)
		return -ENOMEM;

	uint8_t start_code[] = {0, 0, 0, 1};
	size_t off = 0;
	for (size_t i = 0; i < ps_count; ++i) {
		memcpy(&data[off], start_code, sizeof(start_code));
		off += 4;

		size_t offset = 4 - padding_len;
		memcpy(&data[off],
		       &ps_table[i][offset],
		       size_table[i] - offset);
		off += size_table[i] - offset;
	}

	self->avcodec->extradata = data;
	self->avcodec->extradata_size = size;

	int ret = avcodec_open2(self->avcodec, NULL, NULL);
	if (ret < 0) {
		ULOG_ERRNO("avcodec_open2", -ret);
		return ret;
	}

	ret = pthread_create(
		&self->thread, NULL, vdec_ffmpeg_decoder_thread, self);
	if (ret != 0) {
		ret = -ret;
		ULOG_ERRNO("pthread_create", -ret);
		return ret;
	}

	atomic_store(&self->thread_launched, 1);
	return 0;
}


static int set_h264_ps(struct vdec_decoder *base,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format)
{
	const uint8_t *ps_table[] = {sps, pps};
	size_t size_table[] = {sps_size, pps_size};

	return set_ps(base, ps_table, size_table, 2, format);
}


static int set_h265_ps(struct vdec_decoder *base,
		       const uint8_t *vps,
		       size_t vps_size,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format)
{
	const uint8_t *ps_table[] = {vps, sps, pps};
	size_t size_table[] = {vps_size, sps_size, pps_size};

	return set_ps(base, ps_table, size_table, 3, format);
}


static struct mbuf_pool *get_input_buffer_pool(struct vdec_decoder *base)
{
	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


static struct mbuf_coded_video_frame_queue *
get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = base->derived;

	return self->in_queue;
}

const struct vdec_ops vdec_ffmpeg_ops = {
	.get_supported_input_formats = get_supported_input_formats,
	.create = create,
	.flush = flush,
	.stop = stop,
	.destroy = destroy,
	.set_h264_ps = set_h264_ps,
	.set_h265_ps = set_h265_ps,
	.get_input_buffer_pool = get_input_buffer_pool,
	.get_input_buffer_queue = get_input_buffer_queue,
};

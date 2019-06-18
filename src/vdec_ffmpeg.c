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

#define ULOG_TAG vdec_ffmpeg
#include "vdec_priv.h"
ULOG_DECLARE_TAG(vdec_ffmpeg);

#ifdef BUILD_FFMPEG_LIBAV

#	include <libavcodec/avcodec.h>

#	include <video-buffers/vbuf_private.h>
#	ifdef BUILD_LIBVIDEO_BUFFERS_GENERIC
#		include <video-buffers/vbuf_generic.h>
#	else /* BUILD_LIBVIDEO_BUFFERS_GENERIC */
#		error "cannot build vdec_ffmpeg without "\
			"libvideo-buffers-generic"
#	endif /* BUILD_LIBVIDEO_BUFFERS_GENERIC */

#	define VDEC_FFMPEG_OUT_POOL_DEFAULT_MIN_BUF_COUNT 5
#	define VDEC_FFMPEG_DEFAULT_THREAD_COUNT 8
#	define VDEC_FFMPEG_BUF_TYPE_AVFRAME 0x4156464D /* "AVFM" */


struct vdec_ffmpeg {
	struct vdec_decoder *base;
	struct vbuf_queue *in_queue;
	struct vbuf_queue *decoder_queue;
	struct vbuf_pool *out_pool;
	AVCodecContext *avcodec;
	AVPacket avpacket;
	AVFrame *dummy_frame;
	int flushing;

	pthread_t thread;
	int thread_launched;
	int should_stop;
	int flush;
	int flush_discard;

	int needs_ps;
	uint8_t *ps;
	size_t ps_size;
	enum vdec_output_format output_format;
};


struct vdec_ffmpeg_avframe {
	AVFrame *frame;
};


static int
vdec_ffmpeg_avframe_set_data(struct vbuf_buffer *buf, uint8_t *ptr, size_t sz)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf->type != VDEC_FFMPEG_BUF_TYPE_AVFRAME,
				 EINVAL);

	buf->ptr = ptr;
	buf->capacity = sz;
	buf->size = sz;

	return 0;
}


static AVFrame *vdec_ffmpeg_avframe_get_frame(struct vbuf_buffer *buf)
{
	struct vdec_ffmpeg_avframe *avframe;

	ULOG_ERRNO_RETURN_VAL_IF(
		buf->type != VDEC_FFMPEG_BUF_TYPE_AVFRAME, EINVAL, NULL);

	avframe = (struct vdec_ffmpeg_avframe *)buf->specific;

	if (avframe)
		return avframe->frame;
	else
		return NULL;
}


static int vdec_ffmpeg_avframe_alloc_cb(struct vbuf_buffer *buf, void *userdata)
{
	struct vdec_ffmpeg_avframe *avframe = NULL;
	int ret = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	buf->type = VDEC_FFMPEG_BUF_TYPE_AVFRAME;

	avframe = calloc(1, sizeof(*avframe));
	if (avframe == NULL)
		return -ENOMEM;
	buf->specific = (struct vbuf_specific *)avframe;
	avframe->frame = av_frame_alloc();
	if (avframe->frame == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("av_frame_alloc", -ret);
		goto error;
	}

	return 0;

error:
	free(avframe);
	return ret;
}


static int vdec_ffmpeg_avframe_free_cb(struct vbuf_buffer *buf, void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	struct vdec_ffmpeg_avframe *avframe =
		(struct vdec_ffmpeg_avframe *)buf->specific;
	if ((avframe) && (avframe->frame))
		av_frame_free(&avframe->frame);
	free(buf->specific);
	buf->specific = NULL;
	buf->ptr = NULL;

	return 0;
}


static int vdec_ffmpeg_set_frame_metadata(struct vdec_ffmpeg *self,
					  struct vbuf_buffer *in_buf,
					  struct vbuf_buffer *out_buf,
					  AVFrame *frame)
{
	int ret;
	struct vdec_input_metadata *in_meta = NULL;
	struct vdec_output_metadata *out_meta = NULL;
	const uint8_t *user_data;
	uint8_t *base;
	size_t sz;
	unsigned int level = 0, user_data_size;
	struct timespec cur_ts = {0, 0};

	/* Frame metadata */
	ret = vbuf_metadata_get(
		in_buf, self->base, &level, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get:input", -ret);
		return ret;
	}
	ret = vbuf_metadata_copy(in_buf, out_buf, level);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_copy", -ret);
		return ret;
	}
	ret = vbuf_metadata_add(out_buf,
				self->base,
				level,
				sizeof(*out_meta),
				(uint8_t **)&out_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_add:output", -ret);
		return ret;
	}
	out_meta->timestamp = in_meta->timestamp;
	out_meta->index = in_meta->index;
	out_meta->format = self->output_format;
	switch (out_meta->format) {
	case VDEC_OUTPUT_FORMAT_NV12:
		base = (frame->data[0] < frame->data[1]) ? frame->data[0]
							 : frame->data[1];
		sz = frame->linesize[0] * frame->height +
		     frame->linesize[1] * frame->height / 2;
		vdec_ffmpeg_avframe_set_data(out_buf, base, sz);
		out_meta->plane_offset[0] = (frame->data[0] - base);
		out_meta->plane_offset[1] = (frame->data[1] - base);
		out_meta->plane_stride[0] = frame->linesize[0];
		out_meta->plane_stride[1] = frame->linesize[1];
		break;
	case VDEC_OUTPUT_FORMAT_I420:
		base = (frame->data[0] < frame->data[1]) ? frame->data[0]
							 : frame->data[1];
		base = (base < frame->data[2]) ? base : frame->data[2];
		sz = frame->linesize[0] * frame->height +
		     frame->linesize[1] * frame->height / 2 +
		     frame->linesize[2] * frame->height / 2;
		vdec_ffmpeg_avframe_set_data(out_buf, base, sz);
		out_meta->plane_offset[0] = (frame->data[0] - base);
		out_meta->plane_offset[1] = (frame->data[1] - base);
		out_meta->plane_offset[2] = (frame->data[2] - base);
		out_meta->plane_stride[0] = frame->linesize[0];
		out_meta->plane_stride[1] = frame->linesize[1];
		out_meta->plane_stride[2] = frame->linesize[2];
		break;
	default:
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output chroma format", -ret);
		return ret;
	}
	out_meta->width = self->base->width;
	out_meta->height = self->base->height;
	out_meta->sar_width = self->base->sar_width;
	out_meta->sar_height = self->base->sar_height;
	out_meta->crop_left = self->base->crop_left;
	out_meta->crop_top = self->base->crop_top;
	out_meta->crop_width = self->base->crop_width;
	out_meta->crop_height = self->base->crop_height;
	out_meta->full_range = self->base->full_range;
	out_meta->errors = in_meta->errors;
	out_meta->silent = in_meta->silent;
	out_meta->userdata = in_meta->userdata;
	out_meta->input_time = in_meta->input_time;
	out_meta->dequeue_time = in_meta->dequeue_time;

	/* User data */
	user_data = vbuf_get_cuserdata(in_buf);
	user_data_size = vbuf_get_userdata_size(in_buf);
	if ((user_data) && (user_data_size > 0)) {
		ret = vbuf_set_userdata_capacity(out_buf, user_data_size);
		if (ret < (signed)user_data_size) {
			ULOG_ERRNO("vbuf_set_userdata_capacity", -ret);
			vbuf_set_userdata_size(out_buf, 0);
		} else {
			uint8_t *dst = vbuf_get_userdata(out_buf);
			memcpy(dst, user_data, user_data_size);
			vbuf_set_userdata_size(out_buf, user_data_size);
		}
	} else {
		vbuf_set_userdata_size(out_buf, 0);
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &out_meta->output_time);

	if (self->base->dbg.output_yuv != NULL) {
		int dbgret = vdec_dbg_write_yuv_frame(
			self->base->dbg.output_yuv, out_buf, out_meta);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_yuv_frame", -dbgret);
	}

	return 0;
}


static void vdec_ffmpeg_complete_flush(struct vdec_ffmpeg *self)
{
	/* Flush the decoder queue (just in case) */
	int ret = vbuf_queue_flush(self->decoder_queue);
	if (ret < 0)
		ULOG_ERRNO("vbuf_queue_flush:decoder", -ret);

	avcodec_flush_buffers(self->avcodec);
	self->flushing = 0;
	self->needs_ps = 1;

	/* Call the flush callback if defined */
	if (self->base->cbs.flush)
		(*self->base->cbs.flush)(self->base, self->base->userdata);
}


static int vdec_ffmpeg_start_flush(struct vdec_ffmpeg *self)
{
	int ret;

	if (self->flush_discard) {
		/* Flush the input queue */
		ret = vbuf_queue_flush(self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_flush:input", -ret);
			return ret;
		}
	}

	/* Push an empty frame to enter draining mode */
	self->avpacket.data = NULL;
	self->avpacket.size = 0;
	ret = avcodec_send_packet(self->avcodec, &self->avpacket);
	if (ret < 0 && ret != AVERROR_EOF) {
		ULOG_ERRNO("avcodec_send_packet", -ret);
		return ret;
	}

	self->flush = 0;
	self->flushing = 1;

	/* Already flushed, just call flush done */
	if (ret == AVERROR_EOF)
		vdec_ffmpeg_complete_flush(self);

	return 0;
}


static struct vbuf_buffer *
vdec_ffmpeg_buffer_copy_with_ps(struct vdec_ffmpeg *self,
				struct vbuf_buffer *in_buf)
{
	int ret;
	struct vbuf_buffer *out_buf;
	size_t capacity;
	ssize_t userdata_capacity, in_size;
	struct vbuf_cbs cbs;
	const uint8_t *in_data;
	uint8_t *out_data;

	if (!self->base->configured) {
		ULOG_ERRNO("configured", EAGAIN);
		return NULL;
	}

	in_data = vbuf_get_cdata(in_buf);
	in_size = vbuf_get_size(in_buf);
	if (in_size < 0) {
		ULOG_ERRNO("vbuf_get_size", (int)-in_size);
		return NULL;
	}
	capacity = self->ps_size + (size_t)in_size;
	userdata_capacity = vbuf_get_userdata_size(in_buf);
	if (userdata_capacity < 0) {
		ULOG_ERRNO("vbuf_get_userdata_size", (int)-userdata_capacity);
		return NULL;
	}

	/* Allocate the new buffer */
	ret = vbuf_generic_get_cbs(&cbs);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_generic_get_cbs", -ret);
		return NULL;
	}
	ret = vbuf_new(
		capacity, (size_t)userdata_capacity, &cbs, NULL, &out_buf);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_new:out_buf", -ret);
		return NULL;
	}

	/* Copy the data */
	out_data = vbuf_get_data(out_buf);
	memcpy(out_data, self->ps, self->ps_size);
	memcpy(out_data + self->ps_size, in_data, (size_t)in_size);
	vbuf_set_size(out_buf, capacity);

	/* Copy all metadata */
	ret = vbuf_metadata_copy(in_buf, out_buf, 0);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_copy", -ret);
		vbuf_unref(out_buf);
		return NULL;
	}

	/* Copy the user data */
	in_data = vbuf_get_cuserdata(in_buf);
	if ((in_data) && (userdata_capacity > 0)) {
		out_data = vbuf_get_userdata(out_buf);
		memcpy(out_data, in_data, userdata_capacity);
		vbuf_set_userdata_size(out_buf, userdata_capacity);
	}

	return out_buf;
}


static int vdec_ffmpeg_buffer_push_one(struct vdec_ffmpeg *self,
				       struct vbuf_buffer *in_buf)
{
	int ret;
	struct vdec_input_metadata *in_meta = NULL;
	struct timespec cur_ts = {0, 0};
	uint64_t timestamp;
	enum vdec_input_format format;

	if (self->base->config.sync_decoding) {
		ULOGE("push_one not supported in synchronized decoding mode");
		return -ENOSYS;
	}

	ret = vbuf_metadata_get(
		in_buf, self->base, NULL, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get:input", -ret);
		return ret;
	}
	if (in_meta->format != VDEC_INPUT_FORMAT_BYTE_STREAM) {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported format: %s",
			   -ret,
			   vdec_input_format_str(in_meta->format));
		return ret;
	}
	format = in_meta->format;

	timestamp = in_meta->timestamp;
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &in_meta->dequeue_time);

	/* Discard the buffer if the decoder is not configured */
	if (!self->base->configured) {
		vbuf_unref(in_buf);
		return 0;
	}

	/* Copy the buffer adding SPS/PPS if needed */
	if (self->needs_ps) {
		struct vbuf_buffer *buf;
		buf = vdec_ffmpeg_buffer_copy_with_ps(self, in_buf);
		vbuf_unref(in_buf);
		if (buf == NULL)
			return 0;
		in_buf = buf;
		self->needs_ps = 0;
	}

	/* Debug files */
	if (self->base->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_h264_frame(
			self->base->dbg.input_bs, in_buf, format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_h264_frame", -dbgret);
	}
	if (self->base->dbg.h264_analysis != NULL) {
		int dbgret =
			vdec_dbg_parse_h264_frame(self->base, in_buf, format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_h264_frame", -dbgret);
	}

	/* Push the frame */
	self->avpacket.data = (uint8_t *)vbuf_get_cdata(in_buf);
	self->avpacket.size = vbuf_get_size(in_buf);
	self->avpacket.pts = timestamp;
	ret = avcodec_send_packet(self->avcodec, &self->avpacket);
	if (ret < 0) {
		ULOG_ERRNO("avcodec_send_packet", -ret);
		vbuf_unref(in_buf);
		return ret;
	}
	ret = vbuf_queue_push(self->decoder_queue, in_buf);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_push:decoder", -ret);
		vbuf_unref(in_buf);
		return ret;
	}

	vbuf_unref(in_buf);
	return 0;
}


static int vdec_ffmpeg_discard_frame(struct vdec_ffmpeg *self)
{
	int ret, avcodec_ret;
	struct vbuf_buffer *in_buf = NULL;
	struct vdec_input_metadata *in_meta = NULL;

	/* Dequeue the frame from the decoder and drop it */
	avcodec_ret = avcodec_receive_frame(self->avcodec, self->dummy_frame);
	if ((avcodec_ret == -EAGAIN) || (avcodec_ret == AVERROR_EOF)) {
		/* No frame available (all frames were output),
		 * do not dequeue the input buffer */
		if ((avcodec_ret == AVERROR_EOF) && (self->flushing))
			vdec_ffmpeg_complete_flush(self);
		return avcodec_ret;
	} else if (avcodec_ret < 0) {
		ULOG_ERRNO("avcodec_receive_frame", -avcodec_ret);
	}

	/* Get the input buffer (non-blocking);
	 * drop input buffers until the correct timestamp is met,
	 * in case ffmpeg has internally dropped some frames */
	do {
		if (in_buf != NULL)
			vbuf_unref(in_buf);
		ret = vbuf_queue_pop(self->decoder_queue, 0, &in_buf);
		if ((ret < 0) || (in_buf == NULL)) {
			ULOG_ERRNO("vbuf_queue_pop:decoder", -ret);
			break;
		}
		ret = vbuf_metadata_get(
			in_buf, self->base, NULL, NULL, (uint8_t **)&in_meta);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_metadata_get:decoder", -ret);
			break;
		}
	} while (self->dummy_frame->pts > (int64_t)in_meta->timestamp);

	if (in_buf != NULL)
		vbuf_unref(in_buf);

	return 0;
}


static int vdec_ffmpeg_buffer_pop_all(struct vdec_ffmpeg *self)
{
	int ret, err, avcodec_ret;
	AVFrame *frame;
	struct vbuf_buffer *in_buf = NULL;
	struct vbuf_buffer *out_buf = NULL;
	struct vdec_input_metadata *in_meta = NULL;

	if (self->base->config.sync_decoding) {
		ULOGE("pop_all not supported in synchronized decoding mode");
		return -ENOSYS;
	}

	do {
		/* Discard frames if flush is in progress */
		if ((self->flushing) && (self->flush_discard)) {
			err = vdec_ffmpeg_discard_frame(self);
			if (err == AVERROR_EOF)
				break;
			else
				continue;
		}

		/* Get an output buffer (non-blocking) */
		ret = vbuf_pool_get(self->out_pool, 0, &out_buf);
		if ((ret < 0) || (out_buf == NULL)) {
			ULOGW("%s:%d: failed to get an output buffer "
			      "(frame dropped): %d(%s)",
			      __func__,
			      __LINE__,
			      -ret,
			      strerror(-ret));

			err = vdec_ffmpeg_discard_frame(self);
			return ret;
		}
		frame = vdec_ffmpeg_avframe_get_frame(out_buf);

		/* Dequeue the frame from the decoder */
		avcodec_ret = avcodec_receive_frame(self->avcodec, frame);
		if ((avcodec_ret == -EAGAIN) || (avcodec_ret == AVERROR_EOF)) {
			/* No frame available (all frames were output),
			 * do not dequeue the input buffer */
			if ((avcodec_ret == AVERROR_EOF) && (self->flushing))
				vdec_ffmpeg_complete_flush(self);
			vbuf_unref(out_buf);
			break;
		}

		/* Get the input buffer (non-blocking);
		 * drop input buffers until the correct timestamp is met,
		 * in case ffmpeg has internally dropped some frames */
		do {
			if (in_buf != NULL)
				vbuf_unref(in_buf);
			ret = vbuf_queue_pop(self->decoder_queue, 0, &in_buf);
			if ((ret < 0) || (in_buf == NULL)) {
				ULOG_ERRNO("vbuf_queue_pop:decoder", -ret);
				vbuf_unref(out_buf);
				return ret;
			}
			ret = vbuf_metadata_get(in_buf,
						self->base,
						NULL,
						NULL,
						(uint8_t **)&in_meta);
			if (ret < 0) {
				ULOG_ERRNO("vbuf_metadata_get:decoder", -ret);
				vbuf_unref(in_buf);
				vbuf_unref(out_buf);
				return ret;
			}
		} while (frame->pts > (int64_t)in_meta->timestamp);

		if (avcodec_ret < 0) {
			/* Decoding error,
			 * unref the input buffer */
			ULOG_ERRNO("avcodec_receive_frame", -avcodec_ret);
			vbuf_unref(in_buf);
			vbuf_unref(out_buf);
			return avcodec_ret;
		}

		/* Set the metadata */
		ret = vdec_ffmpeg_set_frame_metadata(
			self, in_buf, out_buf, frame);
		if (ret < 0) {
			vbuf_unref(in_buf);
			vbuf_unref(out_buf);
			return ret;
		}

		/* Push the frame (if not silent) */
		ret = vbuf_write_lock(out_buf);
		if (ret < 0)
			ULOG_ERRNO("vbuf_write_lock", -ret);
		if ((in_meta->silent) &&
		    (!self->base->config.output_silent_frames)) {
			ULOGD("silent frame (ignored)");
		} else {
			(*self->base->cbs.frame_output)(
				self->base, 0, out_buf, self->base->userdata);
		}

		/* Unref the buffers */
		vbuf_unref(in_buf);
		in_buf = NULL;
		vbuf_unref(out_buf);
		out_buf = NULL;
	} while (((!self->flush) && (!self->should_stop)) || (self->flushing));

	return 0;
}


static void *vdec_ffmpeg_decoder_thread(void *ptr)
{
	int ret;
	struct vdec_ffmpeg *self = (struct vdec_ffmpeg *)ptr;
	struct vbuf_buffer *in_buf = NULL;
	int timeout;

	if (self->base->config.sync_decoding) {
		ULOGE("decoder thread not supported in "
		      "synchronized decoding mode");
		return NULL;
	}

	while ((!self->should_stop) || (self->flush)) {
		/* Start flush, discarding all frames */
		if ((self->flush) && (self->flush_discard)) {
			ret = vdec_ffmpeg_start_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_ffmpeg_start_flush", -ret);
			goto pop;
		}

		/* Get an input buffer (with timeout) */
		timeout = ((self->flush) && (!self->flush_discard)) ? 0 : 5;
		ret = vbuf_queue_pop(self->in_queue, timeout, &in_buf);
		if (ret == -ETIMEDOUT) {
			if (self->base->configured)
				goto pop;
			else
				continue;
		} else if ((self->flush) && (ret == -EAGAIN)) {
			/* Flush without discarding frames */
			ret = vdec_ffmpeg_start_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_ffmpeg_start_flush", -ret);
			goto pop;
		} else if ((ret < 0) || (in_buf == NULL)) {
			if (!self->should_stop) {
				ULOG_ERRNO("vbuf_queue_pop:input", -ret);
				/* Avoid looping on errors */
				usleep(5000);
			}
			continue;
		}

		/* Push the input frame */
		ret = vdec_ffmpeg_buffer_push_one(self, in_buf);
		if (ret < 0) {
			ULOG_ERRNO("vdec_ffmpeg_buffer_push_one", -ret);
			continue;
		}

	/* codecheck_ignore[INDENTED_LABEL] */
	pop:
		/* Pop output frames */
		ret = vdec_ffmpeg_buffer_pop_all(self);
		if (ret < 0) {
			ULOG_ERRNO("vdec_ffmpeg_buffer_pop_all", -ret);
			continue;
		}
	}

	/* Call the stop callback if defined */
	if (self->base->cbs.stop)
		(*self->base->cbs.stop)(self->base, self->base->userdata);

	return NULL;
}


uint32_t vdec_ffmpeg_get_supported_input_format(void)
{
	return VDEC_INPUT_FORMAT_BYTE_STREAM;
}


int vdec_ffmpeg_new(struct vdec_decoder *base)
{
	int ret = 0;
	struct vdec_ffmpeg *self = NULL;
	AVCodec *codec = NULL;
	struct vbuf_cbs out_buf_cbs;
	unsigned int out_buf_count;
	unsigned int ver = avcodec_version();

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	/* Check the configuration */
	if (base->config.encoding != VDEC_ENCODING_H264) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid encoding", -ret);
		return ret;
	}
	if ((!base->config.sync_decoding) && (base->cbs.frame_output == NULL)) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid frame output callback", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = (struct vdec_derived *)self;

	/* Find the decoder */
	avcodec_register_all();
	av_log_set_level(AV_LOG_VERBOSE);
	if (codec == NULL) {
		codec = avcodec_find_decoder_by_name("h264_cuvid");
		if (codec != NULL) {
			ULOGI("libavcodec version=%u.%u.%u - using "
			      "NVDec (cuvid) H.264 hardware acceleration",
			      (ver >> 16) & 0xFF,
			      (ver >> 8) & 0xFF,
			      ver & 0xFF);
			self->output_format = VDEC_OUTPUT_FORMAT_NV12;
		}
	}
	if (codec == NULL) {
		codec = avcodec_find_decoder(AV_CODEC_ID_H264);
		if (codec != NULL) {
			ULOGI("libavcodec version=%u.%u.%u - using "
			      "CPU H.264 decoding",
			      (ver >> 16) & 0xFF,
			      (ver >> 8) & 0xFF,
			      ver & 0xFF);
			self->output_format = VDEC_OUTPUT_FORMAT_I420;
		}
	}
	if (codec == NULL) {
		ret = -ENOENT;
		ULOG_ERRNO("codec not found", -ret);
		goto error;
	}

	/* Initialize the decoder */
	self->avcodec = avcodec_alloc_context3(codec);
	if (self->avcodec == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("avcodec_alloc_context3", -ret);
		goto error;
	}

	self->avcodec->pix_fmt = AV_PIX_FMT_YUV420P;
	self->avcodec->skip_frame = AVDISCARD_DEFAULT;
	self->avcodec->error_concealment =
		FF_EC_GUESS_MVS | FF_EC_DEBLOCK | FF_EC_FAVOR_INTER;
	self->avcodec->skip_loop_filter = AVDISCARD_DEFAULT;
	self->avcodec->workaround_bugs = FF_BUG_AUTODETECT;
	self->avcodec->codec_type = AVMEDIA_TYPE_VIDEO;
	self->avcodec->codec_id = AV_CODEC_ID_H264;
	self->avcodec->skip_idct = AVDISCARD_DEFAULT;
	self->avcodec->thread_type = FF_THREAD_SLICE;
	if ((!base->config.low_delay) && (!base->config.sync_decoding))
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

	ret = avcodec_open2(self->avcodec, codec, NULL);
	if (ret < 0) {
		ULOG_ERRNO("avcodec_open2", -ret);
		goto error;
	}

	av_init_packet(&self->avpacket);
	self->dummy_frame = av_frame_alloc();
	if (self->dummy_frame == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("av_frame_alloc", -ret);
		goto error;
	}

	/* Allocate the output buffers pool */
	memset(&out_buf_cbs, 0, sizeof(out_buf_cbs));
	out_buf_cbs.alloc = &vdec_ffmpeg_avframe_alloc_cb;
	out_buf_cbs.free = &vdec_ffmpeg_avframe_free_cb;
	out_buf_count = VDEC_FFMPEG_OUT_POOL_DEFAULT_MIN_BUF_COUNT;
	if (base->config.preferred_min_out_buf_count > out_buf_count)
		out_buf_count = base->config.preferred_min_out_buf_count;
	ret = vbuf_pool_new(out_buf_count, 0, 0, &out_buf_cbs, &self->out_pool);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_pool_new:output", -ret);
		goto error;
	}

	if (!base->config.sync_decoding) {
		/* Create the input buffers queue */
		ret = vbuf_queue_new(0, 0, &self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_new:input", -ret);
			goto error;
		}

		/* Create the decoder queue */
		ret = vbuf_queue_new(0, 0, &self->decoder_queue);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_new:decoder", -ret);
			goto error;
		}

		/* Create the decoding thread */
		ret = pthread_create(&self->thread,
				     NULL,
				     vdec_ffmpeg_decoder_thread,
				     (void *)self);
		if (ret != 0) {
			ret = -ret;
			ULOG_ERRNO("pthread_create", -ret);
			goto error;
		} else {
			self->thread_launched = 1;
		}
	}

	return 0;

error:
	/* Cleanup on error */
	vdec_ffmpeg_destroy(base);
	base->derived = NULL;
	return ret;
}


int vdec_ffmpeg_flush(struct vdec_decoder *base, int discard)
{
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	if (base->config.sync_decoding) {
		ULOGE("flush not supported in synchronized decoding mode");
		return -ENOSYS;
	}

	self = (struct vdec_ffmpeg *)base->derived;

	self->flush = 1;
	self->flush_discard = discard;

	return 0;
}


int vdec_ffmpeg_stop(struct vdec_decoder *base)
{
	int err;
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	if (base->config.sync_decoding) {
		ULOGE("stop not supported in synchronized decoding mode");
		return -ENOSYS;
	}

	self = (struct vdec_ffmpeg *)base->derived;

	/* Stop the decoding thread */
	self->should_stop = 1;
	self->base->configured = 0;
	if (self->in_queue != NULL) {
		err = vbuf_queue_abort(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_abort:input", -err);
	}
	if (self->decoder_queue != NULL) {
		err = vbuf_queue_abort(self->decoder_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_abort:decoder", -err);
	}
	if (self->out_pool != NULL) {
		err = vbuf_pool_abort(self->out_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_abort:output", -err);
	}

	return 0;
}


int vdec_ffmpeg_destroy(struct vdec_decoder *base)
{
	int err;
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = (struct vdec_ffmpeg *)base->derived;
	if (self == NULL)
		return 0;

	if (base->config.sync_decoding == 0) {
		/* Stop and join the decoding thread */
		err = vdec_ffmpeg_stop(base);
		if (err < 0)
			ULOG_ERRNO("vdec_ffmpeg_stop", -err);
		if (self->thread_launched) {
			err = pthread_join(self->thread, NULL);
			if (err != 0)
				ULOG_ERRNO("pthread_join", err);
		}
	}

	/* Free the resources */
	if (self->out_pool != NULL) {
		err = vbuf_pool_abort(self->out_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_abort:output", -err);
		err = vbuf_pool_destroy(self->out_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_destroy:output", -err);
	}
	if (self->decoder_queue != NULL) {
		err = vbuf_queue_abort(self->decoder_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_abort:decoder", -err);
		err = vbuf_queue_destroy(self->decoder_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_destroy:decoder", -err);
	}
	if (self->in_queue != NULL) {
		err = vbuf_queue_abort(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_abort:input", -err);
		err = vbuf_queue_destroy(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_destroy:input", -err);
	}
	if (self->avcodec != NULL)
		avcodec_free_context(&self->avcodec);
	if (self->dummy_frame != NULL)
		av_frame_free(&self->dummy_frame);
	free(self->ps);
	free(self);

	return 0;
}


int vdec_ffmpeg_set_sps_pps(struct vdec_decoder *base,
			    const uint8_t *sps,
			    size_t sps_size,
			    const uint8_t *pps,
			    size_t pps_size,
			    enum vdec_input_format format)
{
	struct vdec_ffmpeg *self;
	size_t offset = (format == VDEC_INPUT_FORMAT_RAW_NALU) ? 0 : 4;
	size_t off;
	uint32_t start = htonl(0x00000001);

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);

	self = (struct vdec_ffmpeg *)base->derived;

	/* Copy the SPS/PPS locally */
	self->ps_size = 4 + sps_size - offset + 4 + pps_size - offset;
	self->ps = malloc(self->ps_size);
	if (self->ps == NULL)
		return -ENOMEM;
	off = 0;

	/* SPS */
	memcpy(self->ps + off, &start, sizeof(uint32_t));
	off += 4;
	memcpy(self->ps + off, sps + offset, sps_size - offset);
	off += sps_size - offset;

	/* PPS */
	memcpy(self->ps + off, &start, sizeof(uint32_t));
	off += 4;
	memcpy(self->ps + off, pps + offset, pps_size - offset);
	off += pps_size - offset;

	self->needs_ps = 1;

	return 0;
}


struct vbuf_pool *vdec_ffmpeg_get_input_buffer_pool(struct vdec_decoder *base)
{
	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


struct vbuf_queue *vdec_ffmpeg_get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_ffmpeg *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = (struct vdec_ffmpeg *)base->derived;

	/* Returns NULL in case we are in synchronized decoding mode */
	return self->in_queue;
}


int vdec_ffmpeg_sync_decode(struct vdec_decoder *base,
			    struct vbuf_buffer *in_buf,
			    struct vbuf_buffer **out_buf)
{
	int ret;
	struct vdec_ffmpeg *self;
	AVFrame *frame;
	struct vbuf_buffer *buf = in_buf;
	struct vdec_input_metadata *in_meta = NULL;
	struct timespec cur_ts = {0, 0};

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(in_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(out_buf == NULL, EINVAL);

	if (!base->config.sync_decoding) {
		ULOGE("sync_decode not supported when not in "
		      "synchronized decoding mode");
		return -ENOSYS;
	}

	if (!base->configured) {
		ret = -EAGAIN;
		ULOG_ERRNO("configured", -ret);
		return ret;
	}

	self = (struct vdec_ffmpeg *)base->derived;

	ret = vbuf_metadata_get(
		in_buf, self->base, NULL, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get:input", -ret);
		return ret;
	}
	if (in_meta->format != VDEC_INPUT_FORMAT_BYTE_STREAM) {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported format", -ret);
		return ret;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &in_meta->dequeue_time);

	/* Copy the buffer adding SPS/PPS if needed */
	if (self->needs_ps) {
		buf = vdec_ffmpeg_buffer_copy_with_ps(self, in_buf);
		if (buf == NULL)
			return -ENOMEM;
		self->needs_ps = 0;
	}

	/* Get an output buffer (non-blocking) */
	ret = vbuf_pool_get(self->out_pool, 0, out_buf);
	if ((ret < 0) || (*out_buf == NULL)) {
		ULOGW("%s:%d: failed to get an output buffer : %d(%s)",
		      __func__,
		      __LINE__,
		      -ret,
		      strerror(-ret));
		return ret;
	}
	frame = vdec_ffmpeg_avframe_get_frame(*out_buf);

	/* Debug files */
	if (base->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_h264_frame(
			base->dbg.input_bs, buf, in_meta->format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_h264_frame", -dbgret);
	}
	if (self->base->dbg.h264_analysis != NULL) {
		int dbgret = vdec_dbg_parse_h264_frame(
			self->base, buf, in_meta->format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_h264_frame", -dbgret);
	}

	/* Decode the frame */
	self->avpacket.data = (uint8_t *)vbuf_get_cdata(buf);
	self->avpacket.size = vbuf_get_size(buf);
	ret = avcodec_send_packet(self->avcodec, &self->avpacket);
	if (ret < 0) {
		ULOG_ERRNO("avcodec_send_packet", -ret);
		vbuf_unref(*out_buf);
		*out_buf = NULL;
		return -EIO;
	}
	ret = avcodec_receive_frame(self->avcodec, frame);
	if (ret < 0) {
		ULOG_ERRNO("avcodec_receive_frame", -ret);
		vbuf_unref(*out_buf);
		*out_buf = NULL;
		return -EIO;
	}

	/* Set the metadata */
	ret = vdec_ffmpeg_set_frame_metadata(self, buf, *out_buf, frame);
	if (ret < 0) {
		vbuf_unref(*out_buf);
		*out_buf = NULL;
		return ret;
	}

	ret = vbuf_write_lock(*out_buf);
	if (ret < 0)
		ULOG_ERRNO("vbuf_write_lock", -ret);

	return ((in_meta->silent) && (!base->config.output_silent_frames)) ? 0
									   : 1;
}

#endif /* BUILD_FFMPEG_LIBAV */

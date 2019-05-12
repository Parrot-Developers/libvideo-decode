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

#define ULOG_TAG vdec_mediacodec
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include <stdbool.h>

#include <pthread.h>

#include <media/NdkMediaCodec.h>
#include <media/NdkMediaFormat.h>

#include <futils/timetools.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <video-decode/vdec_internal.h>
#include <video-decode/vdec_mediacodec.h>

enum state {
	RUNNING,
	WAITING_FOR_STOP,
	WAITING_FOR_FLUSH,
};


struct vdec_mediacodec {
	struct vdec_decoder *base;

	AMediaCodec *mc;

	enum state state;

	struct mbuf_coded_video_frame_queue *in_queue;
	struct mbuf_coded_video_frame_queue *meta_queue;
	struct mbuf_raw_video_frame_queue *out_queue;
	struct pomp_evt *out_evt;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool stop_requested;
		bool flush_requested;
		bool eos_requested;
	} push;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool stop_requested;
		bool flush_requested;
		bool eos_requested;
	} pull;

	bool eos_requested;

	bool eos_pending;

	struct vdef_raw_format output_format;
	unsigned int stride;
	unsigned int slice_height;

	struct {
		unsigned int ref;
		unsigned int recovery_point;
		unsigned int fake_frame_num;
		unsigned int max_frame_num;
		struct h264_reader *h264_reader;
	} current_frame;

	bool need_sync;
	bool fake_frame_num;
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecCapabilities.html
 * for reference. */
enum color_format {
	YUV420_PLANAR = 0x00000013,
	YUV420_PACKED_PLANAR = 0x00000014,
	YUV420_SEMIPLANAR = 0x00000015,
	YUV420_PACKED_SEMIPLANAR = 0x00000027,
	TI_YUV420_PACKED_SEMIPLANAR = 0x7F000100,
	QCOM_YUV420_SEMIPLANAR = 0x7FA30C00,
	QCOM_YUV420_PACKED_SEMIPLANAR64X32_TILE2_M8KA = 0x7FA30C03,
	QCOM_YUV420_SEMIPLANAR32_M = 0x7FA30C04,
};


#define NB_SUPPORTED_FORMATS 2
static struct vdef_coded_format supported_formats[NB_SUPPORTED_FORMATS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_h264_byte_stream;
	supported_formats[1] = vdef_h265_byte_stream;
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
	struct vdec_mediacodec *self = base->derived;

	if (self->state == WAITING_FOR_STOP)
		return 0;

	pthread_mutex_lock(&self->push.mutex);
	self->push.stop_requested = true;
	pthread_cond_signal(&self->push.cond);
	pthread_mutex_unlock(&self->push.mutex);

	pthread_mutex_lock(&self->pull.mutex);
	self->pull.stop_requested = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);

	AMediaCodec_stop(self->mc);

	self->state = WAITING_FOR_STOP;

	return 0;
}


static int destroy(struct vdec_decoder *base)
{
	struct vdec_mediacodec *self = base->derived;
	if (self == NULL)
		return 0;

	AMediaCodec_stop(self->mc);

	if (self->pull.thread_created) {
		pthread_mutex_lock(&self->pull.mutex);
		self->pull.stop_requested = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);

		pthread_join(self->pull.thread, NULL);
	}

	if (self->push.thread_created) {
		pthread_mutex_lock(&self->push.mutex);
		self->push.stop_requested = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_join(self->push.thread, NULL);
	}

	if (self->out_evt != NULL) {
		if (pomp_evt_is_attached(self->out_evt, base->loop))
			pomp_evt_detach_from_loop(self->out_evt, base->loop);
		pomp_evt_destroy(self->out_evt);
	}

	if (self->out_queue != NULL)
		mbuf_raw_video_frame_queue_destroy(self->out_queue);

	if (self->meta_queue != NULL)
		mbuf_coded_video_frame_queue_destroy(self->meta_queue);

	if (self->in_queue != NULL) {
		struct pomp_evt *evt;
		mbuf_coded_video_frame_queue_get_event(self->in_queue, &evt);
		if (pomp_evt_is_attached(evt, base->loop))
			pomp_evt_detach_from_loop(evt, base->loop);
		mbuf_coded_video_frame_queue_destroy(self->in_queue);
	}

	if (self->current_frame.h264_reader) {
		int ret = h264_reader_destroy(self->current_frame.h264_reader);
		if (ret < 0)
			ULOG_ERRNO("h264_reader_destroy", -ret);
	}

	AMediaCodec_delete(self->mc);

	pthread_mutex_destroy(&self->push.mutex);
	pthread_cond_destroy(&self->push.cond);

	pthread_mutex_destroy(&self->pull.mutex);
	pthread_cond_destroy(&self->pull.cond);

	free(self);

	return 0;
}

static bool input_filter(struct mbuf_coded_video_frame *frame, void *userdata)
{
	int ret;
	const void *tmp;
	size_t tmplen;
	struct vdef_coded_frame info;

	struct vdec_mediacodec *self = userdata;

	if (self->state == WAITING_FOR_FLUSH || self->eos_pending)
		return false;

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


static void push_frame(struct vdec_mediacodec *self,
		       struct mbuf_coded_video_frame *frame)
{
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us, ts;
	int res = 0;
	const void *frame_data = NULL;
	size_t frame_len;
	struct vdef_coded_frame frame_info;
	ssize_t buf_idx;
	size_t buf_size;
	media_status_t status;
	uint8_t *buf;

	res = mbuf_coded_video_frame_get_frame_info(frame, &frame_info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		goto end;
	}

	ts_us = VDEF_ROUND(frame_info.info.timestamp * 1000000,
			   frame_info.info.timescale);

	if (self->base->config.encoding == VDEF_ENCODING_H264) {
		self->current_frame.ref = 0;
		self->current_frame.recovery_point = 0;
	}

	if (vdec_is_sync_frame(frame, &frame_info)) {
		ULOGI("frame is a sync point");
		self->need_sync = 0;
		self->current_frame.fake_frame_num = 0;
	} else if (self->need_sync) {
		if (self->base->config.encoding == VDEF_ENCODING_H264 &&
		    self->base->config.gen_grey_idr) {
			ULOGI("frame is not an IDR, generating grey IDR");
			size_t size = self->base->video_info.resolution.width *
				      self->base->video_info.resolution.height *
				      3 / 4;
			struct mbuf_mem *mem;
			uint64_t delta;
			uint64_t timestamp = frame_info.info.timestamp;
			res = mbuf_mem_generic_new(size, &mem);
			if (res < 0) {
				ULOG_ERRNO("mbuf_mem_generic_new", -res);
				goto end;
			}

			struct mbuf_coded_video_frame *idr_frame;
			res = vdec_h264_write_grey_idr(self->base,
						       &frame_info,
						       &delta,
						       &timestamp,
						       mem,
						       &idr_frame);
			mbuf_mem_unref(mem);
			if (res < 0) {
				ULOG_ERRNO("vdec_h264_write_grey_idr", -res);
				mbuf_coded_video_frame_unref(idr_frame);
				goto end;
			}

			const void *idr_data = NULL;
			size_t idr_len;
			res = mbuf_coded_video_frame_get_packed_buffer(
				idr_frame, &idr_data, &idr_len);
			if (res < 0) {
				ULOG_ERRNO(
					"mbuf_coded_video_frame_get_packed_buffer",
					-res);
				mbuf_coded_video_frame_unref(idr_frame);
				goto end;
			}

			ssize_t idr_buf_idx;
			size_t idr_buf_size;
			idr_buf_idx =
				AMediaCodec_dequeueInputBuffer(self->mc, -1);
			if (idr_buf_idx < 0) {
				mbuf_coded_video_frame_release_packed_buffer(
					idr_frame, idr_data);
				mbuf_coded_video_frame_unref(idr_frame);
				goto end;
			}

			uint8_t *idr_buf;
			idr_buf = AMediaCodec_getInputBuffer(
				self->mc, idr_buf_idx, &idr_buf_size);
			if ((idr_buf == NULL) || (idr_buf_size < idr_len)) {
				if (idr_buf == NULL)
					ULOGE("idr: null codec buffer");
				if (idr_buf_size < idr_len)
					ULOGE("idr: codec buffer too small");
				(void)AMediaCodec_queueInputBuffer(
					self->mc, idr_buf_idx, 0, 0, 0, 0);
				mbuf_coded_video_frame_release_packed_buffer(
					idr_frame, idr_data);
				mbuf_coded_video_frame_unref(idr_frame);
				goto end;
			}

			memcpy(idr_buf, idr_data, idr_len);

			status = AMediaCodec_queueInputBuffer(self->mc,
							      idr_buf_idx,
							      0,
							      idr_len,
							      timestamp,
							      0);

			mbuf_coded_video_frame_release_packed_buffer(idr_frame,
								     idr_data);
			mbuf_coded_video_frame_unref(idr_frame);
			if (status != AMEDIA_OK) {
				ULOGE("AMediaCodec_queueInputBuffer");
				goto end;
			}

			self->need_sync = false;
			self->current_frame.fake_frame_num = 1;
			ts_us += delta;
		} else {
			ULOGI("frame is not a sync point, discarding frame");
			goto end;
		}
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts);

	res = mbuf_coded_video_frame_add_ancillary_buffer(
		frame, VDEC_ANCILLARY_KEY_DEQUEUE_TIME, &ts, sizeof(ts));
	if (res < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -res);

	res = mbuf_coded_video_frame_get_packed_buffer(
		frame, &frame_data, &frame_len);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer", -res);
		goto end;
	}

	buf_idx = AMediaCodec_dequeueInputBuffer(self->mc, -1);
	if (buf_idx < 0) {
		/* We do not log an error here because we might have interrupted
		 * the wait intentionally from another thread with `flush` or
		 * `stop`. */
		/* TODO: how do we know if something unusual happened and
		 * an input frame is dropped? */
		goto end;
	}

	buf = AMediaCodec_getInputBuffer(self->mc, buf_idx, &buf_size);
	if (buf == NULL) {
		ULOGE("null codec buffer");
		goto end;
	}
	if (buf_size < frame_len) {
		ULOGE("codec buffer too small");
		goto end;
	}

	memcpy(buf, frame_data, frame_len);

	/* Modify the frame_num */
	/* TODO: provide AMediaCodec buffers in our own input pool */
	if (self->base->config.encoding == VDEF_ENCODING_H264 &&
	    self->fake_frame_num) {
		size_t off = 0;
		res = h264_reader_parse(self->current_frame.h264_reader,
					0,
					buf,
					frame_len,
					&off);
		if (res < 0) {
			ULOG_ERRNO("h264_reader_parse", -res);
			goto end;
		}

		if (self->current_frame.ref) {
			/* Update the fake frame_num */
			self->current_frame.fake_frame_num =
				(self->current_frame.recovery_point)
					? 1
					: (self->current_frame.fake_frame_num +
					   1) % self->current_frame
							  .max_frame_num;
		}
	}

	status = AMediaCodec_queueInputBuffer(
		self->mc, buf_idx, 0, frame_len, ts_us, 0);
	if (status != AMEDIA_OK) {
		ULOGE("AMediaCodec_queueInputBuffer");
		goto end;
	}

	res = mbuf_coded_video_frame_queue_push(self->meta_queue, frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push", -res);
		goto end;
	}

	pthread_mutex_lock(&self->pull.mutex);
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);

end:
	if (frame_data)
		mbuf_coded_video_frame_release_packed_buffer(frame, frame_data);
	mbuf_coded_video_frame_unref(frame);
}


/*
 * See
 * https://developer.android.com/reference/android/media/MediaCodec.html#end-of-stream-handling
 */
static void push_eos(struct vdec_mediacodec *self)
{
	ssize_t buf_idx = AMediaCodec_dequeueInputBuffer(self->mc, -1);
	if (buf_idx < 0)
		return;
	media_status_t status = AMediaCodec_queueInputBuffer(
		self->mc,
		buf_idx,
		0,
		0,
		0,
		AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM);
	if (status != AMEDIA_OK)
		ULOGE("AmediaCodec_queueInputBuffer");

	pthread_mutex_lock(&self->pull.mutex);
	self->pull.eos_requested = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);
}


static void on_input_event(struct pomp_evt *evt, void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	pthread_mutex_lock(&self->push.mutex);
	pthread_cond_signal(&self->push.cond);
	pthread_mutex_unlock(&self->push.mutex);
}


static void *push_routine(void *arg)
{
	struct vdec_mediacodec *self = arg;

	pthread_mutex_lock(&self->push.mutex);
	while (true) {
		if (self->push.stop_requested) {
			self->push.stop_requested = false;
			pomp_evt_signal(self->out_evt);
			pthread_mutex_unlock(&self->push.mutex);
			break;
		}

		if (self->push.flush_requested) {
			self->push.flush_requested = false;
			pomp_evt_signal(self->out_evt);
			pthread_cond_wait(&self->push.cond, &self->push.mutex);
			continue;
		}

		struct mbuf_coded_video_frame *frame;
		int res = mbuf_coded_video_frame_queue_pop(self->in_queue,
							   &frame);
		if (res == 0) {
			pthread_mutex_unlock(&self->push.mutex);
			push_frame(self, frame);
			pthread_mutex_lock(&self->push.mutex);
			continue;
		} else if (res == -EAGAIN && self->push.eos_requested) {
			self->push.eos_requested = false;
			pthread_mutex_unlock(&self->push.mutex);
			push_eos(self);
			pthread_mutex_lock(&self->push.mutex);
			continue;
		} else if (res != -EAGAIN) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_pop", -res);
		}

		pthread_cond_wait(&self->push.cond, &self->push.mutex);
	}

	return NULL;
}


static void update_output_format(struct vdec_mediacodec *self)
{
	AMediaFormat *format = AMediaCodec_getOutputFormat(self->mc);

	int32_t color_format;
	if (AMediaFormat_getInt32(format, "color-format", &color_format)) {
		switch (color_format) {
		case YUV420_PLANAR:
		case YUV420_PACKED_PLANAR:
			self->output_format = vdef_i420;
			break;
		case YUV420_SEMIPLANAR:
		case YUV420_PACKED_SEMIPLANAR:
		case TI_YUV420_PACKED_SEMIPLANAR:
		case QCOM_YUV420_SEMIPLANAR:
		case QCOM_YUV420_SEMIPLANAR32_M:
			self->output_format = vdef_nv12;
			break;
		default:
			break;
		}
	} else {
		ULOGE("invalid format declaration: missing color format");
	}

	int32_t stride;
	if (AMediaFormat_getInt32(format, "stride", &stride))
		self->stride = stride;
	else
		ULOGE("invalid format declaration: missing stride");


	int32_t slice_height;
	if (AMediaFormat_getInt32(format, "slice-height", &slice_height))
		self->slice_height = slice_height;
	else
		self->slice_height = self->base->video_info.resolution.height;

	free(format);
}


static void release_mc_mem(void *data, size_t len, void *userdata)
{
	size_t idx = len;
	AMediaCodec *mc = userdata;

	AMediaCodec_releaseOutputBuffer(mc, idx, false);
}


static void pull_frame(struct vdec_mediacodec *self,
		       struct mbuf_coded_video_frame *meta_frame)
{
	int res = 0;
	AMediaCodecBufferInfo buffer_info;
	size_t buffer_index = SIZE_MAX;
	struct mbuf_mem *mem = NULL;
	struct mbuf_raw_video_frame *out_frame = NULL;
	struct vdef_coded_frame meta_info;
	size_t out_size;
	uint8_t *out_data;
	struct vdef_raw_frame out_info;
	struct vmeta_frame *metadata = NULL;
	struct timespec cur_ts;
	uint64_t ts_us;

	while (true) {
		ssize_t status = AMediaCodec_dequeueOutputBuffer(
			self->mc, &buffer_info, -1);

		if (status == AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
			update_output_format(self);
		} else if (status == -3) {
			/*
			 * Output buffers changed message that we ignore. See
			 * https://developer.android.com/reference/android/media/MediaCodec#INFO_OUTPUT_BUFFERS_CHANGED
			 */
		} else if (status < 0) {
			/* We do not log an error here because we might have
			 * interrupted the wait intentionally from another
			 * thread with `flush` or `stop`. */
			/* TODO: how do we know if something unusual
			 * happened? */
			goto end;
		} else {
			buffer_index = status;
			break;
		}
	}

	res = mbuf_coded_video_frame_get_frame_info(meta_frame, &meta_info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		goto end;
	}

	out_data =
		AMediaCodec_getOutputBuffer(self->mc, buffer_index, &out_size) +
		buffer_info.offset;

	/* Since we need to store the buffer index but not the length, we hijack
	 * the mem’s `len` field to store this index. */
	res = mbuf_mem_generic_wrap(
		out_data, buffer_index, release_mc_mem, self->mc, &mem);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_generic_wrap", -res);
		goto end;
	}

	out_info = (struct vdef_raw_frame){
		.info = meta_info.info,
		.format = self->output_format,
	};

	out_info.info.bit_depth = self->base->video_info.bit_depth;
	out_info.info.full_range = self->base->video_info.full_range;
	out_info.info.color_primaries = self->base->video_info.color_primaries;
	out_info.info.transfer_function =
		self->base->video_info.transfer_function;
	out_info.info.matrix_coefs = self->base->video_info.matrix_coefs;
	out_info.info.resolution.width = self->base->video_info.crop.width;
	out_info.info.resolution.height = self->base->video_info.crop.height;
	out_info.info.sar = self->base->video_info.sar;

	if (vdef_raw_format_cmp(&out_info.format, &vdef_nv12)) {
		out_info.plane_stride[0] = self->stride;
		out_info.plane_stride[1] = self->stride;
	} else if (vdef_raw_format_cmp(&out_info.format, &vdef_i420)) {
		out_info.plane_stride[0] = self->stride;
		out_info.plane_stride[1] = self->stride / 2;
		out_info.plane_stride[2] = self->stride / 2;
	}

	res = mbuf_raw_video_frame_new(&out_info, &out_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_new", -res);
		goto end;
	}

	res = mbuf_coded_video_frame_foreach_ancillary_data(
		meta_frame,
		mbuf_raw_video_frame_ancillary_data_copier,
		out_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data", -res);
		goto end;
	}

	res = mbuf_coded_video_frame_get_metadata(meta_frame, &metadata);
	if (res == 0) {
		res = mbuf_raw_video_frame_set_metadata(out_frame, metadata);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_metadata", -res);
			goto end;
		}
	} else if (res == -ENOENT) {
		res = 0;
	} else {
		ULOG_ERRNO("mbuf_coded_video_frame_get_metadata", -res);
		goto end;
	}

	res = mbuf_raw_video_frame_set_plane(
		out_frame,
		0,
		mem,
		0,
		out_info.plane_stride[0] * out_info.info.resolution.height);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
		goto end;
	}

	if (vdef_raw_format_cmp(&out_info.format, &vdef_nv12)) {
		res = mbuf_raw_video_frame_set_plane(
			out_frame,
			1,
			mem,
			self->stride * self->slice_height,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
			goto end;
		}
	} else if (vdef_raw_format_cmp(&out_info.format, &vdef_i420)) {
		res = mbuf_raw_video_frame_set_plane(
			out_frame,
			1,
			mem,
			self->stride * self->slice_height,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
			goto end;
		}
		res = mbuf_raw_video_frame_set_plane(
			out_frame,
			2,
			mem,
			(self->stride * self->slice_height * 5) / 4,
			out_info.plane_stride[2] *
				out_info.info.resolution.height / 2);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
			goto end;
		}
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	res = mbuf_raw_video_frame_add_ancillary_buffer(
		out_frame,
		VDEC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer", -res);
		goto end;
	}

	res = mbuf_raw_video_frame_finalize(out_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_finalize", -res);
		goto end;
	}

	/* Push the frame (if not silent) */
	if ((out_info.info.flags & VDEF_FRAME_FLAG_SILENT) &&
	    (!self->base->config.output_silent_frames)) {
		ULOGD("silent frame (ignored)");
	} else {
		mbuf_raw_video_frame_queue_push(self->out_queue, out_frame);
		pomp_evt_signal(self->out_evt);
	}

end:
	if (metadata)
		vmeta_frame_unref(metadata);
	if (mem)
		mbuf_mem_unref(mem);
	else if (buffer_index != SIZE_MAX)
		AMediaCodec_releaseOutputBuffer(self->mc, buffer_index, false);
	if (out_frame)
		mbuf_raw_video_frame_unref(out_frame);
	mbuf_coded_video_frame_unref(meta_frame);
}


static void *pull_routine(void *arg)
{
	struct vdec_mediacodec *self = arg;

	pthread_mutex_lock(&self->pull.mutex);
	while (true) {
		if (self->pull.stop_requested) {
			self->pull.stop_requested = false;
			pomp_evt_signal(self->out_evt);
			pthread_mutex_unlock(&self->pull.mutex);
			break;
		}

		if (self->pull.flush_requested) {
			self->pull.flush_requested = false;
			pomp_evt_signal(self->out_evt);
			pthread_cond_wait(&self->pull.cond, &self->pull.mutex);
			continue;
		}

		struct mbuf_coded_video_frame *frame;
		int res = mbuf_coded_video_frame_queue_pop(self->meta_queue,
							   &frame);
		if (res == 0) {
			pthread_mutex_unlock(&self->pull.mutex);
			pull_frame(self, frame);
			pthread_mutex_lock(&self->pull.mutex);
			continue;
		} else if (res == -EAGAIN && self->pull.eos_requested) {
			self->pull.eos_requested = false;
			self->eos_requested = true;
			pomp_evt_signal(self->out_evt);
		} else if (res != -EAGAIN) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_pop", -res);
		}

		pthread_cond_wait(&self->pull.cond, &self->pull.mutex);
	}

	return NULL;
}


static void on_output_event(struct pomp_evt *evt, void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	switch (self->state) {
	case RUNNING: {
		while (true) {
			struct mbuf_raw_video_frame *frame;
			int res = mbuf_raw_video_frame_queue_pop(
				self->out_queue, &frame);
			if (res < 0) {
				if (res != -EAGAIN)
					ULOG_ERRNO("mbuf_raw_video_frame_pop",
						   -res);
				break;
			}
			self->base->cbs.frame_output(
				self->base, 0, frame, self->base->userdata);
			mbuf_raw_video_frame_unref(frame);
		}

		pthread_mutex_lock(&self->pull.mutex);
		bool eos_requested = self->eos_requested;
		self->eos_requested = false;
		pthread_mutex_unlock(&self->pull.mutex);

		if (eos_requested) {
			AMediaCodec_flush(self->mc);
			self->eos_pending = false;
			self->need_sync = 1;
			if (self->base->cbs.flush)
				self->base->cbs.flush(self->base,
						      self->base->userdata);
		}
		break;
	}
	case WAITING_FOR_FLUSH:
		pthread_mutex_lock(&self->push.mutex);
		pthread_mutex_lock(&self->pull.mutex);
		bool flush_complete = !self->push.flush_requested &&
				      !self->pull.flush_requested;
		pthread_mutex_unlock(&self->pull.mutex);
		pthread_mutex_unlock(&self->push.mutex);

		if (flush_complete) {
			self->state = RUNNING;

			mbuf_coded_video_frame_queue_flush(self->in_queue);
			mbuf_coded_video_frame_queue_flush(self->meta_queue);
			mbuf_raw_video_frame_queue_flush(self->out_queue);
			AMediaCodec_flush(self->mc);

			if (self->base->cbs.flush != NULL)
				self->base->cbs.flush(self->base,
						      self->base->userdata);
		}
		break;
	case WAITING_FOR_STOP:
		pthread_mutex_lock(&self->push.mutex);
		pthread_mutex_lock(&self->pull.mutex);
		bool stop_complete = !self->push.stop_requested &&
				     !self->pull.stop_requested;
		pthread_mutex_unlock(&self->pull.mutex);
		pthread_mutex_unlock(&self->push.mutex);

		if (stop_complete) {
			if (self->base->cbs.stop != NULL)
				self->base->cbs.stop(self->base,
						     self->base->userdata);
		}
		break;
	default:
		break;
	}
}


static void nalu_end_cb(struct h264_ctx *ctx,
			enum h264_nalu_type type,
			const uint8_t *buf,
			size_t len,
			const struct h264_nalu_header *nh,
			void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	self->current_frame.ref = (((type == H264_NALU_TYPE_SLICE) ||
				    (type == H264_NALU_TYPE_SLICE_IDR)) &&
				   (nh->nal_ref_idc != 0));
}


static void slice_cb(struct h264_ctx *ctx,
		     const uint8_t *buf,
		     size_t len,
		     const struct h264_slice_header *sh,
		     void *userdata)
{
	struct vdec_mediacodec *self = userdata;
	/* Deliberately remove the const of the buf to modify the
	 * frame_num */
	uint8_t *data = (uint8_t *)buf;
	struct h264_slice_header *fsh;
	struct h264_bitstream bs;
	int res = 0;

	if (self->current_frame.fake_frame_num == sh->frame_num)
		return;

	fsh = calloc(1, sizeof(*fsh));
	h264_bs_init(&bs, data, len, 1);

	*fsh = *sh;
	fsh->frame_num = self->current_frame.fake_frame_num;

	res = h264_rewrite_slice_header(&bs, ctx, fsh);
	if (res < 0)
		ULOG_ERRNO("h264_rewrite_slice_header", -res);

	free(fsh);
}


static void sps_cb(struct h264_ctx *ctx,
		   const uint8_t *buf,
		   size_t len,
		   const struct h264_sps *sps,
		   void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	self->current_frame.max_frame_num =
		1 << (sps->log2_max_frame_num_minus4 + 4);
}


static void sei_recovery_point_cb(struct h264_ctx *ctx,
				  const uint8_t *buf,
				  size_t len,
				  const struct h264_sei_recovery_point *sei,
				  void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	self->current_frame.recovery_point = 1;
}


static const struct h264_ctx_cbs h264_cbs = {
	.nalu_end = &nalu_end_cb,
	.slice = &slice_cb,
	.sps = &sps_cb,
	.sei_recovery_point = &sei_recovery_point_cb,
};


static int create(struct vdec_decoder *base)
{
	int res = 0;
	struct mbuf_coded_video_frame_queue_args queue_args;
	struct pomp_evt *evt;

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	switch (base->config.encoding) {
	case VDEF_ENCODING_H264:
	case VDEF_ENCODING_H265:
		break;
	default:
		res = -ENOSYS;
		ULOG_ERRNO("unsupported encoding: '%s'",
			   -res,
			   vdef_encoding_to_str(base->config.encoding));
		return res;
	}

	struct vdec_mediacodec *self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;

	struct vdec_config_mediacodec *specific =
		(struct vdec_config_mediacodec *)vdec_config_get_specific(
			&base->config, VDEC_DECODER_IMPLEM_MEDIACODEC);

	if (specific && specific->fake_frame_num)
		self->fake_frame_num = true;

	if (base->config.encoding == VDEF_ENCODING_H264)
		self->need_sync = true;

	pthread_mutex_init(&self->push.mutex, NULL);
	pthread_cond_init(&self->push.cond, NULL);

	pthread_mutex_init(&self->pull.mutex, NULL);
	pthread_cond_init(&self->pull.cond, NULL);

	const char *mime_type =
		vdef_get_encoding_mime_type(base->config.encoding);

	self->mc = AMediaCodec_createDecoderByType(mime_type);
	if (self->mc == NULL) {
		ULOGE("AMediaCodec_createDecoderByType");
		res = -ENOMEM;
		goto error;
	}

	res = h264_reader_new(
		&h264_cbs, self, &self->current_frame.h264_reader);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_new", -res);
		goto error;
	}

	queue_args = (struct mbuf_coded_video_frame_queue_args){
		.filter = input_filter,
		.filter_userdata = self,
	};
	res = mbuf_coded_video_frame_queue_new_with_args(&queue_args,
							 &self->in_queue);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new_with_args", -res);
		goto error;
	}

	res = mbuf_coded_video_frame_queue_new(&self->meta_queue);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new", -res);
		goto error;
	}

	res = mbuf_raw_video_frame_queue_new(&self->out_queue);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_new", -res);
		goto error;
	}

	mbuf_coded_video_frame_queue_get_event(self->in_queue, &evt);
	res = pomp_evt_attach_to_loop(evt, base->loop, on_input_event, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	self->out_evt = pomp_evt_new();
	if (self->out_evt == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_evt_new", -res);
		goto error;
	}

	res = pomp_evt_attach_to_loop(
		self->out_evt, base->loop, on_output_event, self);
	if (res < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	res = pthread_create(&self->push.thread, NULL, push_routine, self);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_create", -res);
		goto error;
	}
	self->push.thread_created = true;

	res = pthread_create(&self->pull.thread, NULL, pull_routine, self);
	if (res != 0) {
		res = -res;
		ULOG_ERRNO("pthread_create", -res);
		goto error;
	}
	self->pull.thread_created = true;

	return 0;

error:
	destroy(base);
	base->derived = NULL;
	return res;
}


static int flush(struct vdec_decoder *base, int discard)
{
	struct vdec_mediacodec *self = base->derived;

	if (discard) {
		pthread_mutex_lock(&self->push.mutex);
		self->push.flush_requested = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_mutex_lock(&self->pull.mutex);
		self->pull.flush_requested = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);

		AMediaCodec_flush(self->mc);

		self->state = WAITING_FOR_FLUSH;
		self->need_sync = true;
	} else {
		self->eos_pending = true;
		pthread_mutex_lock(&self->push.mutex);
		self->push.eos_requested = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);
	}

	return 0;
}


static int set_ps(struct vdec_decoder *base,
		  const uint8_t *vps,
		  size_t vps_size,
		  const uint8_t *sps,
		  size_t sps_size,
		  const uint8_t *pps,
		  size_t pps_size,
		  const struct vdef_coded_format *coded_format)
{
	ULOG_ERRNO_RETURN_ERR_IF(coded_format->data_format !=
					 VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
				 EINVAL);

	struct vdec_mediacodec *self = base->derived;

	AMediaFormat *format = AMediaFormat_new();
	if (format == NULL)
		return -ENOMEM;

	const char *mime_type =
		vdef_get_encoding_mime_type(base->config.encoding);
	AMediaFormat_setString(format, "mime", mime_type);

	switch (coded_format->encoding) {
	case VDEF_ENCODING_H264:
		self->need_sync = true;
		self->current_frame.ref = 0;
		self->current_frame.recovery_point = 0;
		AMediaFormat_setBuffer(format, "csd-0", (void *)sps, sps_size);
		AMediaFormat_setBuffer(format, "csd-1", (void *)pps, pps_size);
		break;
	case VDEF_ENCODING_H265: {
		size_t ps_size = vps_size + sps_size + pps_size;
		uint8_t *ps_buf = malloc(ps_size);
		if (ps_buf == NULL) {
			AMediaFormat_delete(format);
			return -ENOMEM;
		}
		memcpy(ps_buf, vps, vps_size);
		memcpy(ps_buf + vps_size, sps, sps_size);
		memcpy(ps_buf + vps_size + sps_size, pps, pps_size);

		AMediaFormat_setBuffer(format,
				       "csd-0",
				       ps_buf,
				       vps_size + sps_size + pps_size);

		free(ps_buf);
		break;
	}
	default:
		break;
	}

	AMediaFormat_setInt32(
		format, "width", base->video_info.resolution.width);
	AMediaFormat_setInt32(
		format, "height", base->video_info.resolution.height);

	AMediaFormat_setInt32(format,
			      "max-input-size",
			      base->video_info.resolution.width *
				      base->video_info.resolution.height * 3 /
				      4);

	media_status_t status =
		AMediaCodec_configure(self->mc, format, NULL, NULL, 0);
	AMediaFormat_delete(format);
	if (status != AMEDIA_OK)
		return -EPROTO;

	status = AMediaCodec_start(self->mc);

	update_output_format(self);

	return (status == AMEDIA_OK) ? 0 : -EPROTO;
}


static int set_h264_ps(struct vdec_decoder *base,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format)
{
	int ret;
	struct vdec_mediacodec *self = base->derived;
	int offset =
		format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU ? 0 : 4;
	/* Give SPS and PPS to h264_reader */
	ret = h264_reader_parse_nalu(self->current_frame.h264_reader,
				     0,
				     sps + offset,
				     sps_size - offset);
	if (ret < 0)
		ULOG_ERRNO("h264_reader_parse_nalu", -ret);

	ret = h264_reader_parse_nalu(self->current_frame.h264_reader,
				     0,
				     pps + offset,
				     pps_size - offset);
	if (ret < 0)
		ULOG_ERRNO("h264_reader_parse_nalu", -ret);

	return set_ps(base, NULL, 0, sps, sps_size, pps, pps_size, format);
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
	return set_ps(
		base, vps, vps_size, sps, sps_size, pps, pps_size, format);
}


static struct mbuf_pool *get_input_buffer_pool(struct vdec_decoder *base)
{
	return NULL;
}


static struct mbuf_coded_video_frame_queue *
get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_mediacodec *self = base->derived;
	return self->in_queue;
}

const struct vdec_ops vdec_mediacodec_ops = {
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

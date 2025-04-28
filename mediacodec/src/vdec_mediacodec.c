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

#include "vdec_mediacodec_priv.h"


#define INPUT_DEQUEUE_TIMEOUT_US 8000
#define OUTPUT_DEQUEUE_TIMEOUT_US 8000


static struct vdef_coded_format
	supported_formats[VDEC_MEDIACODEC_NB_SUPPORTED_FORMATS];
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
	return VDEC_MEDIACODEC_NB_SUPPORTED_FORMATS;
}


static void flush_complete_idle(void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	VDEC_LOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	vdec_call_flush_cb(self->base);
}


static void stop_complete_idle(void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	VDEC_LOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	vdec_call_stop_cb(self->base);
}


static int stop(struct vdec_decoder *base)
{
	struct vdec_mediacodec *self = base->derived;

	int state = atomic_exchange(&self->state, WAITING_FOR_STOP);
	if (state == WAITING_FOR_STOP)
		return 0;

	pthread_mutex_lock(&self->push.mutex);
	self->push.stop_requested = true;
	self->push.cond_signalled = true;
	pthread_cond_signal(&self->push.cond);
	pthread_mutex_unlock(&self->push.mutex);

	pthread_mutex_lock(&self->pull.mutex);
	self->pull.stop_requested = true;
	self->pull.cond_signalled = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);

	return 0;
}


static int destroy(struct vdec_decoder *base)
{
	struct vdec_mediacodec *self = base->derived;
	media_status_t status;
	int err;

	if (self == NULL)
		return 0;

	if (self->mc != NULL) {
		status = AMediaCodec_stop(self->mc);
		if (status != AMEDIA_OK) {
			VDEC_LOGE("AMediaCodec_stop (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}

	if (self->mem != NULL) {
		err = mbuf_mem_unref(self->mem);
		if (err != 0)
			VDEC_LOG_ERRNO("mbuf_mem_unref", -err);
	}

	if (self->pull.thread_created) {
		pthread_mutex_lock(&self->pull.mutex);
		self->pull.stop_requested = true;
		self->pull.cond_signalled = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);

		pthread_join(self->pull.thread, NULL);
	}

	if (self->push.thread_created) {
		pthread_mutex_lock(&self->push.mutex);
		self->push.stop_requested = true;
		self->push.cond_signalled = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_join(self->push.thread, NULL);
	}

	/* Remove any leftover idle callbacks */
	err = pomp_loop_idle_remove_by_cookie(self->base->loop, self);
	if (err < 0)
		VDEC_LOG_ERRNO("pomp_loop_idle_remove_by_cookie", -err);

	if (self->out_evt != NULL) {
		if (pomp_evt_is_attached(self->out_evt, base->loop)) {
			err = pomp_evt_detach_from_loop(self->out_evt,
							base->loop);
			if (err < 0)
				VDEC_LOG_ERRNO("pomp_evt_detach_from_loop",
					       -err);
		}
		err = pomp_evt_destroy(self->out_evt);
		if (err < 0)
			VDEC_LOG_ERRNO("pomp_evt_destroy", -err);
	}

	if (self->out_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->out_queue);
		if (err < 0) {
			VDEC_LOG_ERRNO(
				"mbuf_raw_video_frame_queue_destroy(out)",
				-err);
		}
	}

	if (self->meta_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->meta_queue);
		if (err < 0) {
			VDEC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_destroy(meta)",
				-err);
		}
	}

	if (self->in_queue != NULL) {
		struct pomp_evt *evt;
		err = mbuf_coded_video_frame_queue_get_event(self->in_queue,
							     &evt);
		if (err < 0) {
			VDEC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_get_event(in)",
				-err);
		}

		if (pomp_evt_is_attached(evt, base->loop)) {
			err = pomp_evt_detach_from_loop(evt, base->loop);
			if (err < 0)
				VDEC_LOG_ERRNO("pomp_evt_detach_from_loop",
					       -err);
		}

		err = mbuf_coded_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			VDEC_LOG_ERRNO(
				"mbuf_coded_video_frame_queue_destroy(in)",
				-err);
	}

	if (self->dec_name != NULL) {
		AMediaCodec_releaseName(self->mc, self->dec_name);
		self->dec_name = NULL;
	}

	if (self->mc != NULL) {
		status = AMediaCodec_delete(self->mc);
		if (status != AMEDIA_OK) {
			VDEC_LOGE("AMediaCodec_delete (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}

	pthread_mutex_destroy(&self->push.mutex);
	pthread_cond_destroy(&self->push.cond);

	pthread_mutex_destroy(&self->pull.mutex);
	pthread_cond_destroy(&self->pull.cond);

	free(self);
	base->derived = NULL;

	return 0;
}

static bool input_filter(struct mbuf_coded_video_frame *frame, void *userdata)
{
	int ret;
	const void *tmp;
	size_t tmplen;
	struct vdef_coded_frame info;

	struct vdec_mediacodec *self = userdata;

	if ((atomic_load(&self->state) != RUNNING) || self->eos_pending)
		return false;

	ret = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (ret != 0)
		return false;

	/* Pass default filters first */
	if (!vdec_default_input_filter_internal(
		    self->base,
		    frame,
		    &info,
		    supported_formats,
		    VDEC_MEDIACODEC_NB_SUPPORTED_FORMATS))
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


static int insert_grey_idr(struct vdec_mediacodec *self,
			   struct vdef_coded_frame *frame_info,
			   uint64_t *delta)
{
	int res = 0, err;
	size_t size;
	struct mbuf_mem *idr_mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;
	const void *idr_data = NULL;
	size_t idr_len;
	uint64_t timestamp;
	ssize_t status;

	VDEC_LOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	VDEC_LOG_ERRNO_RETURN_ERR_IF(frame_info == NULL, EINVAL);
	VDEC_LOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);

	ssize_t idr_buf_idx = -1;
	status = AMediaCodec_dequeueInputBuffer(self->mc,
						INPUT_DEQUEUE_TIMEOUT_US);
	if (status == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
		res = -EAGAIN;
		goto end;
	} else if (status < 0) {
		VDEC_LOGE(
			"AMediaCodec"
			"_dequeueInputBuffer (%s:%d)",
			media_status_to_str((media_status_t)status),
			(media_status_t)status);
		res = -EPROTO;
		goto end;
	}
	idr_buf_idx = status;

	size = self->base->video_info.resolution.width *
	       self->base->video_info.resolution.height * 3 / 4;
	timestamp = frame_info->info.timestamp;

	/* Create buffer */
	res = mbuf_mem_generic_new(size, &idr_mem);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_mem_generic_new", -res);
		goto end;
	}

	res = vdec_h264_write_grey_idr(
		self->base, frame_info, delta, &timestamp, idr_mem, &idr_frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("vdec_h264_write_grey_idr", -res);
		goto end;
	}

	res = mbuf_coded_video_frame_get_packed_buffer(
		idr_frame, &idr_data, &idr_len);
	if (res < 0) {
		VDEC_LOG_ERRNO(
			"mbuf_coded_video_frame"
			"_get_packed_buffer",
			-res);
		goto end;
	}

	uint8_t *idr_buf;
	size_t idr_buf_size;
	idr_buf = AMediaCodec_getInputBuffer(
		self->mc, idr_buf_idx, &idr_buf_size);
	if ((idr_buf == NULL) || (idr_buf_size < idr_len)) {
		VDEC_LOGE("AMediaCodec_getInputBuffer");
		if (idr_buf == NULL) {
			VDEC_LOGE("idr: null codec buffer");
			res = -EPROTO;
		} else if (idr_buf_size < idr_len) {
			VDEC_LOGE("idr_buf_size (%zu) < idr_len (%zu)",
				  idr_buf_size,
				  idr_len);
			res = -ENOBUFS;
		}
		goto end;
	}

	/* Copy grey IDR frame */
	memcpy(idr_buf, idr_data, idr_len);

	status = AMediaCodec_queueInputBuffer(
		self->mc, idr_buf_idx, 0, idr_len, timestamp, 0);
	if (status != AMEDIA_OK) {
		VDEC_LOGE("AMediaCodec_queueInputBuffer (%s:%d)",
			  media_status_to_str((media_status_t)status),
			  (media_status_t)status);
		res = -EPROTO;
		goto end;
	}
	idr_buf_idx = -1;

	mbuf_coded_video_frame_add_ancillary_buffer(
		idr_frame,
		VDEC_ANCILLARY_KEY_MEDIACODEC_PTS,
		&timestamp,
		sizeof(timestamp));

	res = mbuf_coded_video_frame_queue_push(self->meta_queue, idr_frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_queue_push", -res);
		goto end;
	}

end:
	if (idr_buf_idx >= 0) {
		media_status_t status = AMediaCodec_queueInputBuffer(
			self->mc, idr_buf_idx, 0, 0, 0, 0);
		if (status != AMEDIA_OK) {
			VDEC_LOGE(
				"AMediaCodec"
				"_queueInputBuffer (%s:%d)",
				media_status_to_str(status),
				status);
		}
	}
	if (idr_data) {
		mbuf_coded_video_frame_release_packed_buffer(idr_frame,
							     idr_data);
	}
	if (idr_mem) {
		err = mbuf_mem_unref(idr_mem);
		if (err != 0)
			VDEC_LOG_ERRNO("mbuf_mem_unref", -err);
	}
	if (idr_frame) {
		err = mbuf_coded_video_frame_unref(idr_frame);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	}
	return res;
}


static int push_frame(struct vdec_mediacodec *self,
		      struct mbuf_coded_video_frame *frame)
{
	int err;
	uint64_t delta;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us, ts;
	int res = 0;
	const void *frame_data = NULL;
	size_t frame_len;
	struct vdef_coded_frame frame_info;
	ssize_t buf_idx = -1;
	size_t buf_size;
	media_status_t status;
	ssize_t _status;
	uint8_t *buf;
	struct mbuf_coded_video_frame *meta_frame = NULL;

	res = mbuf_coded_video_frame_get_frame_info(frame, &frame_info);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		goto end;
	}

	ts_us = VDEF_ROUND(frame_info.info.timestamp * 1000000,
			   frame_info.info.timescale);

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts);

	delta = 0;
	if (vdec_is_sync_frame(frame, &frame_info)) {
		VDEC_LOGI("frame is a sync point");
		atomic_store(&self->need_sync, false);
	} else if (atomic_load(&self->need_sync)) {
		if (self->base->config.encoding == VDEF_ENCODING_H264 &&
		    self->base->config.gen_grey_idr) {
			VDEC_LOGI("frame is not an IDR, generating grey IDR");
			res = insert_grey_idr(self, &frame_info, &delta);
			if (res < 0) {
				if (res != -EAGAIN)
					VDEC_LOG_ERRNO("insert_grey_idr", -res);
				goto end;
			}
			atomic_store(&self->need_sync, false);
			ts_us += delta;
		} else {
			VDEC_LOGI(
				"frame is not a sync point, discarding frame");
			res = 0;
			goto end;
		}
	}

	res = mbuf_coded_video_frame_get_packed_buffer(
		frame, &frame_data, &frame_len);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer",
			       -res);
		goto end;
	}

	_status = AMediaCodec_dequeueInputBuffer(self->mc,
						 INPUT_DEQUEUE_TIMEOUT_US);
	if (_status == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
		res = -EAGAIN;
		goto end;
	} else if (_status < 0) {
		VDEC_LOGE(
			"AMediaCodec"
			"_dequeueInputBuffer (%s:%d)",
			media_status_to_str((media_status_t)_status),
			(media_status_t)_status);
		res = -EPROTO;
		goto end;
	}
	buf_idx = _status;

	buf = AMediaCodec_getInputBuffer(self->mc, buf_idx, &buf_size);
	if (buf == NULL) {
		VDEC_LOGE("AMediaCodec_getInputBuffer");
		res = -EPROTO;
		goto end;
	}

	if (buf_size < frame_len) {
		VDEC_LOGE("buf_size (%zu) < frame_len (%zu)",
			  buf_size,
			  frame_len);
		res = -ENOBUFS;
		goto end;
	}

	/* Copy coded frame */
	memcpy(buf, frame_data, frame_len);

	status = AMediaCodec_queueInputBuffer(
		self->mc, buf_idx, 0, frame_len, ts_us, 0);
	if (status != AMEDIA_OK) {
		VDEC_LOGE("AMediaCodec_queueInputBuffer (%s:%d)",
			  media_status_to_str(status),
			  status);
		res = -EPROTO;
		goto end;
	}
	buf_idx = -1;

	atomic_fetch_add(&self->base->counters.pushed, 1);

	res = mbuf_coded_video_frame_add_ancillary_buffer(
		frame, VDEC_ANCILLARY_KEY_DEQUEUE_TIME, &ts, sizeof(ts));
	if (res < 0)
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer",
			       -res);

	mbuf_coded_video_frame_add_ancillary_buffer(
		frame,
		VDEC_ANCILLARY_KEY_MEDIACODEC_PTS,
		&ts_us,
		sizeof(ts_us));

	res = vdec_copy_coded_frame_as_metadata(frame, self->mem, &meta_frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("vdec_copy_coded_frame_as_metadata", -res);
		goto end;
	}

	res = mbuf_coded_video_frame_queue_push(self->meta_queue, meta_frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_queue_push", -res);
		goto end;
	}

	/* Don't wake-up pull routine when flushing or stopping */
	if (atomic_load(&self->state) == RUNNING) {
		pthread_mutex_lock(&self->pull.mutex);
		self->pull.cond_signalled = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);
	}

	res = 0;

end:
	if (buf_idx >= 0) {
		status = AMediaCodec_queueInputBuffer(
			self->mc, buf_idx, 0, 0, 0, 0);
		if (status != AMEDIA_OK) {
			VDEC_LOGE("AMediaCodec_queueInputBuffer (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}
	if (meta_frame) {
		err = mbuf_coded_video_frame_unref(meta_frame);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
	}
	if (frame) {
		if (frame_data) {
			err = mbuf_coded_video_frame_release_packed_buffer(
				frame, frame_data);
			if (err < 0)
				VDEC_LOG_ERRNO(
					"mbuf_coded_video_frame"
					"_release_packed_buffer",
					-err);
		}
		err = mbuf_coded_video_frame_unref(frame);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	}

	return res;
}


/*
 * See
 * https://developer.android.com/reference/android/media/MediaCodec.html#end-of-stream-handling
 */
static int push_eos(struct vdec_mediacodec *self)
{
	ssize_t buf_idx;
	ssize_t status = AMediaCodec_dequeueInputBuffer(
		self->mc, INPUT_DEQUEUE_TIMEOUT_US);
	if (status == AMEDIACODEC_INFO_TRY_AGAIN_LATER) {
		return -EAGAIN;
	} else if (status < 0) {
		VDEC_LOGE(
			"AMediaCodec"
			"_dequeueInputBuffer (%s:%d)",
			media_status_to_str((media_status_t)status),
			(media_status_t)status);
		return -EPROTO;
	}
	buf_idx = status;

	media_status_t _status = AMediaCodec_queueInputBuffer(
		self->mc,
		buf_idx,
		0,
		0,
		0,
		AMEDIACODEC_BUFFER_FLAG_END_OF_STREAM);
	if (_status != AMEDIA_OK) {
		VDEC_LOGE("AMediaCodec_queueInputBuffer (%s:%d)",
			  media_status_to_str(_status),
			  _status);
		return -EPROTO;
	}

	pthread_mutex_lock(&self->pull.mutex);
	self->pull.eos_requested = true;
	self->pull.cond_signalled = true;
	pthread_cond_signal(&self->pull.cond);
	pthread_mutex_unlock(&self->pull.mutex);

	return 0;
}


static void on_input_event(struct pomp_evt *evt, void *userdata)
{
	struct vdec_mediacodec *self = userdata;

	/* Don't wake-up push routine when flushing or stopping */
	if (atomic_load(&self->state) != RUNNING)
		return;

	pthread_mutex_lock(&self->push.mutex);
	self->push.cond_signalled = true;
	pthread_cond_signal(&self->push.cond);
	pthread_mutex_unlock(&self->push.mutex);
}


static void *push_routine(void *arg)
{
	int err;
	struct vdec_mediacodec *self = arg;

	err = pthread_setname_np(pthread_self(), "vdec_mdcdc_push");
	if (err != 0)
		VDEC_LOG_ERRNO("pthread_setname_np", err);

	pthread_mutex_lock(&self->push.mutex);
	while (true) {
		self->push.cond_signalled = false;
		if (self->push.stop_requested) {
			self->push.stop_requested = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VDEC_LOG_ERRNO("pomp_evt_signal", -err);

			pthread_mutex_unlock(&self->push.mutex);
			break;
		}

		if (self->push.flush_requested) {
			self->push.flush_requested = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VDEC_LOG_ERRNO("pomp_evt_signal", -err);
			goto wait;
		}

		struct mbuf_coded_video_frame *frame;
		int res = mbuf_coded_video_frame_queue_peek(self->in_queue,
							    &frame);
		if (res == 0) {
			pthread_mutex_unlock(&self->push.mutex);
			/* Note: push_frame unrefs the frame */
			err = push_frame(self, frame);
			if (err == -EAGAIN) {
				/* Retry later */
				pthread_mutex_lock(&self->push.mutex);
				goto wait;
			} else if (err < 0) {
				VDEC_LOG_ERRNO("push_frame", -err);
			}
			/* Pop the frame for real */
			res = mbuf_coded_video_frame_queue_pop(self->in_queue,
							       &frame);
			if (res < 0) {
				VDEC_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_pop",
					-res);
				pthread_mutex_lock(&self->push.mutex);
				continue;
			}
			err = mbuf_coded_video_frame_unref(frame);
			if (err < 0) {
				VDEC_LOG_ERRNO("mbuf_coded_video_frame_unref",
					       -err);
			}
			pthread_mutex_lock(&self->push.mutex);
			continue;
		} else if (res == -EAGAIN && self->push.eos_requested) {
			pthread_mutex_unlock(&self->push.mutex);
			int err = push_eos(self);
			if (err == 0)
				self->push.eos_requested = false;
			else if (err < 0 && err != -EAGAIN)
				VDEC_LOG_ERRNO("push_eos", -err);
			pthread_mutex_lock(&self->push.mutex);
			continue;
		} else if (res != -EAGAIN) {
			VDEC_LOG_ERRNO("mbuf_coded_video_frame_queue_peek",
				       -res);
		}

		/* clang-format off */
wait:
		/* clang-format on */
		while (!self->push.cond_signalled)
			pthread_cond_wait(&self->push.cond, &self->push.mutex);
	}

	return NULL;
}


static void update_output_format(struct vdec_mediacodec *self)
{
	AMediaFormat *format = AMediaCodec_getOutputFormat(self->mc);
	if (format == NULL) {
		VDEC_LOGE("AMediaCodec_getOutputFormat");
		return;
	}

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
			VDEC_LOGE("unknown color format: %d", color_format);
			break;
		}
	} else {
		VDEC_LOGE("invalid format declaration: missing color format");
	}

	int32_t stride = 0;
	/* Some Samsung devices do not set the codec output format stride and
	 * slice-height fields. */
	if (AMediaFormat_getInt32(format, "stride", &stride))
		self->stride = stride;
	if (self->stride == 0)
		self->stride = self->base->video_info.resolution.width;

	int32_t slice_height = 0;
	if (AMediaFormat_getInt32(format, "slice-height", &slice_height))
		self->slice_height = slice_height;
	if (self->slice_height == 0)
		self->slice_height = self->base->video_info.resolution.height;

	char *fmt = vdef_raw_format_to_str(&self->output_format);
	VDEC_LOGI("%s: format=%s stride=%u slice_height=%u",
		  __func__,
		  fmt,
		  self->stride,
		  self->slice_height);
	free(fmt);
	free(format);
}


static void release_mc_mem(void *data, size_t len, void *userdata)
{
	size_t idx = len;
	AMediaCodec *mc = userdata;

	media_status_t status = AMediaCodec_releaseOutputBuffer(mc, idx, false);
	if (status != AMEDIA_OK) {
		ULOGE("AMediaCodec_releaseOutputBuffer (%s:%d)",
		      media_status_to_str(status),
		      status);
	}
}


static int pull_frame(struct vdec_mediacodec *self,
		      struct mbuf_coded_video_frame *meta_frame)
{
	int err;
	int res = 0;
	AMediaCodecBufferInfo buffer_info;
	size_t buffer_index = SIZE_MAX;
	struct mbuf_mem *mem = NULL;
	struct mbuf_mem *out_mem = NULL;
	struct mbuf_raw_video_frame *frame = NULL;
	struct mbuf_raw_video_frame *out_frame = NULL;
	struct vdef_coded_frame meta_info;
	size_t out_size;
	uint8_t *out_data;
	struct vdef_raw_frame out_info;
	struct vmeta_frame *metadata = NULL;
	struct timespec cur_ts;
	uint64_t ts_us;
	struct mbuf_ancillary_data *adata;
	const uint64_t *input_pts;
	size_t input_pts_len;
	size_t capacity = 0;
	unsigned int plane_stride_align[VDEF_RAW_MAX_PLANE_COUNT];
	size_t plane_size[VDEF_RAW_MAX_PLANE_COUNT];
	size_t plane_stride[VDEF_RAW_MAX_PLANE_COUNT] = {0};

	while (true) {
		ssize_t status = AMediaCodec_dequeueOutputBuffer(
			self->mc, &buffer_info, OUTPUT_DEQUEUE_TIMEOUT_US);
		switch (status) {
		case AMEDIACODEC_INFO_TRY_AGAIN_LATER:
			res = -EAGAIN;
			goto end;
		case AMEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED:
			update_output_format(self);
			break;
		case AMEDIACODEC_INFO_OUTPUT_BUFFERS_CHANGED:
			/*
			 * Output buffers changed message that we ignore. See
			 * https://developer.android.com/reference/android/media/MediaCodec#INFO_OUTPUT_BUFFERS_CHANGED
			 */
			break;
		default:
			if (status < 0) {
				VDEC_LOGE(
					"AMediaCodec"
					"_dequeueOutputBuffer (%s:%d)",
					media_status_to_str(
						(media_status_t)status),
					(media_status_t)status);
				res = -EPROTO;
				goto end;
			}
			buffer_index = status;
			goto end_loop;
		}
	}

end_loop:
	res = mbuf_coded_video_frame_get_ancillary_data(
		meta_frame, VDEC_ANCILLARY_KEY_MEDIACODEC_PTS, &adata);
	if (res < 0 && res != -ENOENT) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_get_ancillary_data",
			       -res);
		goto skip_pts_check;
	} else if (res == -ENOENT) {
		VDEC_LOGD("Input frame has no associated PTS");
		goto skip_pts_check;
	}

	input_pts = mbuf_ancillary_data_get_buffer(adata, &input_pts_len);
	if (input_pts == NULL || input_pts_len != sizeof(*input_pts)) {
		VDEC_LOG_ERRNO("mbuf_ancillary_data_get_buffer", -res);
		mbuf_ancillary_data_unref(adata);
		goto skip_pts_check;
	}

	if (*input_pts != (uint64_t)buffer_info.presentationTimeUs) {
		VDEC_LOGD("Frame PTS mismatch for metadata (expected %" PRIu64
			  ", got %" PRIi64 ")",
			  *input_pts,
			  buffer_info.presentationTimeUs);
	}
	mbuf_ancillary_data_unref(adata);

skip_pts_check:
	res = mbuf_coded_video_frame_get_frame_info(meta_frame, &meta_info);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		goto end;
	}

	out_data =
		AMediaCodec_getOutputBuffer(self->mc, buffer_index, &out_size) +
		buffer_info.offset;
	if (out_data == NULL) {
		res = -EPROTO;
		VDEC_LOGE("AMediaCodec_getOutputBuffer");
		goto end;
	}

	atomic_fetch_add(&self->base->counters.pulled, 1);

	/* Since we need to store the buffer index but not the length, we hijack
	 * the mem’s `len` field to store this index. */
	res = mbuf_mem_generic_wrap(
		out_data, buffer_index, release_mc_mem, self->mc, &mem);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_mem_generic_wrap", -res);
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
	} else {
		res = -ENOSYS;
		VDEC_LOGE("unsupported format: " VDEF_RAW_FORMAT_TO_STR_FMT,
			  VDEF_RAW_FORMAT_TO_STR_ARG(&out_info.format));
		goto end;
	}

	res = mbuf_raw_video_frame_new(&out_info, &frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_new", -res);
		goto end;
	}

	res = mbuf_coded_video_frame_foreach_ancillary_data(
		meta_frame, mbuf_raw_video_frame_ancillary_data_copier, frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_foreach_ancillary_data",
			       -res);
		goto end;
	}

	res = mbuf_coded_video_frame_get_metadata(meta_frame, &metadata);
	if (res == 0) {
		res = mbuf_raw_video_frame_set_metadata(frame, metadata);
		if (res < 0) {
			VDEC_LOG_ERRNO("mbuf_raw_video_frame_set_metadata",
				       -res);
			goto end;
		}
	} else if (res == -ENOENT) {
		res = 0;
	} else {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_get_metadata", -res);
		goto end;
	}

	plane_stride_align[0] = 16;
	res = mbuf_raw_video_frame_set_plane(
		frame,
		0,
		mem,
		0,
		out_info.plane_stride[0] * out_info.info.resolution.height);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
		goto end;
	}

	if (vdef_raw_format_cmp(&out_info.format, &vdef_nv12)) {
		plane_stride_align[1] = 16;
		res = mbuf_raw_video_frame_set_plane(
			frame,
			1,
			mem,
			self->stride * self->slice_height,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (res < 0) {
			VDEC_LOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
			goto end;
		}
	} else if (vdef_raw_format_cmp(&out_info.format, &vdef_i420)) {
		plane_stride_align[1] = 16;
		res = mbuf_raw_video_frame_set_plane(
			frame,
			1,
			mem,
			self->stride * self->slice_height,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (res < 0) {
			VDEC_LOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
			goto end;
		}
		plane_stride_align[2] = 16;
		res = mbuf_raw_video_frame_set_plane(
			frame,
			2,
			mem,
			(self->stride * self->slice_height * 5) / 4,
			out_info.plane_stride[2] *
				out_info.info.resolution.height / 2);
		if (res < 0) {
			VDEC_LOG_ERRNO("mbuf_raw_video_frame_set_plane", -res);
			goto end;
		}
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	res = mbuf_raw_video_frame_add_ancillary_buffer(
		frame, VDEC_ANCILLARY_KEY_OUTPUT_TIME, &ts_us, sizeof(ts_us));
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer",
			       -res);
		goto end;
	}

	res = mbuf_raw_video_frame_finalize(frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_finalize", -res);
		goto end;
	}

	/* Ignore silent frames (depending on configuration) */
	if ((out_info.info.flags & VDEF_FRAME_FLAG_SILENT) &&
	    (!self->base->config.output_silent_frames)) {
		VDEC_LOGD("silent frame (ignored)");
		goto end;
	}

	/* Copy and push the frame */
	res = vdef_calc_raw_frame_size(&out_info.format,
				       &out_info.info.resolution,
				       plane_stride,
				       plane_stride_align,
				       NULL,
				       NULL,
				       plane_size,
				       NULL);
	if (res < 0) {
		VDEC_LOG_ERRNO("vdef_calc_raw_frame_size", -res);
		goto end;
	}
	capacity += plane_size[0];
	if (vdef_raw_format_cmp(&out_info.format, &vdef_nv12)) {
		capacity += plane_size[1];
	} else if (vdef_raw_format_cmp(&out_info.format, &vdef_i420)) {
		capacity += plane_size[1];
		capacity += plane_size[2];
	}

	res = mbuf_mem_generic_new(capacity, &out_mem);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_mem_generic_new", -res);
		goto end;
	}

	res = mbuf_raw_video_frame_copy_with_align(
		frame, out_mem, plane_stride_align, NULL, NULL, &out_frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_copy_with_align", -res);
		goto end;
	}
	res = mbuf_raw_video_frame_finalize(out_frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_finalize", -res);
		goto end;
	}
	res = mbuf_raw_video_frame_queue_push(self->out_queue, out_frame);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_queue_push", -res);
		goto end;
	}
	err = pomp_evt_signal(self->out_evt);
	if (err < 0)
		VDEC_LOG_ERRNO("pomp_evt_signal", -err);

	res = 0;

end:
	if (metadata) {
		err = vmeta_frame_unref(metadata);
		if (err < 0)
			VDEC_LOG_ERRNO("vmeta_frame_unref", -err);
	}
	if (out_mem) {
		err = mbuf_mem_unref(out_mem);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_mem_unref", -err);
	}
	if (mem) {
		err = mbuf_mem_unref(mem);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_mem_unref", -err);
	} else if (buffer_index != SIZE_MAX) {
		/* Buffer wasn't wrapped yet */
		media_status_t status = AMediaCodec_releaseOutputBuffer(
			self->mc, buffer_index, false);
		if (status != AMEDIA_OK) {
			VDEC_LOGE("AMediaCodec_releaseOutputBuffer (%s:%d)",
				  media_status_to_str(status),
				  status);
		}
	}
	if (frame) {
		err = mbuf_raw_video_frame_unref(frame);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
	}
	if (out_frame) {
		err = mbuf_raw_video_frame_unref(out_frame);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_raw_video_frame_unref", -err);
	}
	if (meta_frame) {
		err = mbuf_coded_video_frame_unref(meta_frame);
		if (err < 0)
			VDEC_LOG_ERRNO("mbuf_coded_video_frame_unref", -err);
	}

	return res;
}


static void *pull_routine(void *arg)
{
	int err;
	struct vdec_mediacodec *self = arg;

	err = pthread_setname_np(pthread_self(), "vdec_mdcdc_pull");
	if (err != 0)
		VDEC_LOG_ERRNO("pthread_setname_np", err);

	pthread_mutex_lock(&self->pull.mutex);
	while (true) {
		self->pull.cond_signalled = false;
		if (self->pull.stop_requested) {
			self->pull.stop_requested = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VDEC_LOG_ERRNO("pomp_evt_signal", -err);
			pthread_mutex_unlock(&self->pull.mutex);
			break;
		}

		if (self->pull.flush_requested) {
			self->pull.flush_requested = false;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VDEC_LOG_ERRNO("pomp_evt_signal", -err);
			goto wait;
		}

		struct mbuf_coded_video_frame *frame;
		int res = mbuf_coded_video_frame_queue_peek(self->meta_queue,
							    &frame);
		if (res == 0) {
			pthread_mutex_unlock(&self->pull.mutex);
			/* Note: pull_frame unrefs the frame */
			int err = pull_frame(self, frame);
			if (err == -EAGAIN) {
				/* Retry later */
				pthread_mutex_lock(&self->pull.mutex);
				goto wait;
			} else if (err < 0) {
				VDEC_LOG_ERRNO("pull_frame", -err);
			}
			/* Pop the frame for real */
			res = mbuf_coded_video_frame_queue_pop(self->meta_queue,
							       &frame);
			if (res < 0) {
				VDEC_LOG_ERRNO(
					"mbuf_coded_video_frame_queue_pop",
					-res);
				pthread_mutex_lock(&self->pull.mutex);
				continue;
			}
			err = mbuf_coded_video_frame_unref(frame);
			if (err < 0) {
				VDEC_LOG_ERRNO("mbuf_coded_video_frame_unref",
					       -err);
			}
			pthread_mutex_lock(&self->pull.mutex);
			continue;
		} else if (res == -EAGAIN && self->pull.eos_requested) {
			self->pull.eos_requested = false;
			self->eos_requested = true;
			err = pomp_evt_signal(self->out_evt);
			if (err < 0)
				VDEC_LOG_ERRNO("pomp_evt_signal", -err);
		} else if (res != -EAGAIN) {
			VDEC_LOG_ERRNO("mbuf_coded_video_frame_queue_peek",
				       -res);
		}

		/* clang-format off */
wait:
		/* clang-format on */
		while (!self->pull.cond_signalled)
			pthread_cond_wait(&self->pull.cond, &self->pull.mutex);
	}

	return NULL;
}


static void on_output_event(struct pomp_evt *evt, void *userdata)
{
	int err;
	struct vdec_mediacodec *self = userdata;

	switch (atomic_load(&self->state)) {
	case RUNNING: {
		while (true) {
			struct mbuf_raw_video_frame *frame;
			int res = mbuf_raw_video_frame_queue_pop(
				self->out_queue, &frame);
			if (res < 0) {
				if (res != -EAGAIN)
					VDEC_LOG_ERRNO(
						"mbuf_raw_video_frame_pop",
						-res);
				break;
			}
			vdec_call_frame_output_cb(self->base, 0, frame);
			err = mbuf_raw_video_frame_unref(frame);
			if (err < 0)
				VDEC_LOG_ERRNO("mbuf_raw_video_frame_unref",
					       -err);
		}

		pthread_mutex_lock(&self->pull.mutex);
		bool eos_requested = self->eos_requested;
		self->eos_requested = false;
		pthread_mutex_unlock(&self->pull.mutex);

		if (eos_requested) {
			media_status_t status = AMediaCodec_flush(self->mc);
			if (status != AMEDIA_OK) {
				VDEC_LOGE("AMediaCodec_flush (%s:%d)",
					  media_status_to_str(status),
					  status);
			}

			self->eos_pending = false;
			atomic_store(&self->need_sync, true);

			err = pomp_loop_idle_add_with_cookie(
				self->base->loop,
				flush_complete_idle,
				self,
				self);
			if (err < 0) {
				VDEC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			}
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
			err = mbuf_coded_video_frame_queue_flush(
				self->in_queue);
			if (err < 0) {
				VDEC_LOG_ERRNO(
					"mbuf_coded_video_frame"
					"_queue_flush(in)",
					-err);
			}
			err = mbuf_coded_video_frame_queue_flush(
				self->meta_queue);
			if (err < 0) {
				VDEC_LOG_ERRNO(
					"mbuf_coded_video_frame"
					"_queue_flush(meta)",
					-err);
			}
			err = mbuf_raw_video_frame_queue_flush(self->out_queue);
			if (err < 0) {
				VDEC_LOG_ERRNO(
					"mbuf_raw_video_frame"
					"_queue_flush(out)",
					-err);
			}
			media_status_t status = AMediaCodec_flush(self->mc);
			if (status != AMEDIA_OK) {
				VDEC_LOGE("AMediaCodec_flush (%s:%d)",
					  media_status_to_str(status),
					  status);
			}

			atomic_store(&self->need_sync, true);
			atomic_store(&self->state, RUNNING);

			err = pomp_loop_idle_add_with_cookie(
				self->base->loop,
				flush_complete_idle,
				self,
				self);
			if (err < 0) {
				VDEC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			}
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
			media_status_t status = AMediaCodec_stop(self->mc);
			if (status != AMEDIA_OK) {
				VDEC_LOGE("AMediaCodec_stop (%s:%d)",
					  media_status_to_str(status),
					  status);
			}

			err = pomp_loop_idle_add_with_cookie(self->base->loop,
							     stop_complete_idle,
							     self,
							     self);
			if (err < 0) {
				VDEC_LOG_ERRNO("pomp_loop_idle_add_with_cookie",
					       -err);
			}
		}
		break;
	default:
		break;
	}
}


static int create(struct vdec_decoder *base)
{
	int res = 0;
	struct mbuf_coded_video_frame_queue_args queue_args;
	struct pomp_evt *evt;
	media_status_t status;

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

	if (base->config.encoding == VDEF_ENCODING_H264)
		atomic_store(&self->need_sync, true);

	pthread_mutex_init(&self->push.mutex, NULL);
	pthread_cond_init(&self->push.cond, NULL);

	pthread_mutex_init(&self->pull.mutex, NULL);
	pthread_cond_init(&self->pull.cond, NULL);

	const char *mime_type =
		vdef_get_encoding_mime_type(base->config.encoding);

	/* Create a new mbuf_mem for meta_frame */
	res = mbuf_mem_generic_new(sizeof(uint32_t), &self->mem);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_mem_generic_new", -res);
		goto error;
	}

	self->mc = AMediaCodec_createDecoderByType(mime_type);
	if (self->mc == NULL) {
		VDEC_LOGE("AMediaCodec_createDecoderByType");
		res = -ENOMEM;
		goto error;
	}

	status = AMediaCodec_getName(self->mc, &self->dec_name);
	if (status < 0) {
		VDEC_LOGE(
			"AMediaCodec"
			"_getName (%s:%d)",
			media_status_to_str((media_status_t)status),
			(media_status_t)status);
		res = -EPROTO;
		goto error;
	}

	VDEC_LOGI("mediacodec implementation (%s)", self->dec_name);

	queue_args = (struct mbuf_coded_video_frame_queue_args){
		.filter = input_filter,
		.filter_userdata = self,
	};
	res = mbuf_coded_video_frame_queue_new_with_args(&queue_args,
							 &self->in_queue);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_queue_new_with_args",
			       -res);
		goto error;
	}

	res = mbuf_coded_video_frame_queue_new(&self->meta_queue);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_queue_new", -res);
		goto error;
	}

	res = mbuf_coded_video_frame_queue_get_event(self->in_queue, &evt);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_coded_video_frame_queue_get_event(in)",
			       -res);
		goto error;
	}

	res = pomp_evt_attach_to_loop(evt, base->loop, on_input_event, self);
	if (res < 0) {
		VDEC_LOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	res = mbuf_raw_video_frame_queue_new(&self->out_queue);
	if (res < 0) {
		VDEC_LOG_ERRNO("mbuf_raw_video_frame_queue_new", -res);
		goto error;
	}

	self->out_evt = pomp_evt_new();
	if (self->out_evt == NULL) {
		res = -ENOMEM;
		VDEC_LOG_ERRNO("pomp_evt_new", -res);
		goto error;
	}

	res = pomp_evt_attach_to_loop(
		self->out_evt, base->loop, on_output_event, self);
	if (res < 0) {
		VDEC_LOG_ERRNO("pomp_evt_attach_to_loop", -res);
		goto error;
	}

	res = pthread_create(&self->push.thread, NULL, push_routine, self);
	if (res != 0) {
		res = -res;
		VDEC_LOG_ERRNO("pthread_create", -res);
		goto error;
	}
	self->push.thread_created = true;

	res = pthread_create(&self->pull.thread, NULL, pull_routine, self);
	if (res != 0) {
		res = -res;
		VDEC_LOG_ERRNO("pthread_create", -res);
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
		int state = atomic_exchange(&self->state, WAITING_FOR_FLUSH);
		if (state == WAITING_FOR_FLUSH)
			return 0;

		pthread_mutex_lock(&self->push.mutex);
		self->push.flush_requested = true;
		self->push.cond_signalled = true;
		pthread_cond_signal(&self->push.cond);
		pthread_mutex_unlock(&self->push.mutex);

		pthread_mutex_lock(&self->pull.mutex);
		self->pull.flush_requested = true;
		self->pull.cond_signalled = true;
		pthread_cond_signal(&self->pull.cond);
		pthread_mutex_unlock(&self->pull.mutex);
	} else {
		self->eos_pending = true;
		pthread_mutex_lock(&self->push.mutex);
		self->push.eos_requested = true;
		self->push.cond_signalled = true;
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
	int ret;
	struct vdec_mediacodec *self = base->derived;

	VDEC_LOG_ERRNO_RETURN_ERR_IF(coded_format->data_format !=
					     VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
				     EINVAL);

	AMediaFormat *format = AMediaFormat_new();
	if (format == NULL) {
		ret = -ENOMEM;
		VDEC_LOG_ERRNO("AMediaFormat_new", -ret);
		return ret;
	}

	const char *mime_type =
		vdef_get_encoding_mime_type(base->config.encoding);
	AMediaFormat_setString(format, "mime", mime_type);

	switch (coded_format->encoding) {
	case VDEF_ENCODING_H264:
		atomic_store(&self->need_sync, true);
		AMediaFormat_setBuffer(format, "csd-0", (void *)sps, sps_size);
		AMediaFormat_setBuffer(format, "csd-1", (void *)pps, pps_size);
		break;
	case VDEF_ENCODING_H265: {
		size_t ps_size = vps_size + sps_size + pps_size;
		uint8_t *ps_buf = malloc(ps_size);
		if (ps_buf == NULL) {
			ret = -ENOMEM;
			VDEC_LOG_ERRNO("malloc", -ret);
			goto out;
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

	AMediaFormat_setInt32(format, "low-latency", !!base->config.low_delay);

	/* Vendor-specific low-latency parameters for Samsung Exynos decoders */
	if (strstr(self->dec_name, "exynos")) {
		AMediaFormat_setInt32(format,
				      "vendor.rtc-ext-dec-low-latency.enable",
				      !!base->config.low_delay);
		if (!!base->config.low_delay) {
			AMediaFormat_setInt32(
				format,
				"vendor.sec-dec-output.display-delay.value",
				0);
		}
	}

	media_status_t status =
		AMediaCodec_configure(self->mc, format, NULL, NULL, 0);
	if (status != AMEDIA_OK) {
		ret = -EPROTO;
		VDEC_LOGE("AMediaCodec_configure (%s:%d)",
			  media_status_to_str(status),
			  status);
		goto out;
	}

	status = AMediaCodec_start(self->mc);
	if (status != AMEDIA_OK) {
		ret = -EPROTO;
		VDEC_LOGE("AMediaCodec_start (%s:%d)",
			  media_status_to_str(status),
			  status);
	} else {
		ret = 0;
	}

	update_output_format(self);

out:
	if (format)
		AMediaFormat_delete(format);
	return ret;
}


static int set_h264_ps(struct vdec_decoder *base,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format)
{
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

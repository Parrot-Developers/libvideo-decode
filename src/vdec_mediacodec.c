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
#include "vdec_priv.h"
ULOG_DECLARE_TAG(vdec_mediacodec);

#ifdef BUILD_LIBMEDIACODEC_WRAPPER

#	include <mediacodec-wrapper/libmcw.h>
#	include <video-buffers/vbuf_private.h>


#	define VDEC_MEDIACODEC_H264_MIME_TYPE "video/avc"
#	define VDEC_MEDIACODEC_INPUT_BUFFER_COUNT 20
#	define VDEC_MEDIACODEC_OUTPUT_BUFFER_COUNT 20
#	define VDEC_MEDIACODEC_INFO_OUTPUT_BUFFERS_CHANGED -1003
#	define VDEC_MEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED -1002
#	define VDEC_MEDIACODEC_BUF_TYPE_MDCD 0x4D444344 /* "MDCD" */

#	define VDEC_MEDIACODEC_FORMAT_KEY_SLICE_HEIGHT "slice-height"


struct vdec_mediacodec {
	struct vdec_decoder *base;
	struct vbuf_pool *in_pool;
	struct vbuf_queue *in_queue;
	struct vbuf_queue *decoder_queue;
	struct vbuf_pool *out_pool;
	struct mcw *mcw;
	struct mcw_mediacodec *codec;

	pthread_t thread;
	int thread_launched;
	int should_stop;
	int flush;
	int flush_discard;

	int codec_started;
	enum vdec_output_format output_format;
	int output_format_changed;
	unsigned int stride;
	unsigned int slice_height;
};


struct vdec_mediacodec_mcbuf {
	struct mcw *mcw;
	ssize_t buf_idx;
	off_t offset;
	uint64_t timestamp;
	uint32_t flags;
	int64_t render_time_ns;
};


static int vdec_mediacodec_mcbuf_get_info(struct vbuf_buffer *buf,
					  size_t *idx,
					  off_t *offset,
					  uint64_t *timestamp,
					  uint32_t *flags)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf->type != VDEC_MEDIACODEC_BUF_TYPE_MDCD,
				 EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	if (idx)
		*idx = (size_t)mc->buf_idx;
	if (offset)
		*offset = mc->offset;
	if (timestamp)
		*timestamp = mc->timestamp;
	if (flags)
		*flags = mc->flags;

	return 0;
}


static int vdec_mediacodec_mcbuf_set_info(struct vbuf_buffer *buf,
					  off_t offset,
					  uint64_t timestamp,
					  uint32_t flags)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf->type != VDEC_MEDIACODEC_BUF_TYPE_MDCD,
				 EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	mc->offset = offset;
	mc->timestamp = timestamp;
	mc->flags = flags;

	return 0;
}


static int vdec_mediacodec_mcbuf_get_render_time(struct vbuf_buffer *buf,
						 int64_t *time_ns)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf->type != VDEC_MEDIACODEC_BUF_TYPE_MDCD,
				 EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	if (time_ns)
		*time_ns = mc->render_time_ns;

	return 0;
}


static int vdec_mediacodec_mcbuf_set_render_time(struct vbuf_buffer *buf,
						 int64_t time_ns)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf->type != VDEC_MEDIACODEC_BUF_TYPE_MDCD,
				 EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	mc->render_time_ns = time_ns;

	return 0;
}


static int vdec_mediacodec_mcbuf_alloc_cb(struct vbuf_buffer *buf,
					  void *userdata)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;
	struct mcw *mcw = (struct mcw *)userdata;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(userdata == NULL, EINVAL);

	buf->type = VDEC_MEDIACODEC_BUF_TYPE_MDCD;

	mc = calloc(1, sizeof(struct vdec_mediacodec_mcbuf));
	if (mc == NULL)
		return -ENOMEM;
	buf->specific = (struct vbuf_specific *)mc;
	mc->mcw = mcw;
	mc->buf_idx = -1;

	return 0;
}


static int vdec_mediacodec_mcbuf_free_cb(struct vbuf_buffer *buf,
					 void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(userdata == NULL, EINVAL);

	free(buf->specific);
	buf->specific = NULL;
	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static int vdec_mediacodec_mcbuf_input_pool_get_cb(struct vbuf_buffer *buf,
						   int timeout_ms,
						   void *userdata)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;
	struct mcw_mediacodec *codec = (struct mcw_mediacodec *)userdata;
	ssize_t buf_idx;
	uint8_t *ptr;
	size_t buf_size = 0;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(userdata == NULL, EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	ULOG_ERRNO_RETURN_ERR_IF(mc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mc->mcw == NULL, EINVAL);

	buf_idx = mc->mcw->mediacodec.dequeue_input_buffer(
		codec,
		(timeout_ms > 0) ? (int64_t)timeout_ms * 1000 : timeout_ms);
	if (buf_idx == MCW_INFO_TRY_AGAIN_LATER) {
		return -EAGAIN;
	} else if (buf_idx < 0) {
		ULOGE("failed to dequeue an input buffer, error=%zi", buf_idx);
		return -ENOENT;
	}

	ptr = mc->mcw->mediacodec.get_input_buffer(codec, buf_idx, &buf_size);
	if ((ptr == NULL) || (buf_size <= 0)) {
		ULOGE("failed to get input buffer #%zu", buf_idx);
		return -ENOENT;
	}

	mc->buf_idx = buf_idx;
	mc->offset = 0;
	mc->timestamp = 0;
	mc->flags = 0;
	mc->render_time_ns = 0;
	buf->capacity = buf_size;
	buf->size = 0;
	buf->ptr = ptr;

	return 0;
}


static int vdec_mediacodec_mcbuf_input_pool_put_cb(struct vbuf_buffer *buf,
						   void *userdata)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;
	struct mcw_mediacodec *codec = (struct mcw_mediacodec *)userdata;
	enum mcw_media_status err;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(userdata == NULL, EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	ULOG_ERRNO_RETURN_ERR_IF(mc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mc->mcw == NULL, EINVAL);

	if (mc->buf_idx >= 0) {
		err = mc->mcw->mediacodec.queue_input_buffer(
			codec, mc->buf_idx, 0, 0, 0, 0);
		if (err != MCW_MEDIA_STATUS_OK) {
			ULOGE("failed to queue input buffer #%zu, error=%d",
			      mc->buf_idx,
			      err);
			return -EIO;
		}
	}

	mc->buf_idx = -1;
	mc->offset = 0;
	mc->timestamp = 0;
	mc->flags = 0;
	mc->render_time_ns = 0;
	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static int vdec_mediacodec_input_buffer_queue(struct vbuf_buffer *buf,
					      struct mcw_mediacodec *codec)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;
	enum mcw_media_status err;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(codec == NULL, EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	ULOG_ERRNO_RETURN_ERR_IF(mc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mc->mcw == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mc->buf_idx < 0, EINVAL);

	err = mc->mcw->mediacodec.queue_input_buffer(codec,
						     mc->buf_idx,
						     mc->offset,
						     buf->size,
						     mc->timestamp,
						     mc->flags);
	if (err != MCW_MEDIA_STATUS_OK) {
		ULOGE("failed to queue input buffer #%zu, error=%d",
		      mc->buf_idx,
		      err);
		return -EIO;
	}

	mc->buf_idx = -1;
	mc->offset = 0;
	mc->timestamp = 0;
	mc->flags = 0;
	mc->render_time_ns = 0;
	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static int vdec_mediacodec_mcbuf_output_pool_get_cb(struct vbuf_buffer *buf,
						    int timeout_ms,
						    void *userdata)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;
	struct mcw_mediacodec *codec = (struct mcw_mediacodec *)userdata;
	ssize_t buf_idx;
	uint8_t *ptr;
	size_t buf_size = 0;
	struct mcw_mediacodec_bufferinfo info;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(userdata == NULL, EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	ULOG_ERRNO_RETURN_ERR_IF(mc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mc->mcw == NULL, EINVAL);

	buf_idx = mc->mcw->mediacodec.dequeue_output_buffer(
		codec, &info, (int64_t)timeout_ms * 1000);

	if (buf_idx == MCW_INFO_OUTPUT_BUFFERS_CHANGED) {
		ULOGI("dequeue_output_buffer: output buffers changed");
		return VDEC_MEDIACODEC_INFO_OUTPUT_BUFFERS_CHANGED;
	} else if (buf_idx == MCW_INFO_OUTPUT_FORMAT_CHANGED) {
		ULOGI("dequeue_output_buffer: output format changed");
		return VDEC_MEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED;
	} else if (buf_idx == MCW_INFO_TRY_AGAIN_LATER)
		return -EAGAIN;
	else if (buf_idx < 0) {
		ULOGE("failed to dequeue an output buffer, error=%zi", buf_idx);
		return -ENOENT;
	}

	ptr = mc->mcw->mediacodec.get_output_buffer(codec, buf_idx, &buf_size);
	if ((ptr == NULL) || (buf_size <= 0)) {
		ULOGE("failed to get output buffer #%zu", buf_idx);
		return -ENOENT;
	}

	mc->buf_idx = buf_idx;
	mc->offset = info.offset;
	mc->timestamp = info.presentation_time_us;
	mc->flags = info.flags;
	mc->render_time_ns = 0;
	buf->capacity = info.size;
	buf->size = buf_size;
	buf->ptr = ptr;

	return 0;
}


static int vdec_mediacodec_mcbuf_output_unref_cb(struct vbuf_buffer *buf,
						 void *userdata)
{
	struct vdec_mediacodec_mcbuf *mc = NULL;
	struct mcw_mediacodec *codec = (struct mcw_mediacodec *)userdata;
	enum mcw_media_status err;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(userdata == NULL, EINVAL);

	mc = (struct vdec_mediacodec_mcbuf *)buf->specific;
	ULOG_ERRNO_RETURN_ERR_IF(mc == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mc->mcw == NULL, EINVAL);

	if (mc->buf_idx >= 0) {
		if (mc->render_time_ns > 0)
			err = mc->mcw->mediacodec.release_output_buffer_at_time(
				codec, mc->buf_idx, mc->render_time_ns);
		else
			err = mc->mcw->mediacodec.release_output_buffer(
				codec,
				mc->buf_idx,
				(mc->render_time_ns == 0) ? false : true);
		if (err != MCW_MEDIA_STATUS_OK) {
			ULOGE("failed to release output buffer #%zu, error=%d",
			      mc->buf_idx,
			      err);
		}
	}

	mc->buf_idx = -1;
	mc->offset = 0;
	mc->timestamp = 0;
	mc->flags = 0;
	mc->render_time_ns = 0;
	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static int vdec_mediacodec_get_output_format(struct vdec_mediacodec *self,
					     size_t buf_idx)
{
	int ret;
	struct mcw_mediaformat *format;
	int32_t color_format = 0, stride = 0, slice_height = 0;

	format = self->mcw->mediacodec.get_output_format(self->codec, buf_idx);
	if (format == NULL) {
		ret = -EPROTO;
		ULOG_ERRNO("mediacodec.get_output_format", -ret);
		return ret;
	}

	/* Log all key/value pairs */
	const char *fmt = self->mcw->mediaformat.to_string(format);
	if (fmt != NULL) {
		char *fmt_dup = strdup(fmt);
		char *val, *tmp = NULL;
		val = strtok_r(fmt_dup, ",", &tmp);
		while (val) {
			ULOGD("output color format strings: %s", val);
			val = strtok_r(NULL, ",", &tmp);
		}
		free(fmt_dup);
	}

	self->mcw->mediaformat.get_int32(
		format, self->mcw->mediaformat.KEY_COLOR_FORMAT, &color_format);
	self->mcw->mediaformat.get_int32(
		format, self->mcw->mediaformat.KEY_STRIDE, &stride);
	self->mcw->mediaformat.get_int32(
		format, VDEC_MEDIACODEC_FORMAT_KEY_SLICE_HEIGHT, &slice_height);

	switch (color_format) {
	case MCW_COLOR_FORMAT_YUV420_PLANAR:
	case MCW_COLOR_FORMAT_YUV420_PACKED_PLANAR:
		self->output_format = VDEC_OUTPUT_FORMAT_I420;
		break;
	case MCW_COLOR_FORMAT_YUV420_SEMIPLANAR:
	case MCW_COLOR_FORMAT_YUV420_PACKED_SEMIPLANAR:
	case MCW_COLOR_FORMAT_TI_YUV420PackedSemiPlanar:
	case MCW_COLOR_FORMAT_QCOM_YUV420SemiPlanar:
	case MCW_COLOR_FORMAT_QCOM_YUV420SemiPlanar32m:
		self->output_format = VDEC_OUTPUT_FORMAT_NV12;
		break;
	case MCW_COLOR_FORMAT_QCOM_YUV420PackedSemiPlanar64x32Tile2m8ka:
		self->output_format = VDEC_OUTPUT_FORMAT_NV12MT;
		break;
	default:
		self->output_format = VDEC_OUTPUT_FORMAT_UNKNOWN;
		break;
	}
	self->output_format_changed = 0;
	self->stride = (stride > 0) ? (unsigned)stride : self->base->width;
	self->slice_height = (slice_height > 0) ? (unsigned)slice_height
						: self->base->height;

	ULOGI("output color format: %s (0x%08X), stride=%d slice-height=%d",
	      mcw_color_format_str(color_format),
	      color_format,
	      self->stride,
	      self->slice_height);

	self->mcw->mediaformat.ddelete(format);
	return 0;
}


static int vdec_mediacodec_do_flush(struct vdec_mediacodec *self)
{
	int ret;
	enum mcw_media_status err;

	if (self->flush_discard) {
		/* Flush the input and decoder queues */
		ret = vbuf_queue_flush(self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_flush:input", -ret);
			return ret;
		}
		ret = vbuf_queue_flush(self->decoder_queue);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_flush:decoder", -ret);
			return ret;
		}

		/* Flush the decoder */
		if (self->base->configured) {
			err = self->mcw->mediacodec.flush(self->codec);
			if (err != MCW_MEDIA_STATUS_OK) {
				ret = -EPROTO;
				ULOG_ERRNO("mediacodec.flush mc_error=%d",
					   -ret,
					   err);
				return ret;
			}
		}
	}

	/* Call the flush callback if defined */
	if (self->base->cbs.flush)
		(*self->base->cbs.flush)(self->base, self->base->userdata);
	self->flush = 0;

	return 0;
}


static int vdec_mediacodec_set_frame_metadata(struct vdec_mediacodec *self,
					      struct vbuf_buffer *in_buf,
					      struct vbuf_buffer *out_buf)
{
	int ret;
	struct vdec_input_metadata *in_meta = NULL;
	struct vdec_output_metadata *out_meta = NULL;
	const uint8_t *user_data;
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
	case VDEC_OUTPUT_FORMAT_NV12MT:
		out_meta->plane_offset[0] = 0;
		out_meta->plane_offset[1] = self->stride * self->slice_height;
		out_meta->plane_stride[0] = self->stride;
		out_meta->plane_stride[1] = self->stride;
		break;
	case VDEC_OUTPUT_FORMAT_I420:
		out_meta->plane_offset[0] = 0;
		out_meta->plane_offset[1] = self->stride * self->slice_height;
		out_meta->plane_offset[2] =
			out_meta->plane_offset[1] +
			self->stride / 2 * self->slice_height / 2;
		out_meta->plane_stride[0] = self->stride;
		out_meta->plane_stride[1] = self->stride / 2;
		out_meta->plane_stride[2] = self->stride / 2;
		break;
	case VDEC_OUTPUT_FORMAT_UNKNOWN:
	default:
		out_meta->plane_stride[0] = self->stride;
		break;
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


static int vdec_mediacodec_buffer_push_one(struct vdec_mediacodec *self,
					   struct vbuf_buffer *in_buf)
{
	int ret = 0, err;
	struct vdec_input_metadata *in_meta = NULL, *meta = NULL;
	struct timespec cur_ts = {0, 0};

	/* Copy the buffer in case it is not originating from the
	 * input buffer pool */
	if ((self->in_pool != NULL) &&
	    (vbuf_get_pool(in_buf) != self->in_pool)) {
		/* Get a buffer from the pool (if it fails, the input buffer
		 * will not be dequeued from the input queue and will be
		 * retried later) */
		struct vbuf_buffer *buf = NULL;
		ret = vbuf_pool_get(self->in_pool, 0, &buf);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("vbuf_pool_get", -ret);

			err = vbuf_metadata_get(in_buf,
						self->base,
						NULL,
						NULL,
						(uint8_t **)&in_meta);
			if ((err == 0) && (in_meta) && (!in_meta->ref)) {
				/* Drop the buffer if it's a non-ref frame;
				 * this is a workaround for some phones such
				 * as the Huawei P9 Lite, when no decoder input
				 * buffers are available leading to flushes and
				 * video not being displayed for 1-2 seconds */
				ULOGW("no buffer available in pool, "
				      "dropping non-ref frame to keep up");
				err = vbuf_queue_pop(
					self->in_queue, 0, &in_buf);
				if (err < 0) {
					ULOG_ERRNO("vbuf_queue_pop", -err);
					return ret;
				}
				vbuf_unref(in_buf);
				return 0;
			}

			return ret;
		}

		/* Really dequeue the input buffer */
		ret = vbuf_queue_pop(self->in_queue, 0, &in_buf);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_pop", -ret);
			vbuf_unref(buf);
			return ret;
		}

		/* Copy the buffer */
		ret = vbuf_copy(in_buf, buf);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_copy", -ret);
			vbuf_unref(in_buf);
			vbuf_unref(buf);
			return ret;
		}

		/* Convert to byte stream if necessary */
		ret = vbuf_metadata_get(
			buf, self->base, NULL, NULL, (uint8_t **)&meta);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_metadata_get", -ret);
			vbuf_unref(in_buf);
			vbuf_unref(buf);
			return ret;
		}
		ret = vdec_h264_format_convert(
			buf, meta->format, VDEC_INPUT_FORMAT_BYTE_STREAM);
		if (ret < 0) {
			vbuf_unref(in_buf);
			vbuf_unref(buf);
			return ret;
		}
		meta->format = VDEC_INPUT_FORMAT_BYTE_STREAM;

		vbuf_unref(in_buf);
		in_buf = buf;
	} else {
		/* Really dequeue the input buffer */
		ret = vbuf_queue_pop(self->in_queue, 0, &in_buf);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_pop", -ret);
			return ret;
		}
	}

	/* Discard the buffer if the decoder is not configured */
	if (!self->base->configured)
		goto out;

	/* Metadata check and update */
	ret = vbuf_metadata_get(
		in_buf, self->base, NULL, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get:input", -ret);
		goto out;
	}
	if (in_meta->format != VDEC_INPUT_FORMAT_BYTE_STREAM) {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported format: %s",
			   -ret,
			   vdec_input_format_str(in_meta->format));
		goto out;
	}
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &in_meta->dequeue_time);

	/* Debug files */
	if (self->base->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_h264_frame(
			self->base->dbg.input_bs, in_buf, in_meta->format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_h264_frame", -dbgret);
	}
	if (self->base->dbg.h264_analysis != NULL) {
		int dbgret = vdec_dbg_parse_h264_frame(
			self->base, in_buf, in_meta->format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_h264_frame", -dbgret);
	}

	/* Push the frame */
	ret = vdec_mediacodec_mcbuf_set_info(in_buf, 0, in_meta->timestamp, 0);
	if (ret < 0)
		goto out;
	ret = vbuf_queue_push(self->decoder_queue, in_buf);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_push:decoder", -ret);
		goto out;
	}
	ret = vdec_mediacodec_input_buffer_queue(in_buf, self->codec);
	if (ret < 0)
		goto out;

out:
	vbuf_unref(in_buf);
	return ret;
}


static int vdec_mediacodec_buffer_pop_all(struct vdec_mediacodec *self)
{
	int ret;
	struct vbuf_buffer *in_buf = NULL;
	struct vbuf_buffer *out_buf = NULL;
	struct vbuf_buffer *b;
	struct vdec_input_metadata *in_meta = NULL;
	size_t buf_idx = 0;
	uint64_t timestamp = 0;

	do {
		/* Get an output buffer (non-blocking) */
		ret = vbuf_pool_get(self->out_pool, 0, &out_buf);
		if (ret == VDEC_MEDIACODEC_INFO_OUTPUT_BUFFERS_CHANGED) {
			/* Change of buffers; nothing to do */
			continue;
		} else if (ret == VDEC_MEDIACODEC_INFO_OUTPUT_FORMAT_CHANGED) {
			/* Change of chroma format */
			self->output_format = VDEC_OUTPUT_FORMAT_UNKNOWN;
			self->output_format_changed = 1;
			continue;
		} else if (ret == -EAGAIN) {
			break;
		} else if ((ret < 0) || (out_buf == NULL)) {
			ULOGW("%s:%d: failed to get an output buffer "
			      "(frame dropped): %d(%s)",
			      __func__,
			      __LINE__,
			      -ret,
			      strerror(-ret));
			return ret;
		}

		ret = vdec_mediacodec_mcbuf_get_info(
			out_buf, &buf_idx, NULL, &timestamp, NULL);
		if (ret < 0) {
			vbuf_unref(out_buf);
			return ret;
		}
		if (self->output_format_changed) {
			ret = vdec_mediacodec_get_output_format(self, buf_idx);
			if (ret < 0) {
				ULOG_ERRNO("vdec_mediacodec_get_output_format",
					   -ret);
			}
		}

		/* Get the input buffer (non-blocking) */
		while ((ret = vbuf_queue_peek(self->decoder_queue, 0, 0, &b)) ==
		       0) {
			struct vdec_input_metadata *m;
			ret = vbuf_metadata_get(
				b, self->base, NULL, NULL, (uint8_t **)&m);

			if ((ret == 0) && (m)) {
				if (timestamp == m->timestamp) {
					ret = vbuf_queue_pop(
						self->decoder_queue, 0, &b);
					if (ret < 0) {
						ULOG_ERRNO("vbuf_queue_pop",
							   -ret);
						break;
					}
					in_buf = b;
					in_meta = m;
					break;
				} else if (timestamp > m->timestamp) {
					ret = vbuf_queue_pop(
						self->decoder_queue, 0, &b);
					if (ret < 0) {
						ULOG_ERRNO("vbuf_queue_pop",
							   -ret);
						break;
					}
					ULOGD("discarded input buffer with "
					      "TS %" PRIu64
					      " (expected %" PRIu64 ")",
					      m->timestamp,
					      timestamp);
					vbuf_unref(b);
				} else {
					ULOGW("input buffer not found: "
					      "expected TS %" PRIu64
					      ", got %" PRIu64,
					      timestamp,
					      m->timestamp);
					break;
				}
			} else {
				ULOG_ERRNO("vbuf_metadata_get", -ret);
				break;
			}
		}
		if (in_buf == NULL) {
			ret = -ENOENT;
			ULOG_ERRNO("invalid input buffer", -ret);
			vbuf_unref(out_buf);
			return ret;
		}
		if (in_meta == NULL) {
			ret = -EPROTO;
			ULOG_ERRNO("vbuf_metadata_get:decoder", -ret);
			vbuf_unref(in_buf);
			vbuf_unref(out_buf);
			return ret;
		}

		/* Set the metadata */
		ret = vdec_mediacodec_set_frame_metadata(self, in_buf, out_buf);
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
	} while ((!self->flush) && (!self->should_stop));

	return 0;
}


static void *vdec_mediacodec_decoder_thread(void *ptr)
{
	int ret, eagain_warn = 0;
	struct vdec_mediacodec *self = (struct vdec_mediacodec *)ptr;
	struct vbuf_buffer *in_buf;
	int timeout;

	while ((!self->should_stop) || (self->flush)) {
		in_buf = NULL;

		/* Flush discarding all frames */
		if ((self->flush) && (self->flush_discard)) {
			ret = vdec_mediacodec_do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_mediacodec_do_flush", -ret);
			goto pop;
		}

		/* Get an input buffer (with timeout); just peek, the buffer
		 * will be dequeued by the push_one() function */
		timeout = ((self->flush) && (!self->flush_discard)) ? 0 : 5;
		ret = vbuf_queue_peek(self->in_queue, 0, timeout, &in_buf);
		if (ret == -ETIMEDOUT) {
			if (self->base->configured)
				goto pop;
			else
				continue;
		} else if ((self->flush) && (ret == -EAGAIN)) {
			/* Flush without discarding frames */
			ret = vdec_mediacodec_do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_mediacodec_do_flush", -ret);
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
		ret = vdec_mediacodec_buffer_push_one(self, in_buf);
		if (ret < 0) {
			if (ret == -EAGAIN) {
				/* Retry later (log only once per frame) */
				if (!eagain_warn) {
					ULOGW("%s:%d: no buffer available, "
					      "retrying later",
					      __func__,
					      __LINE__);
					eagain_warn = 1;
				}
				usleep(5000);
			} else {
				ULOG_ERRNO("vdec_mediacodec_buffer_push_one",
					   -ret);
			}
			goto pop;
		}
		eagain_warn = 0;

	/* codecheck_ignore[INDENTED_LABEL] */
	pop:
		/* Pop output frames */
		ret = vdec_mediacodec_buffer_pop_all(self);
		if (ret < 0) {
			ULOG_ERRNO("vdec_mediacodec_buffer_pop_all", -ret);
			continue;
		}
	}

	/* Call the stop callback if defined */
	if (self->base->cbs.stop)
		(*self->base->cbs.stop)(self->base, self->base->userdata);

	return NULL;
}


uint32_t vdec_mediacodec_get_supported_input_format(void)
{
	return VDEC_INPUT_FORMAT_BYTE_STREAM;
}


int vdec_mediacodec_new(struct vdec_decoder *base)
{
	int ret = 0;
	struct vdec_mediacodec *self = NULL;

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
	if (base->config.sync_decoding) {
		ret = -EINVAL;
		ULOG_ERRNO("synchronized decoding is not supported", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = (struct vdec_derived *)self;

	/* Initialize the MediaCodec wrapper */
	ret = mcw_new(
		base->config.android_jvm, MCW_IMPLEMENTATION_AUTO, &self->mcw);
	if (ret < 0) {
		ULOG_ERRNO("mcw_new", -ret);
		goto error;
	}

	/* Initialize the decoder */
	self->codec = self->mcw->mediacodec.create_decoder_by_type(
		VDEC_MEDIACODEC_H264_MIME_TYPE);
	if (self->codec == NULL) {
		ret = -ENOSYS;
		ULOG_ERRNO("failed to create MediaCodec instance", -ret);
		goto error;
	}

	if (base->config.android_jvm != NULL) {
		int sdk_version;
		const char *codec_name;
		sdk_version = mcw_get_sdk_version(base->config.android_jvm);
		codec_name = mcw_get_codec_name(base->config.android_jvm,
						VDEC_MEDIACODEC_H264_MIME_TYPE,
						0);
		ULOGI("Android MediaCodec H.264 decoding - %s implementation - "
		      "SDK version %d - decoder: %s",
		      mcw_implementation_str(self->mcw->implem),
		      sdk_version,
		      (codec_name != NULL) ? codec_name : "UNKNOWN");
	} else {
		ULOGI("Android MediaCodec H.264 decoding - %s implementation",
		      mcw_implementation_str(self->mcw->implem));
	}

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

	/* Create the decoder thread */
	ret = pthread_create(&self->thread,
			     NULL,
			     vdec_mediacodec_decoder_thread,
			     (void *)self);
	if (ret != 0) {
		ret = -ret;
		ULOG_ERRNO("pthread_create", -ret);
		goto error;
	} else {
		self->thread_launched = 1;
	}

	return 0;

error:
	/* Cleanup on error */
	vdec_mediacodec_destroy(base);
	base->derived = NULL;
	return ret;
}


int vdec_mediacodec_flush(struct vdec_decoder *base, int discard)
{
	struct vdec_mediacodec *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = (struct vdec_mediacodec *)base->derived;

	self->flush = 1;
	self->flush_discard = discard;

	return 0;
}


int vdec_mediacodec_stop(struct vdec_decoder *base)
{
	int ret;
	struct vdec_mediacodec *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = (struct vdec_mediacodec *)base->derived;

	/* Stop the decoding thread */
	self->should_stop = 1;
	base->configured = 0;
	if (self->in_pool != NULL) {
		ret = vbuf_pool_abort(self->in_pool);
		if (ret < 0)
			ULOG_ERRNO("vbuf_pool_abort:input", -ret);
	}
	if (self->in_queue != NULL) {
		ret = vbuf_queue_abort(self->in_queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_abort:input", -ret);
	}
	if (self->decoder_queue != NULL) {
		ret = vbuf_queue_abort(self->decoder_queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_abort:decoder", -ret);
	}
	if (self->out_pool != NULL) {
		ret = vbuf_pool_abort(self->out_pool);
		if (ret < 0)
			ULOG_ERRNO("vbuf_pool_abort:output", -ret);
	}

	return 0;
}


int vdec_mediacodec_destroy(struct vdec_decoder *base)
{
	int err;
	struct vdec_mediacodec *self;
	enum mcw_media_status mcw_err;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = (struct vdec_mediacodec *)base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the output thread */
	err = vdec_mediacodec_stop(base);
	if (err < 0)
		ULOG_ERRNO("vdec_mediacodec_stop", -err);
	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
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
	if (self->in_pool != NULL) {
		err = vbuf_pool_abort(self->in_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_abort:input", -err);
		err = vbuf_pool_destroy(self->in_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_destroy:input", -err);
	}
	if (self->mcw != NULL) {
		if (self->codec != NULL) {
			if (self->codec_started) {
				mcw_err =
					self->mcw->mediacodec.stop(self->codec);
				if (mcw_err != MCW_MEDIA_STATUS_OK)
					ULOG_ERRNO(
						"mediacodec.stop mc_error=%d",
						EPROTO,
						mcw_err);
				self->codec_started = 0;
			}
			self->mcw->mediacodec.ddelete(self->codec);
		}
		mcw_destroy(self->mcw);
	}

	free(self);

	return 0;
}


int vdec_mediacodec_set_sps_pps(struct vdec_decoder *base,
				const uint8_t *sps,
				size_t sps_size,
				const uint8_t *pps,
				size_t pps_size,
				enum vdec_input_format format)
{
	int ret = 0;
	struct vdec_mediacodec *self;
	enum mcw_media_status err;
	size_t offset = (format == VDEC_INPUT_FORMAT_RAW_NALU) ? 0 : 4;
	struct mcw_mediaformat *mformat = NULL;
	struct vbuf_cbs in_buf_cbs;
	struct vbuf_cbs out_buf_cbs;
	unsigned int out_buf_count;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format != VDEC_INPUT_FORMAT_BYTE_STREAM,
				 EINVAL);

	self = (struct vdec_mediacodec *)base->derived;

	/* Configure the decoder */
	mformat = self->mcw->mediaformat.nnew();
	if (mformat == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("mediaformat.new", -ret);
		goto out;
	}

	self->mcw->mediaformat.set_string(mformat,
					  self->mcw->mediaformat.KEY_MIME,
					  VDEC_MEDIACODEC_H264_MIME_TYPE);
	self->mcw->mediaformat.set_int32(
		mformat, self->mcw->mediaformat.KEY_WIDTH, 640);
	self->mcw->mediaformat.set_int32(
		mformat, self->mcw->mediaformat.KEY_HEIGHT, 480);
	self->mcw->mediaformat.set_int32(
		mformat,
		self->mcw->mediaformat.KEY_MAX_INPUT_SIZE,
		base->width * base->height * 3 / 4);
	self->mcw->mediaformat.set_buffer(
		mformat, "csd-0", (void *)sps, sps_size);
	self->mcw->mediaformat.set_buffer(
		mformat, "csd-1", (void *)pps, pps_size);

	err = self->mcw->mediacodec.configure(
		self->codec, mformat, NULL, NULL, 0);
	if (err != MCW_MEDIA_STATUS_OK) {
		ret = -EPROTO;
		ULOG_ERRNO("mediacodec.configure mc_error=%d", -ret, err);
		goto out;
	}

	/* Allocate the input buffers pool */
	memset(&in_buf_cbs, 0, sizeof(in_buf_cbs));
	in_buf_cbs.alloc = &vdec_mediacodec_mcbuf_alloc_cb;
	in_buf_cbs.alloc_userdata = (void *)self->mcw;
	in_buf_cbs.free = &vdec_mediacodec_mcbuf_free_cb;
	in_buf_cbs.free_userdata = (void *)self->mcw;
	in_buf_cbs.pool_get = &vdec_mediacodec_mcbuf_input_pool_get_cb;
	in_buf_cbs.pool_get_userdata = (void *)self->codec;
	in_buf_cbs.pool_put = &vdec_mediacodec_mcbuf_input_pool_put_cb;
	in_buf_cbs.pool_put_userdata = (void *)self->codec;
	ret = vbuf_pool_new(VDEC_MEDIACODEC_INPUT_BUFFER_COUNT,
			    0,
			    0,
			    &in_buf_cbs,
			    &self->in_pool);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_pool_new:input", -ret);
		goto out;
	}

	/* Allocate the output buffers pool */
	memset(&out_buf_cbs, 0, sizeof(out_buf_cbs));
	out_buf_cbs.alloc = &vdec_mediacodec_mcbuf_alloc_cb;
	out_buf_cbs.alloc_userdata = (void *)self->mcw;
	out_buf_cbs.unref = &vdec_mediacodec_mcbuf_output_unref_cb;
	out_buf_cbs.unref_userdata = (void *)self->codec;
	out_buf_cbs.free = &vdec_mediacodec_mcbuf_free_cb;
	out_buf_cbs.free_userdata = (void *)self->mcw;
	out_buf_cbs.pool_get = &vdec_mediacodec_mcbuf_output_pool_get_cb;
	out_buf_cbs.pool_get_userdata = (void *)self->codec;
	out_buf_count = VDEC_MEDIACODEC_OUTPUT_BUFFER_COUNT;
	if (base->config.preferred_min_out_buf_count > out_buf_count)
		out_buf_count = base->config.preferred_min_out_buf_count;
	ret = vbuf_pool_new(out_buf_count, 0, 0, &out_buf_cbs, &self->out_pool);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_pool_new:output", -ret);
		goto out;
	}

	/* Start the decoder */
	err = self->mcw->mediacodec.start(self->codec);
	if (err != MCW_MEDIA_STATUS_OK) {
		ret = -EPROTO;
		ULOG_ERRNO("mediacodec.start mc_error=%d", -ret, err);
		goto out;
	}
	self->codec_started = 1;

out:
	if (mformat != NULL)
		self->mcw->mediaformat.ddelete(mformat);

	return ret;
}


struct vbuf_pool *
vdec_mediacodec_get_input_buffer_pool(struct vdec_decoder *base)
{
	struct vdec_mediacodec *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = (struct vdec_mediacodec *)base->derived;

	return self->in_pool;
}


struct vbuf_queue *
vdec_mediacodec_get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_mediacodec *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = (struct vdec_mediacodec *)base->derived;

	return self->in_queue;
}


int vdec_mediacodec_sync_decode(struct vdec_decoder *base,
				struct vbuf_buffer *in_buf,
				struct vbuf_buffer **out_buf)
{
	/* Synchronized decoding is not supported */
	return -ENOSYS;
}

#endif /* BUILD_LIBMEDIACODEC_WRAPPER */

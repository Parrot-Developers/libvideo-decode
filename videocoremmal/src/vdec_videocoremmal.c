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

#define ULOG_TAG vdec_videocoremmal
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


#include "vdec_videocoremmal_priv.h"


#define NB_SUPPORTED_FORMATS 1
static struct vdef_coded_format supported_formats[NB_SUPPORTED_FORMATS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_h264_byte_stream;
}


static void flush_complete(struct vdec_videocoremmal *self)
{
	/* Call the flush callback if defined */
	if (self->base->cbs.flush)
		self->base->cbs.flush(self->base, self->base->userdata);
}


static void stop_complete(struct vdec_videocoremmal *self)
{
	/* Call the stop callback if defined */
	if (self->base->cbs.stop)
		self->base->cbs.stop(self->base, self->base->userdata);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct vdec_videocoremmal *self = userdata;
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
	struct vdec_videocoremmal *self = userdata;
	struct vbuf_buffer *out_buf = NULL;
	int ret;

	do {
		ret = vbuf_queue_pop(self->out_queue, 0, &out_buf);
		if (ret == -EAGAIN) {
			return;
		} else if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_pop:output", -ret);
			return;
		}
		self->base->cbs.frame_output(
			self->base, 0, out_buf, self->base->userdata);
		vbuf_unref(out_buf);
		out_buf = NULL;
	} while (ret == 0);
}


static int vdec_videocoremmal_mmalbuf_set_buf(struct vbuf_buffer *buf,
					      MMAL_BUFFER_HEADER_T *mmal_buf)
{
	struct vdec_videocoremmal_mmalbuf *mb;

	ULOG_ERRNO_RETURN_ERR_IF(buf->type != VDEC_VIDEOCOREMMAL_BUF_TYPE_MMAL,
				 EINVAL);

	mb = (struct vdec_videocoremmal_mmalbuf *)buf->specific;

	mb->mmal_buf = mmal_buf;

	if (mmal_buf == NULL) {
		buf->capacity = 0;
		buf->size = 0;
		buf->ptr = NULL;
	} else {
		buf->capacity = mb->mmal_buf->alloc_size;
		buf->size = 0;
		buf->ptr = mb->mmal_buf->data;
	}

	return 0;
}


static MMAL_BUFFER_HEADER_T *
vdec_videocoremmal_mmalbuf_get_buf(struct vbuf_buffer *buf)
{
	struct vdec_videocoremmal_mmalbuf *mb;

	ULOG_ERRNO_RETURN_VAL_IF(
		buf->type != VDEC_VIDEOCOREMMAL_BUF_TYPE_MMAL, EINVAL, NULL);

	mb = (struct vdec_videocoremmal_mmalbuf *)buf->specific;

	if (mb)
		return mb->mmal_buf;
	else
		return NULL;
}


static int vdec_videocoremmal_mmalbuf_alloc_cb(struct vbuf_buffer *buf,
					       void *userdata)
{
	struct vdec_videocoremmal_mmalbuf *mb = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	buf->type = VDEC_VIDEOCOREMMAL_BUF_TYPE_MMAL;

	mb = calloc(1, sizeof(struct vdec_videocoremmal_mmalbuf));
	if (mb == NULL)
		return -ENOMEM;
	buf->specific = (struct vbuf_specific *)mb;
	mb->mmal_buf = NULL;

	return 0;
}


static int vdec_videocoremmal_mmalbuf_free_cb(struct vbuf_buffer *buf,
					      void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	free(buf->specific);
	buf->specific = NULL;
	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static int vdec_videocoremmal_mmalbuf_unref_cb(struct vbuf_buffer *buf,
					       void *userdata)
{
	struct vdec_videocoremmal_mmalbuf *mb = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	mb = (struct vdec_videocoremmal_mmalbuf *)buf->specific;

	if (mb->mmal_buf != NULL)
		mmal_buffer_header_release(mb->mmal_buf);
	mb->mmal_buf = NULL;
	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static int vdec_videocoremmal_mmalbuf_pool_get_cb(struct vbuf_buffer *buf,
						  int timeout_ms,
						  void *userdata)
{
	int ret;
	MMAL_QUEUE_T *mmal_queue = (MMAL_QUEUE_T *)userdata;
	MMAL_BUFFER_HEADER_T *mmal_buf;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	if (mmal_queue == NULL)
		return vdec_videocoremmal_mmalbuf_set_buf(buf, NULL);

	mmal_buf = mmal_queue_get(mmal_queue);
	if (mmal_buf == NULL) {
		ret = -EAGAIN;
		ULOG_ERRNO("mmal_queue_get", -ret);
		return ret;
	}

	return vdec_videocoremmal_mmalbuf_set_buf(buf, mmal_buf);
}


static void vdec_videocoremmal_control_cb(MMAL_PORT_T *port,
					  MMAL_BUFFER_HEADER_T *buffer)
{
	MMAL_STATUS_T status;

	ULOG_ERRNO_RETURN_IF(port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buffer == NULL, EINVAL);

	if (buffer->cmd == MMAL_EVENT_ERROR) {
		status = *(uint32_t *)buffer->data;
		ULOGE("MMAL error on control port %d(%s)",
		      (int)status,
		      mmal_status_to_string(status));
	} else {
		ULOGW("unknown MMAL event 0x%08X on control port", buffer->cmd);
	}

	mmal_buffer_header_release(buffer);
}


static void vdec_videocoremmal_input_cb(MMAL_PORT_T *port,
					MMAL_BUFFER_HEADER_T *buffer)
{
	ULOG_ERRNO_RETURN_IF(port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buffer == NULL, EINVAL);

	if ((buffer->cmd == 0) && (buffer->user_data != NULL))
		vbuf_unref((struct vbuf_buffer *)buffer->user_data);
}


static void vdec_videocoremmal_output_cb(MMAL_PORT_T *port,
					 MMAL_BUFFER_HEADER_T *buffer)
{
	struct vdec_videocoremmal *self;

	ULOG_ERRNO_RETURN_IF(port == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buffer == NULL, EINVAL);

	self = (struct vdec_videocoremmal *)port->userdata;
	mmal_queue_put(self->mmal_out_queue, buffer);
}


static int
vdec_videocoremmal_return_output_buffers(struct vdec_videocoremmal *self)
{
	int ret = 0;
	MMAL_BUFFER_HEADER_T *buffer;
	MMAL_STATUS_T status;

	while ((buffer = mmal_queue_get(self->mmal_out_pool->queue)) != NULL) {
		status =
			mmal_port_send_buffer(self->decoder->output[0], buffer);
		if (status != MMAL_SUCCESS) {
			ret = -EPROTO;
			ULOG_ERRNO(
				"mmal_port_send_buffer "
				"mmal_status=%d",
				-ret,
				status);
		}
	}

	return ret;
}


/* Flush discarding frames */
static int vdec_videocoremmal_do_flush(struct vdec_videocoremmal *self)
{
	int ret;
	MMAL_STATUS_T status;
	MMAL_BUFFER_HEADER_T *buffer = NULL;

	/* Disable all ports */
	status = mmal_port_disable(self->decoder->input[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_disable:input mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_disable(self->decoder->output[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_disable:output mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_disable(self->decoder->control);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_disable:control mmal_status=%d",
			   EPROTO,
			   status);
	}

	/* Flush the queues */
	ret = vbuf_queue_flush(self->in_queue);
	if (ret < 0)
		ULOG_ERRNO("vbuf_queue_flush:input", -ret);
	ret = vbuf_queue_flush(self->decoder_queue);
	if (ret < 0)
		ULOG_ERRNO("vbuf_queue_flush:decoder", -ret);
	ret = vbuf_queue_flush(self->out_queue);
	if (ret < 0)
		ULOG_ERRNO("vbuf_queue_flush:output", -ret);

	/* Flush all ports */
	status = mmal_port_flush(self->decoder->input[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO(
			"mmal_port_flush:input mmal_status=%d", EPROTO, status);
	}
	status = mmal_port_flush(self->decoder->output[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_flush:output mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_flush(self->decoder->control);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_flush:control mmal_status=%d",
			   EPROTO,
			   status);
	}

	while ((buffer = mmal_queue_get(self->mmal_out_queue)))
		mmal_buffer_header_release(buffer);

	/* Enable all ports */
	status = mmal_port_enable(self->decoder->control,
				  &vdec_videocoremmal_control_cb);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_enable:control mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_enable(self->decoder->output[0],
				  &vdec_videocoremmal_output_cb);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_enable:output mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_enable(self->decoder->input[0],
				  &vdec_videocoremmal_input_cb);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_enable:input mmal_status=%d",
			   EPROTO,
			   status);
	}

	/* Call the flush callback on the loop */
	char message = VDEC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);

	self->flush = 0;

	return 0;
}


/* Flush without discarding frames */
static int vdec_videocoremmal_complete_flush(struct vdec_videocoremmal *self)
{
	int ret;

	self->flushing = 0;

	/* Call the flush callback on the loop */
	char message = VDEC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);

	self->flush = 0;

	return 0;
}


/* Flush without discarding frames */
static int vdec_videocoremmal_start_flush(struct vdec_videocoremmal *self)
{
	int ret;

	/* Flush the decoder */
	self->flushing = 1;

	if (self->last_output_pts >= self->last_input_pts) {
		ret = vdec_videocoremmal_complete_flush(self);
		if (ret < 0)
			ULOG_ERRNO("vdec_videocoremmal_complete_flush", -ret);
	}

	return 0;
}


static void vdec_videocoremmal_do_stop(struct vdec_videocoremmal *self)
{
	int ret;
	MMAL_STATUS_T status;
	MMAL_BUFFER_HEADER_T *buffer = NULL;

	/* Disable all ports */
	status = mmal_port_disable(self->decoder->input[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_disable:input mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_disable(self->decoder->output[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_disable:output mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_disable(self->decoder->control);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_disable:control mmal_status=%d",
			   EPROTO,
			   status);
	}

	/* Flush all ports */
	status = mmal_port_flush(self->decoder->input[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO(
			"mmal_port_flush:input mmal_status=%d", EPROTO, status);
	}
	status = mmal_port_flush(self->decoder->output[0]);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_flush:output mmal_status=%d",
			   EPROTO,
			   status);
	}
	status = mmal_port_flush(self->decoder->control);
	if (status != MMAL_SUCCESS) {
		ULOG_ERRNO("mmal_port_flush:control mmal_status=%d",
			   EPROTO,
			   status);
	}

	while ((buffer = mmal_queue_get(self->mmal_out_queue)))
		mmal_buffer_header_release(buffer);

	/* Call the stop callback on the loop */
	char message = VDEC_MSG_STOP;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);
}


static int
vdec_videocoremmal_set_frame_metadata(struct vdec_videocoremmal *self,
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
	out_meta->frame.format = self->output_format;
	out_meta->frame.info = in_meta->frame.info;
	if (vdef_raw_format_cmp(&out_meta->frame.format, &vdef_nv12)) {
		out_meta->plane_offset[0] = 0;
		out_meta->plane_offset[1] =
			self->stride * self->base->video_info.resolution.height;
		out_meta->frame.plane_stride[0] = self->stride;
		out_meta->frame.plane_stride[1] = self->stride;
	} else if (vdef_raw_format_cmp(&out_meta->frame.format, &vdef_i420)) {
		out_meta->plane_offset[0] = 0;
		out_meta->plane_offset[1] =
			self->stride * self->base->video_info.resolution.height;
		out_meta->plane_offset[2] =
			self->stride *
			self->base->video_info.resolution.height * 5 / 4;
		out_meta->frame.plane_stride[0] = self->stride;
		out_meta->frame.plane_stride[1] = self->stride / 2;
		out_meta->frame.plane_stride[2] = self->stride / 2;
	} else if (vdef_raw_format_cmp(&out_meta->frame.format,
				       &vdef_mmal_opaque)) {
		out_meta->frame.plane_stride[0] = self->stride;
	} else {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output chroma format", -ret);
		return ret;
	}
	out_meta->frame.info.bit_depth = self->base->video_info.bit_depth;
	out_meta->frame.info.full_range = self->base->video_info.full_range;
	out_meta->frame.info.color_primaries =
		self->base->video_info.color_primaries;
	out_meta->frame.info.transfer_function =
		self->base->video_info.transfer_function;
	out_meta->frame.info.matrix_coefs = self->base->video_info.matrix_coefs;
	out_meta->frame.info.resolution.width =
		self->base->video_info.crop.width;
	out_meta->frame.info.resolution.height =
		self->base->video_info.crop.height;
	out_meta->frame.info.sar = self->base->video_info.sar;
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


static int
vdec_videocoremmal_insert_grey_idr(struct vdec_videocoremmal *self,
				   struct vdec_input_metadata *in_meta,
				   unsigned int level,
				   uint64_t *delta)
{
	int ret;
	struct h264_ctx *ctx = h264_reader_get_ctx(self->base->reader.h264);
	uint64_t timestamp;
	MMAL_BUFFER_HEADER_T *mmal_buf = NULL;
	MMAL_STATUS_T status;
	struct vbuf_buffer *idr_buf = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(in_meta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);

	ret = vbuf_pool_get(self->in_pool, 0, &idr_buf);
	if (ret < 0) {
		if (ret != -EAGAIN)
			ULOG_ERRNO("vbuf_pool_get", -ret);
		return ret;
	}

	timestamp = in_meta->frame.info.timestamp;

	ret = vdec_h264_write_grey_idr(
		self->base, in_meta, level, delta, &timestamp, idr_buf);
	if (ret < 0) {
		ULOG_ERRNO("vdec_h264_write_grey_idr", -ret);
		goto out;
	}

	/* Push the frame */
	mmal_buf = vdec_videocoremmal_mmalbuf_get_buf(idr_buf);
	if (mmal_buf == NULL) {
		ret = -EPROTO;
		ULOG_ERRNO("vdec_videocoremmal_mmalbuf_get_buf:input", -ret);
		goto out;
	}
	mmal_buf->cmd = 0;
	mmal_buf->length = vbuf_get_size(idr_buf);
	mmal_buf->offset = 0;
	mmal_buf->flags = MMAL_BUFFER_HEADER_FLAG_FRAME;
	mmal_buf->pts = in_meta->frame.info.timestamp;
	self->last_input_pts = in_meta->frame.info.timestamp;
	mmal_buf->dts = in_meta->frame.info.timestamp;
	mmal_buf->user_data = idr_buf;

	status = mmal_port_send_buffer(self->decoder->input[0], mmal_buf);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO(
			"mmal_port_send_buffer mmal_status=%d", -ret, status);
		goto out;
	}

out:
	if (idr_buf)
		vbuf_unref(idr_buf);

	return ret;
}


static int vdec_videocoremmal_buffer_push_one(struct vdec_videocoremmal *self,
					      struct vbuf_buffer *in_buf)
{
	int ret = 0;
	struct vdec_input_metadata *in_meta = NULL, *meta = NULL;
	struct timespec cur_ts = {0, 0};
	MMAL_BUFFER_HEADER_T *mmal_buf = NULL;
	MMAL_STATUS_T status;
	uint64_t delta = 0;
	unsigned int level = 0;

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
		ret = vdec_format_convert(
			buf, meta->frame.format, VDEF_H264_BYTE_STREAM);
		if (ret < 0) {
			vbuf_unref(in_buf);
			vbuf_unref(buf);
			return ret;
		}
		meta->frame.format = VDEF_H264_BYTE_STREAM;

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
	if (ret < 0)
		goto out;
	if (in_meta->frame.format != VDEF_H264_BYTE_STREAM) {
		ret = -ENOSYS;
		ULOG_ERRNO(
			"unsupported format: " VDEF_CODED_FORMAT_TO_STR_FMT,
			-ret,
			VDEF_CODED_FORMAT_TO_STR_ARG2(in_meta->frame.format));
		goto out;
	}
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &in_meta->dequeue_time);

	if (self->need_sync) {
		if (vdec_is_sync_frame(in_buf, in_meta)) {
			ULOGI("frame is a sync point");
			self->need_sync = 0;
		} else {
			if (self->base->config.encoding == VDEF_ENCODING_H264 &&
			    self->base->config.gen_grey_idr) {
				ULOGI("frame is not an IDR, "
				      "generating grey IDR");
				ret = vdec_videocoremmal_insert_grey_idr(
					self, in_meta, level, &delta);
				if (ret < 0) {
					ULOG_ERRNO(
						"vdec_videocoremmal_insert_grey_idr",
						-ret);
					return ret;
				}
				self->need_sync = 0;
			} else {
				ULOGI("frame is not a sync point, "
				      "discarding frame");
				goto out;
			}
		}
	}

	/* Debug files */
	if (self->base->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_frame(self->base->dbg.input_bs,
						  in_buf,
						  &in_meta->frame.format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_frame", -dbgret);
	}
	if (self->base->dbg.analysis != NULL) {
		int dbgret = vdec_dbg_parse_frame(
			self->base, in_buf, in_meta->frame.format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_frame", -dbgret);
	}

	/* Send the input buffer for decoding */
	mmal_buf = vdec_videocoremmal_mmalbuf_get_buf(in_buf);
	if (mmal_buf == NULL) {
		ret = -EPROTO;
		ULOG_ERRNO("vdec_videocoremmal_mmalbuf_get_buf:input", -ret);
		goto out;
	}
	mmal_buf->cmd = 0;
	mmal_buf->length = vbuf_get_size(in_buf);
	mmal_buf->offset = 0;
	mmal_buf->flags = MMAL_BUFFER_HEADER_FLAG_FRAME;
	mmal_buf->pts = in_meta->frame.info.timestamp + delta;
	self->last_input_pts = in_meta->frame.info.timestamp + delta;
	mmal_buf->dts = in_meta->frame.info.timestamp + delta;
	mmal_buf->user_data = in_buf;
	ret = vbuf_ref(in_buf);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_ref", -ret);
		goto out;
	}
	ret = vbuf_queue_push(self->decoder_queue, in_buf);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_push:decoder", -ret);
		goto out;
	}
	status = mmal_port_send_buffer(self->decoder->input[0], mmal_buf);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO(
			"mmal_port_send_buffer mmal_status=%d", -ret, status);
		goto out;
	}

out:
	vbuf_unref(in_buf);
	return ret;
}


static int vdec_videocoremmal_format_changed(struct vdec_videocoremmal *self,
					     MMAL_BUFFER_HEADER_T *buffer)
{
	int ret = 0;
	MMAL_STATUS_T status;
	MMAL_BUFFER_HEADER_T *buf;
	MMAL_EVENT_FORMAT_CHANGED_T *ev = NULL;
	MMAL_ES_FORMAT_T *format_out = NULL;

	if (self == NULL)
		return -EINVAL;
	if (buffer == NULL)
		return -EINVAL;

	ev = mmal_event_format_changed_get(buffer);

	status = mmal_port_disable(self->decoder->output[0]);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_port_disable:output mmal_status=%d",
			   -ret,
			   status);
		return ret;
	}

	while ((buf = mmal_queue_get(self->mmal_out_queue)))
		mmal_buffer_header_release(buf);

	/* Copy the new output port format */
	format_out = self->decoder->output[0]->format;
	mmal_format_copy(format_out, ev->format);

	/* Set the format of the output port */
	if (vdef_raw_format_cmp(&self->output_format, &vdef_mmal_opaque)) {
		format_out->encoding = MMAL_ENCODING_OPAQUE;
		status = mmal_port_parameter_set_boolean(
			self->decoder->output[0],
			MMAL_PARAMETER_ZERO_COPY,
			MMAL_TRUE);
		if (status != MMAL_SUCCESS) {
			ret = -EPROTO;
			ULOG_ERRNO(
				"mmal_port_parameter_set:ZERO_COPY "
				"mmal_status=%d",
				-ret,
				status);
			return ret;
		}
		status = mmal_port_parameter_set_uint32(
			self->decoder->output[0],
			MMAL_PARAMETER_EXTRA_BUFFERS,
			VDEC_VIDEOCOREMMAL_OUT_NUM_EXTRA_BUFFERS);
		if (status != MMAL_SUCCESS) {
			ret = -EPROTO;
			ULOG_ERRNO(
				"mmal_port_parameter_set:EXTRA_BUFFERS "
				"mmal_status=%d",
				-ret,
				status);
			return ret;
		}
	} else {
		format_out->encoding = MMAL_ENCODING_I420;
		format_out->encoding_variant = MMAL_ENCODING_I420;
	}

	status = mmal_port_format_commit(self->decoder->output[0]);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_port_format_commit:output mmal_status=%d",
			   -ret,
			   status);
	}

	status = mmal_port_enable(self->decoder->output[0],
				  &vdec_videocoremmal_output_cb);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO(
			"mmal_port_enable:output mmal_status=%d", -ret, status);
	}

	return ret;
}


static int vdec_videocoremmal_buffer_pop_all(struct vdec_videocoremmal *self)
{
	int ret = 0;
	struct vbuf_buffer *out_buf = NULL;
	struct vbuf_buffer *in_buf = NULL, *b;
	struct vdec_input_metadata *in_meta = NULL;
	MMAL_BUFFER_HEADER_T *buffer;

	buffer = mmal_queue_get(self->mmal_out_queue);
	while ((ret == 0) && (buffer != NULL)) {
		if (buffer->cmd == MMAL_EVENT_FORMAT_CHANGED) {
			ULOGI("format changed event");
			ret = vdec_videocoremmal_format_changed(self, buffer);
			goto out;
		} else if (buffer->cmd != 0) {
			ULOGW("unknown MMAL event 0x%08X on output port",
			      buffer->cmd);
			goto out;
		} else if (buffer->length == 0) {
			goto out;
		}
		self->last_output_pts = buffer->pts;

		/* Get the input buffer (non-blocking) */
		while ((ret = vbuf_queue_pop(self->decoder_queue, 0, &b)) ==
		       0) {
			struct vdec_input_metadata *m;
			ret = vbuf_metadata_get(
				b, self->base, NULL, NULL, (uint8_t **)&m);

			if ((ret == 0) && (m) &&
			    ((unsigned)buffer->pts ==
			     m->frame.info.timestamp)) {
				in_buf = b;
				in_meta = m;
				break;
			} else {
				if (m) {
					ULOGD("discarded input buffer with "
					      "TS %" PRIu64
					      " (expected %" PRIu64 ")",
					      m->frame.info.timestamp,
					      buffer->pts);
				}
				vbuf_unref(b);
			}
		}
		if (in_buf == NULL) {
			ret = -ENOENT;
			ULOG_ERRNO("invalid input buffer", -ret);
			goto out;
		}
		if (in_meta == NULL) {
			ret = -EPROTO;
			ULOG_ERRNO("vbuf_metadata_get:decoder", -ret);
			goto out;
		}

		/* Discard frame if flush is in progress */
		if (self->flushing) {
			if ((unsigned)buffer->pts >= self->last_input_pts)
				ret = vdec_videocoremmal_complete_flush(self);
			goto out;
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
			goto out;
		}
		mmal_buffer_header_acquire(buffer);
		ret = vdec_videocoremmal_mmalbuf_set_buf(out_buf, buffer);
		if (ret < 0)
			goto out;

		/* Set the metadata */
		ret = vdec_videocoremmal_set_frame_metadata(
			self, in_buf, out_buf);
		if (ret < 0)
			goto out;

		/* Push the frame (if not silent) */
		ret = vbuf_write_lock(out_buf);
		if (ret < 0)
			ULOG_ERRNO("vbuf_write_lock", -ret);
		if ((in_meta->silent) &&
		    (!self->base->config.output_silent_frames)) {
			ULOGD("silent frame (ignored)");
		} else {
			ret = vbuf_queue_push(self->out_queue, out_buf);
			if (ret < 0) {
				ULOG_ERRNO("vbuf_queue_push:output", -ret);
				return ret;
			}
		}

	/* codecheck_ignore[INDENTED_LABEL] */
	out:
		if (buffer != NULL)
			mmal_buffer_header_release(buffer);
		if (out_buf) {
			vbuf_unref(out_buf);
			out_buf = NULL;
		}
		if (in_buf) {
			vbuf_unref(in_buf);
			in_buf = NULL;
		}
		if (ret == 0)
			buffer = mmal_queue_get(self->mmal_out_queue);
	}

	return ret;
}


static void *vdec_videocoremmal_decoder_thread(void *ptr)
{
	int ret, eagain_warn = 0;
	struct vdec_videocoremmal *self = (struct vdec_videocoremmal *)ptr;
	struct vbuf_buffer *in_buf = NULL;
	int timeout;

	while ((!self->should_stop) || (self->flush)) {
		/* Return all available output buffers to the output port */
		if (self->base->configured) {
			ret = vdec_videocoremmal_return_output_buffers(self);
			if (ret < 0) {
				ULOG_ERRNO(
					"vdec_videocoremmal_return_"
					"output_buffers",
					-ret);
				goto pop;
			}
		}

		/* Flush, discarding all frames */
		if ((self->flush) && (self->flush_discard)) {
			ret = vdec_videocoremmal_do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_videocoremmal_do_flush", -ret);
			if (self->base->config.encoding == VDEF_ENCODING_H264)
				self->need_sync = 1;
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
			ret = vdec_videocoremmal_start_flush(self);
			if (ret < 0) {
				ULOG_ERRNO("vdec_videocoremmal_start_flush",
					   -ret);
			}
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
		ret = vdec_videocoremmal_buffer_push_one(self, in_buf);
		if (ret < 0) {
			if (ret == -EAGAIN) {
				/* Retry later (log only once per frame) */
				if (!eagain_warn) {
					ULOGW("no buffer available, "
					      "retrying later...");
					eagain_warn = 1;
				}
				usleep(5000);
			} else {
				ULOG_ERRNO("vdec_videocoremmal_buffer_push_one",
					   -ret);
			}
			continue;
		}
		eagain_warn = 0;

	/* codecheck_ignore[INDENTED_LABEL] */
	pop:
		/* Pop output frames */
		ret = vdec_videocoremmal_buffer_pop_all(self);
		if (ret < 0) {
			ULOG_ERRNO("vdec_videocoremmal_buffer_pop_all", -ret);
			continue;
		}
	}

	vdec_videocoremmal_do_stop(self);

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
	int ret;
	struct vdec_videocoremmal *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the decoding thread */
	self->should_stop = 1;
	base->configured = 0;
	if (self->out_pool != NULL) {
		ret = vbuf_pool_abort(self->out_pool);
		if (ret < 0)
			ULOG_ERRNO("vbuf_pool_abort:output", -ret);
	}
	if (self->out_queue != NULL) {
		ret = vbuf_queue_abort(self->out_queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_abort:output", -ret);
	}
	if (self->decoder_queue != NULL) {
		ret = vbuf_queue_abort(self->decoder_queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_abort:decoder", -ret);
	}
	if (self->in_queue != NULL) {
		ret = vbuf_queue_abort(self->in_queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_abort:input", -ret);
	}
	if (self->in_pool != NULL) {
		ret = vbuf_pool_abort(self->in_pool);
		if (ret < 0)
			ULOG_ERRNO("vbuf_pool_abort:input", -ret);
	}

	return 0;
}


static int destroy(struct vdec_decoder *base)
{
	int err;
	MMAL_STATUS_T status;
	struct vdec_videocoremmal *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the output thread */
	err = stop(base);
	if (err < 0)
		ULOG_ERRNO("vdec_videocoremmal_stop", -err);
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
	if (self->out_queue_evt != NULL) {
		err = pomp_evt_detach_from_loop(self->out_queue_evt,
						base->loop);
		if (err < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->out_queue != NULL) {
		err = vbuf_queue_abort(self->out_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_abort:output", -err);
		err = vbuf_queue_destroy(self->out_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_destroy:output", -err);
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
	if (self->mbox != NULL) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			ULOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	if (self->decoder) {
		status = mmal_component_destroy(self->decoder);
		if (status != MMAL_SUCCESS) {
			ULOG_ERRNO(
				"mmal_component_destroy "
				"mmal_status=%d",
				EPROTO,
				status);
		}
	}
	if (self->mmal_in_pool != NULL)
		mmal_pool_destroy(self->mmal_in_pool);
	if (self->mmal_out_pool != NULL) {
		if (self->output_format == VDEF_BCM_MMAL_OPAQUE) {
			mmal_port_pool_destroy(self->decoder->output[0],
					       self->mmal_out_pool);
		} else {
			mmal_pool_destroy(self->mmal_out_pool);
		}
	}
	if (self->mmal_out_queue)
		mmal_queue_destroy(self->mmal_out_queue);
	mmal_vc_deinit();
	free(self);

	return 0;
}


static int create(struct vdec_decoder *base)
{
	int ret = 0;
	struct vdec_videocoremmal *self = NULL;
	MMAL_STATUS_T status;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	/* Check the configuration */
	if (base->config.encoding != VDEF_ENCODING_H264) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid encoding", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;

	self->output_format =
		(base->config.preferred_output_format == VDEF_BCM_MMAL_OPAQUE)
			? VDEF_BCM_MMAL_OPAQUE
			: VDEF_I420;

	/* Initialize the VideoCore */
	status = mmal_vc_init();
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_vc_init mmal_status=%d", -ret, status);
		goto error;
	}

	self->mmal_out_queue = mmal_queue_create();
	if (self->mmal_out_queue == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("mmal_queue_create", -ret);
		goto error;
	}

	/* Create the video decoder component */
	status = mmal_component_create(MMAL_COMPONENT_DEFAULT_VIDEO_DECODER,
				       &self->decoder);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO(
			"mmal_component_create mmal_status=%d", -ret, status);
		goto error;
	}

	ULOGI("Broadcom VideoCore MMAL API version=%d.%d - "
	      "using component '%s'",
	      MMAL_VERSION_MAJOR,
	      MMAL_VERSION_MINOR,
	      MMAL_COMPONENT_DEFAULT_VIDEO_DECODER);

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

	/* Create the ouput buffers queue */
	ret = vbuf_queue_new(0, 0, &self->out_queue);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_new:output", -ret);
		goto error;
	}
	self->out_queue_evt = vbuf_queue_get_evt(self->out_queue);
	if (self->out_queue_evt == NULL) {
		ret = -ENODEV;
		ULOG_ERRNO("vbuf_queue_get_evt", -ret);
		goto error;
	}
	ret = pomp_evt_attach_to_loop(
		self->out_queue_evt, base->loop, &out_queue_evt_cb, self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	if (self->base->config.encoding == VDEF_ENCODING_H264)
		self->need_sync = 1;

	/* Create the decoder thread */
	ret = pthread_create(&self->thread,
			     NULL,
			     vdec_videocoremmal_decoder_thread,
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
	destroy(base);
	base->derived = NULL;
	return ret;
}


static int flush(struct vdec_decoder *base, int discard)
{
	struct vdec_videocoremmal *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	self->flush = 1;
	self->flush_discard = discard;

	return 0;
}


static int set_h264_ps(struct vdec_decoder *base,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format)
{
	int ret = 0, err;
	struct vdec_videocoremmal *self;
	size_t ps_size, off;
	struct vbuf_cbs buf_cbs;
	MMAL_STATUS_T status;
	MMAL_ES_FORMAT_T *format_in = NULL;
	MMAL_ES_FORMAT_T *format_out = NULL;
	unsigned int output_size;

	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	size_t offset = (format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU)
				? 0
				: 4;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);

	self = base->derived;

	self->stride = base->video_info.resolution.width;
	output_size = VDEF_ALIGN(base->video_info.resolution.width, 32) *
		      VDEF_ALIGN(base->video_info.resolution.height, 16) * 3 /
		      2;

	/* Set the format of the video decoder input port */
	format_in = self->decoder->input[0]->format;
	format_in->type = MMAL_ES_TYPE_VIDEO;
	format_in->encoding = MMAL_ENCODING_H264;
	format_in->es->video.width =
		VDEF_ALIGN(base->video_info.resolution.width, 32)
			format_in->es->video.height =
			VDEF_ALIGN(base->video_info.resolution.height, 16)
				format_in->es->video.frame_rate.num = 30;
	format_in->es->video.frame_rate.den = 1;
	format_in->es->video.par.num = base->video_info.sar.width;
	format_in->es->video.par.den = base->video_info.sar.height;
	format_in->flags = MMAL_ES_FORMAT_FLAG_FRAMED;

	/* Alloc the SPS/PPS buffer for the input format */
	ps_size = 4 + sps_size - offset + 4 + pps_size - offset;
	status = mmal_format_extradata_alloc(format_in, ps_size);
	if (status != MMAL_SUCCESS) {
		ret = -ENOMEM;
		ULOG_ERRNO("mmal_format_extradata_alloc mmal_status=%d",
			   -ret,
			   status);
		goto error;
	}
	format_in->extradata_size = ps_size;
	off = 0;

	/* SPS */
	*(uint32_t *)(format_in->extradata + off) = htonl(0x00000001);
	off += 4;
	memcpy(format_in->extradata + off, sps + offset, sps_size - offset);
	off += sps_size - offset;

	/* PPS */
	*(uint32_t *)(format_in->extradata + off) = htonl(0x00000001);
	off += 4;
	memcpy(format_in->extradata + off, pps + offset, pps_size - offset);
	off += pps_size - offset;

	status = mmal_port_format_commit(self->decoder->input[0]);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_port_format_commit:input mmal_status=%d",
			   -ret,
			   status);
		goto error;
	}

	/* Set the format of the video decoder output port */
	format_out = self->decoder->output[0]->format;
	if (self->output_format == VDEF_BCM_MMAL_OPAQUE) {
		format_out->encoding = MMAL_ENCODING_OPAQUE;
		status = mmal_port_parameter_set_boolean(
			self->decoder->output[0],
			MMAL_PARAMETER_ZERO_COPY,
			MMAL_TRUE);
		if (status != MMAL_SUCCESS) {
			ret = -EPROTO;
			ULOG_ERRNO(
				"mmal_port_parameter_set:ZERO_COPY "
				"mmal_status=%d",
				-ret,
				status);
			goto error;
		}
		status = mmal_port_parameter_set_uint32(
			self->decoder->output[0],
			MMAL_PARAMETER_EXTRA_BUFFERS,
			VDEC_VIDEOCOREMMAL_OUT_NUM_EXTRA_BUFFERS);
		if (status != MMAL_SUCCESS) {
			ret = -EPROTO;
			ULOG_ERRNO(
				"mmal_port_parameter_set:EXTRA_BUFFERS "
				"mmal_status=%d",
				-ret,
				status);
			goto error;
		}
	} else {
		format_out->encoding = MMAL_ENCODING_I420;
		format_out->encoding_variant = MMAL_ENCODING_I420;
	}
	status = mmal_port_format_commit(self->decoder->output[0]);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_port_format_commit:output mmal_status=%d",
			   -ret,
			   status);
		goto error;
	}

	/* Create MMAL input pool */
	self->decoder->input[0]->buffer_num =
		self->decoder->input[0]->buffer_num_min;
	if (base->config.preferred_min_in_buf_count >
	    self->decoder->input[0]->buffer_num)
		self->decoder->input[0]->buffer_num =
			base->config.preferred_min_in_buf_count;
	if (self->decoder->input[0]->buffer_num_recommended >
	    self->decoder->input[0]->buffer_num)
		self->decoder->input[0]->buffer_num =
			self->decoder->input[0]->buffer_num_recommended;
	self->decoder->input[0]->buffer_size =
		base->video_info.resolution.width *
		base->video_info.resolution.height * 3 / 4;
	if (self->decoder->input[0]->buffer_size_min >
	    self->decoder->input[0]->buffer_size)
		self->decoder->input[0]->buffer_size =
			self->decoder->input[0]->buffer_size_min;
	if (self->decoder->input[0]->buffer_size_recommended >
	    self->decoder->input[0]->buffer_size)
		self->decoder->input[0]->buffer_size =
			self->decoder->input[0]->buffer_size_recommended;
	ULOGI("input buffers num=%d size=%d",
	      self->decoder->input[0]->buffer_num,
	      self->decoder->input[0]->buffer_size);
	self->mmal_in_pool =
		mmal_pool_create(self->decoder->input[0]->buffer_num,
				 self->decoder->input[0]->buffer_size);
	if (self->mmal_in_pool == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("mmal_pool_create:input", -ret);
		goto error;
	}
	memset(&buf_cbs, 0, sizeof(buf_cbs));
	buf_cbs.alloc = &vdec_videocoremmal_mmalbuf_alloc_cb;
	buf_cbs.free = &vdec_videocoremmal_mmalbuf_free_cb;
	buf_cbs.unref = &vdec_videocoremmal_mmalbuf_unref_cb;
	buf_cbs.pool_get = &vdec_videocoremmal_mmalbuf_pool_get_cb;
	buf_cbs.pool_get_userdata = (void *)self->mmal_in_pool->queue;
	ret = vbuf_pool_new(self->decoder->input[0]->buffer_num,
			    0,
			    0,
			    &buf_cbs,
			    &self->in_pool);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_pool_new:input", -ret);
		goto error;
	}

	/* Create MMAL output pool and vbuf pool */
	self->decoder->output[0]->buffer_num =
		VDEC_VIDEOCOREMMAL_OUT_BUFFERS_COUNT;
	if (base->config.preferred_min_out_buf_count >
	    self->decoder->output[0]->buffer_num)
		self->decoder->output[0]->buffer_num =
			base->config.preferred_min_out_buf_count;
	if (self->decoder->output[0]->buffer_num_min >
	    self->decoder->output[0]->buffer_num)
		self->decoder->output[0]->buffer_num =
			self->decoder->output[0]->buffer_num_min;
	self->decoder->output[0]->buffer_size = output_size;
	if (self->decoder->output[0]->buffer_size_min >
	    self->decoder->output[0]->buffer_size)
		self->decoder->output[0]->buffer_size =
			self->decoder->output[0]->buffer_size_min;
	ULOGI("output buffers num=%d size=%d",
	      self->decoder->output[0]->buffer_num,
	      self->decoder->output[0]->buffer_size);
	if (self->output_format == VDEF_BCM_MMAL_OPAQUE) {
		self->mmal_out_pool = mmal_port_pool_create(
			self->decoder->output[0],
			self->decoder->output[0]->buffer_num,
			self->decoder->output[0]->buffer_size);
		if (self->mmal_out_pool == NULL) {
			ret = -ENOMEM;
			ULOG_ERRNO("mmal_port_pool_create:output", -ret);
			goto error;
		}
	} else {
		self->mmal_out_pool =
			mmal_pool_create(self->decoder->output[0]->buffer_num,
					 self->decoder->output[0]->buffer_size);
		if (self->mmal_out_pool == NULL) {
			ret = -ENOMEM;
			ULOG_ERRNO("mmal_pool_create:output", -ret);
			goto error;
		}
	}
	memset(&buf_cbs, 0, sizeof(buf_cbs));
	buf_cbs.alloc = &vdec_videocoremmal_mmalbuf_alloc_cb;
	buf_cbs.free = &vdec_videocoremmal_mmalbuf_free_cb;
	buf_cbs.unref = &vdec_videocoremmal_mmalbuf_unref_cb;
	buf_cbs.pool_get = &vdec_videocoremmal_mmalbuf_pool_get_cb;
	buf_cbs.pool_get_userdata = NULL;
	ret = vbuf_pool_new(self->decoder->output[0]->buffer_num,
			    0,
			    0,
			    &buf_cbs,
			    &self->out_pool);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_pool_new:output", -ret);
		goto error;
	}

	/* Enable the input/output ports and the component */
	self->decoder->input[0]->userdata = (void *)self;
	self->decoder->output[0]->userdata = (void *)self;
	self->decoder->control->userdata = (void *)self;
	status = mmal_port_enable(self->decoder->control,
				  &vdec_videocoremmal_control_cb);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_port_enable mmal_status=%d", -ret, status);
		goto error;
	}
	status = mmal_port_enable(self->decoder->input[0],
				  &vdec_videocoremmal_input_cb);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_port_enable mmal_status=%d", -ret, status);
		goto error;
	}
	status = mmal_port_enable(self->decoder->output[0],
				  &vdec_videocoremmal_output_cb);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO("mmal_port_enable mmal_status=%d", -ret, status);
		goto error;
	}
	status = mmal_component_enable(self->decoder);
	if (status != MMAL_SUCCESS) {
		ret = -EPROTO;
		ULOG_ERRNO(
			"mmal_component_enable mmal_status=%d", -ret, status);
		goto error;
	}

	/* Return all output buffers to the output port */
	ret = vdec_videocoremmal_return_output_buffers(self);
	if (ret < 0)
		goto error;

	return 0;

error:
	if (self->in_pool != NULL) {
		err = vbuf_pool_destroy(self->in_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_destroy:input", -err);
		self->in_pool = NULL;
	}
	if (self->out_pool != NULL) {
		err = vbuf_pool_destroy(self->out_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_destroy:output", -err);
		self->out_pool = NULL;
	}
	if (self->mmal_in_pool != NULL) {
		mmal_pool_destroy(self->mmal_in_pool);
		self->mmal_in_pool = NULL;
	}
	if (self->mmal_out_pool != NULL) {
		if (self->output_format == VDEF_BCM_MMAL_OPAQUE) {
			mmal_port_pool_destroy(self->decoder->output[0],
					       self->mmal_out_pool);
		} else {
			mmal_pool_destroy(self->mmal_out_pool);
		}
		self->mmal_out_pool = NULL;
	}
	return ret;
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
	return -ENOSYS;
}


static struct vbuf_pool *get_input_buffer_pool(struct vdec_decoder *base)
{
	struct vdec_videocoremmal *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = base->derived;

	return self->in_pool;
}


static struct vbuf_queue *get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_videocoremmal *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = base->derived;

	return self->in_queue;
}

const struct vdec_ops vdec_videocoremmal_ops = {
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

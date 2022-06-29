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

#define ULOG_TAG vdec_turbojpeg
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

#include "vdec_turbojpeg_priv.h"


#define NB_SUPPORTED_FORMATS 1
#define DEFAULT_OUT_BUF_COUNT 10

static struct vdef_coded_format supported_formats[NB_SUPPORTED_FORMATS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;

static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_jpeg_jfif;
}

static void flush_complete(struct vdec_turbojpeg *self)
{
	/* Call the flush callback if defined */
	vdec_call_flush_cb(self->base);
}

static void stop_complete(struct vdec_turbojpeg *self)
{
	/* Call the stop callback if defined */
	vdec_call_stop_cb(self->base);
}

static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct vdec_turbojpeg *self = userdata;
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

static void dec_error_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct vdec_turbojpeg *self = userdata;
	int status = atomic_exchange(&self->status, 0);

	vdec_call_frame_output_cb(self->base, status, NULL);
}


static void dec_out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct vdec_turbojpeg *self = userdata;
	struct mbuf_raw_video_frame *out_frame = NULL;
	int ret;
	do {
		ret = mbuf_raw_video_frame_queue_pop(self->dec_out_queue,
						     &out_frame);
		if (ret == -EAGAIN) {
			return;
		} else if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop:dec_out",
				   -ret);
			return;
		}
		vdec_call_frame_output_cb(self->base, 0, out_frame);
		mbuf_raw_video_frame_unref(out_frame);
	} while (ret == 0);
}

static int decode_frame(struct vdec_turbojpeg *self,
			struct mbuf_coded_video_frame *in_frame)
{
	int ret;
	size_t len;
	struct vdef_coded_frame in_info;
	unsigned char *plane_data[VDEF_RAW_MAX_PLANE_COUNT] = {0};
	size_t plane_size[VDEF_RAW_MAX_PLANE_COUNT] = {0};
	const void *in_data;
	struct mbuf_raw_video_frame *out_frame = NULL;
	struct vdef_raw_frame out_info = {0};
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	struct mbuf_mem *out_mem = NULL;
	struct vmeta_frame *metadata = NULL;
	size_t offset = 0;
	void *mem_data = NULL;
	size_t mem_data_size = 0;
	unsigned int i;

	if (!in_frame)
		return 0;

	ret = mbuf_coded_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -ret);
		return ret;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	if (!vdef_coded_format_intersect(
		    &in_info.format, supported_formats, NB_SUPPORTED_FORMATS)) {
		ret = -ENOSYS;
		ULOG_ERRNO(
			"unsupported format:"
			" " VDEF_CODED_FORMAT_TO_STR_FMT,
			-ret,
			VDEF_CODED_FORMAT_TO_STR_ARG(&in_info.format));
		return ret;
	}

	ret = mbuf_coded_video_frame_get_packed_buffer(
		in_frame, &in_data, &len);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer", -ret);
		goto out;
	}

	/* Get a mem from the pool */
	ret = mbuf_pool_get(self->out_pool, &out_mem);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_pool_get", -ret);
		goto out;
	}

	ret = mbuf_mem_get_data(out_mem, &mem_data, &mem_data_size);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}

	offset = 0;
	/* Compute frame size depending on format */
	ret = vdef_calc_raw_frame_size(&self->output_format,
				       &self->base->video_info.resolution,
				       NULL,
				       NULL,
				       NULL,
				       NULL,
				       &(plane_size[0]),
				       NULL);
	if (ret < 0) {
		ULOG_ERRNO("vdef_calc_raw_frame_size", -ret);
		goto out;
	}

	for (i = 0; i < vdef_get_raw_frame_plane_count(&self->output_format);
	     i++) {
		plane_data[i] = (unsigned char *)mem_data + offset;
		offset += plane_size[i];
	}

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		in_frame,
		VDEC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -ret);

	/* TODO check for tjDecompressToYUVPlanes flags */
	ret = tjDecompressToYUVPlanes(self->tj_handler,
				      in_data,
				      len,
				      (unsigned char **)&plane_data,
				      self->base->video_info.resolution.width,
				      NULL,
				      self->base->video_info.resolution.height,
				      0);
	if (ret < 0) {
		ULOGE("%s", tjGetErrorStr());
		goto out;
	}

	ret = mbuf_coded_video_frame_release_packed_buffer(in_frame, in_data);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_release_packed_buffer",
			   -ret);
	}
	in_data = NULL;

	out_info.info = in_info.info;
	out_info.format = self->output_format;
	if (vdef_raw_format_cmp(&out_info.format, &vdef_i420)) {
		unsigned int stride = self->base->video_info.resolution.width;
		out_info.plane_stride[0] = stride;
		out_info.plane_stride[1] = stride / 2;
		out_info.plane_stride[2] = stride / 2;
	} else {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output chroma format", -ret);
		goto out;
	}
	out_info.info.bit_depth = self->base->video_info.bit_depth;
	out_info.info.full_range = self->base->video_info.full_range;
	out_info.info.color_primaries = self->base->video_info.color_primaries;
	out_info.info.transfer_function =
		self->base->video_info.transfer_function;
	out_info.info.matrix_coefs = self->base->video_info.matrix_coefs;
	out_info.info.resolution.width =
		self->base->video_info.resolution.width;
	out_info.info.resolution.height =
		self->base->video_info.resolution.height;
	out_info.info.sar = self->base->video_info.sar;

	/* Raw frame creation */
	ret = mbuf_raw_video_frame_new(&out_info, &out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto out;
	}

	/* Set planes */
	offset = 0;
	for (unsigned int i = 0;
	     i < vdef_get_raw_frame_plane_count(&self->output_format);
	     i++) {
		ret = mbuf_raw_video_frame_set_plane(
			out_frame, i, out_mem, offset, plane_size[i]);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane", -ret);
			goto out;
		}
		offset += plane_size[i];
	}

	/* Frame ancillary data */
	ret = mbuf_coded_video_frame_foreach_ancillary_data(
		in_frame,
		mbuf_raw_video_frame_ancillary_data_copier,
		out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_foreach_ancillary_data",
			   -ret);
		goto out;
	}

	/* Frame metadata */
	ret = mbuf_coded_video_frame_get_metadata(in_frame, &metadata);
	if (ret == 0) {
		ret = mbuf_raw_video_frame_set_metadata(out_frame, metadata);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_set_metadata", -ret);
			goto out;
		}
	} else if ((ret < 0) && (ret != -ENOENT)) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_metadata", -ret);
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		out_frame,
		VDEC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer", -ret);

	/* Output the frame */
	ret = mbuf_raw_video_frame_finalize(out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);
		goto out;
	}

	ret = mbuf_raw_video_frame_queue_push(self->dec_out_queue, out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_push", -ret);
		goto out;
	}

out:
	if (out_frame)
		mbuf_raw_video_frame_unref(out_frame);

	if (out_mem)
		mbuf_mem_unref(out_mem);

	if (in_data) {
		ret = mbuf_coded_video_frame_release_packed_buffer(in_frame,
								   in_data);
		if (ret < 0)
			ULOG_ERRNO(
				"mbuf_coded_video_frame_release_packed_buffer",
				-ret);
	}

	if (metadata)
		vmeta_frame_unref(metadata);

	if (ret < 0) {
		/* Notify error */
		atomic_store(&self->status, ret);
		pomp_evt_signal(self->dec_error_evt);
	}
	return ret;
}

static int complete_flush(struct vdec_turbojpeg *self)
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

		/* Flush the decoder output queue */
		ret = mbuf_raw_video_frame_queue_flush(self->dec_out_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_flush:dec_out",
				   -ret);
			return ret;
		}
	}

	atomic_store(&self->flushing, false);
	atomic_store(&self->flush_discard, false);

	/* Call the flush callback on the loop */
	char message = VDEC_MSG_FLUSH;
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);

	return ret;
}

static void check_input_queue(struct vdec_turbojpeg *self)
{
	int ret, err = 0;
	struct mbuf_coded_video_frame *in_frame;

	ret = mbuf_coded_video_frame_queue_peek(self->in_queue, &in_frame);
	while (ret >= 0) {
		/* Push the input frame */
		/* Decode the frame */
		ret = decode_frame(self, in_frame);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("decode_frame", -ret);
			err = -ENOSPC;
		}
		if (in_frame) {
			mbuf_coded_video_frame_unref(in_frame);
			/* Pop the frame for real */
			ret = mbuf_coded_video_frame_queue_pop(self->in_queue,
							       &in_frame);
			if (ret < 0) {
				ULOG_ERRNO("mbuf_coded_video_frame_queue_pop",
					   -ret);
				break;
			}
			mbuf_coded_video_frame_unref(in_frame);
		}
		if (err)
			break;
		/* Peek the next frame */
		ret = mbuf_coded_video_frame_queue_peek(self->in_queue,
							&in_frame);
		if (ret < 0 && ret != -EAGAIN && ret != -ENOSPC)
			ULOG_ERRNO("mbuf_coded_video_frame_queue_peek", -ret);
		if (self->flushing && ret == -EAGAIN) {
			in_frame = NULL;
			if ((atomic_load(&self->flushing)) &&
			    (!atomic_load(&self->flush_discard))) {
				ret = complete_flush(self);
				if (ret < 0)
					ULOG_ERRNO("complete_flush", -ret);
				continue;

				/* Else we proceed to call decode_frame()
				 * without an input frame to flush the
				 * decoder */
			}
		}
	}

	if (self->flushing && ret == -EAGAIN) {
		if (((atomic_load(&self->flushing)) &&
		     (!atomic_load(&self->flush_discard)))) {
			ret = complete_flush(self);
			if (ret < 0)
				ULOG_ERRNO("complete_flush", -ret);
		}
	}
}

static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	struct vdec_turbojpeg *self = userdata;
	check_input_queue(self);
}


static void *decoder_thread(void *ptr)
{
	int ret, timeout;
	struct vdec_turbojpeg *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
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

	while ((!atomic_load(&self->should_stop)) ||
	       (atomic_load(&self->flushing))) {
		/* Start flush, discarding all frames */
		if ((atomic_load(&self->flushing)) &&
		    (atomic_load(&self->flush_discard))) {
			ret = complete_flush(self);
			if (ret < 0)
				ULOG_ERRNO("complete_flush", -ret);
			continue;
		}

		/* Wait for an input buffer (without dequeueing it) */
		timeout = ((atomic_load(&self->flushing)) &&
			   (!atomic_load(&self->flush_discard)))
				  ? 0
				  : 5;
		ret = pomp_loop_wait_and_process(loop, timeout);
		if (ret < 0 && ret != -ETIMEDOUT) {
			ULOG_ERRNO("pomp_loop_wait_and_process", -ret);
			if (!self->should_stop) {
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
	if (in_queue_evt != NULL && pomp_evt_is_attached(in_queue_evt, loop)) {
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

static int flush(struct vdec_decoder *base, int discard_)
{
	struct vdec_turbojpeg *self;
	bool discard = discard_;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	atomic_store(&self->flushing, true);
	atomic_store(&self->flush_discard, discard);

	return 0;
}

static int stop(struct vdec_decoder *base)
{
	struct vdec_turbojpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the decoding thread */
	atomic_store(&self->should_stop, true);
	return 0;
}

static int destroy(struct vdec_decoder *base)
{
	int err;
	struct vdec_turbojpeg *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the decoding thread */
	err = stop(base);
	if (err < 0)
		ULOG_ERRNO("stop", -err);
	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->dec_error_evt != NULL) {
		if (pomp_evt_is_attached(self->dec_error_evt, base->loop)) {
			err = pomp_evt_detach_from_loop(self->dec_error_evt,
							base->loop);
			if (err < 0)
				ULOG_ERRNO("pomp_evt_detach_from_loop", -err);
		}

		pomp_evt_destroy(self->dec_error_evt);
		self->dec_error_evt = NULL;
	}
	if (self->in_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_queue_destroy:input",
				   -err);
	}
	if (self->dec_out_queue_evt != NULL &&
	    pomp_evt_is_attached(self->dec_out_queue_evt, base->loop)) {
		err = pomp_evt_detach_from_loop(self->dec_out_queue_evt,
						base->loop);
		if (err < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->dec_out_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->dec_out_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_queue_destroy:dec_out",
				   -err);
	}
	if (self->out_pool != NULL) {
		err = mbuf_pool_destroy(self->out_pool);
		if (err < 0)
			ULOG_ERRNO("mbuf_pool_destroy:dec_out", -err);
	}
	if (self->mbox) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			ULOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	if (self->tj_handler) {
		err = tjDestroy(self->tj_handler);
		if (err < 0)
			ULOGE("%s", tjGetErrorStr());
	}

	xfree((void **)&self);
	base->derived = NULL;
	return 0;
}

static bool input_filter(struct mbuf_coded_video_frame *frame, void *userdata)
{
	int ret;
	const void *tmp;
	size_t tmplen;
	struct vdec_turbojpeg *self = userdata;
	struct vdef_coded_frame info;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, false);

	if (atomic_load(&self->flushing))
		return false;

	ret = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (ret < 0)
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
	struct vdec_turbojpeg *self = NULL;
	struct mbuf_coded_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	(void)pthread_once(&supported_formats_is_init,
			   initialize_supported_formats);

	/* Check the configuration */
	if (base->config.encoding != VDEF_ENCODING_MJPEG) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid encoding: %s",
			   -ret,
			   vdef_encoding_to_str(base->config.encoding));
		return ret;
	}

	if (vdef_is_raw_format_valid(&base->config.preferred_output_format) &&
	    !vdef_raw_format_cmp(&base->config.preferred_output_format,
				 &vdef_i420)) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid preferred_output_format", -ret);
		goto error;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->base = base;
	base->derived = self;
	/* Output format is hardcoded as I420 */
	self->output_format = vdef_i420;
	self->thread_launched = false;

	queue_args.filter_userdata = self;

	ULOGI("TurboJPEG implementation");

	self->tj_handler = tjInitDecompress();
	if (!self->tj_handler) {
		ULOGE("%s", tjGetErrorStr());
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

	/* Create the input buffers queue */
	ret = mbuf_coded_video_frame_queue_new_with_args(&queue_args,
							 &self->in_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new_with_args:input",
			   -ret);
		goto error;
	}

	/* Create the decoder output buffers queue */
	ret = mbuf_raw_video_frame_queue_new(&self->dec_out_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_new:dec_out", -ret);
		goto error;
	}

	ret = mbuf_raw_video_frame_queue_get_event(self->dec_out_queue,
						   &self->dec_out_queue_evt);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -ret);
		goto error;
	}

	ret = pomp_evt_attach_to_loop(self->dec_out_queue_evt,
				      base->loop,
				      &dec_out_queue_evt_cb,
				      self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	self->dec_error_evt = pomp_evt_new();
	if (self->dec_error_evt == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("pomp_evt_new", -ret);
		goto error;
	}

	ret = pomp_evt_attach_to_loop(
		self->dec_error_evt, base->loop, &dec_error_evt_cb, self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	/* Create the decoder thread */
	ret = pthread_create(&self->thread, NULL, decoder_thread, (void *)self);
	if (ret != 0) {
		ret = -ret;
		ULOG_ERRNO("pthread_create", -ret);
		goto error;
	}

	self->thread_launched = true;

	return 0;

error:
	/* Cleanup on error */
	destroy(base);
	return ret;
}

static struct mbuf_pool *get_input_buffer_pool(struct vdec_decoder *base)
{
	/* No input buffer pool allocated: use the application's */
	return NULL;
}

static struct mbuf_coded_video_frame_queue *
get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_turbojpeg *self;

	self = (struct vdec_turbojpeg *)base->derived;

	return self->in_queue;
}

static int set_jpeg_params(struct vdec_decoder *base)
{
	int ret;
	struct vdec_turbojpeg *self;
	size_t pool_count = DEFAULT_OUT_BUF_COUNT;
	ssize_t pool_size;
	size_t plane_size[VDEF_RAW_MAX_PLANE_COUNT] = {0};

	self = (struct vdec_turbojpeg *)base->derived;

	/* Use preferred_min_out_buf_count if greater than default pool count */
	if (base->config.preferred_min_out_buf_count != 0 &&
	    base->config.preferred_min_out_buf_count > pool_count)
		pool_count = base->config.preferred_min_out_buf_count;

	/* Compute frame size depending on format */
	ret = vdef_calc_raw_frame_size(&self->output_format,
				       &base->video_info.resolution,
				       NULL,
				       NULL,
				       NULL,
				       NULL,
				       &(plane_size[0]),
				       NULL);
	if (ret < 0) {
		ULOG_ERRNO("vdef_calc_raw_frame_size", -ret);
		return ret;
	}

	pool_size = 0;
	for (unsigned int i = 0;
	     i < vdef_get_raw_frame_plane_count(&self->output_format);
	     i++) {
		pool_size += plane_size[i];
	}

	/* Create pool for raw output frames */
	ret = mbuf_pool_new(mbuf_mem_generic_impl,
			    pool_size,
			    pool_count,
			    MBUF_POOL_NO_GROW,
			    0,
			    "vdec_turbojpeg",
			    &self->out_pool);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_pool_new", -ret);
		return ret;
	}

	return 0;
}

const struct vdec_ops vdec_turbojpeg_ops = {
	.get_supported_input_formats = get_supported_input_formats,
	.create = create,
	.flush = flush,
	.stop = stop,
	.destroy = destroy,
	.get_input_buffer_pool = get_input_buffer_pool,
	.get_input_buffer_queue = get_input_buffer_queue,
	.set_jpeg_params = set_jpeg_params,
};

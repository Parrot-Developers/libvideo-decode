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

#define ULOG_TAG vdec
#include "vdec_priv.h"
ULOG_DECLARE_TAG(vdec);


#define VDEC_OPS_NULL                                                          \
	{                                                                      \
		.get_supported_input_format = NULL, .new = NULL,               \
		.flush = NULL, .stop = NULL, .destroy = NULL,                  \
		.set_sps_pps = NULL, .get_input_buffer_pool = NULL,            \
		.get_input_buffer_queue = NULL, .sync_decode = NULL,           \
	}


static const struct vdec_ops vdec_ops[] = {
	[VDEC_DECODER_IMPLEM_AUTO] = VDEC_OPS_NULL,

#ifdef BUILD_FFMPEG_LIBAV
	[VDEC_DECODER_IMPLEM_FFMPEG] = VDEC_OPS_FFMPEG,
#else /* BUILD_FFMPEG_LIBAV */
	[VDEC_DECODER_IMPLEM_FFMPEG] = VDEC_OPS_NULL,
#endif /* BUILD_FFMPEG_LIBAV */

#ifdef BUILD_LIBMEDIACODEC_WRAPPER
	[VDEC_DECODER_IMPLEM_MEDIACODEC] = VDEC_OPS_MEDIACODEC,
#else /* BUILD_LIBMEDIACODEC_WRAPPER */
	[VDEC_DECODER_IMPLEM_MEDIACODEC] = VDEC_OPS_NULL,
#endif /* BUILD_LIBMEDIACODEC_WRAPPER */

#ifdef USE_VIDEOTOOLBOX
	[VDEC_DECODER_IMPLEM_VIDEOTOOLBOX] = VDEC_OPS_VIDEOTOOLBOX,
#else /* USE_VIDEOTOOLBOX */
	[VDEC_DECODER_IMPLEM_VIDEOTOOLBOX] = VDEC_OPS_NULL,
#endif /* USE_VIDEOTOOLBOX */

#ifdef BUILD_MMAL
	[VDEC_DECODER_IMPLEM_VIDEOCOREMMAL] = VDEC_OPS_VIDEOCOREMMAL,
#else /* BUILD_MMAL */
	[VDEC_DECODER_IMPLEM_VIDEOCOREMMAL] = VDEC_OPS_NULL,
#endif /* BUILD_MMAL */
};


static int vdec_get_implem(enum vdec_decoder_implem *implem)
{
	int ret = -ENOSYS;

	ULOG_ERRNO_RETURN_ERR_IF(implem == NULL, EINVAL);

#ifdef BUILD_MMAL
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_VIDEOCOREMMAL)) {
		*implem = VDEC_DECODER_IMPLEM_VIDEOCOREMMAL;
		ret = 0;
	}
#endif /* BUILD_MMAL */

#ifdef USE_VIDEOTOOLBOX
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_VIDEOTOOLBOX)) {
		*implem = VDEC_DECODER_IMPLEM_VIDEOTOOLBOX;
		ret = 0;
	}
#endif /* USE_VIDEOTOOLBOX */

#ifdef BUILD_LIBMEDIACODEC_WRAPPER
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_MEDIACODEC)) {
		*implem = VDEC_DECODER_IMPLEM_MEDIACODEC;
		ret = 0;
	}
#endif /* BUILD_LIBMEDIACODEC_WRAPPER */

#ifdef BUILD_FFMPEG_LIBAV
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_FFMPEG)) {
		*implem = VDEC_DECODER_IMPLEM_FFMPEG;
		ret = 0;
	}
#endif /* BUILD_FFMPEG_LIBAV */

	return ret;
}


uint32_t vdec_get_supported_input_format(enum vdec_decoder_implem implem)
{
	int ret;

	ret = vdec_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, 0);

	if (vdec_ops[implem].get_supported_input_format) {
		return (*vdec_ops[implem].get_supported_input_format)();
	} else {
		ULOG_ERRNO("get_supported_input_format", ENOSYS);
		return 0;
	}
}


int vdec_new(const struct vdec_config *config,
	     const struct vdec_cbs *cbs,
	     void *userdata,
	     struct vdec_decoder **ret_obj)
{
	int ret, dbgret;
	struct vdec_decoder *self = NULL;
	const char *env_dbg_dir;
	const char *env_dbg_flags;

	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->frame_output == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	env_dbg_dir = getenv("VDEC_DBG_DIR");
	env_dbg_flags = getenv("VDEC_DBG_FLAGS");

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->cbs = *cbs;
	self->userdata = userdata;
	self->config = *config;
	self->config.name = xstrdup(config->name);

	/* Override debug config with environment if set */
	if (env_dbg_dir != NULL)
		self->config.dbg_dir = strdup(env_dbg_dir);
	else
		self->config.dbg_dir = xstrdup(config->dbg_dir);
	if (env_dbg_flags != NULL)
		self->config.dbg_flags = strtol(env_dbg_flags, NULL, 0);

	ret = vdec_get_implem(&self->config.implem);
	if (ret < 0)
		goto error;

	self->sar_width = 1;
	self->sar_height = 1;
	self->ops = vdec_ops[self->config.implem];

	if ((self->ops.new == NULL) || (self->ops.destroy == NULL)) {
		ret = -EPROTO;
		goto error;
	}

	ret = (*self->ops.new)(self);
	if (ret < 0)
		goto error;

	if (self->config.dbg_dir != NULL) {
		dbgret = vdec_dbg_create_files(self);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_create_files", -dbgret);
	}

	*ret_obj = self;
	return 0;

error:
	vdec_destroy(self);
	*ret_obj = NULL;
	return ret;
}


int vdec_flush(struct vdec_decoder *self, int discard)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->ops.flush)
		return (*self->ops.flush)(self, discard);
	else
		return -ENOSYS;
}


int vdec_stop(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->ops.stop)
		return (*self->ops.stop)(self);
	else
		return -ENOSYS;
}


int vdec_destroy(struct vdec_decoder *self)
{
	int ret = 0, dbgret;

	if (self == NULL)
		return 0;

	if (self->ops.destroy)
		ret = (*self->ops.destroy)(self);

	if (ret == 0) {
		dbgret = vdec_dbg_close_files(self);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_close_files", -dbgret);
		xfree((void **)&self->config.name);
		xfree((void **)&self->config.dbg_dir);
		free(self);
	}

	return ret;
}


int vdec_set_sps_pps(struct vdec_decoder *self,
		     const uint8_t *sps,
		     size_t sps_size,
		     const uint8_t *pps,
		     size_t pps_size,
		     enum vdec_input_format format)
{
	int ret;
	size_t offset = (format == VDEC_INPUT_FORMAT_RAW_NALU) ? 0 : 4;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);

	/* Get the video dimensions */
	ret = vdec_h264_get_video_info(
		self, sps, sps_size, pps, pps_size, format);
	if (ret < 0)
		return ret;

	/* Log video info */
	ULOGI("dimensions: width=%u height=%u SAR=%u:%u",
	      self->width,
	      self->height,
	      self->sar_width,
	      self->sar_height);
	ULOGI("crop: left=%u top=%u width=%u height=%u",
	      self->crop_left,
	      self->crop_top,
	      self->crop_width,
	      self->crop_height);
	if ((self->time_scale != 0) && (self->num_units_in_tick != 0)) {
		/* Note: a tick corresponds to a field, not a frame,
		 * thus the factor 2 */
		ULOGI("declared framerate: %u/%u -> %.3f fps",
		      (self->time_scale % 2) ? self->time_scale
					     : self->time_scale / 2,
		      (self->time_scale % 2) ? self->num_units_in_tick * 2
					     : self->num_units_in_tick,
		      (float)self->time_scale / 2. / self->num_units_in_tick);
	}
	if ((self->nal_hrd_bitrate != 0) && (self->nal_hrd_cpb_size != 0)) {
		ULOGI("declared NAL bitrate: %u bit/s (CPB size %u bits)",
		      self->nal_hrd_bitrate,
		      self->nal_hrd_cpb_size);
	}
	if ((self->vcl_hrd_bitrate != 0) && (self->vcl_hrd_cpb_size != 0)) {
		ULOGI("declared VCL bitrate: %u bit/s (CPB size %u bits)",
		      self->vcl_hrd_bitrate,
		      self->vcl_hrd_cpb_size);
	}

	if (self->ops.set_sps_pps) {
		ret = (*self->ops.set_sps_pps)(
			self, sps, sps_size, pps, pps_size, format);
	} else {
		ret = -ENOSYS;
	}

	self->configured = (ret == 0) ? 1 : 0;

	/* Debug files */
	if (self->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_h264_sps_pps(self->dbg.input_bs,
							 sps,
							 sps_size,
							 pps,
							 pps_size,
							 format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_h264_sps_pps", -dbgret);
	}
	if (self->dbg.h264_analysis != NULL) {
		int dbgret = vdec_dbg_parse_h264_sps_pps(
			self, sps, sps_size, pps, pps_size, format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_h264_sps_pps", -dbgret);
	}

	return ret;
}


struct vbuf_pool *vdec_get_input_buffer_pool(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	if (self->ops.get_input_buffer_pool)
		return (*self->ops.get_input_buffer_pool)(self);
	else
		return NULL;
}


struct vbuf_queue *vdec_get_input_buffer_queue(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	if (self->ops.get_input_buffer_queue)
		return (*self->ops.get_input_buffer_queue)(self);
	else
		return NULL;
}


int vdec_get_video_dimensions(struct vdec_decoder *self,
			      unsigned int *width,
			      unsigned int *height,
			      unsigned int *sar_width,
			      unsigned int *sar_height,
			      unsigned int *crop_left,
			      unsigned int *crop_top,
			      unsigned int *crop_width,
			      unsigned int *crop_height)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (!self->configured) {
		ULOG_ERRNO("decoder is not configured", EAGAIN);
		return -EAGAIN;
	}

	if (width != NULL)
		*width = self->width;
	if (height != NULL)
		*height = self->height;
	if (sar_width != NULL)
		*sar_width = self->sar_width;
	if (sar_height != NULL)
		*sar_height = self->sar_height;
	if (crop_left != NULL)
		*crop_left = self->crop_left;
	if (crop_top != NULL)
		*crop_top = self->crop_top;
	if (crop_width != NULL)
		*crop_width = self->crop_width;
	if (crop_height != NULL)
		*crop_height = self->crop_height;

	return 0;
}


int vdec_sync_decode(struct vdec_decoder *self,
		     struct vbuf_buffer *in_buf,
		     struct vbuf_buffer **out_buf)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(in_buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(out_buf == NULL, EINVAL);

	if (self->ops.sync_decode)
		return (*self->ops.sync_decode)(self, in_buf, out_buf);
	else
		return -ENOSYS;
}


enum vdec_decoder_implem vdec_get_used_implem(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == NULL, EINVAL, VDEC_DECODER_IMPLEM_AUTO);

	return self->config.implem;
}


const char *vdec_decoder_implem_str(enum vdec_decoder_implem implem)
{
	switch (implem) {
	case VDEC_DECODER_IMPLEM_FFMPEG:
		return "FFMPEG";
	case VDEC_DECODER_IMPLEM_MEDIACODEC:
		return "MEDIACODEC";
	case VDEC_DECODER_IMPLEM_VIDEOTOOLBOX:
		return "VIDEOTOOLBOX";
	case VDEC_DECODER_IMPLEM_VIDEOCOREMMAL:
		return "VIDEOCOREMMAL";
	default:
		return "UNKNOWN";
	}
}


const char *vdec_encoding_str(enum vdec_encoding enc)
{
	switch (enc) {
	case VDEC_ENCODING_H264:
		return "H264";
	default:
		return "UNKNOWN";
	}
}


const char *vdec_input_format_str(enum vdec_input_format fmt)
{
	switch (fmt) {
	case VDEC_INPUT_FORMAT_RAW_NALU:
		return "RAW_NALU";
	case VDEC_INPUT_FORMAT_BYTE_STREAM:
		return "BYTE_STREAM";
	case VDEC_INPUT_FORMAT_AVCC:
		return "AVCC";
	default:
		return "UNKNOWN";
	}
}


const char *vdec_output_format_str(enum vdec_output_format fmt)
{
	switch (fmt) {
	case VDEC_OUTPUT_FORMAT_I420:
		return "I420";
	case VDEC_OUTPUT_FORMAT_YV12:
		return "YV12";
	case VDEC_OUTPUT_FORMAT_NV12:
		return "NV12";
	case VDEC_OUTPUT_FORMAT_NV21:
		return "NV21";
	case VDEC_OUTPUT_FORMAT_NV12MT:
		return "NV12MT";
	case VDEC_OUTPUT_FORMAT_MMAL_OPAQUE:
		return "MMAL_OPAQUE";
	case VDEC_OUTPUT_FORMAT_UNKNOWN:
	default:
		return "UNKNOWN";
	}
}

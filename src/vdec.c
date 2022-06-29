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


static const struct vdec_ops *implem_ops(enum vdec_decoder_implem implem)
{
	switch (implem) {
#ifdef BUILD_LIBVIDEO_DECODE_FFMPEG
	case VDEC_DECODER_IMPLEM_FFMPEG:
		return &vdec_ffmpeg_ops;
#endif

#ifdef BUILD_LIBVIDEO_DECODE_MEDIACODEC
	case VDEC_DECODER_IMPLEM_MEDIACODEC:
		return &vdec_mediacodec_ops;
#endif
#ifdef BUILD_LIBVIDEO_DECODE_VIDEOCOREMMAL
	case VDEC_DECODER_IMPLEM_VIDEOCOREMMAL:
		return &vdec_videocoremmal_ops;
#endif
#ifdef BUILD_LIBVIDEO_DECODE_VIDEOTOOLBOX
	case VDEC_DECODER_IMPLEM_VIDEOTOOLBOX:
		return &vdec_videotoolbox_ops;
#endif
#ifdef BUILD_LIBVIDEO_DECODE_HISI
	case VDEC_DECODER_IMPLEM_HISI:
		return &vdec_hisi_ops;
#endif
#ifdef BUILD_LIBVIDEO_DECODE_AML
	case VDEC_DECODER_IMPLEM_AML:
		return &vdec_aml_ops;
#endif
#ifdef BUILD_LIBVIDEO_DECODE_TURBOJPEG
	case VDEC_DECODER_IMPLEM_TURBOJPEG:
		return &vdec_turbojpeg_ops;
#endif
	default:
		return NULL;
	}
}


static int vdec_get_implem(enum vdec_decoder_implem *implem)
{
	ULOG_ERRNO_RETURN_ERR_IF(implem == NULL, EINVAL);

#ifdef BUILD_LIBVIDEO_DECODE_AML
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_AML)) {
		*implem = VDEC_DECODER_IMPLEM_AML;
		return 0;
	}
#endif /* BUILD_LIBVIDEO_DECODE_AML */

#ifdef BUILD_LIBVIDEO_DECODE_HISI
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_HISI)) {
		*implem = VDEC_DECODER_IMPLEM_HISI;
		return 0;
	}
#endif /* BUILD_LIBVIDEO_DECODE_HISI */

#ifdef BUILD_LIBVIDEO_DECODE_VIDEOCOREMMAL
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_VIDEOCOREMMAL)) {
		*implem = VDEC_DECODER_IMPLEM_VIDEOCOREMMAL;
		return 0;
	}
#endif /* BUILD_LIBVIDEO_DECODE_VIDEOCOREMMAL */

#ifdef BUILD_LIBVIDEO_DECODE_VIDEOTOOLBOX
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_VIDEOTOOLBOX)) {
		*implem = VDEC_DECODER_IMPLEM_VIDEOTOOLBOX;
		return 0;
	}
#endif /* BUILD_LIBVIDEO_DECODE_VIDEOTOOLBOX */

#ifdef BUILD_LIBVIDEO_DECODE_MEDIACODEC
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_MEDIACODEC)) {
		*implem = VDEC_DECODER_IMPLEM_MEDIACODEC;
		return 0;
	}
#endif /* BUILD_LIBVIDEO_DECODE_MEDIACODEC */

#ifdef BUILD_LIBVIDEO_DECODE_FFMPEG
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_FFMPEG)) {
		*implem = VDEC_DECODER_IMPLEM_FFMPEG;
		return 0;
	}
#endif /* BUILD_LIBVIDEO_DECODE_FFMPEG */

#ifdef BUILD_LIBVIDEO_DECODE_TURBOJPEG
	if ((*implem == VDEC_DECODER_IMPLEM_AUTO) ||
	    (*implem == VDEC_DECODER_IMPLEM_TURBOJPEG)) {
		*implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
		return 0;
	}
#endif /* BUILD_LIBVIDEO_DECODE_TURBOJPEG */

	return -ENOSYS;
}


static void log_video_info(struct video_info *info, int level)
{
	/* Log photo/video info */
	ULOG_PRI(level,
		 "dimensions: width=%u height=%u SAR=%u:%u",
		 info->resolution.width,
		 info->resolution.height,
		 info->sar.width,
		 info->sar.height);
	ULOG_PRI(level,
		 "crop: left=%u top=%u width=%u height=%u",
		 info->crop.left,
		 info->crop.top,
		 info->crop.width,
		 info->crop.height);
	if ((info->framerate.num != 0) && (info->framerate.den != 0)) {
		ULOG_PRI(level,
			 "declared framerate: %u/%u -> %.3f fps",
			 info->framerate.num,
			 info->framerate.den,
			 (float)info->framerate.num /
				 (float)info->framerate.den);
	}
	ULOG_PRI(level,
		 "%d bits, color primaries: %s, transfer function: %s, "
		 "matrix coefficients: %s, full range: %d",
		 info->bit_depth,
		 vdef_color_primaries_to_str(info->color_primaries),
		 vdef_transfer_function_to_str(info->transfer_function),
		 vdef_matrix_coefs_to_str(info->matrix_coefs),
		 info->full_range);
	if ((info->nal_hrd_bitrate != 0) && (info->nal_hrd_cpb_size != 0)) {
		ULOG_PRI(level,
			 "declared NAL bitrate: "
			 "%u bit/s (CPB size %u bits)",
			 info->nal_hrd_bitrate,
			 info->nal_hrd_cpb_size);
	}
	if ((info->vcl_hrd_bitrate != 0) && (info->vcl_hrd_cpb_size != 0)) {
		ULOG_PRI(level,
			 "declared VCL bitrate: "
			 "%u bit/s (CPB size %u bits)",
			 info->vcl_hrd_bitrate,
			 info->vcl_hrd_cpb_size);
	}
}


int vdec_get_supported_input_formats(enum vdec_decoder_implem implem,
				     const struct vdef_coded_format **formats)
{
	int ret;
	ULOG_ERRNO_RETURN_ERR_IF(!formats, EINVAL);

	ret = vdec_get_implem(&implem);
	if (ret < 0) {
		ULOG_ERRNO("vdec_get_implem", -ret);
		return ret;
	}

	return implem_ops(implem)->get_supported_input_formats(formats);
}


enum vdec_decoder_implem vdec_get_auto_implem(void)
{
	int ret;
	enum vdec_decoder_implem implem = VDEC_DECODER_IMPLEM_AUTO;

	ret = vdec_get_implem(&implem);
	ULOG_ERRNO_RETURN_VAL_IF(ret < 0, -ret, VDEC_DECODER_IMPLEM_AUTO);

	return implem;
}


enum vdec_decoder_implem
vdec_get_auto_implem_by_coded_format(struct vdef_coded_format *format)
{
	int res = 0, count;
	const struct vdef_coded_format *supported_input_formats;

	ULOG_ERRNO_RETURN_VAL_IF(
		format == NULL, EINVAL, VDEC_DECODER_IMPLEM_AUTO);

	for (enum vdec_decoder_implem implem = VDEC_DECODER_IMPLEM_AUTO + 1;
	     implem < VDEC_DECODER_IMPLEM_MAX;
	     implem++) {

		res = vdec_get_implem(&implem);
		if (res < 0)
			continue;

		count = implem_ops(implem)->get_supported_input_formats(
			&supported_input_formats);
		if (count < 0)
			continue;

		if (!vdef_coded_format_intersect(
			    format, supported_input_formats, count))
			continue;

		return implem;
	}

	return VDEC_DECODER_IMPLEM_AUTO;
}


static void h264_nalu_begin_cb(struct h264_ctx *ctx,
			       enum h264_nalu_type type,
			       const uint8_t *buf,
			       size_t len,
			       const struct h264_nalu_header *nh,
			       void *userdata)
{
	struct vdec_decoder *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);

	vdec_dbg_h264_nalu_begin(self, *nh, type);
}


static void h264_slice_cb(struct h264_ctx *ctx,
			  const uint8_t *buf,
			  size_t len,
			  const struct h264_slice_header *sh,
			  void *userdata)
{
	struct vdec_decoder *self = userdata;
	struct h264_nalu_header nh;
	int res;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);
	ULOG_ERRNO_RETURN_IF(sh == NULL, EINVAL);

	res = h264_parse_nalu_header(buf, len, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_nalu_header", -res);
		return;
	}
	vdec_dbg_h264_slice(self, nh, sh);
}


static const struct h264_ctx_cbs h264_cbs = {
	.nalu_begin = h264_nalu_begin_cb,
	.slice = h264_slice_cb,
};


static void h265_nalu_begin_cb(struct h265_ctx *ctx,
			       enum h265_nalu_type type,
			       const uint8_t *buf,
			       size_t len,
			       const struct h265_nalu_header *nh,
			       void *userdata)
{
	struct vdec_decoder *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);
	ULOG_ERRNO_RETURN_IF(nh == NULL, EINVAL);

	vdec_dbg_h265_nalu_begin(self, type);
}


static const struct h265_ctx_cbs h265_cbs = {
	.nalu_begin = h265_nalu_begin_cb,
};


int vdec_new(struct pomp_loop *loop,
	     const struct vdec_config *config,
	     const struct vdec_cbs *cbs,
	     void *userdata,
	     struct vdec_decoder **ret_obj)
{
	int ret, dbgret;
	struct vdec_decoder *self = NULL;
	const char *env_dbg_dir;
	const char *env_dbg_flags;

	ULOG_ERRNO_RETURN_ERR_IF(loop == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(config == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(cbs->frame_output == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	env_dbg_dir = getenv("VDEC_DBG_DIR");
	env_dbg_flags = getenv("VDEC_DBG_FLAGS");

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;

	self->loop = loop;
	self->cbs = *cbs;
	self->userdata = userdata;
	self->config = *config;
	self->config.name = xstrdup(config->name);
	atomic_init(&self->last_timestamp, UINT64_MAX);

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

	self->video_info.sar.width = 1;
	self->video_info.sar.height = 1;
	self->ops = implem_ops(self->config.implem);

	ret = self->ops->create(self);
	if (ret < 0)
		goto error;

	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		ret = h264_reader_new(&h264_cbs, self, &self->reader.h264);
		if (ret < 0)
			ULOG_ERRNO("h264_reader_new", -ret);
		break;

	case VDEF_ENCODING_H265:
		ret = h265_reader_new(&h265_cbs, self, &self->reader.h265);
		if (ret < 0)
			ULOG_ERRNO("h265_reader_new", -ret);
		break;
	default:
		break;
	}

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

	return self->ops->flush(self, discard);
}


int vdec_stop(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	return self->ops->stop(self);
}


int vdec_destroy(struct vdec_decoder *self)
{
	int ret = 0, dbgret;

	if (self == NULL)
		return 0;

	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		if (self->reader.h264 != NULL) {
			ret = h264_reader_destroy(self->reader.h264);
			if (ret < 0)
				ULOG_ERRNO("h264_reader_destroy", -ret);
		}
		break;

	case VDEF_ENCODING_H265:
		if (self->reader.h265 != NULL) {
			ret = h265_reader_destroy(self->reader.h265);
			if (ret < 0)
				ULOG_ERRNO("h265_reader_destroy", -ret);
		}
		break;
	default:
		break;
	}


	ret = self->ops->destroy(self);

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


int vdec_set_jpeg_params(struct vdec_decoder *self,
			 const struct vdef_format_info *format_info)
{
	int ret = 0, count;
	const struct vdef_coded_format *supported_input_formats;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format_info == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((format_info->resolution.width == 0) ||
					 (format_info->resolution.height == 0),
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((format_info->framerate.num == 0) ||
					 (format_info->framerate.den == 0),
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->config.encoding != VDEF_ENCODING_MJPEG,
				 EINVAL);

	count = self->ops->get_supported_input_formats(
		&supported_input_formats);
	if (count < 0) {
		ULOG_ERRNO("get_supported_input_formats", -count);
		return -EINVAL;
	}

	if (!vdef_coded_format_intersect(
		    &vdef_jpeg_jfif, supported_input_formats, count)) {
		ULOG_ERRNO("unsupported input formats", EINVAL);
		return -EINVAL;
	}

	self->video_info = (const struct video_info){
		.resolution = {format_info->resolution.width,
			       format_info->resolution.height},
		.bit_depth = format_info->bit_depth,
		.sar = format_info->sar,
		.crop = {0,
			 0,
			 format_info->resolution.width,
			 format_info->resolution.height},
		.full_range = format_info->full_range,
		.color_primaries = format_info->color_primaries,
		.transfer_function = format_info->transfer_function,
		.matrix_coefs = format_info->matrix_coefs,
		.framerate.num = format_info->framerate.num,
		.framerate.den = format_info->framerate.den,
	};

	if ((self->video_info.bit_depth != 8) ||
	    (self->video_info.color_primaries != VDEF_COLOR_PRIMARIES_SRGB) ||
	    (self->video_info.transfer_function !=
	     VDEF_TRANSFER_FUNCTION_BT709) ||
	    (self->video_info.matrix_coefs != VDEF_MATRIX_COEFS_SRGB)) {
		ULOG_ERRNO("invalid input format", EINVAL);
		log_video_info(&self->video_info, ULOG_ERR);
		return -EINVAL;
	}

	log_video_info(&self->video_info, ULOG_INFO);

	if (!self->ops->set_jpeg_params) {
		self->configured = 1;
		return 0;
	}

	ret = self->ops->set_jpeg_params(self);
	self->configured = (ret == 0) ? 1 : 0;
	return ret;
}


int vdec_set_h264_ps(struct vdec_decoder *self,
		     const uint8_t *sps,
		     size_t sps_size,
		     const uint8_t *pps,
		     size_t pps_size,
		     const struct vdef_coded_format *format)
{
	int ret, count;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format->encoding != self->config.encoding,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format->encoding != VDEF_ENCODING_H264,
				 EINVAL);

	size_t offset = (format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU)
				? 0
				: 4;
	size_t size = 0, off = 0;
	uint8_t start_code[] = {0, 0, 0, 1};
	struct h264_info info = {0};
	struct h264_ctx *ctx = h264_reader_get_ctx(self->reader.h264);
	const struct vdef_coded_format *supported_input_formats;
	uint32_t sc;

	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->configured, EALREADY);

	/* SPS and PPS parsing */
	size = 4 + sps_size - offset + 4 + pps_size - offset;

	uint8_t *data = malloc(size);
	if (data == NULL)
		return -ENOMEM;

	memcpy(&data[off], start_code, sizeof(start_code));
	off += 4;
	memcpy(&data[off], &sps[offset], sps_size - offset);
	off += sps_size - offset;

	memcpy(&data[off], start_code, sizeof(start_code));
	off += 4;
	memcpy(&data[off], &pps[offset], pps_size - offset);

	ret = h264_reader_parse(self->reader.h264, 0, data, size, &off);
	if (ret < 0) {
		ULOG_ERRNO("h264_reader_parse", -ret);
		goto out;
	}

	/* Get the video info */
	ret = h264_ctx_get_info(ctx, &info);
	if (ret < 0) {
		ULOG_ERRNO("h264_ctx_get_info", -ret);
		goto out;
	}

	self->video_info = (const struct video_info){
		.resolution = {info.width, info.height},
		.bit_depth = info.bit_depth_luma,
		.sar = {info.sar_width, info.sar_height},
		.crop = {info.crop_left,
			 info.crop_top,
			 info.crop_width,
			 info.crop_height},
		.full_range = info.full_range,
		.color_primaries =
			vdef_color_primaries_from_h264(info.colour_primaries),
		.transfer_function = vdef_transfer_function_from_h264(
			info.transfer_characteristics),
		.matrix_coefs =
			vdef_matrix_coefs_from_h264(info.matrix_coefficients),
		.framerate.num = info.framerate_num,
		.framerate.den = info.framerate_den,
		.nal_hrd_bitrate = info.nal_hrd_bitrate,
		.nal_hrd_cpb_size = info.nal_hrd_cpb_size,
		.vcl_hrd_bitrate = info.vcl_hrd_bitrate,
		.vcl_hrd_cpb_size = info.vcl_hrd_cpb_size,
	};

	log_video_info(&self->video_info, ULOG_INFO);

	count = self->ops->get_supported_input_formats(
		&supported_input_formats);
	if (count < 0) {
		ULOG_ERRNO("get_supported_input_formats", -count);
		goto out;
	}

	if (self->ops->set_h264_ps) {
		if (vdef_coded_format_intersect(
			    format, supported_input_formats, count)) {
			ret = self->ops->set_h264_ps(
				self, sps, sps_size, pps, pps_size, format);
		} else {
			if (vdef_coded_format_intersect(&vdef_h264_raw_nalu,
							supported_input_formats,
							count)) {
				ret = self->ops->set_h264_ps(
					self,
					&data[4],
					sps_size - offset,
					&data[8 + sps_size - offset],
					pps_size - offset,
					&vdef_h264_raw_nalu);
			} else if (vdef_coded_format_intersect(
					   &vdef_h264_byte_stream,
					   supported_input_formats,
					   count)) {
				ret = self->ops->set_h264_ps(
					self,
					data,
					4 + sps_size - offset,
					&data[4 + sps_size - offset],
					4 + pps_size - offset,
					&vdef_h264_byte_stream);
			} else if (vdef_coded_format_intersect(
					   &vdef_h264_avcc,
					   supported_input_formats,
					   count)) {
				sc = htonl(sps_size - offset);
				memcpy(data, &sc, sizeof(sc));
				sc = htonl(pps_size - offset);
				memcpy(&data[4 + sps_size - offset],
				       &sc,
				       sizeof(sc));
				ret = self->ops->set_h264_ps(
					self,
					data,
					4 + sps_size - offset,
					&data[4 + sps_size - offset],
					4 + pps_size - offset,
					&vdef_h264_avcc);
			} else {
				ret = -ENOSYS;
				ULOG_ERRNO(
					"unsupported format:"
					" " VDEF_CODED_FORMAT_TO_STR_FMT,
					-ret,
					VDEF_CODED_FORMAT_TO_STR_ARG(format));
			}
		}
	} else {
		ret = -ENOSYS;
	}


	self->configured = (ret == 0) ? 1 : 0;

	/* Debug files */
	if (self->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_h264_ps(self->dbg.input_bs,
						    sps,
						    sps_size,
						    pps,
						    pps_size,
						    format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_h264_ps", -dbgret);
	}

out:
	if (data)
		free(data);
	return ret;
}


int vdec_set_h265_ps(struct vdec_decoder *self,
		     const uint8_t *vps,
		     size_t vps_size,
		     const uint8_t *sps,
		     size_t sps_size,
		     const uint8_t *pps,
		     size_t pps_size,
		     const struct vdef_coded_format *format)
{
	int ret, count;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format->encoding != self->config.encoding,
				 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format->encoding != VDEF_ENCODING_H265,
				 EINVAL);

	size_t offset = (format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU)
				? 0
				: 4;
	size_t size = 0, off = 0;
	uint8_t start_code[] = {0, 0, 0, 1};
	struct h265_info info = {0};
	struct h265_ctx *ctx = h265_reader_get_ctx(self->reader.h265);
	const struct vdef_coded_format *supported_input_formats;
	uint32_t sc;

	ULOG_ERRNO_RETURN_ERR_IF((vps == NULL) || (vps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(self->configured, EALREADY);

	/* VPS, SPS and PPS parsing */
	size = 4 + vps_size - offset + 4 + sps_size - offset + 4 + pps_size -
	       offset;

	uint8_t *data = malloc(size);
	if (data == NULL)
		return -ENOMEM;

	memcpy(&data[off], start_code, sizeof(start_code));
	off += 4;
	memcpy(&data[off], &vps[offset], vps_size - offset);
	off += vps_size - offset;

	memcpy(&data[off], start_code, sizeof(start_code));
	off += 4;
	memcpy(&data[off], &sps[offset], sps_size - offset);
	off += sps_size - offset;

	memcpy(&data[off], start_code, sizeof(start_code));
	off += 4;
	memcpy(&data[off], &pps[offset], pps_size - offset);

	ret = h265_reader_parse(self->reader.h265, 0, data, size, &off);
	if (ret < 0) {
		ULOG_ERRNO("h265_reader_parse", -ret);
		goto out;
	}

	/* Get the video info */
	ret = h265_ctx_get_info(ctx, &info);
	if (ret < 0) {
		ULOG_ERRNO("h264_ctx_get_info", -ret);
		goto out;
	}

	self->video_info = (const struct video_info){
		.resolution = {info.width, info.height},
		.bit_depth = info.bit_depth_luma,
		.sar = {info.sar_width, info.sar_height},
		.crop = {info.crop_left,
			 info.crop_top,
			 info.crop_width,
			 info.crop_height},
		.full_range = info.full_range,
		.color_primaries =
			vdef_color_primaries_from_h265(info.colour_primaries),
		.transfer_function = vdef_transfer_function_from_h265(
			info.transfer_characteristics),
		.matrix_coefs =
			vdef_matrix_coefs_from_h265(info.matrix_coefficients),
		.framerate.num = info.framerate_num,
		.framerate.den = info.framerate_den,
		.nal_hrd_bitrate = info.nal_hrd_bitrate,
		.nal_hrd_cpb_size = info.nal_hrd_cpb_size,
		.vcl_hrd_bitrate = info.vcl_hrd_bitrate,
		.vcl_hrd_cpb_size = info.vcl_hrd_cpb_size,
	};

	log_video_info(&self->video_info, ULOG_INFO);

	if (self->ops->set_h265_ps) {
		count = self->ops->get_supported_input_formats(
			&supported_input_formats);
		if (count < 0) {
			ULOG_ERRNO("get_supported_input_formats", -count);
			goto out;
		}

		if (vdef_coded_format_intersect(
			    format, supported_input_formats, count))
			ret = self->ops->set_h265_ps(self,
						     vps,
						     vps_size,
						     sps,
						     sps_size,
						     pps,
						     pps_size,
						     format);
		else {
			if (vdef_coded_format_intersect(&vdef_h265_raw_nalu,
							supported_input_formats,
							count))
				ret = self->ops->set_h265_ps(
					self,
					&data[4],
					vps_size - offset,
					&data[8 + vps_size - offset],
					sps_size - offset,
					&data[12 + vps_size - offset +
					      sps_size - offset],
					pps_size - offset,
					&vdef_h265_raw_nalu);
			else if (vdef_coded_format_intersect(
					 &vdef_h265_byte_stream,
					 supported_input_formats,
					 count))
				ret = self->ops->set_h265_ps(
					self,
					data,
					4 + vps_size - offset,
					&data[4 + vps_size - offset],
					4 + sps_size - offset,
					&data[8 + vps_size - offset + sps_size -
					      offset],
					4 + pps_size - offset,
					&vdef_h265_byte_stream);
			else if (vdef_coded_format_intersect(
					 &vdef_h265_hvcc,
					 supported_input_formats,
					 count)) {
				sc = htonl(vps_size - offset);
				memcpy(data, &sc, sizeof(sc));
				sc = htonl(sps_size - offset);
				memcpy(&data[4 + vps_size - offset],
				       &sc,
				       sizeof(sc));
				sc = htonl(pps_size - offset);
				memcpy(&data[8 + vps_size - offset + sps_size -
					     offset],
				       &sc,
				       sizeof(sc));
				ret = self->ops->set_h265_ps(
					self,
					data,
					4 + vps_size - offset,
					&data[4 + vps_size - offset],
					4 + sps_size - offset,
					&data[8 + vps_size - offset + sps_size -
					      offset],
					4 + pps_size - offset,
					&vdef_h265_hvcc);
			} else {
				ret = -ENOSYS;
				ULOG_ERRNO(
					"unsupported format:"
					" " VDEF_CODED_FORMAT_TO_STR_FMT,
					-ret,
					VDEF_CODED_FORMAT_TO_STR_ARG(format));
			}
		}
	} else {
		ret = -ENOSYS;
	}

	self->configured = (ret == 0) ? 1 : 0;

	/* Debug files */
	if (self->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_h265_ps(self->dbg.input_bs,
						    vps,
						    vps_size,
						    sps,
						    sps_size,
						    pps,
						    pps_size,
						    format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_h264_ps", -dbgret);
	}

out:
	if (data)
		free(data);
	return ret;
}


struct mbuf_pool *vdec_get_input_buffer_pool(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->ops->get_input_buffer_pool(self);
}


struct mbuf_coded_video_frame_queue *
vdec_get_input_buffer_queue(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(self == NULL, EINVAL, NULL);

	return self->ops->get_input_buffer_queue(self);
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
		*width = self->video_info.resolution.width;
	if (height != NULL)
		*height = self->video_info.resolution.height;
	if (sar_width != NULL)
		*sar_width = self->video_info.sar.width;
	if (sar_height != NULL)
		*sar_height = self->video_info.sar.height;
	if (crop_left != NULL)
		*crop_left = self->video_info.crop.left;
	if (crop_top != NULL)
		*crop_top = self->video_info.crop.top;
	if (crop_width != NULL)
		*crop_width = self->video_info.crop.width;
	if (crop_height != NULL)
		*crop_height = self->video_info.crop.height;

	return 0;
}


enum vdec_decoder_implem vdec_get_used_implem(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_VAL_IF(
		self == NULL, EINVAL, VDEC_DECODER_IMPLEM_AUTO);

	return self->config.implem;
}

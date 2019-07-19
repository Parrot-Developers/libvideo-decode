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


static FILE *vdec_dbg_create_file(const char *dir,
				  void *ctx,
				  const char *name,
				  const char *mode)
{
	int res;
	FILE *file = NULL;
	uint64_t epoch_sec = 0;
	int32_t utc_offset_sec = 0;
	struct tm tm;
	char *path = NULL;

	time_local_get(&epoch_sec, &utc_offset_sec);
	time_local_to_tm(epoch_sec, utc_offset_sec, &tm);

	res = asprintf(&path,
		       "%s/vdec_%04d%02d%02d_%02d%02d%02d_%d_%p_%s",
		       dir,
		       tm.tm_year + 1900,
		       tm.tm_mon + 1,
		       tm.tm_mday,
		       tm.tm_hour,
		       tm.tm_min,
		       tm.tm_sec,
		       getpid(),
		       ctx,
		       name);
	if (res <= 0) {
		ULOG_ERRNO("asprintf", ENOMEM);
		return NULL;
	}

	file = fopen(path, mode);
	if (file == NULL) {
		ULOGW("failed to create debug file '%s': err=%d(%s)",
		      path,
		      errno,
		      strerror(errno));
	}

	free(path);
	return file;
}


static void vdec_dbg_h264_nalu_begin_cb(struct h264_ctx *ctx,
					enum h264_nalu_type type,
					const uint8_t *buf,
					size_t len,
					void *userdata)
{
	int res;
	struct vdec_decoder *self = userdata;
	struct h264_nalu_header nh;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);

	if (self->dbg.h264_analysis == NULL)
		return;

	if ((type == H264_NALU_TYPE_SLICE) ||
	    (type == H264_NALU_TYPE_SLICE_DPA) ||
	    (type == H264_NALU_TYPE_SLICE_DPB) ||
	    (type == H264_NALU_TYPE_SLICE_DPC) ||
	    (type == H264_NALU_TYPE_SLICE_IDR))
		return;

	res = h264_parse_nalu_header(buf, len, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_nalu_header", -res);
		return;
	}

	fprintf(self->dbg.h264_analysis,
		"%d %d %d %d %d %d %d\n",
		self->dbg.h264_frame_index,
		nh.nal_unit_type,
		nh.nal_ref_idc,
		-1,
		-1,
		-1,
		-1);
}


static void vdec_dbg_h264_slice_cb(struct h264_ctx *ctx,
				   const uint8_t *buf,
				   size_t len,
				   const struct h264_slice_header *sh,
				   void *userdata)
{
	int res;
	struct vdec_decoder *self = userdata;
	struct h264_nalu_header nh;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);
	ULOG_ERRNO_RETURN_IF(sh == NULL, EINVAL);

	if (self->dbg.h264_analysis == NULL)
		return;

	res = h264_parse_nalu_header(buf, len, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_nalu_header", -res);
		return;
	}

	fprintf(self->dbg.h264_analysis,
		"%d %d %d %d %d %d %d\n",
		self->dbg.h264_frame_index,
		nh.nal_unit_type,
		nh.nal_ref_idc,
		sh->frame_num,
		sh->pic_order_cnt_lsb,
		sh->slice_type,
		sh->first_mb_in_slice);
}


static const struct h264_ctx_cbs h264_cbs = {
	.nalu_begin = &vdec_dbg_h264_nalu_begin_cb,
	.slice = &vdec_dbg_h264_slice_cb,
};


int vdec_dbg_create_files(struct vdec_decoder *self)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if ((self->config.dbg_flags & VDEC_DBG_FLAG_INPUT_BITSTREAM) != 0) {
		self->dbg.input_bs = vdec_dbg_create_file(
			self->config.dbg_dir, self, "input_bs.264", "wb");
	}

	if ((self->config.dbg_flags & VDEC_DBG_FLAG_OUTPUT_YUV) != 0) {
		self->dbg.output_yuv = vdec_dbg_create_file(
			self->config.dbg_dir, self, "output_yuv.yuv", "wb");
	}

	if ((self->config.dbg_flags & VDEC_DBG_FLAG_H264_ANALYSIS) != 0) {
		self->dbg.h264_analysis = vdec_dbg_create_file(
			self->config.dbg_dir, self, "h264_analysis.csv", "w");

		if (self->dbg.h264_analysis != NULL) {
			fprintf(self->dbg.h264_analysis,
				"index nal_unit_type "
				"nal_ref_idc frame_num pic_order_cnt_lsb "
				"slice_type first_mb_in_slice\n");

			res = h264_reader_new(
				&h264_cbs, self, &self->dbg.h264_reader);
			if (res < 0)
				ULOG_ERRNO("h264_reader_new", -res);
		}
	}

	return 0;
}


int vdec_dbg_close_files(struct vdec_decoder *self)
{
	int res;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->dbg.input_bs != NULL) {
		fclose(self->dbg.input_bs);
		self->dbg.input_bs = NULL;
	}

	if (self->dbg.output_yuv != NULL) {
		fclose(self->dbg.output_yuv);
		self->dbg.output_yuv = NULL;
	}

	if (self->dbg.h264_analysis != NULL) {
		fclose(self->dbg.h264_analysis);
		self->dbg.h264_analysis = NULL;
	}

	if (self->dbg.h264_reader != NULL) {
		res = h264_reader_destroy(self->dbg.h264_reader);
		if (res < 0)
			ULOG_ERRNO("h264_reader_destroy", -res);
	}

	return 0;
}


int vdec_dbg_write_h264_sps_pps(FILE *file,
				const uint8_t *sps,
				size_t sps_size,
				const uint8_t *pps,
				size_t pps_size,
				enum vdec_input_format format)
{
	size_t offset = 0;
	int res, start_code = 0;

	ULOG_ERRNO_RETURN_ERR_IF(file == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size == 0), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size == 0), EINVAL);

	switch (format) {
	case VDEC_INPUT_FORMAT_RAW_NALU:
		start_code = 1;
		break;
	case VDEC_INPUT_FORMAT_BYTE_STREAM:
		break;
	case VDEC_INPUT_FORMAT_AVCC:
		if ((sps_size <= 4) || (pps_size <= 4)) {
			res = -EINVAL;
			ULOG_ERRNO("invalid SPS or PPS size", -res);
			return res;
		}
		offset = 4;
		start_code = 1;
		break;
	default:
		res = -EINVAL;
		ULOG_ERRNO("invalid format: %d", -res, format);
		return res;
	}

	/* SPS */
	if (start_code)
		fwrite("\x00\x00\x00\x01", 1, 4, file);
	fwrite(sps + offset, 1, sps_size - offset, file);

	/* PPS */
	if (start_code)
		fwrite("\x00\x00\x00\x01", 1, 4, file);
	fwrite(pps + offset, 1, pps_size - offset, file);

	return 0;
}


int vdec_dbg_write_h264_frame(FILE *file,
			      struct vbuf_buffer *buf,
			      enum vdec_input_format format)
{
	int res;
	const uint8_t *data;
	ssize_t res1;
	size_t size, nalusize, offset = 0;

	ULOG_ERRNO_RETURN_ERR_IF(file == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	data = vbuf_get_cdata(buf);
	res1 = vbuf_get_size(buf);
	if ((data == NULL) || (res1 <= 0)) {
		res = -EPROTO;
		ULOG_ERRNO(
			"invalid buffer (data=%p size=%zi)", -res, data, res1);
		return res;
	}
	size = res1;

	/* Byte stream */
	if (format == VDEC_INPUT_FORMAT_BYTE_STREAM) {
		fwrite(data, 1, size, file);
		return 0;
	} else if (format != VDEC_INPUT_FORMAT_AVCC) {
		res = -EINVAL;
		ULOG_ERRNO("unsupported format: %s",
			   -res,
			   vdec_input_format_str(format));
		return res;
	}

	/* AVCC */
	while (offset < size) {
		if (offset + 4 >= size) {
			ULOGE("invalid size");
			return -EPROTO;
		}
		nalusize = ((uint32_t)data[offset] << 24) +
			   ((uint32_t)data[offset + 1] << 16) +
			   ((uint32_t)data[offset + 2] << 8) +
			   (uint32_t)data[offset + 3];
		offset += 4;
		if (offset + nalusize > size) {
			ULOGE("invalid NALU size");
			return -EPROTO;
		}
		fwrite("\x00\x00\x00\x01", 1, 4, file);
		fwrite(data + offset, 1, nalusize, file);
		offset += nalusize;
	}

	return 0;
}


int vdec_dbg_parse_h264_sps_pps(struct vdec_decoder *self,
				const uint8_t *sps,
				size_t sps_size,
				const uint8_t *pps,
				size_t pps_size,
				enum vdec_input_format format)
{
	int res;
	size_t offset = 0;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size == 0), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size == 0), EINVAL);

	switch (format) {
	case VDEC_INPUT_FORMAT_RAW_NALU:
		break;
	case VDEC_INPUT_FORMAT_BYTE_STREAM:
	case VDEC_INPUT_FORMAT_AVCC:
		if ((sps_size <= 4) || (pps_size <= 4))
			return -EINVAL;
		offset = 4;
		break;
	default:
		res = -EINVAL;
		ULOG_ERRNO("invalid format: %d", -res, format);
		return res;
	}

	res = h264_reader_parse_nalu(
		self->dbg.h264_reader, 0, sps + offset, sps_size - offset);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_parse_nalu:sps", -res);
		return res;
	}

	res = h264_reader_parse_nalu(
		self->dbg.h264_reader, 0, pps + offset, pps_size - offset);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_parse_nalu:pps", -res);
		return res;
	}

	return 0;
}


int vdec_dbg_parse_h264_frame(struct vdec_decoder *self,
			      struct vbuf_buffer *buf,
			      enum vdec_input_format format)
{
	int res = 0;
	const uint8_t *data;
	ssize_t res1;
	size_t size, off = 0, nalusize, offset = 0;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	data = vbuf_get_cdata(buf);
	res1 = vbuf_get_size(buf);
	if ((data == NULL) || (res1 <= 0)) {
		res = -EPROTO;
		ULOG_ERRNO(
			"invalid buffer (data=%p size=%zi)", -res, data, res1);
		return res;
	}
	size = res1;

	/* Byte stream */
	if (format == VDEC_INPUT_FORMAT_BYTE_STREAM) {
		res = h264_reader_parse(
			self->dbg.h264_reader, 0, data, size, &off);
		if (res < 0)
			ULOG_ERRNO("h264_reader_parse", -res);
		self->dbg.h264_frame_index++;
		return res;
	} else if (format != VDEC_INPUT_FORMAT_AVCC) {
		res = -EINVAL;
		ULOG_ERRNO("unsupported format: %s",
			   -res,
			   vdec_input_format_str(format));
		return res;
	}

	/* AVCC */
	while (offset < size) {
		if (offset + 4 >= size) {
			ULOGE("invalid size");
			return -EPROTO;
		}
		nalusize = ((uint32_t)data[offset] << 24) +
			   ((uint32_t)data[offset + 1] << 16) +
			   ((uint32_t)data[offset + 2] << 8) +
			   (uint32_t)data[offset + 3];
		offset += 4;
		if (offset + nalusize > size) {
			ULOGE("invalid NALU size");
			return -EPROTO;
		}
		res = h264_reader_parse(self->dbg.h264_reader,
					0,
					data + offset,
					nalusize,
					&off);
		if (res < 0) {
			ULOG_ERRNO("h264_reader_parse", -res);
			break;
		}
		offset += nalusize;
	}

	self->dbg.h264_frame_index++;
	return res;
}


int vdec_dbg_write_yuv_frame(FILE *file,
			     struct vbuf_buffer *buf,
			     struct vdec_output_metadata *meta)
{
	int res;
	const uint8_t *d, *data;
	unsigned int i;
	ssize_t res1;

	ULOG_ERRNO_RETURN_ERR_IF(file == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(meta == NULL, EINVAL);

	d = vbuf_get_cdata(buf);
	res1 = vbuf_get_size(buf);
	if ((d == NULL) || (res1 <= 0)) {
		res = -EPROTO;
		ULOG_ERRNO("invalid buffer (data=%p size=%zi)", -res, d, res1);
		return res;
	}

	switch (meta->format) {
	case VDEC_OUTPUT_FORMAT_NV12:
		data = d + meta->plane_offset[0];
		for (i = 0; i < meta->height; i++) {
			fwrite(data, 1, meta->width, file);
			data += meta->plane_stride[0];
		}
		data = d + meta->plane_offset[1];
		for (i = 0; i < meta->height / 2; i++) {
			fwrite(data, 1, meta->width, file);
			data += meta->plane_stride[1];
		}
		break;
	case VDEC_OUTPUT_FORMAT_I420:
		data = d + meta->plane_offset[0];
		for (i = 0; i < meta->height; i++) {
			fwrite(data, 1, meta->width, file);
			data += meta->plane_stride[0];
		}
		data = d + meta->plane_offset[1];
		for (i = 0; i < meta->height / 2; i++) {
			fwrite(data, 1, meta->width / 2, file);
			data += meta->plane_stride[1];
		}
		data = d + meta->plane_offset[2];
		for (i = 0; i < meta->height / 2; i++) {
			fwrite(data, 1, meta->width / 2, file);
			data += meta->plane_stride[2];
		}
		break;
	default:
		return 0;
	}

	return 0;
}

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

#define ULOG_TAG vdec_core
#include "vdec_core_priv.h"

ULOG_DECLARE_TAG(ULOG_TAG);


static FILE *
create_file(const char *dir, void *ctx, const char *name, const char *mode)
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


void vdec_dbg_h264_nalu_begin(struct vdec_decoder *self,
			      struct h264_nalu_header nh,
			      enum h264_nalu_type type)
{
	if (self->dbg.analysis == NULL)
		return;

	if ((type == H264_NALU_TYPE_SLICE) ||
	    (type == H264_NALU_TYPE_SLICE_DPA) ||
	    (type == H264_NALU_TYPE_SLICE_DPB) ||
	    (type == H264_NALU_TYPE_SLICE_DPC) ||
	    (type == H264_NALU_TYPE_SLICE_IDR))
		return;

	fprintf(self->dbg.analysis,
		"%d %d %d %d %d %d %d\n",
		self->dbg.frame_index,
		nh.nal_unit_type,
		nh.nal_ref_idc,
		-1,
		-1,
		-1,
		-1);
}


void vdec_dbg_h264_slice(struct vdec_decoder *self,
			 struct h264_nalu_header nh,
			 const struct h264_slice_header *sh)
{
	if (self->dbg.analysis == NULL)
		return;

	fprintf(self->dbg.analysis,
		"%d %d %d %d %d %d %d\n",
		self->dbg.frame_index,
		nh.nal_unit_type,
		nh.nal_ref_idc,
		sh->frame_num,
		sh->pic_order_cnt_lsb,
		sh->slice_type,
		sh->first_mb_in_slice);
}


void vdec_dbg_h265_nalu_begin(struct vdec_decoder *self,
			      enum h265_nalu_type type)
{
	if (self->dbg.analysis == NULL)
		return;

	fprintf(self->dbg.analysis,
		"%d %d %d %d %d %d %d\n",
		self->dbg.frame_index,
		type,
		-1,
		-1,
		-1,
		-1,
		-1);
}


int vdec_dbg_create_files(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if ((self->config.dbg_flags & VDEC_DBG_FLAG_INPUT_BITSTREAM) != 0) {
		const char *filename =
			self->config.encoding == VDEF_ENCODING_H265
				? "input.h265"
				: "input.h264";

		self->dbg.input_bs =
			create_file(self->config.dbg_dir, self, filename, "wb");
	}

	if ((self->config.dbg_flags & VDEC_DBG_FLAG_OUTPUT_YUV) != 0) {
		self->dbg.output_yuv = create_file(
			self->config.dbg_dir, self, "output.yuv", "wb");
	}

	if ((self->config.dbg_flags & VDEC_DBG_FLAG_ANALYSIS) != 0) {
		self->dbg.analysis = create_file(
			self->config.dbg_dir, self, "analysis.csv", "w");

		if (self->dbg.analysis != NULL) {
			fprintf(self->dbg.analysis,
				"index nal_unit_type "
				"nal_ref_idc frame_num pic_order_cnt_lsb "
				"slice_type first_mb_in_slice\n");
		}
	}

	return 0;
}


int vdec_dbg_close_files(struct vdec_decoder *self)
{
	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);

	if (self->dbg.input_bs != NULL) {
		fclose(self->dbg.input_bs);
		self->dbg.input_bs = NULL;
	}

	if (self->dbg.output_yuv != NULL) {
		fclose(self->dbg.output_yuv);
		self->dbg.output_yuv = NULL;
	}

	if (self->dbg.analysis != NULL) {
		fclose(self->dbg.analysis);
		self->dbg.analysis = NULL;
	}

	return 0;
}


static int write_ps(FILE *file,
		    const uint8_t **ps_table,
		    const size_t *size_table,
		    size_t len,
		    const struct vdef_coded_format *format)
{
	ULOG_ERRNO_RETURN_ERR_IF(file == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ps_table == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(size_table == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);

	size_t offset =
		(format->data_format == VDEF_CODED_DATA_FORMAT_AVCC) ? 4 : 0;
	int start_code_missing =
		(format->data_format != VDEF_CODED_DATA_FORMAT_BYTE_STREAM);

	const uint8_t start_code[] = {0, 0, 0, 1};
	for (size_t i = 0; i < len; ++i) {
		if (start_code_missing)
			fwrite(start_code, 1, 4, file);
		fwrite(&ps_table[i][offset], 1, size_table[i] - offset, file);
	}

	return 0;
}


int vdec_dbg_write_h264_ps(FILE *file,
			   const uint8_t *sps,
			   size_t sps_size,
			   const uint8_t *pps,
			   size_t pps_size,
			   const struct vdef_coded_format *format)
{
	const uint8_t *ps_table[] = {sps, pps};
	size_t size_table[] = {sps_size, pps_size};

	return write_ps(file, ps_table, size_table, 2, format);
}


int vdec_dbg_write_h265_ps(FILE *file,
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

	return write_ps(file, ps_table, size_table, 3, format);
}


int vdec_dbg_write_frame(FILE *file, struct mbuf_coded_video_frame *frame)
{
	int res;
	struct vdef_coded_frame infos;

	ULOG_ERRNO_RETURN_ERR_IF(file == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &infos);
	if (res != 0)
		return res;

	enum vdef_coded_data_format fmt = infos.format.data_format;

	int nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	if (nalu_count < 0)
		return nalu_count;

	for (int i = 0; i < nalu_count; i++) {
		const void *data;
		struct vdef_nalu nalu;
		res = mbuf_coded_video_frame_get_nalu(frame, i, &data, &nalu);
		if (res != 0)
			break;
		switch (fmt) {
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
			fwrite(data, 1, nalu.size, file);
			break;
		case VDEF_CODED_DATA_FORMAT_AVCC:
			fwrite("\x00\x00\x00\x01", 1, 4, file);
			fwrite((uint8_t *)data + 4, 1, nalu.size - 4, file);
			break;
		case VDEF_CODED_DATA_FORMAT_RAW_NALU:
			fwrite("\x00\x00\x00\x01", 1, 4, file);
			fwrite(data, 1, nalu.size, file);
			break;
		default:
			break;
		}
		res = mbuf_coded_video_frame_release_nalu(frame, i, data);
		if (res != 0)
			break;
	}
	return res;
}


static int parse(struct vdec_decoder *self, const uint8_t *data, size_t size)
{
	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		return h264_reader_parse_nalu(self->reader.h264, 0, data, size);

	case VDEF_ENCODING_H265:
		return h265_reader_parse_nalu(self->reader.h265, 0, data, size);

	default:
		return 0;
	}
}


int vdec_dbg_parse_frame(struct vdec_decoder *self,
			 struct mbuf_coded_video_frame *frame)
{
	int res;
	struct vdef_coded_frame infos;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &infos);
	if (res != 0)
		return res;

	enum vdef_coded_data_format fmt = infos.format.data_format;

	int nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	if (nalu_count < 0)
		return nalu_count;
	for (int i = 0; i < nalu_count; i++) {
		const void *data;
		const uint8_t *raw_nalu;
		struct vdef_nalu nalu;
		res = mbuf_coded_video_frame_get_nalu(frame, i, &data, &nalu);
		if (res != 0)
			break;
		raw_nalu = data;
		switch (fmt) {
		case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		case VDEF_CODED_DATA_FORMAT_AVCC:
			raw_nalu += 4;
			nalu.size -= 4;
			break;
		default:
			break;
		}
		res = parse(self, raw_nalu, nalu.size);
		int res2 = mbuf_coded_video_frame_release_nalu(frame, i, data);
		if (res2 != 0) {
			res = res2;
			break;
		}
	}

	self->dbg.frame_index++;
	return res;
}


int vdec_dbg_write_yuv_frame(FILE *file, struct mbuf_raw_video_frame *frame)
{
	int res;
	struct vdef_raw_frame infos;

	ULOG_ERRNO_RETURN_ERR_IF(file == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);

	res = mbuf_raw_video_frame_get_frame_info(frame, &infos);
	if (res != 0)
		return res;

	int nplanes = vdef_get_raw_frame_plane_count(&infos.format);

	/* TODO: This function will write the frame as-is, including strides.
	 * we could check for strides, and if present, remove them using
	 * mbuf_raw_video_frame_copy() in the future */

	for (int i = 0; i < nplanes; i++) {
		const void *data;
		size_t len;
		res = mbuf_raw_video_frame_get_plane(frame, i, &data, &len);
		if (res != 0)
			break;
		fwrite(data, 1, len, file);
		res = mbuf_raw_video_frame_release_plane(frame, i, data);
		if (res != 0)
			break;
	}

	return res;
}

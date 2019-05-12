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


bool vdec_h264_is_idr(struct mbuf_coded_video_frame *frame,
		      struct vdef_coded_frame *info)
{
	int err = 0;
	int nalu_count;

	ULOG_ERRNO_RETURN_VAL_IF(frame == NULL, EINVAL, false);
	ULOG_ERRNO_RETURN_VAL_IF(info == NULL, EINVAL, false);

	nalu_count = mbuf_coded_video_frame_get_nalu_count(frame);
	if (nalu_count < 0) {
		err = nalu_count;
		ULOG_ERRNO("mbuf_coded_video_frame_get_nalu_count", -err);
		return false;
	}
	for (int i = 0; i < nalu_count; i++) {
		const void *data;
		const uint8_t *raw_nalu;
		struct vdef_nalu nalu;

		err = mbuf_coded_video_frame_get_nalu(frame, i, &data, &nalu);
		if (err < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_get_nalu", -err);
			return false;
		}

		/* If the type is "unknown", read it from data */
		if (nalu.h264.type == H264_NALU_TYPE_UNKNOWN) {
			raw_nalu = data;
			if (info->format.data_format !=
			    VDEF_CODED_DATA_FORMAT_RAW_NALU)
				raw_nalu += 4;
			nalu.h264.type =
				(enum h264_nalu_type)(*raw_nalu & 0x1F);
		}

		err = mbuf_coded_video_frame_release_nalu(frame, i, data);
		if (err < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_release_nalu", -err);
			return false;
		}

		/* As for each frame, trust the nalu type if given */
		if (nalu.h264.type == H264_NALU_TYPE_SLICE_IDR)
			return true;
		else if (nalu.h264.type == H264_NALU_TYPE_SLICE)
			return false;
	}
	return false;
}


int vdec_h264_write_grey_idr(struct vdec_decoder *self,
			     struct vdef_coded_frame *in_frame_info,
			     uint64_t *delta,
			     uint64_t *timestamp,
			     struct mbuf_mem *idr_mem,
			     struct mbuf_coded_video_frame **idr_frame)
{
	int ret;
	struct h264_slice_header *sh = NULL;
	uint32_t mb_total, sc;
	struct h264_bitstream bs;
	struct h264_ctx *ctx = h264_reader_get_ctx(self->reader.h264);
	uint8_t *data;
	size_t size = 0;
	struct vdef_coded_frame idr_frame_info;
	struct vdef_nalu nalu;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(in_frame_info == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(timestamp == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(idr_mem == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(idr_frame == NULL, EINVAL);

	*idr_frame = NULL;

	if (!self->configured) {
		ULOGW("%s: decoder is not configured", __func__);
		return -EAGAIN;
	}

	idr_frame_info = *in_frame_info;
	idr_frame_info.info.capture_timestamp = 0;
	idr_frame_info.type = VDEF_CODED_FRAME_TYPE_IDR;
	idr_frame_info.info.flags = VDEF_FRAME_FLAG_SILENT;

	*delta = (in_frame_info->info.timestamp == 0) ? 1 : 0;
	if (in_frame_info->info.timestamp != 0)
		idr_frame_info.info.timestamp--;
	*timestamp = idr_frame_info.info.timestamp;

	/* Start NALU */
	ret = h264_ctx_clear_nalu(ctx);
	if (ret < 0) {
		ULOG_ERRNO("h264_ctx_clear_nalu", -ret);
		return ret;
	}

	/* Setup NALU header */
	struct h264_nalu_header nh = {
		.nal_ref_idc = 3,
		.nal_unit_type = H264_NALU_TYPE_SLICE_IDR,
	};
	ret = h264_ctx_set_nalu_header(ctx, &nh);
	if (ret < 0) {
		ULOG_ERRNO("h264_ctx_set_nalu_header", -ret);
		goto out;
	}

	/* Setup slice header */
	sh = calloc(1, sizeof(*sh));
	if (sh == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("calloc", -ret);
		goto out;
	}

	sh->first_mb_in_slice = 0;
	sh->slice_type = H264_SLICE_TYPE_I;
	sh->frame_num = 0;
	sh->pic_order_cnt_lsb = 0;
	sh->redundant_pic_cnt = 0;
	sh->direct_spatial_mv_pred_flag = 0;
	sh->slice_qp_delta = 0;
	sh->disable_deblocking_filter_idc = 2;
	sh->slice_alpha_c0_offset_div2 = 0;
	sh->slice_beta_offset_div2 = 0;
	sh->drpm.long_term_reference_flag =
		(in_frame_info->info.flags & VDEF_FRAME_FLAG_USES_LTR) ? 1 : 0;
	ret = h264_ctx_set_slice_header(ctx, sh);
	if (ret < 0) {
		ULOG_ERRNO("h264_ctx_set_slice_header", -ret);
		goto out;
	}

	/* Setup bitstream */
	ret = mbuf_mem_get_data(idr_mem, (void **)&data, &size);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -ret);
		goto out;
	}

	memset(&bs, 0, sizeof(bs));
	if (size <= 4) {
		ret = -ENOBUFS;
		ULOG_ERRNO("", -ret);
		goto out;
	}

	h264_bs_init(&bs, data + 4, size - 4, 1);

	/* Write slice */
	mb_total = VDEF_ROUND_UP(self->video_info.resolution.height, 16) *
		   VDEF_ROUND_UP(self->video_info.resolution.width, 16);

	h264_write_grey_i_slice(&bs, ctx, mb_total);

	switch (in_frame_info->format.data_format) {
	case VDEF_CODED_DATA_FORMAT_AVCC:
		sc = htonl(bs.off);
		memcpy(data, &sc, sizeof(sc));
		break;
	case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		sc = htonl(0x00000001);
		memcpy(data, &sc, sizeof(sc));
		break;
	default:
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported data format", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_new(&idr_frame_info, idr_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		goto out;
	}
	nalu = (struct vdef_nalu){
		.size = bs.off + 4,
		.h264.type = H264_NALU_TYPE_SLICE_IDR,
		.h264.slice_type = H264_SLICE_TYPE_I,
		.h264.slice_mb_count = mb_total,
	};
	ret = mbuf_coded_video_frame_add_nalu(*idr_frame, idr_mem, 0, &nalu);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -ret);
		goto out;
	}
	ret = mbuf_coded_video_frame_finalize(*idr_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto out;
	}

out:
	if (ret != 0) {
		if (*idr_frame != NULL) {
			mbuf_coded_video_frame_unref(*idr_frame);
			*idr_frame = NULL;
		}
	}
	free(sh);
	return ret;
}

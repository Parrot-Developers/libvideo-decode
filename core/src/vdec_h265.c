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


bool vdec_h265_is_idr(struct mbuf_coded_video_frame *frame,
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
		if (nalu.h265.type == H265_NALU_TYPE_UNKNOWN) {
			raw_nalu = data;
			if (info->format.data_format !=
			    VDEF_CODED_DATA_FORMAT_RAW_NALU)
				raw_nalu += 4;
			nalu.h265.type =
				(enum h265_nalu_type)((*raw_nalu & 0x3E) >> 1);
		}

		err = mbuf_coded_video_frame_release_nalu(frame, i, data);
		if (err < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_release_nalu", -err);
			return false;
		}

		/* As for each frame, trust the nalu type if given */
		switch (nalu.h265.type) {
		case H265_NALU_TYPE_IDR_W_RADL:
		case H265_NALU_TYPE_IDR_N_LP:
			return true;
		case H265_NALU_TYPE_BLA_W_LP:
		case H265_NALU_TYPE_BLA_W_RADL:
		case H265_NALU_TYPE_BLA_N_LP:
		case H265_NALU_TYPE_CRA_NUT:
			return false;
		default:
			break;
		}
	}
	return false;
}

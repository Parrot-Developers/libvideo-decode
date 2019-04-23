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


#define VDEC_H264_EXTENDED_SAR 255


const unsigned int vdec_h264_sar[17][2] = {
	{1, 1},
	{1, 1},
	{12, 11},
	{10, 11},
	{16, 11},
	{40, 33},
	{24, 11},
	{20, 11},
	{32, 11},
	{80, 33},
	{18, 11},
	{15, 11},
	{64, 33},
	{160, 99},
	{4, 3},
	{3, 2},
	{2, 1},
};


int vdec_h264_get_video_info(struct vdec_decoder *self,
			     const uint8_t *sps,
			     size_t sps_size,
			     const uint8_t *pps,
			     size_t pps_size,
			     enum vdec_input_format format)
{
	int ret;
	struct h264_sps *_sps = NULL;
	struct h264_sps_derived sps_derived;
	size_t offset = (format == VDEC_INPUT_FORMAT_RAW_NALU) ? 0 : 4;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);

	_sps = calloc(1, sizeof(*_sps));
	if (_sps == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		return -ENOMEM;
	}

	ret = h264_parse_sps(sps + offset, sps_size - offset, _sps);
	if (ret < 0) {
		ULOG_ERRNO("h264_parse_sps", -ret);
		goto exit;
	}

	ret = h264_get_sps_derived(_sps, &sps_derived);
	if (ret < 0) {
		ULOG_ERRNO("h264_get_sps_derived", -ret);
		goto exit;
	}

	self->width = sps_derived.PicWidthInSamplesLuma;
	self->height = sps_derived.FrameHeightInMbs * 16;
	self->crop_left = 0;
	self->crop_top = 0;
	self->crop_width = self->width;
	self->crop_height = self->height;
	if (_sps->frame_cropping_flag) {
		self->crop_left =
			_sps->frame_crop_left_offset * sps_derived.CropUnitX;
		self->crop_width = self->width - _sps->frame_crop_right_offset *
							 sps_derived.CropUnitX;
		self->crop_top =
			_sps->frame_crop_top_offset * sps_derived.CropUnitY;
		self->crop_height =
			self->height -
			_sps->frame_crop_bottom_offset * sps_derived.CropUnitY;
	}

	self->sar_width = 1;
	self->sar_height = 1;
	if (_sps->vui_parameters_present_flag) {
		if (_sps->vui.aspect_ratio_info_present_flag) {
			if (_sps->vui.aspect_ratio_idc ==
			    VDEC_H264_EXTENDED_SAR) {
				self->sar_width = _sps->vui.sar_width;
				self->sar_height = _sps->vui.sar_height;
			} else if (_sps->vui.aspect_ratio_idc <= 16) {
				self->sar_width = vdec_h264_sar
					[_sps->vui.aspect_ratio_idc][0];
				self->sar_height = vdec_h264_sar
					[_sps->vui.aspect_ratio_idc][1];
			}
		}
		self->full_range = _sps->vui.video_full_range_flag;
		if (_sps->vui.timing_info_present_flag) {
			self->num_units_in_tick = _sps->vui.num_units_in_tick;
			self->time_scale = _sps->vui.time_scale;
		}
		if (_sps->vui.nal_hrd_parameters_present_flag) {
			self->nal_hrd_bitrate =
				(_sps->vui.nal_hrd.cpb[0]
					 .bit_rate_value_minus1 +
				 1)
				<< (6 + _sps->vui.nal_hrd.bit_rate_scale);
			self->nal_hrd_cpb_size =
				(_sps->vui.nal_hrd.cpb[0]
					 .cpb_size_value_minus1 +
				 1)
				<< (4 + _sps->vui.nal_hrd.cpb_size_scale);
		}
		if (_sps->vui.vcl_hrd_parameters_present_flag) {
			self->vcl_hrd_bitrate =
				(_sps->vui.vcl_hrd.cpb[0]
					 .bit_rate_value_minus1 +
				 1)
				<< (6 + _sps->vui.vcl_hrd.bit_rate_scale);
			self->vcl_hrd_cpb_size =
				(_sps->vui.vcl_hrd.cpb[0]
					 .cpb_size_value_minus1 +
				 1)
				<< (4 + _sps->vui.vcl_hrd.cpb_size_scale);
		}
	}

	ret = 0;

exit:
	free(_sps);
	return ret;
}


int vdec_h264_format_convert(struct vbuf_buffer *buf,
			     enum vdec_input_format current_format,
			     enum vdec_input_format target_format)
{
	int res = 0;
	uint8_t *data;
	uint32_t nalu_len, start_code = htonl(0x00000001);
	ssize_t res1;
	size_t len, offset, start, end;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	if (vbuf_is_write_locked(buf)) {
		res = -EPERM;
		ULOG_ERRNO("buffer is locked", -res);
		return res;
	}

	/* No conversion needed */
	if (current_format == target_format)
		return 0;

	/* Only support byte stream to AVCC or AVCC to byte stream */
	if ((current_format == VDEC_INPUT_FORMAT_RAW_NALU) ||
	    (target_format == VDEC_INPUT_FORMAT_RAW_NALU)) {
		res = -ENOSYS;
		ULOG_ERRNO("cannot convert raw NALU formats", -res);
		return res;
	}

	data = vbuf_get_data(buf);
	res1 = vbuf_get_size(buf);
	if ((data == NULL) || (res1 <= 0)) {
		res = -EPROTO;
		ULOG_ERRNO(
			"invalid buffer (data=%p size=%zi)", -res, data, res1);
		return res;
	}
	len = res1;

	if ((current_format == VDEC_INPUT_FORMAT_AVCC) &&
	    (target_format == VDEC_INPUT_FORMAT_BYTE_STREAM)) {
		/* AVCC to byte stream */
		offset = 0;
		while (offset < len) {
			memcpy(&nalu_len, data, sizeof(uint32_t));
			nalu_len = ntohl(nalu_len);
			memcpy(data, &start_code, sizeof(uint32_t));
			data += 4 + nalu_len;
			offset += 4 + nalu_len;
		}
	} else if ((current_format == VDEC_INPUT_FORMAT_BYTE_STREAM) &&
		   (target_format == VDEC_INPUT_FORMAT_AVCC)) {
		/* byte stream to AVCC */
		res = 0;
		offset = 0;
		while (offset < len) {
			res = h264_find_nalu(data, len - offset, &start, &end);
			if (res < 0) {
				if (res != -EAGAIN)
					break;
				else
					res = 0;
			}
			if (start != 4) {
				res = -EPROTO;
				break;
			}
			nalu_len = end - start;
			memcpy(data, &nalu_len, sizeof(uint32_t));
			data += 4 + nalu_len;
			offset += 4 + nalu_len;
		}
	} else {
		res = -ENOSYS;
		ULOG_ERRNO("unsupported conversion (%s to %s)",
			   -res,
			   vdec_input_format_str(current_format),
			   vdec_input_format_str(target_format));
		return res;
	}

	return res;
}

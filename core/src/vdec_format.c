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
#include <futils/timetools.h>


int vdec_format_convert(struct mbuf_coded_video_frame *frame,
			const struct vdef_coded_format *target_format)
{
	int res;
	uint8_t *data;
	size_t len;
	struct vdef_coded_frame info;
	const struct vdef_coded_format *current_format;

	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(target_format == NULL, EINVAL);

	res = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -res);
		return res;
	}
	current_format = &info.format;

	/* No conversion needed */
	if (vdef_coded_format_cmp(current_format, target_format))
		return 0;

	res = mbuf_coded_video_frame_get_rw_packed_buffer(
		frame, (void **)&data, &len);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_rw_packed_buffer", -res);
		goto out;
	}

	if (current_format->encoding != target_format->encoding) {
		res = -ENOSYS;
		ULOG_ERRNO("encoding mismatch", -res);
		goto out;
	}
	if ((current_format->encoding != VDEF_ENCODING_H264) &&
	    (current_format->encoding != VDEF_ENCODING_H265)) {
		res = -ENOSYS;
		ULOG_ERRNO("unsupported encoding", -res);
		goto out;
	}

	/* Only support byte stream to AVCC/HVCC or AVCC/HVCC to byte stream */
	if ((current_format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU) ||
	    (target_format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU)) {
		res = -ENOSYS;
		ULOG_ERRNO("cannot convert raw NALU formats", -res);
		goto out;
	}

	if (current_format->data_format == VDEF_CODED_DATA_FORMAT_AVCC &&
	    target_format->data_format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		/* AVCC/HVCC to byte stream */
		switch (current_format->encoding) {
		case VDEF_ENCODING_H264:
			res = h264_avcc_to_byte_stream(data, len);
			if (res < 0) {
				ULOG_ERRNO("h264_avcc_to_byte_stream", -res);
				goto out;
			}
			break;
		case VDEF_ENCODING_H265:
			res = h265_hvcc_to_byte_stream(data, len);
			if (res < 0) {
				ULOG_ERRNO("h265_hvcc_to_byte_stream", -res);
				goto out;
			}
			break;
		default:
			break;
		}
	} else if (current_format->data_format ==
			   VDEF_CODED_DATA_FORMAT_BYTE_STREAM &&
		   target_format->data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		/* Byte stream to AVCC */
		switch (current_format->encoding) {
		case VDEF_ENCODING_H264:
			res = h264_byte_stream_to_avcc(data, len);
			if (res < 0) {
				ULOG_ERRNO("h264_byte_stream_to_avcc", -res);
				goto out;
			}
			break;
		case VDEF_ENCODING_H265:
			res = h265_byte_stream_to_hvcc(data, len);
			if (res < 0) {
				ULOG_ERRNO("h265_byte_stream_to_hvcc", -res);
				goto out;
			}
			break;
		default:
			break;
		}
	} else {
		res = -ENOSYS;
		ULOG_ERRNO("unsupported conversion (%s to %s)",
			   -res,
			   vdef_coded_data_format_to_str(
				   current_format->data_format),
			   vdef_coded_data_format_to_str(
				   target_format->data_format));
		return res;
	}


out:
	mbuf_coded_video_frame_release_rw_packed_buffer(frame, data);
	return res;
}


bool vdec_is_sync_frame(struct mbuf_coded_video_frame *frame,
			struct vdef_coded_frame *info)
{
	ULOG_ERRNO_RETURN_VAL_IF(frame == NULL, EINVAL, false);
	ULOG_ERRNO_RETURN_VAL_IF(info == NULL, EINVAL, false);

	/* If the frame is declared as an IDR, trust it */
	if (info->type == VDEF_CODED_FRAME_TYPE_IDR)
		return true;
	/* If the frame is declared as anything else, trust it too */
	if (info->type != VDEF_CODED_FRAME_TYPE_UNKNOWN)
		return false;

	/* If the frame is of type UNKNOWN, parse the frame */
	switch (info->format.encoding) {
	case VDEF_ENCODING_H264:
		return vdec_h264_is_idr(frame, info);
	case VDEF_ENCODING_H265:
		return vdec_h265_is_idr(frame, info);
	default:
		ULOGE("unsupported encoding (%s)",
		      vdef_encoding_to_str(info->format.encoding));
		return false;
	}
}


bool vdec_default_input_filter(struct mbuf_coded_video_frame *frame,
			       void *userdata)
{
	int ret;
	bool accept;
	struct vdec_decoder *decoder = userdata;
	const struct vdef_coded_format *supported_formats;
	struct vdef_coded_frame frame_info;

	if (!frame || !decoder)
		return false;

	ret = mbuf_coded_video_frame_get_frame_info(frame, &frame_info);
	if (ret != 0)
		return false;

	ret = decoder->ops->get_supported_input_formats(&supported_formats);
	if (ret < 0)
		return false;
	accept = vdec_default_input_filter_internal(
		decoder, frame, &frame_info, supported_formats, ret);
	if (accept)
		vdec_default_input_filter_internal_confirm_frame(
			decoder, frame, &frame_info);
	return accept;
}


bool vdec_default_input_filter_internal(
	struct vdec_decoder *decoder,
	struct mbuf_coded_video_frame *frame,
	struct vdef_coded_frame *frame_info,
	const struct vdef_coded_format *supported_formats,
	unsigned int nb_supported_formats)
{
	if (!vdef_coded_format_intersect(&frame_info->format,
					 supported_formats,
					 nb_supported_formats)) {
		ULOG_ERRNO(
			"unsupported format:"
			" " VDEF_CODED_FORMAT_TO_STR_FMT,
			EPROTO,
			VDEF_CODED_FORMAT_TO_STR_ARG(&frame_info->format));
		return false;
	}

	if (frame_info->info.timestamp <= decoder->last_timestamp &&
	    decoder->last_timestamp != UINT64_MAX) {
		ULOG_ERRNO("non-strictly-monotonic timestamp (%" PRIu64
			   " <= %" PRIu64 ")",
			   EPROTO,
			   frame_info->info.timestamp,
			   decoder->last_timestamp);
		return false;
	}

	return true;
}


void vdec_default_input_filter_internal_confirm_frame(
	struct vdec_decoder *decoder,
	struct mbuf_coded_video_frame *frame,
	struct vdef_coded_frame *frame_info)
{
	int err;
	uint64_t ts_us;
	struct timespec cur_ts = {0, 0};

	/* Save frame timestamp to last_timestamp */
	decoder->last_timestamp = frame_info->info.timestamp;

	/* Set the input time ancillary data to the frame */
	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);
	err = mbuf_coded_video_frame_add_ancillary_buffer(
		frame, VDEC_ANCILLARY_KEY_INPUT_TIME, &ts_us, sizeof(ts_us));
	if (err < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -err);
}


int vdec_copy_coded_frame_as_metadata(struct mbuf_coded_video_frame *frame,
				      struct mbuf_mem *mem,
				      struct mbuf_coded_video_frame **ret_obj)
{
	int ret = 0, err;
	struct vdef_coded_frame info;
	struct vmeta_frame *metadata = NULL;
	struct mbuf_coded_video_frame *meta_frame = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mem == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(ret_obj == NULL, EINVAL);

	ret = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		return ret;
	}

	ret = mbuf_coded_video_frame_new(&info, &meta_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_new", -ret);
		return ret;
	}

	/* libmedia-buffers does not accept to finalize a mbuf_coded_video_frame
	 * that does not provide any NALU */
	struct vdef_nalu nalu = {
		.size = 0,
	};
	ret = mbuf_coded_video_frame_add_nalu(meta_frame, mem, 0, &nalu);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -ret);
		goto failure;
	}

	/* Copy ancillary data */
	ret = mbuf_coded_video_frame_foreach_ancillary_data(
		frame,
		mbuf_coded_video_frame_ancillary_data_copier,
		meta_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_foreach_ancillary_data",
			   -ret);
		goto failure;
	}

	/* Frame metadata */
	ret = mbuf_coded_video_frame_get_metadata(frame, &metadata);
	if (ret == 0) {
		ret = mbuf_coded_video_frame_set_metadata(meta_frame, metadata);
		vmeta_frame_unref(metadata);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_set_metadata", -ret);
			goto failure;
		}
	} else if (ret != -ENOENT) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_metadata", -ret);
		goto failure;
	}

	ret = mbuf_coded_video_frame_finalize(meta_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize", -ret);
		goto failure;
	}

	*ret_obj = meta_frame;

failure:
	if ((ret < 0) && (meta_frame)) {
		err = mbuf_coded_video_frame_unref(meta_frame);
		if (err < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_unref", -ret);
	}

	return ret;
}

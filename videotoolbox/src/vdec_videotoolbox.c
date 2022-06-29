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

#define ULOG_TAG vdec_videotoolbox
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);


#include "vdec_videotoolbox_priv.h"


#define NB_SUPPORTED_FORMATS 2
static struct vdef_coded_format supported_formats[NB_SUPPORTED_FORMATS];
static pthread_once_t supported_formats_is_init = PTHREAD_ONCE_INIT;
static void initialize_supported_formats(void)
{
	supported_formats[0] = vdef_h264_avcc;
	supported_formats[1] = vdef_h265_hvcc;
}


static void flush_complete(struct vdec_videotoolbox *self)
{
	vdec_call_flush_cb(self->base);
	self->need_sync = 1;
}


static void stop_complete(struct vdec_videotoolbox *self)
{
	vdec_call_stop_cb(self->base);
}


static void decoder_error(struct vdec_videotoolbox *self, int error)
{
	vdec_call_frame_output_cb(self->base, error, NULL);
}


static void mbox_cb(int fd, uint32_t revents, void *userdata)
{
	struct vdec_videotoolbox *self = userdata;
	int ret;
	struct vdec_videotoolbox_message message;

	do {
		/* Read from the mailbox */
		ret = mbox_peek(self->mbox, &message);
		if (ret < 0) {
			if (ret != -EAGAIN)
				ULOG_ERRNO("mbox_peek", -ret);
			break;
		}

		switch (message.type) {
		case VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH:
			flush_complete(self);
			break;
		case VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_STOP:
			stop_complete(self);
			break;
		case VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_ERROR:
			decoder_error(self, message.error);
			break;
		default:
			ULOGE("unknown message type: %d", message.type);
			break;
		}
	} while (ret == 0);
}


static void out_queue_evt_cb(struct pomp_evt *evt, void *userdata)
{
	struct vdec_videotoolbox *self = userdata;
	struct mbuf_raw_video_frame *frame;
	int ret;

	do {
		ret = mbuf_raw_video_frame_queue_pop(self->out_queue, &frame);
		if (ret == -EAGAIN) {
			return;
		} else if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_pop", -ret);
			return;
		}
		vdec_call_frame_output_cb(self->base, 0, frame);
		mbuf_raw_video_frame_unref(frame);
	} while (ret == 0);
}


static CFDataRef vdec_videotoolbox_avcc_create(const uint8_t *sps,
					       size_t sps_size,
					       const uint8_t *pps,
					       size_t pps_size)
{
	CFDataRef data_ref = NULL;
	uint8_t *data = NULL;
	size_t size = 0, offset = 0;

	size = 8 + sps_size + 3 + pps_size;
	data = malloc(size);
	if (data == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		return NULL;
	}

	/* See ISO/IEC 14496-15 chap 5.3.3.1 */
	data[offset++] = 1; /* version */
	data[offset++] = sps[1]; /* profile */
	data[offset++] = sps[2]; /* profile_compat */
	data[offset++] = sps[3]; /* level */
	data[offset++] = 0xFF; /* 6 bits reserved + 2 bits NALU size len */
	data[offset++] = 0xE1; /* 3 bits reserved + 5 bits SPS count */
	data[offset++] = (sps_size >> 8) & 0xFF; /* SPS size (high) */
	data[offset++] = (sps_size >> 0) & 0xFF; /* SPS size (low) */
	memcpy(data + offset, sps, sps_size); /* SPS */
	offset += sps_size;
	data[offset++] = 1; /* PPS count */
	data[offset++] = (pps_size >> 8) & 0xFF; /* PPS size (high) */
	data[offset++] = (pps_size >> 0) & 0xFF; /* PPS size (low) */
	memcpy(data + offset, pps, pps_size); /* PPS */
	offset += pps_size;

	data_ref = CFDataCreate(kCFAllocatorDefault, data, size);
	free(data);

	return data_ref;
}


static CFDataRef vdec_videotoolbox_hvcc_create(const uint8_t *vps,
					       size_t vps_size,
					       const uint8_t *sps,
					       size_t sps_size,
					       const uint8_t *pps,
					       size_t pps_size)
{
	CFDataRef data_ref = NULL;
	uint8_t *data = NULL;
	size_t size = 0, offset = 0;
	uint8_t version = 1;
	uint8_t general_profile_space = 0;
	uint8_t general_tier_flag = 0;
	uint8_t general_profile_idc = 1;
	uint32_t general_profile_compatibility_flags = 0x60000000;
	uint64_t general_constraint_indicator_flags = 0;
	uint8_t general_level_idc = 0;
	uint16_t min_spatial_segmentation_idc = 0;
	uint8_t parallelism_type = 0;
	uint8_t chroma_format = 0;
	uint8_t bit_depth_luma_minus8 = 0;
	uint8_t bit_depth_chroma_minus8 = 0;
	uint16_t avg_framerate = 0;
	uint8_t constant_framerate = 0;
	uint8_t num_temporeral_layers = 0;
	uint8_t temporal_id_nested = 0;
	uint8_t length_size_minus1 = 3;
	uint8_t num_of_arrays = 3;
	uint8_t array_completeness = 1;
	uint16_t num_nalus = 1;

	size = 23 + 5 * 3 + vps_size + sps_size + pps_size;
	data = calloc(size, sizeof(*data));
	if (data == NULL) {
		ULOG_ERRNO("malloc", ENOMEM);
		return NULL;
	}

	/* See ISO/IEC 14496-15 chap 8.3.3.1.2 */
	data[offset++] = version;
	data[offset++] = general_profile_space << 6 | general_tier_flag << 5 |
			 general_profile_idc;
	data[offset++] = (general_profile_compatibility_flags >> 24) & 0xFF;
	data[offset++] = (general_profile_compatibility_flags >> 16) & 0xFF;
	data[offset++] = (general_profile_compatibility_flags >> 8) & 0xFF;
	data[offset++] = (general_profile_compatibility_flags >> 0) & 0xFF;
	data[offset++] = (general_constraint_indicator_flags >> 40) & 0xFF;
	data[offset++] = (general_constraint_indicator_flags >> 32) & 0xFF;
	data[offset++] = (general_constraint_indicator_flags >> 24) & 0xFF;
	data[offset++] = (general_constraint_indicator_flags >> 16) & 0xFF;
	data[offset++] = (general_constraint_indicator_flags >> 8) & 0xFF;
	data[offset++] = (general_constraint_indicator_flags >> 0) & 0xFF;
	data[offset++] = general_level_idc;
	data[offset++] = 0xF0 | ((min_spatial_segmentation_idc >> 8) & 0x0F);
	data[offset++] = ((min_spatial_segmentation_idc >> 0) & 0x0F);
	data[offset++] = 0xFC | (parallelism_type & 0x03);
	data[offset++] = 0xFC | (chroma_format & 0x03);
	data[offset++] = 0xF8 | (bit_depth_luma_minus8 & 0x07);
	data[offset++] = 0xF8 | (bit_depth_chroma_minus8 & 0x07);
	data[offset++] = (avg_framerate >> 8) & 0xFF;
	data[offset++] = (avg_framerate >> 0) & 0xFF;
	data[offset++] = constant_framerate << 6 | num_temporeral_layers << 5 |
			 temporal_id_nested << 2 | length_size_minus1;
	data[offset++] = num_of_arrays;

	/* VPS array slot */
	data[offset++] = array_completeness << 7 | H265_NALU_TYPE_VPS_NUT;
	data[offset++] = (num_nalus >> 8) & 0xFF;
	data[offset++] = (num_nalus >> 0) & 0xFF;
	data[offset++] = (vps_size >> 8) & 0xFF;
	data[offset++] = (vps_size >> 0) & 0xFF;
	memcpy(data + offset, vps, vps_size);
	offset += vps_size;

	/* SPS array slot */
	data[offset++] = array_completeness << 7 | H265_NALU_TYPE_SPS_NUT;
	data[offset++] = (num_nalus >> 8) & 0xFF;
	data[offset++] = (num_nalus >> 0) & 0xFF;
	data[offset++] = (sps_size >> 8) & 0xFF;
	data[offset++] = (sps_size >> 0) & 0xFF;
	memcpy(data + offset, sps, sps_size);
	offset += sps_size;

	/* PPS array slot */
	data[offset++] = array_completeness << 7 | H265_NALU_TYPE_PPS_NUT;
	data[offset++] = (num_nalus >> 8) & 0xFF;
	data[offset++] = (num_nalus >> 0) & 0xFF;
	data[offset++] = (pps_size >> 8) & 0xFF;
	data[offset++] = (pps_size >> 0) & 0xFF;
	memcpy(data + offset, pps, pps_size);
	offset += pps_size;

	data_ref = CFDataCreate(kCFAllocatorDefault, data, size);
	free(data);

	return data_ref;
}


static CFMutableDictionaryRef
vdec_videotoolbox_decoder_config_create(const uint8_t *vps,
					size_t vps_size,
					const uint8_t *sps,
					size_t sps_size,
					const uint8_t *pps,
					size_t pps_size,
					enum vdef_encoding video_codec)
{
	int err = 0;
	CFMutableDictionaryRef decoder_config = NULL, codec_config = NULL;
	CFDataRef codec_data = NULL;

	decoder_config =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  0,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (decoder_config == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

#if !TARGET_OS_IPHONE
	CFDictionarySetValue(
		decoder_config,
		/* codecheck_ignore[LONG_LINE] */
		kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder,
		kCFBooleanTrue);
#endif /* !TARGET_OS_IPHONE */

	codec_config =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  1,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (codec_config == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

	switch (video_codec) {
	case VDEF_ENCODING_H264:
		codec_data = vdec_videotoolbox_avcc_create(
			sps, sps_size, pps, pps_size);
		if (codec_data == NULL) {
			err = -ENOMEM;
			ULOG_ERRNO("vdec_videotoolbox_avcc_create", -err);
			goto out;
		}
		CFDictionarySetValue(codec_config, CFSTR("avcC"), codec_data);
		break;
	case VDEF_ENCODING_H265:
		codec_data = vdec_videotoolbox_hvcc_create(
			vps, vps_size, sps, sps_size, pps, pps_size);
		if (codec_data == NULL) {
			err = -ENOMEM;
			ULOG_ERRNO("vdec_videotoolbox_hvcc_create", -err);
			goto out;
		}
		CFDictionarySetValue(codec_config, CFSTR("hvcC"), codec_data);
		break;
	default:
		break;
	}
	CFDictionarySetValue(
		decoder_config,
		kCMFormatDescriptionExtension_SampleDescriptionExtensionAtoms,
		codec_config);

out:
	if (codec_data != NULL)
		CFRelease(codec_data);
	if (codec_config != NULL)
		CFRelease(codec_config);
	if ((err != 0) && (decoder_config != NULL))
		CFRelease(decoder_config);
	return (err == 0) ? decoder_config : NULL;
}


static CFMutableDictionaryRef
vdec_videotoolbox_buffer_attr_create(unsigned int width,
				     unsigned int height,
				     int full_range)
{
	int err = 0;
	CFMutableDictionaryRef buffer_attr = NULL, io_surface_properties = NULL;
	CFNumberRef w = NULL, h = NULL, pix_fmt = NULL;
	int fmt = full_range ? kCVPixelFormatType_420YpCbCr8BiPlanarFullRange
			     : kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;

	buffer_attr =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  4,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (buffer_attr == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

	/* Empty means use default */
	io_surface_properties =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  0,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (io_surface_properties == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

	pix_fmt =
		CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &fmt);
	if (pix_fmt == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFNumberCreate", -err);
		goto out;
	}

	w = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &width);
	if (w == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFNumberCreate", -err);
		goto out;
	}

	h = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &height);
	if (h == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFNumberCreate", -err);
		goto out;
	}

	CFDictionarySetValue(buffer_attr,
			     kCVPixelBufferIOSurfacePropertiesKey,
			     io_surface_properties);
	CFDictionarySetValue(
		buffer_attr, kCVPixelBufferPixelFormatTypeKey, pix_fmt);
	CFDictionarySetValue(buffer_attr, kCVPixelBufferWidthKey, w);
	CFDictionarySetValue(buffer_attr, kCVPixelBufferHeightKey, h);
#if TARGET_OS_IPHONE
	CFDictionarySetValue(buffer_attr,
			     kCVPixelBufferOpenGLESCompatibilityKey,
			     kCFBooleanTrue);
#else /* TARGET_OS_IPHONE */
#	if 0
	/* TODO: this crashes when rendering */
	CFDictionarySetValue(buffer_attr,
		kCVPixelBufferIOSurfaceOpenGLTextureCompatibilityKey,
		kCFBooleanTrue);
#	endif
#endif /* TARGET_OS_IPHONE */

out:
	if (pix_fmt != NULL)
		CFRelease(pix_fmt);
	if (w != NULL)
		CFRelease(w);
	if (h != NULL)
		CFRelease(h);
	if (io_surface_properties != NULL)
		CFRelease(io_surface_properties);
	if ((err != 0) && (buffer_attr != NULL))
		CFRelease(buffer_attr);
	return (err == 0) ? buffer_attr : NULL;
}


static int vdec_videotoolbox_do_flush(struct vdec_videotoolbox *self)
{
	int ret;
	OSStatus osstatus;

	if (self->flush_discard) {
		/* Flush the queues */
		ret = mbuf_coded_video_frame_queue_flush(self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_queue_flush", -ret);
			return ret;
		}
		ret = mbuf_raw_video_frame_queue_flush(self->out_queue);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_flush", -ret);
			return ret;
		}
	}

	/* Flush the decoder */
	self->flushing = 1;
	if (self->base->configured) {
		osstatus = VTDecompressionSessionWaitForAsynchronousFrames(
			self->decompress_ref);
		if (osstatus != noErr && osstatus != kVTInvalidSessionErr) {
			ret = -EPROTO;
			ULOG_ERRNO(
				"VTDecompressionSessionWaitFor"
				"AsynchronousFrames status=%d",
				-ret,
				(int)osstatus);
			return ret;
		}
	}
	self->flushing = 0;

	/* Call the flush callback on the loop */
	struct vdec_videotoolbox_message message = {
		.type = VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH,
	};
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);

	self->flush = 0;

	return 0;
}


static void
vdec_videotoolbox_cvpb_mbuf_release(void *data, size_t len, void *userdata)
{
	CVPixelBufferRef ref = userdata;

	CVPixelBufferUnlockBaseAddress(ref, 0);
	CVPixelBufferRelease(ref);
}


static int
vdec_videotoolbox_set_frame_metadata(struct vdec_videotoolbox *self,
				     struct mbuf_coded_video_frame *in_frame,
				     struct mbuf_raw_video_frame **out_frame,
				     CVPixelBufferRef pixel_buffer_ref)
{
	int ret = 0;
	struct timespec cur_ts = {0, 0};
	uint64_t ts_us;
	OSType pixel_format;
	uint8_t *base;
	uint8_t *plane;
	struct mbuf_mem *out_mem = NULL;
	struct vmeta_frame *metadata = NULL;
	struct vdef_raw_frame out_info;
	struct vdef_coded_frame in_info;
	CVPixelBufferRef ref;
	OSStatus status;

	ret = mbuf_coded_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		goto out;
	}

	pixel_format = CVPixelBufferGetPixelFormatType(pixel_buffer_ref);
	out_info.info = in_info.info;
	switch (pixel_format) {
	case kCVPixelFormatType_420YpCbCr8Planar:
	case kCVPixelFormatType_420YpCbCr8PlanarFullRange:
		out_info.format = vdef_i420;
		out_info.plane_stride[0] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 0);
		out_info.plane_stride[1] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 1);
		out_info.plane_stride[2] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 2);
		break;
	case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
	case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
		out_info.format = vdef_nv12;
		out_info.plane_stride[0] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 0);
		out_info.plane_stride[1] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 1);
		break;
	default:
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output format", -ret);
		goto out;
	}
	out_info.info.bit_depth = self->base->video_info.bit_depth;
	out_info.info.full_range = self->base->video_info.full_range;
	out_info.info.color_primaries = self->base->video_info.color_primaries;
	out_info.info.transfer_function =
		self->base->video_info.transfer_function;
	out_info.info.matrix_coefs = self->base->video_info.matrix_coefs;
	out_info.info.resolution.width = self->base->video_info.crop.width;
	out_info.info.resolution.height = self->base->video_info.crop.height;
	out_info.info.sar = self->base->video_info.sar;

	ret = mbuf_raw_video_frame_new(&out_info, out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_new", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_foreach_ancillary_data(
		in_frame,
		mbuf_raw_video_frame_ancillary_data_copier,
		*out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_foreach_ancillary_data",
			   -ret);
		goto out;
	}

	ref = CVPixelBufferRetain(pixel_buffer_ref);

	status = CVPixelBufferLockBaseAddress(ref, 0);
	if (status != noErr) {
		ret = -EPROTO;
		goto out;
	}

	ret = mbuf_mem_generic_wrap(CVPixelBufferGetBaseAddress(ref),
				    CVPixelBufferGetDataSize(ref),
				    vdec_videotoolbox_cvpb_mbuf_release,
				    ref,
				    &out_mem);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_generic_wrap", -ret);
		goto out;
	}

	ret = mbuf_coded_video_frame_get_metadata(in_frame, &metadata);
	if (ret == 0) {
		ret = mbuf_raw_video_frame_set_metadata(*out_frame, metadata);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_metadata", -ret);
			goto out;
		}
	} else if (ret == -ENOENT) {
		ret = 0;
		/* No metadata, nothing to do */
	} else {
		ULOG_ERRNO("mbuf_coded_video_frame_get_metadata", -ret);
		goto out;
	}

	base = CVPixelBufferGetBaseAddress(pixel_buffer_ref);
	switch (pixel_format) {
	case kCVPixelFormatType_420YpCbCr8Planar:
	case kCVPixelFormatType_420YpCbCr8PlanarFullRange:
		plane = CVPixelBufferGetBaseAddressOfPlane(pixel_buffer_ref, 0);
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			0,
			out_mem,
			plane - base,
			out_info.plane_stride[0] *
				out_info.info.resolution.height);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]", -ret);
			goto out;
		}
		plane = CVPixelBufferGetBaseAddressOfPlane(pixel_buffer_ref, 1);
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			1,
			out_mem,
			plane - base,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[1]", -ret);
			goto out;
		}
		plane = CVPixelBufferGetBaseAddressOfPlane(pixel_buffer_ref, 2);
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			2,
			out_mem,
			plane - base,
			out_info.plane_stride[2] *
				out_info.info.resolution.height / 2);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[2]", -ret);
			goto out;
		}
		break;
	case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
	case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
		plane = CVPixelBufferGetBaseAddressOfPlane(pixel_buffer_ref, 0);
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			0,
			out_mem,
			plane - base,
			out_info.plane_stride[0] *
				out_info.info.resolution.height);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[0]", -ret);
			goto out;
		}
		plane = CVPixelBufferGetBaseAddressOfPlane(pixel_buffer_ref, 1);
		ret = mbuf_raw_video_frame_set_plane(
			*out_frame,
			1,
			out_mem,
			plane - base,
			out_info.plane_stride[1] *
				out_info.info.resolution.height / 2);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_set_plane[1]", -ret);
			goto out;
		}
		break;
	default:
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output format", -ret);
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_raw_video_frame_add_ancillary_buffer(
		*out_frame,
		VDEC_ANCILLARY_KEY_OUTPUT_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_add_ancillary_buffer", -ret);

out:
	if (ret < 0 && *out_frame) {
		mbuf_raw_video_frame_unref(*out_frame);
		*out_frame = NULL;
	}
	if (out_mem)
		mbuf_mem_unref(out_mem);
	if (metadata)
		vmeta_frame_unref(metadata);
	return ret;
}


static int
vdec_videotoolbox_insert_grey_idr(struct vdec_videotoolbox *self,
				  struct vdef_coded_frame *in_frame_info,
				  uint64_t *delta)
{
	int ret;
	struct mbuf_mem *idr_mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;
	uint64_t timestamp;
	size_t out_buf_size;
	const void *frame_data = NULL;
	size_t frame_len;
	CMSampleTimingInfo timing_info[1];
	OSStatus osstatus;
	CMBlockBufferRef block_buffer_ref = NULL;
	CMSampleBufferRef sample_buffer_ref = NULL;
	CFArrayRef attachments = NULL;
	CFMutableDictionaryRef dict = NULL;
	VTDecodeFrameFlags decode_flags = 0;
	VTDecodeInfoFlags info_flags = 0;

	ULOG_ERRNO_RETURN_ERR_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(in_frame_info == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(delta == NULL, EINVAL);

	/* Create buffer */
	out_buf_size = self->base->video_info.resolution.width *
		       self->base->video_info.resolution.height * 3 / 4;
	ret = mbuf_mem_generic_new(out_buf_size, &idr_mem);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_mem_generic_new", -ret);
		goto out;
	}

	timestamp = in_frame_info->info.timestamp;

	ret = vdec_h264_write_grey_idr(self->base,
				       in_frame_info,
				       delta,
				       &timestamp,
				       idr_mem,
				       &idr_frame);

	if (ret < 0) {
		ULOG_ERRNO("vdec_h264_write_grey_idr", -ret);
		goto out;
	}

	/* Create the sample buffer */
	ret = mbuf_coded_video_frame_get_packed_buffer(
		idr_frame, &frame_data, &frame_len);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer", -ret);
		goto out;
	}

	memset(timing_info, 0, sizeof(timing_info));
	timing_info[0].duration = kCMTimeInvalid;
	timing_info[0].presentationTimeStamp =
		CMTimeMake(timestamp, in_frame_info->info.timescale);
	timing_info[0].decodeTimeStamp = kCMTimeInvalid;

	osstatus = CMBlockBufferCreateWithMemoryBlock(kCFAllocatorDefault,
						      (void *)frame_data,
						      frame_len,
						      kCFAllocatorNull,
						      NULL,
						      0,
						      frame_len,
						      0,
						      &block_buffer_ref);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("CMBlockBufferCreateWithMemoryBlock status=%d",
			   -ret,
			   (int)osstatus);
		goto out;
	}

	osstatus = CMSampleBufferCreate(kCFAllocatorDefault,
					block_buffer_ref,
					TRUE,
					NULL,
					NULL,
					self->format_desc_ref,
					1,
					1,
					timing_info,
					0,
					NULL,
					&sample_buffer_ref);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO(
			"CMSampleBufferCreate status=%d", -ret, (int)osstatus);
		goto out;
	}

	attachments = CMSampleBufferGetSampleAttachmentsArray(sample_buffer_ref,
							      TRUE);
	dict = (CFMutableDictionaryRef)CFArrayGetValueAtIndex(attachments, 0);
	CFDictionarySetValue(dict,
			     kCMSampleAttachmentKey_DisplayImmediately,
			     kCFBooleanTrue);
	decode_flags |= kVTDecodeFrame_EnableAsynchronousDecompression;

	osstatus = VTDecompressionSessionDecodeFrame(self->decompress_ref,
						     sample_buffer_ref,
						     decode_flags,
						     idr_frame,
						     &info_flags);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("VTDecompressionSessionDecodeFrame status=%d",
			   -ret,
			   (int)osstatus);
		goto out;
	}

out:
	if (idr_mem)
		mbuf_mem_unref(idr_mem);
	if (block_buffer_ref != NULL)
		CFRelease(block_buffer_ref);
	if (sample_buffer_ref != NULL)
		CFRelease(sample_buffer_ref);
	if (ret != 0 && frame_data)
		mbuf_coded_video_frame_release_packed_buffer(idr_frame,
							     frame_data);
	if (ret != 0 && idr_frame)
		mbuf_coded_video_frame_unref(idr_frame);

	return ret;
}


static int
vdec_videotoolbox_buffer_push_one(struct vdec_videotoolbox *self,
				  struct mbuf_coded_video_frame *in_frame)
{
	int ret = 0;
	struct vdef_coded_frame in_info;
	struct timespec cur_ts = {0, 0};
	uint64_t delta = 0;
	uint64_t ts_us;
	CMBlockBufferRef block_buffer_ref = NULL;
	CMSampleBufferRef sample_buffer_ref = NULL;
	CFArrayRef attachments = NULL;
	CFMutableDictionaryRef dict = NULL;
	CMSampleTimingInfo timing_info[1];
	VTDecodeFrameFlags decode_flags = 0;
	VTDecodeInfoFlags info_flags = 0;
	OSStatus osstatus;
	const void *frame_data = NULL;
	size_t frame_len;

	ret = mbuf_coded_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info", -ret);
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &ts_us);

	ret = mbuf_coded_video_frame_add_ancillary_buffer(
		in_frame,
		VDEC_ANCILLARY_KEY_DEQUEUE_TIME,
		&ts_us,
		sizeof(ts_us));
	if (ret != 0)
		ULOG_ERRNO("mbuf_coded_video_frame_add_ancillary_buffer", -ret);

	/* Discard the frame if the decoder is not configured */
	if (!self->base->configured) {
		mbuf_coded_video_frame_unref(in_frame);
		return 0;
	}

	if (self->need_sync) {
		if (vdec_is_sync_frame(in_frame, &in_info)) {
			ULOGI("frame is a sync point");
			self->need_sync = 0;
		} else {
			if (self->base->config.encoding == VDEF_ENCODING_H264 &&
			    self->base->config.gen_grey_idr) {
				ULOGI("frame is not an IDR, "
				      "generating grey IDR");
				ret = vdec_videotoolbox_insert_grey_idr(
					self, &in_info, &delta);
				if (ret < 0) {
					ULOG_ERRNO(
						"vdec_videotoolbox_insert_grey_idr",
						-ret);
					goto out;
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
						  in_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_frame", -dbgret);
	}
	if (self->base->dbg.analysis != NULL) {
		int dbgret = vdec_dbg_parse_frame(self->base, in_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_frame", -dbgret);
	}

	/* Create the sample buffer */
	memset(timing_info, 0, sizeof(timing_info));
	timing_info[0].duration = kCMTimeInvalid;
	timing_info[0].presentationTimeStamp = CMTimeMake(
		in_info.info.timestamp + delta, in_info.info.timescale);
	timing_info[0].decodeTimeStamp = kCMTimeInvalid;

	ret = mbuf_coded_video_frame_get_packed_buffer(
		in_frame, &frame_data, &frame_len);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_buffer", -ret);
		goto out;
	}

	osstatus = CMBlockBufferCreateWithMemoryBlock(kCFAllocatorDefault,
						      (void *)frame_data,
						      frame_len,
						      kCFAllocatorNull,
						      NULL,
						      0,
						      frame_len,
						      0,
						      &block_buffer_ref);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("CMBlockBufferCreateWithMemoryBlock status=%d",
			   -ret,
			   (int)osstatus);
		goto out;
	}

	osstatus = CMSampleBufferCreate(kCFAllocatorDefault,
					block_buffer_ref,
					TRUE,
					NULL,
					NULL,
					self->format_desc_ref,
					1,
					1,
					timing_info,
					0,
					NULL,
					&sample_buffer_ref);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO(
			"CMSampleBufferCreate status=%d", -ret, (int)osstatus);
		goto out;
	}

	attachments = CMSampleBufferGetSampleAttachmentsArray(sample_buffer_ref,
							      TRUE);
	dict = (CFMutableDictionaryRef)CFArrayGetValueAtIndex(attachments, 0);
	CFDictionarySetValue(dict,
			     kCMSampleAttachmentKey_DisplayImmediately,
			     kCFBooleanTrue);
	decode_flags |= kVTDecodeFrame_EnableAsynchronousDecompression;

	/* Push the frame */
	osstatus = VTDecompressionSessionDecodeFrame(self->decompress_ref,
						     sample_buffer_ref,
						     decode_flags,
						     in_frame,
						     &info_flags);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("VTDecompressionSessionDecodeFrame status=%d",
			   -ret,
			   (int)osstatus);
		goto out;
	}

out:
	if (block_buffer_ref != NULL)
		CFRelease(block_buffer_ref);
	if (sample_buffer_ref != NULL)
		CFRelease(sample_buffer_ref);
	if (ret != 0) {
		if (frame_data)
			mbuf_coded_video_frame_release_packed_buffer(
				in_frame, frame_data);
		mbuf_coded_video_frame_unref(in_frame);
	}
	return ret;
}


static void vdec_videotoolbox_frame_output_cb(void *decompress_output_ref_con,
					      void *source_frame_ref_con,
					      OSStatus status,
					      VTDecodeInfoFlags info_flags,
					      CVImageBufferRef image_buffer,
					      CMTime presentation_timestamp,
					      CMTime presentation_duration)
{
	int ret;
	struct vdec_videotoolbox *self =
		(struct vdec_videotoolbox *)decompress_output_ref_con;
	struct mbuf_coded_video_frame *in_frame = source_frame_ref_con;
	struct vdef_coded_frame in_info;
	struct mbuf_raw_video_frame *out_frame = NULL;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (status == kVTVideoDecoderBadDataErr) {
		ret = -EBADMSG;
		ULOG_ERRNO("decoder error %d (bad data), resync required",
			   -ret,
			   (int)status);
		/* Call the frame callback with an error on the loop */
		struct vdec_videotoolbox_message message = {
			.type = VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_ERROR,
			.error = ret,
		};
		ret = mbox_push(self->mbox, &message);
		if (ret < 0)
			ULOG_ERRNO("mbox_push", -ret);
		goto out;
	} else if (status != noErr) {
		ULOG_ERRNO("decoder error %d", EPROTO, (int)status);
		goto out;
	}

	ULOG_ERRNO_RETURN_IF(image_buffer == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(in_frame == NULL, EINVAL);

	if (!self->base->configured)
		goto out;

	/* Discard the buffer when flushing with frames discarding */
	if ((self->flushing) && (self->flush_discard)) {
		ULOGI("frame discarded (flushing)"); /* TODO: debug */
		goto out;
	}

	ret = vdec_videotoolbox_set_frame_metadata(
		self, in_frame, &out_frame, (CVPixelBufferRef)image_buffer);
	if (ret < 0) {
		ULOG_ERRNO("vdec_videotoolbox_set_frame_metadata", -ret);
		goto out;
	}

	ret = mbuf_raw_video_frame_finalize(out_frame);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_finalize", -ret);
		goto out;
	}

	if (self->base->dbg.output_yuv != NULL) {
		int dbgret = vdec_dbg_write_yuv_frame(
			self->base->dbg.output_yuv, out_frame);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_yuv_frame", -dbgret);
	}

	ret = mbuf_coded_video_frame_get_frame_info(in_frame, &in_info);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_get_frame_info:decoder",
			   -ret);
		goto out;
	}

	/* Push the frame (if not silent) */
	if ((in_info.info.flags & VDEF_FRAME_FLAG_SILENT) &&
	    (!self->base->config.output_silent_frames)) {
		ULOGD("silent frame (ignored)");
	} else {
		ret = mbuf_raw_video_frame_queue_push(self->out_queue,
						      out_frame);
		if (ret < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_queue_push", -ret);
			goto out;
		}
	}

out:
	/* Unref the buffers */
	if (in_frame != NULL) {
		/* The packed buffer is still ref from the
		 * vdec_videotoolbox_buffer_push_one function, so get the
		 * pointer, and release it twice here */
		const void *frame_data;
		size_t frame_len;
		if (mbuf_coded_video_frame_get_packed_buffer(
			    in_frame, &frame_data, &frame_len) == 0) {
			mbuf_coded_video_frame_release_packed_buffer(
				in_frame, frame_data);
			mbuf_coded_video_frame_release_packed_buffer(
				in_frame, frame_data);
		}
		mbuf_coded_video_frame_unref(in_frame);
	}
	if (out_frame != NULL)
		mbuf_raw_video_frame_unref(out_frame);
}


static void input_event_cb(struct pomp_evt *evt, void *userdata)
{
	int ret;
	struct mbuf_coded_video_frame *frame;
	struct vdec_videotoolbox *self = userdata;

	ret = mbuf_coded_video_frame_queue_pop(self->in_queue, &frame);
	while (ret == 0) {
		/* Push the input frame */
		ret = vdec_videotoolbox_buffer_push_one(self, frame);
		if (ret < 0) {
			ULOG_ERRNO("vdec_videotoolbox_buffer_push_one", -ret);
			ret = 0;
			break;
		}
		ret = mbuf_coded_video_frame_queue_pop(self->in_queue, &frame);
	}
	if (ret != -EAGAIN)
		ULOG_ERRNO("mbuf_coded_video_frame_queue_pop", -ret);
	if (self->flush && ret == -EAGAIN) {
		/* Flush without discarding frames */
		ret = vdec_videotoolbox_do_flush(self);
		if (ret < 0)
			ULOG_ERRNO("vdec_videotoolbox_do_flush", -ret);
	}
}


static void *vdec_videotoolbox_decoder_thread(void *ptr)
{
	int ret;
	struct vdec_videotoolbox *self = ptr;
	struct pomp_loop *loop = NULL;
	struct pomp_evt *in_queue_evt = NULL;
	int timeout;

	switch (self->base->config.encoding) {
	case VDEF_ENCODING_H264:
		if (__builtin_available(iOS 11.0, macOS 10.13, tvos 11.0, *)) {
			ULOGI("Apple VideoToolbox H.264 decoding - "
			      "hardware acceleration %s supported",
			      VTIsHardwareDecodeSupported(
				      kCMVideoCodecType_H264)
				      ? "is"
				      : "is not");
		} else {
			ULOGI("Apple VideoToolbox H.264 decoding");
		}
		break;

	case VDEF_ENCODING_H265:
		if (__builtin_available(iOS 11.0, macOS 10.13, tvos 11.0, *)) {
			ULOGI("Apple VideoToolbox H.265 decoding - "
			      "hardware acceleration %s supported",
			      VTIsHardwareDecodeSupported(
				      kCMVideoCodecType_HEVC)
				      ? "is"
				      : "is not");
		} else {
			ULOGI("Apple VideoToolbox H.265 decoding");
		}
		break;
	default:
		break;
	}

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

	while ((!self->should_stop) || (self->flush)) {
		/* Flush discarding all frames */
		if ((self->flush) && (self->flush_discard)) {
			ret = vdec_videotoolbox_do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_videotoolbox_do_flush", -ret);
			continue;
		}

		timeout = ((self->flush) && (!self->flush_discard)) ? 0 : 5;

		ret = pomp_loop_wait_and_process(loop, timeout);
		if (ret < 0 && ret != -ETIMEDOUT) {
			ULOG_ERRNO("pomp_loop_wait_and_process", -ret);
			if (!self->should_stop) {
				/* Avoid looping on errors */
				usleep(5000);
			}
			continue;
		}
	}

	/* Call the stop callback on the loop */
	struct vdec_videotoolbox_message message = {
		.type = VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_STOP,
	};
	ret = mbox_push(self->mbox, &message);
	if (ret < 0)
		ULOG_ERRNO("mbox_push", -ret);

exit:
	if (in_queue_evt != NULL) {
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


static int stop(struct vdec_decoder *base)
{
	struct vdec_videotoolbox *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	/* Stop the decoding thread */
	self->should_stop = 1;
	base->configured = 0;

	return 0;
}


static int destroy(struct vdec_decoder *base)
{
	int err;
	struct vdec_videotoolbox *self;
	OSStatus osstatus;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the output thread */
	err = stop(base);
	if (err < 0)
		ULOG_ERRNO("vdec_videotoolbox_stop", -err);
	if (self->decompress_ref) {
		osstatus = VTDecompressionSessionWaitForAsynchronousFrames(
			self->decompress_ref);
		if (osstatus != noErr) {
			ULOG_ERRNO(
				"VTDecompressionSessionWaitFor"
				"AsynchronousFrames status=%d",
				EPROTO,
				(int)osstatus);
		}
	}
	if (self->thread_launched) {
		err = pthread_join(self->thread, NULL);
		if (err != 0)
			ULOG_ERRNO("pthread_join", err);
	}

	/* Free the resources */
	if (self->out_queue_evt != NULL) {
		err = pomp_evt_detach_from_loop(self->out_queue_evt,
						base->loop);
		if (err < 0)
			ULOG_ERRNO("pomp_evt_detach_from_loop", -err);
	}
	if (self->out_queue != NULL) {
		err = mbuf_raw_video_frame_queue_destroy(self->out_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_queue_destroy", -err);
	}
	if (self->in_queue != NULL) {
		err = mbuf_coded_video_frame_queue_destroy(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_queue_destroy",
				   -err);
	}
	if (self->mbox != NULL) {
		err = pomp_loop_remove(base->loop,
				       mbox_get_read_fd(self->mbox));
		if (err < 0)
			ULOG_ERRNO("pomp_loop_remove", -err);
		mbox_destroy(self->mbox);
	}
	if (self->decompress_ref) {
		VTDecompressionSessionInvalidate(self->decompress_ref);
		CFRelease(self->decompress_ref);
	}
	if (self->format_desc_ref)
		CFRelease(self->format_desc_ref);
	free(self);

	return 0;
}


static bool input_filter(struct mbuf_coded_video_frame *frame, void *userdata)
{
	const void *tmp;
	size_t tmplen;
	struct vdef_coded_frame info;
	struct vdec_videotoolbox *self = userdata;
	int ret;

	ret = mbuf_coded_video_frame_get_frame_info(frame, &info);
	if (ret != 0)
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
	struct vdec_videotoolbox *self = NULL;
	struct mbuf_coded_video_frame_queue_args queue_args = {
		.filter = input_filter,
	};

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	/* Check the configuration */
	if (base->config.encoding != VDEF_ENCODING_H264 &&
	    base->config.encoding != VDEF_ENCODING_H265) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid encoding", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = self;
	queue_args.filter_userdata = self;

	switch (base->config.encoding) {
	case VDEF_ENCODING_H264:
		self->need_sync = 1;
		break;
	default:
		break;
	}

	/* Initialize the mailbox for inter-thread messages  */
	self->mbox = mbox_new(sizeof(struct vdec_videotoolbox_message));
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

	/* Create the ouput buffers queue */
	ret = mbuf_raw_video_frame_queue_new(&self->out_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_new", -ret);
		goto error;
	}
	ret = mbuf_raw_video_frame_queue_get_event(self->out_queue,
						   &self->out_queue_evt);
	if (ret != 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_queue_get_event", -ret);
		goto error;
	}
	ret = pomp_evt_attach_to_loop(
		self->out_queue_evt, base->loop, &out_queue_evt_cb, self);
	if (ret < 0) {
		ULOG_ERRNO("pomp_evt_attach_to_loop", -ret);
		goto error;
	}

	/* Create the input buffers queue */
	ret = mbuf_coded_video_frame_queue_new_with_args(&queue_args,
							 &self->in_queue);
	if (ret < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_new_with_filter",
			   -ret);
		goto error;
	}

	/* Create the decoder thread */
	ret = pthread_create(&self->thread,
			     NULL,
			     vdec_videotoolbox_decoder_thread,
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
	struct vdec_videotoolbox *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = base->derived;

	self->flush = 1;
	self->flush_discard = discard;

	return 0;
}


static int set_ps(struct vdec_decoder *base,
		  const uint8_t *vps,
		  size_t vps_size,
		  const uint8_t *sps,
		  size_t sps_size,
		  const uint8_t *pps,
		  size_t pps_size,
		  const struct vdef_coded_format *format)
{
	int ret = 0;
	struct vdec_videotoolbox *self;
	const uint8_t *vps_tmp = NULL;
	size_t vps_size_tmp = 0;
	OSStatus osstatus;
	CFMutableDictionaryRef decoder_config = NULL, buffer_attr = NULL;
	VTDecompressionOutputCallbackRecord cb;
	CMVideoCodecType codec_type;

	ULOG_ERRNO_RETURN_ERR_IF(format == NULL, EINVAL);
	size_t offset = (format->data_format == VDEF_CODED_DATA_FORMAT_RAW_NALU)
				? 0
				: 4;
	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(
		(format->encoding != VDEF_ENCODING_H264) &&
			(format->encoding != VDEF_ENCODING_H265),
		EINVAL);
	if (format->encoding == VDEF_ENCODING_H265)
		ULOG_ERRNO_RETURN_ERR_IF((vps == NULL) || (vps_size <= offset),
					 EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);

	self = base->derived;

	switch (format->encoding) {
	case VDEF_ENCODING_H264:
		codec_type = kCMVideoCodecType_H264;
		break;
	case VDEF_ENCODING_H265:
		codec_type = kCMVideoCodecType_HEVC;
		vps_tmp = vps + offset;
		vps_size_tmp = vps_size - offset;
		break;
	default:
		break;
	}

	decoder_config =
		vdec_videotoolbox_decoder_config_create(vps_tmp,
							vps_size_tmp,
							sps + offset,
							sps_size - offset,
							pps + offset,
							pps_size - offset,
							format->encoding);
	if (decoder_config == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("vdec_videotoolbox_decoder_config_create", -ret);
		goto error;
	}

	osstatus = CMVideoFormatDescriptionCreate(kCFAllocatorDefault,
						  codec_type,
						  base->video_info.crop.width,
						  base->video_info.crop.height,
						  decoder_config,
						  &self->format_desc_ref);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("CMVideoFormatDescriptionCreate status=%d",
			   -ret,
			   (int)osstatus);
		goto error;
	}

	buffer_attr = vdec_videotoolbox_buffer_attr_create(
		base->video_info.crop.width,
		base->video_info.crop.height,
		base->video_info.full_range);
	if (buffer_attr == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("vdec_videotoolbox_buffer_attr_create", -ret);
		goto error;
	}

	cb.decompressionOutputCallback = &vdec_videotoolbox_frame_output_cb;
	cb.decompressionOutputRefCon = self;

	osstatus = VTDecompressionSessionCreate(kCFAllocatorDefault,
						self->format_desc_ref,
						NULL,
						buffer_attr,
						&cb,
						&self->decompress_ref);
	if (osstatus != noErr) {
		ret = -EPROTO;
		ULOG_ERRNO("VTDecompressionSessionCreate status=%d",
			   -ret,
			   (int)osstatus);
		goto error;
	}

	if (decoder_config != NULL)
		CFRelease(decoder_config);
	if (buffer_attr != NULL)
		CFRelease(buffer_attr);

	return 0;

error:
	if (self->decompress_ref != NULL) {
		VTDecompressionSessionInvalidate(self->decompress_ref);
		CFRelease(self->decompress_ref);
	}
	if (self->format_desc_ref != NULL)
		CFRelease(self->format_desc_ref);
	if (decoder_config != NULL)
		CFRelease(decoder_config);
	if (buffer_attr != NULL)
		CFRelease(buffer_attr);
	return ret;
}


static int set_h264_ps(struct vdec_decoder *base,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format)
{
	return set_ps(base, NULL, 0, sps, sps_size, pps, pps_size, format);
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
	return set_ps(
		base, vps, vps_size, sps, sps_size, pps, pps_size, format);
}


static struct mbuf_pool *get_input_buffer_pool(struct vdec_decoder *base)
{
	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


static struct mbuf_coded_video_frame_queue *
get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_videotoolbox *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = base->derived;

	return self->in_queue;
}


const struct vdec_ops vdec_videotoolbox_ops = {
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

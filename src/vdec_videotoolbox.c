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
#include "vdec_priv.h"
ULOG_DECLARE_TAG(vdec_videotoolbox);

#ifdef USE_VIDEOTOOLBOX

#	include <VideoToolbox/VideoToolbox.h>

#	include <video-buffers/vbuf_private.h>


#	define VDEC_VIDEOTOOLBOX_OUT_POOL_DEFAULT_MIN_BUF_COUNT 5
#	define VDEC_VIDEOTOOLBOX_BUF_TYPE_CVBUFFER 0x43564246 /* "CVBF" */


struct vdec_videotoolbox {
	struct vdec_decoder *base;
	struct vbuf_queue *in_queue;
	struct vbuf_pool *out_pool;
	CMVideoFormatDescriptionRef format_desc_ref;
	VTDecompressionSessionRef decompress_ref;
	int flushing;

	pthread_t thread;
	int thread_launched;
	int should_stop;
	int flush;
	int flush_discard;
};


struct vdec_videotoolbox_cvbuffer {
	CVBufferRef ref;
	CVPixelBufferLockFlags lock_flags;
	int cpu_locked;
};


static int vdec_videotoolbox_cvbuffer_set_buffer(struct vbuf_buffer *buf,
						 CVBufferRef ref,
						 int cpu_lock,
						 int read_only)
{
	struct vdec_videotoolbox_cvbuffer *cvbuf = NULL;
	OSStatus status;

	ULOG_ERRNO_RETURN_ERR_IF(
		buf->type != VDEC_VIDEOTOOLBOX_BUF_TYPE_CVBUFFER, EINVAL);

	cvbuf = (struct vdec_videotoolbox_cvbuffer *)buf->specific;

	if (cvbuf->ref) {
		if (cvbuf->cpu_locked) {
			status = CVPixelBufferUnlockBaseAddress(
				(CVPixelBufferRef)cvbuf->ref,
				cvbuf->lock_flags);
			if (status != noErr) {
				ULOG_ERRNO(
					"CVPixelBufferUnlockBaseAddress "
					"status=%d",
					EPROTO,
					(int)status);
			}
			cvbuf->cpu_locked = 0;
		}
		CVBufferRelease(cvbuf->ref);
		cvbuf->ref = NULL;
		buf->capacity = 0;
		buf->size = 0;
		buf->ptr = NULL;
	}

	if (ref) {
		cvbuf->ref = CVBufferRetain(ref);
		if (cpu_lock) {
			cvbuf->lock_flags =
				(read_only) ? kCVPixelBufferLock_ReadOnly : 0;
			status = CVPixelBufferLockBaseAddress(
				(CVPixelBufferRef)cvbuf->ref,
				cvbuf->lock_flags);
			if (status != noErr) {
				int ret = -EPROTO;
				ULOG_ERRNO(
					"CVPixelBufferLockBaseAddress "
					"status=%d",
					-ret,
					(int)status);
				CVBufferRelease(cvbuf->ref);
				cvbuf->ref = NULL;
				return ret;
			}
			cvbuf->cpu_locked = 1;
			buf->capacity = CVPixelBufferGetDataSize(
				(CVPixelBufferRef)cvbuf->ref);
			buf->size = buf->capacity;
			buf->ptr = CVPixelBufferGetBaseAddress(
				(CVPixelBufferRef)cvbuf->ref);
		}
	}

	return 0;
}


static int vdec_videotoolbox_cvbuffer_alloc_cb(struct vbuf_buffer *buf,
					       void *userdata)
{
	struct vdec_videotoolbox_cvbuffer *cvbuf = NULL;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	buf->type = VDEC_VIDEOTOOLBOX_BUF_TYPE_CVBUFFER;

	cvbuf = calloc(1, sizeof(struct vdec_videotoolbox_cvbuffer));
	if (cvbuf == NULL)
		return -ENOMEM;
	buf->specific = (struct vbuf_specific *)cvbuf;

	return 0;
}


static int vdec_videotoolbox_cvbuffer_free_cb(struct vbuf_buffer *buf,
					      void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	free(buf->specific);
	buf->specific = NULL;
	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static int vdec_videotoolbox_cvbuffer_unref_cb(struct vbuf_buffer *buf,
					       void *userdata)
{
	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);

	vdec_videotoolbox_cvbuffer_set_buffer(buf, NULL, 0, 0);

	buf->capacity = 0;
	buf->size = 0;
	buf->ptr = NULL;

	return 0;
}


static CFDataRef vdec_videotoolbox_avcc_create(const uint8_t *sps,
					       size_t sps_size,
					       const uint8_t *pps,
					       size_t pps_size)
{
	CFDataRef data_ref = NULL;
	uint8_t *data;
	size_t size, offset = 0;

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


static CFMutableDictionaryRef
vdec_videotoolbox_decoder_config_create(const uint8_t *sps,
					size_t sps_size,
					const uint8_t *pps,
					size_t pps_size)
{
	int err = 0;
	CFMutableDictionaryRef decoder_config = NULL, avc_config = NULL;
	CFDataRef avcc_data = NULL;

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

#	if !TARGET_OS_IPHONE
	CFDictionarySetValue(
		decoder_config,
		/* codecheck_ignore[LONG_LINE] */
		kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder,
		kCFBooleanTrue);
#	endif /* !TARGET_OS_IPHONE */

	avc_config =
		CFDictionaryCreateMutable(kCFAllocatorDefault,
					  1,
					  &kCFTypeDictionaryKeyCallBacks,
					  &kCFTypeDictionaryValueCallBacks);
	if (avc_config == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("CFDictionaryCreateMutable", -err);
		goto out;
	}

	avcc_data = vdec_videotoolbox_avcc_create(sps, sps_size, pps, pps_size);
	if (avcc_data == NULL) {
		err = -ENOMEM;
		ULOG_ERRNO("vdec_videotoolbox_avcc_create", -err);
		goto out;
	}

	CFDictionarySetValue(avc_config, CFSTR("avcC"), avcc_data);
	CFDictionarySetValue(
		decoder_config,
		kCMFormatDescriptionExtension_SampleDescriptionExtensionAtoms,
		avc_config);

out:
	if (avcc_data != NULL)
		CFRelease(avcc_data);
	if (avc_config != NULL)
		CFRelease(avc_config);
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
#	if TARGET_OS_IPHONE
	CFDictionarySetValue(buffer_attr,
			     kCVPixelBufferOpenGLESCompatibilityKey,
			     kCFBooleanTrue);
#	else /* TARGET_OS_IPHONE */
#		if 0
	/* TODO: this crashes when rendering */
	CFDictionarySetValue(buffer_attr,
		kCVPixelBufferIOSurfaceOpenGLTextureCompatibilityKey,
		kCFBooleanTrue);
#		endif
#	endif /* TARGET_OS_IPHONE */

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
		/* Flush the input queue */
		ret = vbuf_queue_flush(self->in_queue);
		if (ret < 0) {
			ULOG_ERRNO("vbuf_queue_flush:input", -ret);
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

	/* Call the flush callback if defined */
	if (self->base->cbs.flush)
		(*self->base->cbs.flush)(self->base, self->base->userdata);
	self->flush = 0;

	return 0;
}


static int
vdec_videotoolbox_set_frame_metadata(struct vdec_videotoolbox *self,
				     struct vbuf_buffer *in_buf,
				     struct vbuf_buffer *out_buf,
				     CVPixelBufferRef pixel_buffer_ref)
{
	int ret;
	struct vdec_input_metadata *in_meta = NULL;
	struct vdec_output_metadata *out_meta = NULL;
	const uint8_t *user_data;
	unsigned int level = 0, user_data_size;
	struct timespec cur_ts = {0, 0};
	OSType pixel_format;
	uint8_t *base;

	/* Frame metadata */
	ret = vbuf_metadata_get(
		in_buf, self->base, &level, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get:input", -ret);
		return ret;
	}
	ret = vbuf_metadata_copy(in_buf, out_buf, level);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_copy", -ret);
		return ret;
	}
	ret = vbuf_metadata_add(out_buf,
				self->base,
				level,
				sizeof(*out_meta),
				(uint8_t **)&out_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_add:output", -ret);
		return ret;
	}
	out_meta->timestamp = in_meta->timestamp;
	out_meta->index = in_meta->index;
	pixel_format = CVPixelBufferGetPixelFormatType(pixel_buffer_ref);
	base = CVPixelBufferGetBaseAddress(pixel_buffer_ref);
	switch (pixel_format) {
	case kCVPixelFormatType_420YpCbCr8Planar:
	case kCVPixelFormatType_420YpCbCr8PlanarFullRange:
		out_meta->format = VDEC_OUTPUT_FORMAT_I420;
		out_meta->plane_offset[0] =
			((uint8_t *)CVPixelBufferGetBaseAddressOfPlane(
				 pixel_buffer_ref, 0) -
			 base);
		out_meta->plane_offset[1] =
			((uint8_t *)CVPixelBufferGetBaseAddressOfPlane(
				 pixel_buffer_ref, 1) -
			 base);
		out_meta->plane_offset[2] =
			((uint8_t *)CVPixelBufferGetBaseAddressOfPlane(
				 pixel_buffer_ref, 2) -
			 base);
		out_meta->plane_stride[0] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 0);
		out_meta->plane_stride[1] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 1);
		out_meta->plane_stride[2] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 2);
		break;
	case kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange:
	case kCVPixelFormatType_420YpCbCr8BiPlanarFullRange:
		out_meta->format = VDEC_OUTPUT_FORMAT_NV12;
		out_meta->plane_offset[0] =
			((uint8_t *)CVPixelBufferGetBaseAddressOfPlane(
				 pixel_buffer_ref, 0) -
			 base);
		out_meta->plane_offset[1] =
			((uint8_t *)CVPixelBufferGetBaseAddressOfPlane(
				 pixel_buffer_ref, 1) -
			 base);
		out_meta->plane_stride[0] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 0);
		out_meta->plane_stride[1] =
			CVPixelBufferGetBytesPerRowOfPlane(pixel_buffer_ref, 1);
		break;
	default:
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported output chroma format", -ret);
		return ret;
	}
	out_meta->width = self->base->width;
	out_meta->height = self->base->height;
	out_meta->sar_width = self->base->sar_width;
	out_meta->sar_height = self->base->sar_height;
	out_meta->crop_left = self->base->crop_left;
	out_meta->crop_top = self->base->crop_top;
	out_meta->crop_width = self->base->crop_width;
	out_meta->crop_height = self->base->crop_height;
	out_meta->full_range = self->base->full_range;
	out_meta->errors = in_meta->errors;
	out_meta->silent = in_meta->silent;
	out_meta->userdata = in_meta->userdata;
	out_meta->input_time = in_meta->input_time;
	out_meta->dequeue_time = in_meta->dequeue_time;

	/* User data */
	user_data = vbuf_get_cuserdata(in_buf);
	user_data_size = vbuf_get_userdata_size(in_buf);
	if ((user_data) && (user_data_size > 0)) {
		ret = vbuf_set_userdata_capacity(out_buf, user_data_size);
		if (ret < (signed)user_data_size) {
			ULOG_ERRNO("vbuf_set_userdata_capacity", ENOMEM);
			vbuf_set_userdata_size(out_buf, 0);
		} else {
			uint8_t *dst = vbuf_get_userdata(out_buf);
			memcpy(dst, user_data, user_data_size);
			vbuf_set_userdata_size(out_buf, user_data_size);
		}
	} else {
		vbuf_set_userdata_size(out_buf, 0);
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &out_meta->output_time);

	if (self->base->dbg.output_yuv != NULL) {
		int dbgret = vdec_dbg_write_yuv_frame(
			self->base->dbg.output_yuv, out_buf, out_meta);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_yuv_frame", -dbgret);
	}

	return 0;
}


static int vdec_videotoolbox_buffer_push_one(struct vdec_videotoolbox *self,
					     struct vbuf_buffer *in_buf)
{
	int ret = 0;
	struct vdec_input_metadata *in_meta = NULL;
	struct timespec cur_ts = {0, 0};
	CMBlockBufferRef block_buffer_ref = NULL;
	CMSampleBufferRef sample_buffer_ref = NULL;
	CFArrayRef attachments = NULL;
	CFMutableDictionaryRef dict = NULL;
	CMSampleTimingInfo timing_info[1];
	VTDecodeFrameFlags decode_flags = 0;
	VTDecodeInfoFlags info_flags = 0;
	OSStatus osstatus;

	ret = vbuf_metadata_get(
		in_buf, self->base, NULL, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get:input", -ret);
		return ret;
	}
	if (in_meta->format != VDEC_INPUT_FORMAT_AVCC) {
		ret = -ENOSYS;
		ULOG_ERRNO("unsupported format: %s",
			   -ret,
			   vdec_input_format_str(in_meta->format));
		return ret;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &in_meta->dequeue_time);

	/* Discard the buffer if the decoder is not configured */
	if (!self->base->configured) {
		vbuf_unref(in_buf);
		return 0;
	}

	/* Debug files */
	if (self->base->dbg.input_bs != NULL) {
		int dbgret = vdec_dbg_write_h264_frame(
			self->base->dbg.input_bs, in_buf, in_meta->format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_write_h264_frame", -dbgret);
	}
	if (self->base->dbg.h264_analysis != NULL) {
		int dbgret = vdec_dbg_parse_h264_frame(
			self->base, in_buf, in_meta->format);
		if (dbgret < 0)
			ULOG_ERRNO("vdec_dbg_parse_h264_frame", -dbgret);
	}

	/* Create the sample buffer */
	memset(timing_info, 0, sizeof(timing_info));
	timing_info[0].duration = kCMTimeInvalid;
	timing_info[0].presentationTimeStamp =
		CMTimeMake(in_meta->timestamp, 1000000);
	timing_info[0].decodeTimeStamp = kCMTimeInvalid;

	osstatus = CMBlockBufferCreateWithMemoryBlock(
		kCFAllocatorDefault,
		(void *)vbuf_get_cdata(in_buf),
		vbuf_get_size(in_buf),
		kCFAllocatorNull,
		NULL,
		0,
		vbuf_get_size(in_buf),
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
	ret = vbuf_ref(in_buf);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_ref", -ret);
		goto out;
	}
	osstatus = VTDecompressionSessionDecodeFrame(self->decompress_ref,
						     sample_buffer_ref,
						     decode_flags,
						     in_buf,
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
	vbuf_unref(in_buf);
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
	struct vbuf_buffer *in_buf = (struct vbuf_buffer *)source_frame_ref_con;
	struct vbuf_buffer *out_buf = NULL;
	struct vdec_input_metadata *in_meta = NULL;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (status == kVTVideoDecoderBadDataErr) {
		ret = -EBADMSG;
		ULOG_ERRNO("decoder error %d (bad data), resync required",
			   -ret,
			   (int)status);
		(*self->base->cbs.frame_output)(
			self->base, ret, NULL, self->base->userdata);
		goto out;
	} else if (status != noErr) {
		ULOG_ERRNO("decoder error %d", EPROTO, (int)status);
		goto out;
	}

	ULOG_ERRNO_RETURN_IF(image_buffer == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(in_buf == NULL, EINVAL);

	ret = vbuf_metadata_get(
		in_buf, self->base, NULL, NULL, (uint8_t **)&in_meta);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_metadata_get:input", -ret);
		goto out;
	}

	if (!self->base->configured)
		goto out;

	/* Discard the buffer when flushing with frames discarding */
	if ((self->flushing) && (self->flush_discard)) {
		ULOGI("frame discarded (flushing)"); /* TODO: debug */
		goto out;
	}

	/* Get an output buffer (non-blocking) */
	ret = vbuf_pool_get(self->out_pool, 0, &out_buf);
	if ((ret < 0) || (out_buf == NULL)) {
		ULOGW("%s:%d: failed to get an output buffer "
		      "(frame dropped): %d(%s)",
		      __func__,
		      __LINE__,
		      -ret,
		      strerror(-ret));
		goto out;
	}
	ret = vdec_videotoolbox_cvbuffer_set_buffer(
		out_buf, image_buffer, 1, 1);
	if (ret < 0) {
		ULOG_ERRNO("vdec_videotoolbox_cvbuffer_set_buffer", -ret);
		goto out;
	}

	/* Set the metadata */
	ret = vdec_videotoolbox_set_frame_metadata(
		self, in_buf, out_buf, (CVPixelBufferRef)image_buffer);
	if (ret < 0) {
		ULOG_ERRNO("vdec_videotoolbox_set_frame_metadata", -ret);
		goto out;
	}

	/* Push the frame (if not silent) */
	ret = vbuf_write_lock(out_buf);
	if (ret < 0)
		ULOG_ERRNO("vbuf_write_lock", -ret);
	if ((in_meta->silent) && (!self->base->config.output_silent_frames)) {
		ULOGD("silent frame (ignored)");
	} else {
		(*self->base->cbs.frame_output)(
			self->base, 0, out_buf, self->base->userdata);
	}

out:
	/* Unref the buffers */
	if (in_buf != NULL)
		vbuf_unref(in_buf);
	if (out_buf != NULL)
		vbuf_unref(out_buf);
}


static void *vdec_videotoolbox_decoder_thread(void *ptr)
{
	int ret;
	struct vdec_videotoolbox *self = (struct vdec_videotoolbox *)ptr;
	struct vbuf_buffer *in_buf = NULL;
	int timeout;

	while ((!self->should_stop) || (self->flush)) {
		/* Flush discarding all frames */
		if ((self->flush) && (self->flush_discard)) {
			ret = vdec_videotoolbox_do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_videotoolbox_do_flush", -ret);
			continue;
		}

		/* Get an input buffer (with timeout) */
		timeout = ((self->flush) && (!self->flush_discard)) ? 0 : 5;
		ret = vbuf_queue_pop(self->in_queue, timeout, &in_buf);
		if (ret == -ETIMEDOUT) {
			continue;
		} else if ((self->flush) && (ret == -EAGAIN)) {
			/* Flush without discarding frames */
			ret = vdec_videotoolbox_do_flush(self);
			if (ret < 0)
				ULOG_ERRNO("vdec_videotoolbox_do_flush", -ret);
			continue;
		} else if ((ret < 0) || (in_buf == NULL)) {
			if (!self->should_stop) {
				ULOG_ERRNO("vbuf_queue_pop:input", -ret);
				/* Avoid looping on errors */
				usleep(5000);
			}
			continue;
		}

		/* Push the input frame */
		ret = vdec_videotoolbox_buffer_push_one(self, in_buf);
		if (ret < 0) {
			ULOG_ERRNO("vdec_videotoolbox_buffer_push_one", -ret);
			continue;
		}
	}

	/* Call the stop callback if defined */
	if (self->base->cbs.stop)
		(*self->base->cbs.stop)(self->base, self->base->userdata);

	return NULL;
}


uint32_t vdec_videotoolbox_get_supported_input_format(void)
{
	return VDEC_INPUT_FORMAT_AVCC;
}


int vdec_videotoolbox_new(struct vdec_decoder *base)
{
	int ret = 0;
	struct vdec_videotoolbox *self = NULL;
	struct vbuf_cbs out_buf_cbs;
	unsigned int out_buf_count;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	/* Check the configuration */
	if (base->config.encoding != VDEC_ENCODING_H264) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid encoding", -ret);
		return ret;
	}
	if ((!base->config.sync_decoding) && (base->cbs.frame_output == NULL)) {
		ret = -EINVAL;
		ULOG_ERRNO("invalid frame output callback", -ret);
		return ret;
	}
	if (base->config.sync_decoding) {
		ret = -EINVAL;
		ULOG_ERRNO("synchronized decoding is not supported", -ret);
		return ret;
	}

	self = calloc(1, sizeof(*self));
	if (self == NULL)
		return -ENOMEM;
	self->base = base;
	base->derived = (struct vdec_derived *)self;

	if (__builtin_available(iOS 11.0, macOS 10.13, tvos 11.0, *)) {
		ULOGI("Apple VideoToolbox H.264 decoding - "
		      "hardware acceleration %s supported",
		      VTIsHardwareDecodeSupported(kCMVideoCodecType_H264)
			      ? "is"
			      : "is not");
	} else {
		ULOGI("Apple VideoToolbox H.264 decoding");
	}

	/* Allocate the output buffers pool */
	memset(&out_buf_cbs, 0, sizeof(out_buf_cbs));
	out_buf_cbs.alloc = &vdec_videotoolbox_cvbuffer_alloc_cb;
	out_buf_cbs.free = &vdec_videotoolbox_cvbuffer_free_cb;
	out_buf_cbs.unref = &vdec_videotoolbox_cvbuffer_unref_cb;
	out_buf_count = VDEC_VIDEOTOOLBOX_OUT_POOL_DEFAULT_MIN_BUF_COUNT;
	if (base->config.preferred_min_out_buf_count > out_buf_count)
		out_buf_count = base->config.preferred_min_out_buf_count;
	ret = vbuf_pool_new(out_buf_count, 0, 0, &out_buf_cbs, &self->out_pool);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_pool_new:output", -ret);
		goto error;
	}

	/* Create the input buffers queue */
	ret = vbuf_queue_new(0, 0, &self->in_queue);
	if (ret < 0) {
		ULOG_ERRNO("vbuf_queue_new:input", -ret);
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
	vdec_videotoolbox_destroy(base);
	base->derived = NULL;
	return ret;
}


int vdec_videotoolbox_flush(struct vdec_decoder *base, int discard)
{
	struct vdec_videotoolbox *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = (struct vdec_videotoolbox *)base->derived;

	self->flush = 1;
	self->flush_discard = discard;

	return 0;
}


int vdec_videotoolbox_stop(struct vdec_decoder *base)
{
	int ret;
	struct vdec_videotoolbox *self;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = (struct vdec_videotoolbox *)base->derived;

	/* Stop the decoding thread */
	self->should_stop = 1;
	base->configured = 0;
	if (self->in_queue != NULL) {
		ret = vbuf_queue_abort(self->in_queue);
		if (ret < 0)
			ULOG_ERRNO("vbuf_queue_abort:input", -ret);
	}
	if (self->out_pool != NULL) {
		ret = vbuf_pool_abort(self->out_pool);
		if (ret < 0)
			ULOG_ERRNO("vbuf_pool_abort:output", -ret);
	}

	return 0;
}


int vdec_videotoolbox_destroy(struct vdec_decoder *base)
{
	int err;
	struct vdec_videotoolbox *self;
	OSStatus osstatus;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);

	self = (struct vdec_videotoolbox *)base->derived;
	if (self == NULL)
		return 0;

	/* Stop and join the output thread */
	err = vdec_videotoolbox_stop(base);
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
	if (self->out_pool != NULL) {
		err = vbuf_pool_abort(self->out_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_abort:output", -err);
		err = vbuf_pool_destroy(self->out_pool);
		if (err < 0)
			ULOG_ERRNO("vbuf_pool_destroy:output", -err);
	}
	if (self->in_queue != NULL) {
		err = vbuf_queue_abort(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_abort:input", -err);
		err = vbuf_queue_destroy(self->in_queue);
		if (err < 0)
			ULOG_ERRNO("vbuf_queue_destroy:input", -err);
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


int vdec_videotoolbox_set_sps_pps(struct vdec_decoder *base,
				  const uint8_t *sps,
				  size_t sps_size,
				  const uint8_t *pps,
				  size_t pps_size,
				  enum vdec_input_format format)
{
	int ret = 0;
	struct vdec_videotoolbox *self;
	size_t offset = (format == VDEC_INPUT_FORMAT_RAW_NALU) ? 0 : 4;
	OSStatus osstatus;
	CFMutableDictionaryRef decoder_config = NULL, buffer_attr = NULL;
	VTDecompressionOutputCallbackRecord cb;

	ULOG_ERRNO_RETURN_ERR_IF(base == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((sps == NULL) || (sps_size <= offset), EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF((pps == NULL) || (pps_size <= offset), EINVAL);

	self = (struct vdec_videotoolbox *)base->derived;

	decoder_config =
		vdec_videotoolbox_decoder_config_create(sps + offset,
							sps_size - offset,
							pps + offset,
							pps_size - offset);
	if (decoder_config == NULL) {
		ret = -ENOMEM;
		ULOG_ERRNO("vdec_videotoolbox_decoder_config_create", -ret);
		goto error;
	}

	osstatus = CMVideoFormatDescriptionCreate(kCFAllocatorDefault,
						  kCMVideoCodecType_H264,
						  base->crop_width,
						  base->crop_height,
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
		base->crop_width, base->crop_height, base->full_range);
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


struct vbuf_pool *
vdec_videotoolbox_get_input_buffer_pool(struct vdec_decoder *base)
{
	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	/* No input buffer pool allocated: use the application's */
	return NULL;
}


struct vbuf_queue *
vdec_videotoolbox_get_input_buffer_queue(struct vdec_decoder *base)
{
	struct vdec_videotoolbox *self;

	ULOG_ERRNO_RETURN_VAL_IF(base == NULL, EINVAL, NULL);

	self = (struct vdec_videotoolbox *)base->derived;

	/* Returns NULL in case we are in synchronized decoding mode */
	return self->in_queue;
}


int vdec_videotoolbox_sync_decode(struct vdec_decoder *base,
				  struct vbuf_buffer *in_buf,
				  struct vbuf_buffer **out_buf)
{
	/* Synchronized decoding is not supported */
	return -ENOSYS;
}

#endif /* USE_VIDEOTOOLBOX */

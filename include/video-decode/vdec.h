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

#ifndef _VDEC_H_
#define _VDEC_H_

#include <stdint.h>
#include <unistd.h>

#include <libpomp.h>
#include <video-decode/vdec_core.h>
#include <video-defs/vdefs.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef VDEC_API_EXPORTS
#	ifdef _WIN32
#		define VDEC_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define VDEC_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !VDEC_API_EXPORTS */
#	define VDEC_API
#endif /* !VDEC_API_EXPORTS */


/**
 * Get the supported input buffer data formats for the given
 * decoder implementation.
 * Each implementation supports at least one input format,
 * and optionally more. All input buffers need to be in one of
 * the supported formats, otherwise they will be discarded.
 * The returned formats array is a static array whose size is the return value
 * of this function. If this function returns an error (negative errno value),
 * then the value of *formats is undefined.
 * @param implem: decoder implementation
 * @param formats: pointer to the supported formats list (output)
 * @return the size of the formats array, or a negative errno on error.
 */
VDEC_API int
vdec_get_supported_input_formats(enum vdec_decoder_implem implem,
				 const struct vdef_coded_format **formats);


/**
 * Get the implementation that will be chosen in case VDEC_DECODER_IMPLEM_AUTO
 * is used.
 * @return the decoder implementation, or VDEC_DECODER_IMPLEM_AUTO in case of
 * error
 */
VDEC_API enum vdec_decoder_implem vdec_get_auto_implem(void);


/**
 * Get an implementation for a given coded format
 * @param format: coded format to support
 * @return the decoder implementation, or VDEC_DECODER_IMPLEM_AUTO in case of
 * error
 */
VDEC_API enum vdec_decoder_implem
vdec_get_auto_implem_by_coded_format(struct vdef_coded_format *format);

/**
 * Create a decoder instance.
 * The configuration and callbacks structures must be filled.
 * The instance handle is returned through the ret_obj parameter.
 * When no longer needed, the instance must be freed using the
 * vdec_destroy() function.
 * @param loop: event loop to use
 * @param config: decoder configuration
 * @param cbs: decoder callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: decoder instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_new(struct pomp_loop *loop,
		      const struct vdec_config *config,
		      const struct vdec_cbs *cbs,
		      void *userdata,
		      struct vdec_decoder **ret_obj);


/**
 * Flush the decoder.
 * This function flushes all queues and optionally discards all buffers
 * retained by the decoder. If the buffers are not discarded the frame
 * output callback is called for each frame when the decoding is complete.
 * The function is asynchronous and returns immediately. When flushing is
 * complete the flush callback function is called if defined. After flushing
 * the decoder new input buffers can still be queued but should start with a
 * synchronization frame (e.g. IDR frame or start of refresh).
 * @param self: decoder instance handle
 * @param discard: if null, all pending buffers are output, otherwise they
 *        are discarded
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_flush(struct vdec_decoder *self, int discard);


/**
 * Stop the decoder.
 * This function stops any running threads. The function is asynchronous and
 * returns immediately. When stopping is complete the stop callback function
 * is called if defined. After stopping the decoder no new input buffers
 * can be queued and the decoder instance must be freed using the
 * vdec_destroy() function.
 * @param self: decoder instance handle
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_stop(struct vdec_decoder *self);


/**
 * Free a decoder instance.
 * This function frees all resources associated with a decoder instance.
 * @note this function blocks until all internal threads (if any) can be
 * joined; therefore the application should call vdec_stop() and wait for
 * the stop callback function to be called before calling vdec_destroy().
 * @param self: decoder instance handle
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_destroy(struct vdec_decoder *self);


/**
 * Set the JPEG parameters for decoding.
 * This function must be called prior to decoding (i.e. pushing buffer into
 * the input queue). The ownership of the format_info struct stays
 * with the caller. It is the caller's responsibility to ensure that the
 * instance is configured to decode a MJPEG/JPEG stream.
 * @param self decoder instance handle
 * @param[in] format_info: image format information
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_set_jpeg_params(struct vdec_decoder *self,
				  const struct vdef_format_info *format_info);


/**
 * Set the H264 parameter sets for decoding.
 * This function must be called prior to decoding (i.e. pushing buffer into
 * the input queue) with the H.264 SPS and PPS. The SPS and PPS data will be
 * copied internally if necessary. The ownership of the SPS and PPS buffers
 * stays with the caller. It is the caller's responsibility to ensure that
 * the instance is configured to decode a H.264 stream.
 * @param self decoder instance handle
 * @param[in] sps: pointer to the SPS data
 * @param[in] sps_size: SPS size
 * @param[in] pps: pointer to the PPS data
 * @param[in] pps_size: PPS size
 * @param[in] format: SPS and PPS data format
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_set_h264_ps(struct vdec_decoder *self,
			      const uint8_t *sps,
			      size_t sps_size,
			      const uint8_t *pps,
			      size_t pps_size,
			      const struct vdef_coded_format *format);


/**
 * Set the H.265 parameter sets for decoding.
 * This function must be called prior to decoding (i.e. pushing buffers
 * into the input queue) with the H.265 VPS, SPS and
 * PPS. Ownership of buffers is retained by the caller. It is the caller's
 * responsibility to ensure that the instance is configured to decode a H.265
 * stream.
 * @param[in] self: decoder instance handle
 * @param[in] vps: VPS buffer pointer
 * @param[in] vps_size: VPS buffer size
 * @param[in] sps: SPS buffer pointer
 * @param[in] sps_size: SPS buffer size
 * @param[in] pps: PPS buffer pointer
 * @param[in] pps_size: PPS buffer size
 * @param[in] format: data format
 * @return 0 on success, negative errno value in case of error.
 */
VDEC_API int vdec_set_h265_ps(struct vdec_decoder *self,
			      const uint8_t *vps,
			      size_t vps_size,
			      const uint8_t *sps,
			      size_t sps_size,
			      const uint8_t *pps,
			      size_t pps_size,
			      const struct vdef_coded_format *format);


/**
 * Get the input buffer pool.
 * The input buffer pool is defined only for implementations that require
 * using input memories from the decoder's own pool. This function must
 * be called prior to decoding and if the returned value is not NULL the
 * input buffer pool should be used to get input memories. If the input
 * memories provided are not originating from the pool, they will be copied
 * resulting in a loss of performance.
 * @param self: decoder instance handle
 * @return a pointer on the input memory pool on success, NULL in case of
 * error or if no pool is used
 */
VDEC_API struct mbuf_pool *
vdec_get_input_buffer_pool(struct vdec_decoder *self);


/**
 * Get the input frame queue.
 * This function must be called prior to decoding and the input
 * frame queue must be used to push input frames for decoding.
 * @param self: decoder instance handle
 * @return a pointer on the input frame queue on success, NULL in case of error
 */
VDEC_API struct mbuf_coded_video_frame_queue *
vdec_get_input_buffer_queue(struct vdec_decoder *self);


/**
 * Get the video dimensions.
 * @param self: decoder instance handle
 * @param width: video width (output, optional, can be null)
 * @param height: video height (output, optional, can be null)
 * @param sar_width: video SAR width (output, optional, can be null)
 * @param sar_width: video SAR height (output, optional, can be null)
 * @param crop_left: video crop X coordinate (output, optional, can be null)
 * @param crop_top: video crop Y coordinate (output, optional, can be null)
 * @param crop_width: video crop width (output, optional, can be null)
 * @param crop_height: video crop height (output, optional, can be null)
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_get_video_dimensions(struct vdec_decoder *self,
				       unsigned int *width,
				       unsigned int *height,
				       unsigned int *sar_width,
				       unsigned int *sar_height,
				       unsigned int *crop_left,
				       unsigned int *crop_top,
				       unsigned int *crop_width,
				       unsigned int *crop_height);


/**
 * Get the decoder implementation used.
 * @param self: decoder instance handle
 * @return the decoder implementation used, or VDEC_DECODER_IMPLEM_AUTO
 * in case of error
 */
VDEC_API enum vdec_decoder_implem
vdec_get_used_implem(struct vdec_decoder *self);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VDEC_H_ */

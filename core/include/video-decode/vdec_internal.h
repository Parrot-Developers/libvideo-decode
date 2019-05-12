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

#ifndef _VDEC_INTERNAL_H_
#define _VDEC_INTERNAL_H_

#include <stdio.h>

#include <h264/h264.h>
#include <h265/h265.h>
#include <video-decode/vdec_core.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* To be used for all public API */
#ifdef VDEC_API_EXPORTS
#	ifdef _WIN32
#		define VDEC_INTERNAL_API __declspec(dllexport)
#	else /* !_WIN32 */
#		define VDEC_INTERNAL_API __attribute__((visibility("default")))
#	endif /* !_WIN32 */
#else /* !VDEC_API_EXPORTS */
#	define VDEC_INTERNAL_API
#endif /* !VDEC_API_EXPORTS */


struct vdec_ops {
	int (*get_supported_input_formats)(
		const struct vdef_coded_format **formats);

	int (*create)(struct vdec_decoder *base);

	int (*flush)(struct vdec_decoder *base, int discard);

	int (*stop)(struct vdec_decoder *base);

	int (*destroy)(struct vdec_decoder *base);

	int (*set_h264_ps)(struct vdec_decoder *base,
			   const uint8_t *sps,
			   size_t sps_size,
			   const uint8_t *pps,
			   size_t pps_size,
			   const struct vdef_coded_format *format);

	int (*set_h265_ps)(struct vdec_decoder *base,
			   const uint8_t *vps,
			   size_t vps_size,
			   const uint8_t *sps,
			   size_t sps_size,
			   const uint8_t *pps,
			   size_t pps_size,
			   const struct vdef_coded_format *format);

	struct mbuf_pool *(*get_input_buffer_pool)(struct vdec_decoder *base);

	struct mbuf_coded_video_frame_queue *(*get_input_buffer_queue)(
		struct vdec_decoder *base);
};


struct video_info {
	/* Picture resolution in pixels */
	struct vdef_dim resolution;

	uint8_t bit_depth;

	/* Source aspect ratio size (1 if unknown) */
	struct vdef_dim sar;

	/* Crop rectangle in pixels */
	struct vdef_rect crop;

	/* Full range flag */
	int full_range;

	/* Color primaries */
	enum vdef_color_primaries color_primaries;

	/* Transfer function */
	enum vdef_transfer_function transfer_function;

	/* Matrix coefficients */
	enum vdef_matrix_coefs matrix_coefs;

	/* Declared framerate from time_scale and num_units_in_tick
	 * (0 if unknown) */
	struct vdef_frac framerate;

	/* NAL HRD bitrate (0 if unknown) */
	uint32_t nal_hrd_bitrate;

	/* NAL HRD CPB size (0 if unknown) */
	uint32_t nal_hrd_cpb_size;

	/* VCL HRD bitrate (0 if unknown) */
	uint32_t vcl_hrd_bitrate;

	/* VCL HRD CPB size (0 if unknown) */
	uint32_t vcl_hrd_cpb_size;
};


struct vdec_decoder {
	void *derived;
	const struct vdec_ops *ops;
	struct pomp_loop *loop;
	struct vdec_cbs cbs;
	void *userdata;
	struct vdec_config config;
	int configured;
	struct video_info video_info;
	union {
		struct h264_reader *h264;
		struct h265_reader *h265;
	} reader;
	struct {
		char *dir;
		unsigned int frame_index;
		FILE *input_bs;
		FILE *output_yuv;
		FILE *analysis;
	} dbg;
	uint64_t last_timestamp;
};


VDEC_INTERNAL_API int
vdec_format_convert(struct mbuf_coded_video_frame *frame,
		    const struct vdef_coded_format *target_format);


VDEC_INTERNAL_API bool vdec_is_sync_frame(struct mbuf_coded_video_frame *frame,
					  struct vdef_coded_frame *info);

/**
 * Default filter for the input frame queue.
 * This function is intended to be used as a standalone input filter.
 * It will call vdec_default_input_filter_internal(), and then
 * vdec_default_input_filter_internal_confirm_frame() if the former returned
 * true.
 *
 * @param frame: The frame to filter.
 * @param userdata: The vdec_decoder structure.
 *
 * @return true if the frame passes the checks, false otherwise
 */
VDEC_API bool vdec_default_input_filter(struct mbuf_coded_video_frame *frame,
					void *userdata);

/**
 * Default filter for the input frame queue.
 * This filter does the following checks:
 * - frame is in a supported format
 * - frame timestamp is strictly monotonic
 * This version is intended to be used by custom filters, to avoid calls to
 * mbuf_coded_video_frame_get_frame_info() or get_supported_input_formats().
 *
 * @warning This function does NOT check input validity. Arguments must not be
 * NULL, except for supported_formats if nb_supported_formats is zero.
 *
 * @param decoder: The base video decoder.
 * @param frame: The frame to filter.
 * @param frame_info: The associated vdef_coded_frame.
 * @param supported_formats: The formats supported by the implementation.
 * @param nb_supported_formats: The size of the supported_formats array.
 *
 * @return true if the frame passes the checks, false otherwise
 */
VDEC_INTERNAL_API bool vdec_default_input_filter_internal(
	struct vdec_decoder *decoder,
	struct mbuf_coded_video_frame *frame,
	struct vdef_coded_frame *frame_info,
	const struct vdef_coded_format *supported_formats,
	unsigned int nb_supported_formats);

/**
 * Filter update function.
 * This function should be called at the end of a custom filter. It registers
 * that the frame was accepted. This function saves the frame timestamp for
 * monotonic checks, and sets the VDEC_ANCILLARY_KEY_INPUT_TIME ancillary data
 * on the frame.
 *
 * @param decoder: The base video decoder.
 * @param frame: The accepted frame.
 * @param frame_info: The associated vdef_coded_frame.
 */
VDEC_INTERNAL_API void vdec_default_input_filter_internal_confirm_frame(
	struct vdec_decoder *decoder,
	struct mbuf_coded_video_frame *frame,
	struct vdef_coded_frame *frame_info);


VDEC_INTERNAL_API bool vdec_h264_is_idr(struct mbuf_coded_video_frame *frame,
					struct vdef_coded_frame *info);


VDEC_INTERNAL_API
int vdec_h264_write_grey_idr(struct vdec_decoder *self,
			     struct vdef_coded_frame *in_frame_info,
			     uint64_t *delta,
			     uint64_t *timestamp,
			     struct mbuf_mem *idr_mem,
			     struct mbuf_coded_video_frame **idr_frame);


VDEC_INTERNAL_API bool vdec_h265_is_idr(struct mbuf_coded_video_frame *frame,
					struct vdef_coded_frame *info);


VDEC_INTERNAL_API int vdec_dbg_create_files(struct vdec_decoder *vdec);


VDEC_INTERNAL_API int vdec_dbg_close_files(struct vdec_decoder *vdec);


VDEC_INTERNAL_API int
vdec_dbg_write_h264_ps(FILE *file,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format);


VDEC_INTERNAL_API int
vdec_dbg_write_h265_ps(FILE *file,
		       const uint8_t *vps,
		       size_t vps_size,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format);


VDEC_INTERNAL_API
int vdec_dbg_write_frame(FILE *file, struct mbuf_coded_video_frame *frame);


VDEC_INTERNAL_API int
vdec_dbg_parse_h264_ps(struct vdec_decoder *vdec,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format);


VDEC_INTERNAL_API int
vdec_dbg_parse_h265_ps(struct vdec_decoder *vdec,
		       const uint8_t *vps,
		       size_t vps_size,
		       const uint8_t *sps,
		       size_t sps_size,
		       const uint8_t *pps,
		       size_t pps_size,
		       const struct vdef_coded_format *format);


VDEC_INTERNAL_API
int vdec_dbg_parse_frame(struct vdec_decoder *vdec,
			 struct mbuf_coded_video_frame *frame);


VDEC_INTERNAL_API int
vdec_dbg_write_yuv_frame(FILE *file, struct mbuf_raw_video_frame *frame);


VDEC_INTERNAL_API void vdec_dbg_h264_nalu_begin(struct vdec_decoder *self,
						struct h264_nalu_header nh,
						enum h264_nalu_type type);


VDEC_INTERNAL_API void vdec_dbg_h264_slice(struct vdec_decoder *self,
					   struct h264_nalu_header nh,
					   const struct h264_slice_header *sh);


VDEC_INTERNAL_API void vdec_dbg_h265_nalu_begin(struct vdec_decoder *self,
						enum h265_nalu_type type);


VDEC_INTERNAL_API struct vdec_config_impl *
vdec_config_get_specific(struct vdec_config *config,
			 enum vdec_decoder_implem implem);


/**
 * Copy coded frame as frame info, meta data and ancillary data
 * without copying the data related to the NALUs.
 * It prevents holding the input coded frames too long
 * on systems with memory constraint.
 * @param frame: Pointer to the frame object.
 * @param mem: Pointer to the memory.
 * @param ret_obj: [out] Pointer to the new frame object.
 * @return 0 on success, negative errno on error.
 */
VDEC_INTERNAL_API int
vdec_copy_coded_frame_as_metadata(struct mbuf_coded_video_frame *frame,
				  struct mbuf_mem *mem,
				  struct mbuf_coded_video_frame **ret_obj);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VDEC_INTERNAL_H_ */

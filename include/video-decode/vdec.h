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

#include <video-buffers/vbuf.h>

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


/* Debug files flags. Can be set in configuration as well as in
 * VDEC_DBG_FLAGS environment variables (with VDEC_DBG_DIR specifying the
 * output directory. */

/* Input bitstream */
#define VDEC_DBG_FLAG_INPUT_BITSTREAM (1 << 0)

/* Output YUV */
#define VDEC_DBG_FLAG_OUTPUT_YUV (1 << 1)

/* H.264 parsing and analysis */
#define VDEC_DBG_FLAG_H264_ANALYSIS (1 << 2)


/* Forward declarations */
struct vdec_decoder;


/* Supported decoder implementations */
enum vdec_decoder_implem {
	/* Automatically select decoder */
	VDEC_DECODER_IMPLEM_AUTO = 0,

	/* FFmpeg decoder implementation */
	VDEC_DECODER_IMPLEM_FFMPEG,

	/* Android MediaCodec decoder implementation */
	VDEC_DECODER_IMPLEM_MEDIACODEC,

	/* Apple VideoToolbox decoder implementation */
	VDEC_DECODER_IMPLEM_VIDEOTOOLBOX,

	/* Broadcom VideoCore MMAL decoder implementation */
	VDEC_DECODER_IMPLEM_VIDEOCOREMMAL,
};


/* Supported encodings */
enum vdec_encoding {
	/* ISO/IEC 14496-10 AVC / ITU-T H.264 */
	VDEC_ENCODING_H264 = 0,
};


/* Input buffers data format */
enum vdec_input_format {
	/* H.264 raw NAL units */
	VDEC_INPUT_FORMAT_RAW_NALU = (1 << 0),

	/* H.264 Annex B byte stream format */
	VDEC_INPUT_FORMAT_BYTE_STREAM = (1 << 1),

	/* AVCC format (4-bytes NALU length in network order) */
	VDEC_INPUT_FORMAT_AVCC = (1 << 2),
};


/* Output buffers data format */
enum vdec_output_format {
	/* Unknown data format (or 'any' for preferred_output_format
	 * in struct vdec_config) */
	VDEC_OUTPUT_FORMAT_UNKNOWN = 0,
	VDEC_OUTPUT_FORMAT_ANY = VDEC_OUTPUT_FORMAT_UNKNOWN,

	/* "I420" YUV 4:2:0 planar (3 planes, YUV order) */
	VDEC_OUTPUT_FORMAT_I420 = (1 << 0),

	/* "YV12" YUV 4:2:0 planar (3 planes, YVU order) */
	VDEC_OUTPUT_FORMAT_YV12 = (1 << 1),

	/* "NV12" YUV 4:2:0 semi-planar (2 planes, Y + interleaved UV) */
	VDEC_OUTPUT_FORMAT_NV12 = (1 << 2),

	/* "NV21" YUV 4:2:0 semi-planar (2 planes, Y + interleaved VU) */
	VDEC_OUTPUT_FORMAT_NV21 = (1 << 3),

	/* "NV12MT" YUV 4:2:0 semi-planar tiled (2 planes,
	 * Y + interleaved UV, 64x32 tiles, width aligned on 128,
	 * height aligned on 32) */
	VDEC_OUTPUT_FORMAT_NV12MT = (1 << 4),

	/* Broadcom VideoCore MMAL opaque buffer */
	VDEC_OUTPUT_FORMAT_MMAL_OPAQUE = (1 << 5),
};


/* Decoder intital configuration */
struct vdec_config {
	/* Decoder instance name (optional, can be null, copied internally) */
	const char *name;

	/* Decoder implementation (AUTO means no preference,
	 * use the default implementation for the platform) */
	enum vdec_decoder_implem implem;

	/* Encoding type */
	enum vdec_encoding encoding;

	/* Input buffer pool preferred minimum buffer count, used
	 * only if the implementation uses it's own input buffer pool
	 * (0 means no preference, use the default value) */
	unsigned int preferred_min_in_buf_count;

	/* Output buffer pool preferred minimum buffer count
	 * (0 means no preference, use the default value) */
	unsigned int preferred_min_out_buf_count;

	/* Preferred decoding thread count (0 means no preference,
	 * use the default value; 1 means no multi-threading;
	 * only relevant for CPU decoding implementations) */
	unsigned int preferred_thread_count;

	/* Favor low delay decoding (e.g. for a live stream) */
	int low_delay;

	/* Synchronized decoding (use the vdec_sync_decode() function) */
	int sync_decoding;

	/* Output silent frames */
	int output_silent_frames;

	/* Preferred output buffers data format (optional,
	 * can be VDEC_OUTPUT_FORMAT_ANY) */
	enum vdec_output_format preferred_output_format;

	/* Android Java VM pointer (only used on Android, set to
	 * NULL otherwise) */
	void *android_jvm;

	/* Debug output directory */
	const char *dbg_dir;

	/* Debug flags */
	uint32_t dbg_flags;
};


/* Decoder callback functions */
struct vdec_cbs {
	/* Frame output callback function (only used when not in synchronized
	 * decoding mode; mandatory in that case).
	 * The library retains ownership of the output buffer and the
	 * application must reference it if needed after returning from the
	 * callback function. The status is 0 in case of success, a negative
	 * errno otherwise. In case of error no frame is output and out_buf
	 * is NULL. An error -EBADMSG means a resync is required (IDR frame).
	 * @param dec: decoder instance handle
	 * @param status: frame output status
	 * @param out_buf: output buffer
	 * @param userdata: user data pointer */
	void (*frame_output)(struct vdec_decoder *dec,
			     int status,
			     struct vbuf_buffer *out_buf,
			     void *userdata);

	/* Flush callback function, called when flushing is complete (only
	 * used when not in synchronized decoding mode; optional).
	 * @param dec: decoder instance handle
	 * @param userdata: user data pointer */
	void (*flush)(struct vdec_decoder *dec, void *userdata);

	/* Stop callback function, called when stoping is complete (only
	 * used when not in synchronized decoding mode; optional).
	 * @param dec: decoder instance handle
	 * @param userdata: user data pointer */
	void (*stop)(struct vdec_decoder *dec, void *userdata);
};


/* Input buffer metadata */
struct vdec_input_metadata {
	/* Decoding timestamp in microseconds (monotonic) */
	uint64_t timestamp;

	/* Frame index */
	unsigned int index;

	/* Input bitstream format */
	enum vdec_input_format format;

	/* Frame is complete (syntactically complete) */
	int complete;

	/* Frame has errors (concealed missing slices or error propagation) */
	int errors;

	/* Reference frame (used as reference for future frames) */
	int ref;

	/* Silent frame (should be decoded but not displayed) */
	int silent;

	/* Buffer user data, passed to the output buffer
	 * (optional, can be null) */
	void *userdata;

	/* Frame input queue time in microseconds
	 * (for performance monitoring; set by the application) */
	uint64_t input_time;

	/* Frame input dequeue time in microseconds
	 * (for performance monitoring; set by the library) */
	uint64_t dequeue_time;
};


/* Output buffer metadata */
struct vdec_output_metadata {
	/* Decoding timestamp in microseconds (monotonic) */
	uint64_t timestamp;

	/* Frame index */
	unsigned int index;

	/* Output chroma format */
	enum vdec_output_format format;

	/* Planes offset in bytes (the planes count depends on the
	 * chroma format; 0 for unused planes) */
	size_t plane_offset[4];

	/* Planes stride in bytes (the planes count depends on the
	 * chroma format; 0 for unused planes) */
	size_t plane_stride[4];

	/* Frame width in pixel units */
	unsigned int width;

	/* Frame height in pixel units */
	unsigned int height;

	/* Sample aspect ratio width (no unit; full
	 * aspect ratio is sar_width / sar_height) */
	unsigned int sar_width;

	/* Sample aspect ratio height (no unit; full
	 * aspect ratio is sar_width / sar_height) */
	unsigned int sar_height;

	/* Left crop in pixel units */
	unsigned int crop_left;

	/* Top crop in pixel units */
	unsigned int crop_top;

	/* Crop width in pixel units */
	unsigned int crop_width;

	/* Crop height in pixel units */
	unsigned int crop_height;

	/* Video signal range: 0 = Y [16..235], Cb&Cr [16..240]
	 * 1 = Y&Cb&Cr [0..255] */
	int full_range;

	/* Frame has errors (concealed missing slices or error propagation) */
	int errors;

	/* Silent frame (should be decoded but not displayed) */
	int silent;

	/* Buffer user data, passed from the input buffer
	 * (optional, can be null) */
	void *userdata;

	/* Frame input queue time in microseconds
	 * (for performance monitoring) */
	uint64_t input_time;

	/* Frame input dequeue time in microseconds
	 * (for performance monitoring) */
	uint64_t dequeue_time;

	/* Frame output time in microseconds
	 * (for performance monitoring) */
	uint64_t output_time;
};


/**
 * Get the supported input buffer data formats for the given
 * decoder implementation.
 * Every implementation supports at least one input format,
 * and optionally more. All input buffers need to be in one of
 * the supported formats, otherwise they will be discarded.
 * @param implem: decoder implementation
 * @return bit field of the supported input formats
 */
VDEC_API uint32_t
vdec_get_supported_input_format(enum vdec_decoder_implem implem);


/**
 * Create a decoder instance.
 * The configuration and callbacks structures must be filled.
 * The instance handle is returned through the ret_obj parameter.
 * When no longer needed, the instance must be freed using the
 * vdec_destroy() function.
 * @param config: decoder configuration
 * @param cbs: decoder callback functions
 * @param userdata: callback functions user data (optional, can be null)
 * @param ret_obj: decoder instance handle (output)
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_new(const struct vdec_config *config,
		      const struct vdec_cbs *cbs,
		      void *userdata,
		      struct vdec_decoder **ret_obj);


/**
 * Flush the decoder.
 * This function flushes all queues and optionally discards all buffers
 * retained by the decoder. If the buffers are not discarded the frame
 * output callback is called for each frame when the decoding is complete.
 * It is only used when not in synchronized decoding mode. The function is
 * asynchronous and returns immediately. When flushing is complete the
 * flush callback function is called if defined. After flushing the
 * decoder new input buffers can still be queued but should start with a
 * syncronization frame (e.g. IDR frame or start of refresh).
 * @param self: decoder instance handle
 * @param discard: if null, all pending buffers are output, otherwise they
 *        are discarded
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_flush(struct vdec_decoder *self, int discard);


/**
 * Stop the decoder.
 * This function stops any running threads. It is only used when not in
 * synchronized decoding mode. The function is asynchronous and returns
 * immediately. When stopping is complete the stop callback function
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
 * Set the parameter sets for decoding.
 * This function must be called prior to decoding (i.e. pushing buffer
 * into the input queue or calling the vdec_sync_decode() function) with
 * the H.264 SPS and PPS.
 * The SPS and PPS data will be copied internally if necessary.
 * The ownership of the SPS and PPS buffers stays with the caller.
 * @param self: decoder instance handle
 * @param sps: pointer to the SPS data
 * @param sps_size: SPS size
 * @param pps: pointer to the PPS data
 * @param pps_size: PPS size
 * @param format: SPS and PPS data format
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_set_sps_pps(struct vdec_decoder *self,
			      const uint8_t *sps,
			      size_t sps_size,
			      const uint8_t *pps,
			      size_t pps_size,
			      enum vdec_input_format format);


/**
 * Get the input buffer pool.
 * The input buffer pool is defined only for implementations that require
 * using input buffers from the decoder's own pool. This function must
 * be called prior to decoding and if the returned value is not NULL the
 * input buffer pool should be used to get input buffers. If the input
 * buffers provided are not originating from the pool, they will be copied
 * resulting in a loss of performance.
 * This function is used when both in synchronized and asynchronized
 * decoding modes.
 * @param self: decoder instance handle
 * @return a pointer on the input buffer pool on success, NULL in case of
 * error of if no pool is used
 */
VDEC_API struct vbuf_pool *
vdec_get_input_buffer_pool(struct vdec_decoder *self);


/**
 * Get the input buffer queue.
 * This function must be called prior to decoding and the input
 * buffer queue must be used to push input buffers for decoding.
 * This function is only used when not in synchronized decoding mode.
 * @param self: decoder instance handle
 * @return a pointer on the input buffer queue on success, NULL in case of error
 */
VDEC_API struct vbuf_queue *
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
 * Synchronous decoding function.
 * This function decodes a frame from an input buffer and returns a
 * decoded frame output buffer.
 * This function is only used when in synchronized decoding mode.
 * The function does not ref/unref the input buffer. This is the
 * caller's responsibility.
 * The caller has ownership of the returned output buffer and must
 * unreference it when no longer needed.
 * @param self: decoder instance handle
 * @param in_buf: input buffer
 * @param out_buf: output buffer (output)
 * @return 0 on success, negative errno value in case of error
 */
VDEC_API int vdec_sync_decode(struct vdec_decoder *self,
			      struct vbuf_buffer *in_buf,
			      struct vbuf_buffer **out_buf);

/**
 * Get the decoder implementation used.
 * @param self: decoder instance handle
 * @return the decoder implementation used, or VDEC_DECODER_IMPLEM_AUTO
 * in case of error
 */
VDEC_API enum vdec_decoder_implem
vdec_get_used_implem(struct vdec_decoder *self);


/**
 * ToString function for enum vdec_decoder_implem.
 * @param implem: implementation value to convert
 * @return a string description of the implementation
 */
VDEC_API const char *vdec_decoder_implem_str(enum vdec_decoder_implem implem);


/**
 * ToString function for enum vdec_encoding.
 * @param enc: encoding value to convert
 * @return a string description of the encoding
 */
VDEC_API const char *vdec_encoding_str(enum vdec_encoding enc);


/**
 * ToString function for enum vdec_input_format.
 * @param fmt: format value to convert
 * @return a string description of the format
 */
VDEC_API const char *vdec_input_format_str(enum vdec_input_format fmt);


/**
 * ToString function for enum vdec_output_format.
 * @param fmt: format value to convert
 * @return a string description of the format
 */
VDEC_API const char *vdec_output_format_str(enum vdec_output_format fmt);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VDEC_H_ */

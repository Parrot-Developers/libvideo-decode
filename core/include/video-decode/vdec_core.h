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

#ifndef _VDEC_CORE_H_
#define _VDEC_CORE_H_

#include <stdint.h>
#include <unistd.h>

#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_raw_video_frame.h>
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
 * mbuf ancillary data key for the input timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define VDEC_ANCILLARY_KEY_INPUT_TIME "vdec.input_time"

/**
 * mbuf ancillary data key for the dequeue timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define VDEC_ANCILLARY_KEY_DEQUEUE_TIME "vdec.dequeue_time"

/**
 * mbuf ancillary data key for the output timestamp.
 *
 * Content is a 64bits microseconds value on a monotonic clock
 */
#define VDEC_ANCILLARY_KEY_OUTPUT_TIME "vdec.output_time"


/* Debug files flags. Can be set in configuration as well as in
 * VDEC_DBG_FLAGS environment variables (with VDEC_DBG_DIR specifying the
 * output directory. */

/* Input bitstream */
#define VDEC_DBG_FLAG_INPUT_BITSTREAM (1 << 0)

/* Output YUV */
#define VDEC_DBG_FLAG_OUTPUT_YUV (1 << 1)

/* H.264 parsing and analysis */
#define VDEC_DBG_FLAG_ANALYSIS (1 << 2)


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

	/* HiSilicon decoder implementation */
	VDEC_DECODER_IMPLEM_HISI,

	/* Amlogic decoder implementation */
	VDEC_DECODER_IMPLEM_AML,
};


/* Decoder initial configuration, implementation specific extension
 * Each implementation might provide implementation specific configuration with
 * a structure compatible with this base structure (i.e. which starts with the
 * same implem field). */
struct vdec_config_impl {
	/* Decoder implementation for this extension */
	enum vdec_decoder_implem implem;
};


/* Decoder initial configuration */
struct vdec_config {
	/* Decoder instance name (optional, can be null, copied internally) */
	const char *name;

	/* Decoder implementation (AUTO means no preference,
	 * use the default implementation for the platform) */
	enum vdec_decoder_implem implem;

	/* Encoding type */
	enum vdef_encoding encoding;

	/* Input buffer pool preferred minimum buffer count, used
	 * only if the implementation uses its own input buffer pool
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

	/* Generate an H.264 grey IDR frame for synchronization if needed */
	int gen_grey_idr;

	/* Output silent frames */
	int output_silent_frames;

	/* Preferred output buffers data format (optional, 0 means any) */
	struct vdef_raw_format preferred_output_format;

	/* Android Java VM pointer (only used on Android, set to
	 * NULL otherwise) */
	void *android_jvm;

	/* Debug output directory */
	const char *dbg_dir;

	/* Debug flags */
	uint32_t dbg_flags;

	/* Implementation specific extensions (optional, can be NULL)
	 * If not null, implem_cfg must match the following requirements:
	 *  - this->implem_cfg->implem == this->implem
	 *  - this->implem != VDEC_DECODER_IMPLEM_AUTO
	 *  - The real type of implem_cfg must be the implementation specific
	 *    structure, not struct vdec_config_impl */
	struct vdec_config_impl *implem_cfg;
};


/* Decoder callback functions */
struct vdec_cbs {
	/* Frame output callback function (mandatory).
	 * The library retains ownership of the output frame and the
	 * application must reference it if needed after returning from the
	 * callback function. The status is 0 in case of success, a negative
	 * errno otherwise. In case of error no frame is output and frame
	 * is NULL. An error -EBADMSG means a resync is required (IDR frame).
	 * @param dec: decoder instance handle
	 * @param status: frame output status
	 * @param frame: output frame
	 * @param userdata: user data pointer */
	void (*frame_output)(struct vdec_decoder *dec,
			     int status,
			     struct mbuf_raw_video_frame *frame,
			     void *userdata);

	/* Flush callback function, called when flushing is complete (optional).
	 * @param dec: decoder instance handle
	 * @param userdata: user data pointer */
	void (*flush)(struct vdec_decoder *dec, void *userdata);

	/* Stop callback function, called when stopping is complete (optional).
	 * @param dec: decoder instance handle
	 * @param userdata: user data pointer */
	void (*stop)(struct vdec_decoder *dec, void *userdata);
};


/**
 * ToString function for enum vdec_decoder_implem.
 * @param implem: implementation value to convert
 * @return a string description of the implementation
 */
VDEC_API const char *vdec_decoder_implem_str(enum vdec_decoder_implem implem);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* !_VDEC_CORE_H_ */

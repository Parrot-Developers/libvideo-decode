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

#ifndef _VDEC_MEDIACODEC_PRIV_H_
#define _VDEC_MEDIACODEC_PRIV_H_

#include <inttypes.h>
#include <stdatomic.h>
#include <stdbool.h>

#include <pthread.h>

#include <media/NdkMediaCodec.h>
#include <media/NdkMediaFormat.h>

#include <futils/timetools.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <video-decode/vdec_internal.h>
#include <video-decode/vdec_mediacodec.h>


#define VDEC_ANCILLARY_KEY_MEDIACODEC_PTS "vdec.mediacodec.pts"


enum state {
	RUNNING,
	WAITING_FOR_STOP,
	WAITING_FOR_FLUSH,
};


/* See
 * https://developer.android.com/reference/android/media/MediaCodecInfo.CodecCapabilities.html
 * for reference. */
enum color_format {
	YUV420_PLANAR = 0x00000013,
	YUV420_PACKED_PLANAR = 0x00000014,
	YUV420_SEMIPLANAR = 0x00000015,
	YUV420_PACKED_SEMIPLANAR = 0x00000027,
	TI_YUV420_PACKED_SEMIPLANAR = 0x7F000100,
	QCOM_YUV420_SEMIPLANAR = 0x7FA30C00,
	QCOM_YUV420_PACKED_SEMIPLANAR64X32_TILE2_M8KA = 0x7FA30C03,
	QCOM_YUV420_SEMIPLANAR32_M = 0x7FA30C04,
};


struct vdec_mediacodec {
	struct vdec_decoder *base;

	AMediaCodec *mc;

	atomic_int state;

	struct mbuf_coded_video_frame_queue *in_queue;
	struct mbuf_coded_video_frame_queue *meta_queue;
	struct mbuf_raw_video_frame_queue *out_queue;
	struct mbuf_mem *mem;
	struct pomp_evt *out_evt;

	char *dec_name;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool cond_signalled;
		bool stop_requested;
		bool flush_requested;
		bool eos_requested;
	} push;

	struct {
		pthread_t thread;
		bool thread_created;
		pthread_mutex_t mutex;
		pthread_cond_t cond;
		bool cond_signalled;
		bool stop_requested;
		bool flush_requested;
		bool eos_requested;
	} pull;

	bool eos_requested;
	bool eos_pending;

	struct vdef_raw_format output_format;
	unsigned int stride;
	unsigned int slice_height;

	atomic_bool need_sync;
};


/* Type conversion functions between Mediacodec & venc types */
#include "vdec_mediacodec_convert.h"

#endif /* !_VDEC_MEDIACODEC_PRIV_H_ */

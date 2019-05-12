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

#ifndef _VDEC_VIDEOTOOLBOX_PRIV_H_
#define _VDEC_VIDEOTOOLBOX_PRIV_H_

#include <arpa/inet.h>

#include <VideoToolbox/VideoToolbox.h>

#include <futils/futils.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-decode/vdec_core.h>
#include <video-decode/vdec_internal.h>
#include <video-decode/vdec_videotoolbox.h>

#include <pthread.h>


#define VDEC_VIDEOTOOLBOX_OUT_POOL_DEFAULT_MIN_BUF_COUNT 10


enum vdec_videotoolbox_message_type {
	VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_FLUSH = 'f',
	VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_STOP = 's',
	VDEC_VIDEOTOOLBOX_MESSAGE_TYPE_ERROR = 'e',
};


struct vdec_videotoolbox_message {
	enum vdec_videotoolbox_message_type type;
	int error;
};


struct vdec_videotoolbox {
	struct vdec_decoder *base;
	struct mbuf_coded_video_frame_queue *in_queue;
	struct mbuf_raw_video_frame_queue *out_queue;
	struct pomp_evt *out_queue_evt;
	CMVideoFormatDescriptionRef format_desc_ref;
	VTDecompressionSessionRef decompress_ref;
	int flushing;

	pthread_t thread;
	int thread_launched;
	int should_stop;
	int flush;
	int flush_discard;
	struct mbox *mbox;
	int need_sync;
};


struct vdec_videotoolbox_cvbuffer {
	CVBufferRef ref;
	CVPixelBufferLockFlags lock_flags;
	int cpu_locked;
};

#endif /* !_VDEC_VIDEOTOOLBOX_PRIV_H_ */

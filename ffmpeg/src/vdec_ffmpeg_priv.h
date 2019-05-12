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

#ifndef _VDEC_FFMPEG_PRIV_H_
#define _VDEC_FFMPEG_PRIV_H_

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <libavcodec/avcodec.h>
#include <libavutil/avutil.h>
#include <libavutil/pixdesc.h>
#include <pthread.h>
#include <stdatomic.h>

#include <futils/mbox.h>
#include <futils/timetools.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <video-decode/vdec_ffmpeg.h>
#include <video-decode/vdec_internal.h>

#define VDEC_FFMPEG_DEFAULT_THREAD_COUNT 8

#define VDEC_MSG_FLUSH 'f'
#define VDEC_MSG_STOP 's'


struct vdec_ffmpeg {
	struct vdec_decoder *base;
	struct mbuf_coded_video_frame_queue *in_queue;
	struct mbuf_coded_video_frame_queue *decoder_queue;
	struct mbuf_raw_video_frame_queue *out_queue;
	struct pomp_evt *out_queue_evt;
	AVCodecContext *avcodec;
	AVBufferRef *hw_device_ctx;
	AVPacket avpacket;
	AVFrame *dummy_frame;
	atomic_int flushing;

	pthread_t thread;
	atomic_int thread_launched;
	atomic_int should_stop;
	atomic_int flush;
	atomic_int flush_discard;
	struct mbox *mbox;
	atomic_int need_sync;
};

#endif /* _VDEC_FFMPEG_PRIV_H_ */

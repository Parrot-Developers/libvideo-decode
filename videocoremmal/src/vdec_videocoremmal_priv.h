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

#ifndef _VDEC_VIDEOCOREMMAL_PRIV_H_
#define _VDEC_VIDEOCOREMMAL_PRIV_H_

#include <arpa/inet.h>

#include <interface/mmal/mmal.h>
#include <interface/mmal/util/mmal_default_components.h>
#include <interface/mmal/util/mmal_util.h>
#include <interface/mmal/util/mmal_util_params.h>
#include <interface/mmal/vc/mmal_vc_api.h>

#include <video-buffers/vbuf_internals.h>
#include <video-decode/vdec_core.h>
#include <video-decode/vdec_internal.h>
#include <video-decode/vdec_videocoremmal.h>


#define VDEC_VIDEOCOREMMAL_OUT_BUFFERS_COUNT 20
#define VDEC_VIDEOCOREMMAL_OUT_NUM_EXTRA_BUFFERS 20
#define VDEC_VIDEOCOREMMAL_BUF_TYPE_MMAL 0x4D4D414C /* "MDCD" */

#define VDEC_MSG_FLUSH 'f'
#define VDEC_MSG_STOP 's'


struct vdec_videocoremmal {
	struct vdec_decoder *base;
	struct vbuf_pool *in_pool;
	struct vbuf_queue *in_queue;
	struct vbuf_queue *decoder_queue;
	struct vbuf_pool *out_pool;
	struct vbuf_queue *out_queue;
	struct pomp_evt *out_queue_evt;
	MMAL_COMPONENT_T *decoder;
	MMAL_POOL_T *mmal_in_pool;
	MMAL_POOL_T *mmal_out_pool;
	MMAL_QUEUE_T *mmal_out_queue;
	int flushing;
	uint64_t last_input_pts;
	uint64_t last_output_pts;

	pthread_t thread;
	int thread_launched;
	int should_stop;
	int flush;
	int flush_discard;
	struct mbox *mbox;
	int need_sync;

	struct vdef_raw_format output_format;
	unsigned int stride;
};


struct vdec_videocoremmal_mmalbuf {
	MMAL_BUFFER_HEADER_T *mmal_buf;
};


#endif /* !_VDEC_VIDEOCOREMMAL_PRIV_H_ */

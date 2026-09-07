/**
 * Copyright (c) 2026 Parrot Drones SAS
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

#ifndef _VDEC_TEST_H_
#define _VDEC_TEST_H_

#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <CUnit/Automated.h>
#include <CUnit/Basic.h>
#include <CUnit/CUnit.h>

#include <video-decode/vdec.h>
#include <video-decode/vdec_internal.h>

#define FN(_name) (char *)(_name)
#define ARRAY_SIZE(_x) (sizeof(_x) / sizeof((_x)[0]))

static inline bool implem_available(enum vdec_decoder_implem implem)
{
	const struct vdef_coded_format *formats = NULL;
	return vdec_get_supported_input_formats(implem, &formats) > 0;
}

#define VDEC_TEST_MOCK_MAX_CAPTURED 256

/* Shared mock vdec_cbs implementation: captures every output frame (with a
 * reference taken, so it survives past the callback) and records whether the
 * flush/stop callbacks fired. */
struct mock_vdec_cbs_ctx {
	struct mbuf_raw_video_frame *captured[VDEC_TEST_MOCK_MAX_CAPTURED];
	int frame_count;
	int status[VDEC_TEST_MOCK_MAX_CAPTURED];
	bool flush_done;
	bool stop_done;
};

void mock_vdec_cbs_ctx_reset(struct mock_vdec_cbs_ctx *ctx);

void mock_vdec_cbs_ctx_clear(struct mock_vdec_cbs_ctx *ctx);

void mock_vdec_frame_output_cb(struct vdec_decoder *dec,
			       int status,
			       struct mbuf_raw_video_frame *frame,
			       void *userdata);

void mock_vdec_flush_cb(struct vdec_decoder *dec, void *userdata);

void mock_vdec_stop_cb(struct vdec_decoder *dec, void *userdata);

extern CU_TestInfo g_vdec_test_core[];
extern CU_TestInfo g_vdec_test_dbg[];
extern CU_TestInfo g_vdec_test_format[];
extern CU_TestInfo g_vdec_test_h264[];
extern CU_TestInfo g_vdec_test_h265[];
extern CU_TestInfo g_vdec_test_config[];
extern CU_TestInfo g_vdec_test_lifecycle[];

#endif /* !_VDEC_TEST_H_ */

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

#include "vdec_test.h"

void mock_vdec_cbs_ctx_reset(struct mock_vdec_cbs_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
}


void mock_vdec_cbs_ctx_clear(struct mock_vdec_cbs_ctx *ctx)
{
	for (int i = 0; i < ctx->frame_count; i++) {
		if (ctx->captured[i] != NULL)
			mbuf_raw_video_frame_unref(ctx->captured[i]);
	}
	memset(ctx, 0, sizeof(*ctx));
}


void mock_vdec_frame_output_cb(struct vdec_decoder *dec,
			       int status,
			       struct mbuf_raw_video_frame *frame,
			       void *userdata)
{
	struct mock_vdec_cbs_ctx *ctx = userdata;

	(void)dec;

	if (ctx->frame_count >= VDEC_TEST_MOCK_MAX_CAPTURED)
		return;

	ctx->status[ctx->frame_count] = status;
	if (frame != NULL)
		mbuf_raw_video_frame_ref(frame);
	ctx->captured[ctx->frame_count] = frame;
	ctx->frame_count++;
}


void mock_vdec_flush_cb(struct vdec_decoder *dec, void *userdata)
{
	struct mock_vdec_cbs_ctx *ctx = userdata;

	(void)dec;
	ctx->flush_done = true;
}


void mock_vdec_stop_cb(struct vdec_decoder *dec, void *userdata)
{
	struct mock_vdec_cbs_ctx *ctx = userdata;

	(void)dec;
	ctx->stop_done = true;
}


static CU_SuiteInfo s_suites[] = {
	{FN("core"), NULL, NULL, g_vdec_test_core},
	{FN("dbg"), NULL, NULL, g_vdec_test_dbg},
	{FN("format"), NULL, NULL, g_vdec_test_format},
	{FN("h264"), NULL, NULL, g_vdec_test_h264},
	{FN("h265"), NULL, NULL, g_vdec_test_h265},
	{FN("config"), NULL, NULL, g_vdec_test_config},
	{FN("lifecycle"), NULL, NULL, g_vdec_test_lifecycle},

	CU_SUITE_INFO_NULL,
};


static void run_automated(void)
{
	CU_automated_run_tests();
	CU_list_tests_to_file();
}


static void run_basic(void)
{
	CU_basic_set_mode(CU_BRM_VERBOSE);
	CU_basic_run_tests();
}


int main(void)
{
	const char *filename;

	CU_initialize_registry();
	CU_register_suites(s_suites);

	filename = getenv("CUNIT_OUT_NAME");
	if (filename != NULL)
		CU_set_output_filename(filename);

	if (getenv("CUNIT_AUTOMATED") != NULL)
		run_automated();
	else
		run_basic();

	CU_cleanup_registry();

	return 0;
}

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

/*
 * vdec_decoder_implem_str() (core/src/vdec_enums.c): pure enum switch, no
 * dependencies. Exhaustive coverage of every branch, including the sentinel
 * VDEC_DECODER_IMPLEM_MAX and VDEC_DECODER_IMPLEM_AUTO (both fall to the
 * default "UNKNOWN" case, since the switch has no case for either).
 */

static void test_decoder_implem_str(void)
{
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_AUTO), "UNKNOWN");
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_FFMPEG), "FFMPEG");
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_MEDIACODEC),
		"MEDIACODEC");
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_VIDEOTOOLBOX),
		"VIDEOTOOLBOX");
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_HISI), "HISI");
	CU_ASSERT_STRING_EQUAL(vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_AML),
			       "AML");
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_TURBOJPEG),
		"TURBOJPEG");
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_QCOM), "QCOM");
	CU_ASSERT_STRING_EQUAL(vdec_decoder_implem_str(VDEC_DECODER_IMPLEM_MAX),
			       "UNKNOWN");
	CU_ASSERT_STRING_EQUAL(
		vdec_decoder_implem_str((enum vdec_decoder_implem)999),
		"UNKNOWN");
}


/*
 * vdec_config_get_specific() (core/src/vdec_enums.c): pure 3-way branch on a
 * hand-built struct vdec_config, no decoder instance needed.
 */

static void test_config_get_specific_no_implem_cfg(void)
{
	struct vdec_config config;
	struct vdec_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_FFMPEG;
	config.implem_cfg = NULL;

	specific =
		vdec_config_get_specific(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	CU_ASSERT_PTR_NULL(specific);
}


static void test_config_get_specific_implem_mismatch(void)
{
	struct vdec_config config;
	struct vdec_config_impl impl_cfg = {.implem =
						    VDEC_DECODER_IMPLEM_FFMPEG};
	struct vdec_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_AUTO;
	config.implem_cfg = &impl_cfg;

	/* config.implem != the requested implem */
	specific =
		vdec_config_get_specific(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	CU_ASSERT_PTR_NULL(specific);
}


static void test_config_get_specific_implem_cfg_mismatch(void)
{
	struct vdec_config config;
	struct vdec_config_impl impl_cfg = {.implem = VDEC_DECODER_IMPLEM_AUTO};
	struct vdec_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_FFMPEG;
	config.implem_cfg = &impl_cfg;

	/* config.implem_cfg->implem != config.implem */
	specific =
		vdec_config_get_specific(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	CU_ASSERT_PTR_NULL(specific);
}


static void test_config_get_specific_match(void)
{
	struct vdec_config config;
	struct vdec_config_impl impl_cfg = {.implem =
						    VDEC_DECODER_IMPLEM_FFMPEG};
	struct vdec_config_impl *specific;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_FFMPEG;
	config.implem_cfg = &impl_cfg;

	specific =
		vdec_config_get_specific(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	CU_ASSERT_PTR_EQUAL(specific, &impl_cfg);
}


CU_TestInfo g_vdec_test_core[] = {
	{FN("decoder-implem-str"), &test_decoder_implem_str},
	{FN("config-get-specific-no-implem-cfg"),
	 &test_config_get_specific_no_implem_cfg},
	{FN("config-get-specific-implem-mismatch"),
	 &test_config_get_specific_implem_mismatch},
	{FN("config-get-specific-implem-cfg-mismatch"),
	 &test_config_get_specific_implem_cfg_mismatch},
	{FN("config-get-specific-match"), &test_config_get_specific_match},

	CU_TEST_INFO_NULL,
};

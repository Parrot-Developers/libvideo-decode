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

#include <stdlib.h>
#include <unistd.h>

/*
 * vdec_new() argument guards (src/vdec.c).
 */

static void dummy_frame_output_cb(struct vdec_decoder *dec,
				  int status,
				  struct mbuf_raw_video_frame *frame,
				  void *userdata)
{
	(void)dec;
	(void)status;
	(void)frame;
	(void)userdata;
}


static void fill_valid_h264_config(struct vdec_config *config,
				   enum vdec_decoder_implem implem)
{
	memset(config, 0, sizeof(*config));
	config->implem = implem;
	config->encoding = VDEF_ENCODING_H264;
}


static void test_vdec_new_null_args(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);

	res = vdec_new(NULL, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_new(loop, NULL, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_new(loop, &config, NULL, NULL, &dec);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_new(loop, &config, &cbs, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


static void test_vdec_new_missing_frame_output_cb(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {0};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);

	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL(res, -EINVAL);

	pomp_loop_destroy(loop);
}


/*
 * dec_name: vdec_new() asprintf's it from config.name when given, else from
 * an auto-incrementing numeric id ("%02d"). struct vdec_decoder is fully
 * defined in video-decode/vdec_internal.h, so dec->dec_name can be read back
 * directly instead of only inferred from observable behavior (e.g. log
 * output).
 */

static void test_vdec_new_named_decoder(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	config.name = "my-decoder";
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec->dec_name);
	CU_ASSERT_STRING_EQUAL(dec->dec_name, "my-decoder");

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/*
 * Environment variable overrides. vdec_new() copies config, then overrides
 * a handful of fields from environment variables if set. struct vdec_decoder
 * is fully defined in video-decode/vdec_internal.h, so the resulting
 * self->config can be read back directly rather than only inferred from
 * observable behavior.
 */

static void test_vdec_new_env_low_delay_override(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	setenv("VDEC_LOW_DELAY", "1", 1);
	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	config.low_delay = 0;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	unsetenv("VDEC_LOW_DELAY");
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);
	CU_ASSERT_EQUAL(dec->config.low_delay, 1);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_new_env_preferred_thread_count_override(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	setenv("VDEC_PREFERRED_THREAD_COUNT", "2", 1);
	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	config.preferred_thread_count = 0;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	unsetenv("VDEC_PREFERRED_THREAD_COUNT");
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);
	CU_ASSERT_EQUAL(dec->config.preferred_thread_count, 2u);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_new_env_dbg_flags_override(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	setenv("VDEC_DBG_FLAGS", "3", 1);
	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	config.dbg_flags = 0;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	unsetenv("VDEC_DBG_FLAGS");
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);
	CU_ASSERT_EQUAL(dec->config.dbg_flags, 3u);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_new_env_dbg_dir_override(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	char dir_template[] = "/tmp/vdec_test_dbg_XXXXXX";
	char *dir;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	dir = mkdtemp(dir_template);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dir);

	setenv("VDEC_DBG_DIR", dir, 1);
	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	config.dbg_dir = NULL;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	unsetenv("VDEC_DBG_DIR");
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec->config.dbg_dir);
	CU_ASSERT_STRING_EQUAL(dec->config.dbg_dir, dir);

	vdec_destroy(dec);
	rmdir(dir);

out:
	pomp_loop_destroy(loop);
}


/*
 * Implementation resolution.
 */

static void test_vdec_new_auto_resolves_to_ffmpeg(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	enum vdec_decoder_implem used;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_AUTO);
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	used = vdec_get_used_implem(dec);
	CU_ASSERT_EQUAL(used, VDEC_DECODER_IMPLEM_FFMPEG);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_new_unbuilt_implem_returns_enosys(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);

	/* This product config does not build MEDIACODEC/VIDEOTOOLBOX/HISI/AML/
	 * QCOM (only FFMPEG and TURBOJPEG). Requesting one of these explicitly
	 * must fail with -ENOSYS rather than silently falling back to AUTO. */
	if (implem_available(VDEC_DECODER_IMPLEM_MEDIACODEC))
		goto out;

	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_MEDIACODEC);
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL(res, -ENOSYS);
	CU_ASSERT_PTR_NULL(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_new_ffmpeg_succeeds(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	enum vdec_decoder_implem used;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	used = vdec_get_used_implem(dec);
	CU_ASSERT_EQUAL(used, VDEC_DECODER_IMPLEM_FFMPEG);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_new_turbojpeg_succeeds(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	enum vdec_decoder_implem used;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		goto out;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	used = vdec_get_used_implem(dec);
	CU_ASSERT_EQUAL(used, VDEC_DECODER_IMPLEM_TURBOJPEG);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* vdec_set_jpeg_params() (src/vdec.c) rejects any format_info whose
 * bit_depth/color_primaries/transfer_function/matrix_coefs aren't the single
 * combination it hardcodes support for (8-bit, BT.709, sRGB primaries) -
 * every other JPEG test in this suite already uses a fully valid
 * format_info, so this branch was otherwise never reached. No real JPEG
 * asset is needed: this function only touches self->video_info/self->ops,
 * never the pixel data itself. */
static void test_vdec_set_jpeg_params_invalid_format(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	struct vdef_format_info format_info;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		goto out;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	memset(&format_info, 0, sizeof(format_info));
	format_info.resolution.width = 64;
	format_info.resolution.height = 64;
	format_info.framerate.num = 1;
	format_info.framerate.den = 1;
	/* Everything else is valid except bit_depth. */
	format_info.bit_depth = 10;
	format_info.color_primaries = VDEF_COLOR_PRIMARIES_SRGB;
	format_info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	format_info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;

	res = vdec_set_jpeg_params(dec, &format_info);
	CU_ASSERT_EQUAL(res, -EINVAL);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* Asymmetry pinned down, not fixed (see workspace CLAUDE.md / plan): the
 * ffmpeg backend's create() looks up a codec by (encoding) via its internal
 * backend map; a request for an encoding it doesn't support at all (as
 * opposed to a malformed encoding value) falls through to "codec not found"
 * -> -ENOENT, not -EINVAL. This differs from turbojpeg's create(), which
 * explicitly checks `config.encoding != VDEF_ENCODING_MJPEG` and returns
 * -EINVAL. */
static void test_vdec_new_ffmpeg_bad_encoding_returns_enoent(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	/* ffmpeg's backend map only has H264/H265 entries in this product
	 * config: MJPEG has no matching entry -> "codec not found". */
	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	config.encoding = VDEF_ENCODING_MJPEG;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL(res, -ENOENT);
	CU_ASSERT_PTR_NULL(dec);

out:
	pomp_loop_destroy(loop);
}


/* Pinning test: config.implem_cfg is a shallow/raw pointer copy in vdec_new()
 * (`self->config = *config;`), not deep-copied. Pointer identity, and the
 * pointee's content, survive vdec_new() unchanged. */
struct fake_impl_cfg {
	enum vdec_decoder_implem implem;
	int marker;
};

static void test_vdec_new_implem_cfg_pointer_identity_preserved(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	struct fake_impl_cfg fake_cfg = {
		.implem = VDEC_DECODER_IMPLEM_FFMPEG,
		.marker = 1234,
	};
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	config.implem_cfg = (struct vdec_config_impl *)&fake_cfg;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	CU_ASSERT_PTR_EQUAL(dec->config.implem_cfg,
			    (struct vdec_config_impl *)&fake_cfg);
	CU_ASSERT_EQUAL(
		((struct fake_impl_cfg *)dec->config.implem_cfg)->marker, 1234);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* turbojpeg's create() only accepts config.preferred_output_format ==
 * vdef_i420 or vdef_nv12 (or unset) - anything else, including vdef_nv21,
 * is rejected synchronously from vdec_new() itself. */
static void test_vdec_new_turbojpeg_rejects_nv21_output_format(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		goto out;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	config.preferred_output_format = vdef_nv21;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL(res, -EINVAL);
	CU_ASSERT_PTR_NULL(dec);

out:
	pomp_loop_destroy(loop);
}


/*
 * Other guarded entry points, no backend needed.
 */

static void test_vdec_get_used_implem_null_self(void)
{
	enum vdec_decoder_implem implem = vdec_get_used_implem(NULL);
	CU_ASSERT_EQUAL(implem, VDEC_DECODER_IMPLEM_AUTO);
}


static void test_vdec_destroy_null_is_noop(void)
{
	int res = vdec_destroy(NULL);
	CU_ASSERT_EQUAL(res, 0);
}


static void test_vdec_flush_stop_null_self(void)
{
	int res;

	res = vdec_flush(NULL, 0);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_stop(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_vdec_get_input_buffer_pool_and_queue_null_self(void)
{
	CU_ASSERT_PTR_NULL(vdec_get_input_buffer_pool(NULL));
	CU_ASSERT_PTR_NULL(vdec_get_input_buffer_queue(NULL));
}


/* Neither backend built in this product config allocates its own input
 * buffer pool (both ffmpeg's and turbojpeg's get_input_buffer_pool() just
 * `return NULL;` after the guard) - callers are expected to supply their
 * own pool and only use vdec_get_input_buffer_queue(), exactly like
 * tools/vdec.c's own configure() does (falls back to mbuf_pool_new() when
 * vdec_get_input_buffer_pool() returns NULL). */
static void test_vdec_get_input_buffer_pool_real_decoder_returns_null(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	CU_ASSERT_PTR_NULL(vdec_get_input_buffer_pool(dec));
	CU_ASSERT_PTR_NOT_NULL(vdec_get_input_buffer_queue(dec));

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* Same as above, but for turbojpeg specifically: get_input_buffer_pool() is
 * implemented separately per backend (each its own translation unit), so
 * exercising it via an ffmpeg decoder only covers ffmpeg's copy - turbojpeg's
 * own `return NULL;` needs its own real decoder instance. */
static void test_vdec_get_input_buffer_pool_turbojpeg_returns_null(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		goto out;

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	CU_ASSERT_PTR_NULL(vdec_get_input_buffer_pool(dec));
	CU_ASSERT_PTR_NOT_NULL(vdec_get_input_buffer_queue(dec));

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_get_supported_input_formats_null_formats(void)
{
	int res = vdec_get_supported_input_formats(VDEC_DECODER_IMPLEM_FFMPEG,
						   NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_vdec_get_supported_input_formats_ffmpeg(void)
{
	const struct vdef_coded_format *formats = NULL;
	int res;

	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		return;

	res = vdec_get_supported_input_formats(VDEC_DECODER_IMPLEM_FFMPEG,
					       &formats);
	CU_ASSERT_TRUE(res > 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(formats);
}


static void test_vdec_get_all_supported_input_formats_null_formats(void)
{
	int res = vdec_get_all_supported_input_formats(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_vdec_get_all_supported_input_formats_success(void)
{
	const struct vdef_coded_format *formats = NULL;
	int res;

	res = vdec_get_all_supported_input_formats(&formats);
	/* At least ffmpeg is always built in this product config. */
	CU_ASSERT_TRUE(res > 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(formats);
}


static void test_vdec_get_auto_implem_matches_ffmpeg(void)
{
	enum vdec_decoder_implem implem;

	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		return;

	implem = vdec_get_auto_implem();
	CU_ASSERT_EQUAL(implem, VDEC_DECODER_IMPLEM_FFMPEG);
}


static void test_vdec_get_video_dimensions_null_self(void)
{
	unsigned int width = 0;
	int res = vdec_get_video_dimensions(
		NULL, &width, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_vdec_get_video_dimensions_not_configured(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct vdec_config config;
	struct vdec_cbs cbs = {.frame_output = &dummy_frame_output_cb};
	struct vdec_decoder *dec = NULL;
	unsigned int width = 0;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;

	fill_valid_h264_config(&config, VDEC_DECODER_IMPLEM_FFMPEG);
	res = vdec_new(loop, &config, &cbs, NULL, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	/* No vdec_set_h264_ps() call yet: self->configured is still 0. */
	res = vdec_get_video_dimensions(
		dec, &width, NULL, NULL, NULL, NULL, NULL, NULL, NULL);
	CU_ASSERT_EQUAL(res, -EAGAIN);

	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_get_auto_implem_by_coded_format_null_args(void)
{
	enum vdec_decoder_implem implem =
		vdec_get_auto_implem_by_coded_format(NULL);
	CU_ASSERT_EQUAL(implem, VDEC_DECODER_IMPLEM_AUTO);
}


static void test_vdec_get_auto_implem_by_coded_format_matches(void)
{
	enum vdec_decoder_implem implem;

	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		return;

	implem = vdec_get_auto_implem_by_coded_format(&vdef_h264_byte_stream);
	CU_ASSERT_NOT_EQUAL(implem, VDEC_DECODER_IMPLEM_AUTO);
}


static void test_vdec_get_auto_implem_by_coded_format_no_match(void)
{
	enum vdec_decoder_implem implem;
	struct vdef_coded_format format = {
		.encoding = (enum vdef_encoding)999,
		.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
	};

	implem = vdec_get_auto_implem_by_coded_format(&format);
	CU_ASSERT_EQUAL(implem, VDEC_DECODER_IMPLEM_AUTO);
}


CU_TestInfo g_vdec_test_config[] = {
	{FN("vdec-new-null-args"), &test_vdec_new_null_args},
	{FN("vdec-new-missing-frame-output-cb"),
	 &test_vdec_new_missing_frame_output_cb},
	{FN("vdec-new-named-decoder"), &test_vdec_new_named_decoder},
	{FN("vdec-new-env-low-delay-override"),
	 &test_vdec_new_env_low_delay_override},
	{FN("vdec-new-env-preferred-thread-count-override"),
	 &test_vdec_new_env_preferred_thread_count_override},
	{FN("vdec-new-env-dbg-flags-override"),
	 &test_vdec_new_env_dbg_flags_override},
	{FN("vdec-new-env-dbg-dir-override"),
	 &test_vdec_new_env_dbg_dir_override},
	{FN("vdec-new-auto-resolves-to-ffmpeg"),
	 &test_vdec_new_auto_resolves_to_ffmpeg},
	{FN("vdec-new-unbuilt-implem-returns-enosys"),
	 &test_vdec_new_unbuilt_implem_returns_enosys},
	{FN("vdec-new-ffmpeg-succeeds"), &test_vdec_new_ffmpeg_succeeds},
	{FN("vdec-new-turbojpeg-succeeds"), &test_vdec_new_turbojpeg_succeeds},
	{FN("set-jpeg-params-invalid-format"),
	 &test_vdec_set_jpeg_params_invalid_format},
	{FN("vdec-new-ffmpeg-bad-encoding-returns-enoent"),
	 &test_vdec_new_ffmpeg_bad_encoding_returns_enoent},
	{FN("vdec-new-implem-cfg-pointer-identity-preserved"),
	 &test_vdec_new_implem_cfg_pointer_identity_preserved},
	{FN("vdec-new-turbojpeg-rejects-nv21-output-format"),
	 &test_vdec_new_turbojpeg_rejects_nv21_output_format},
	{FN("get-used-implem-null-self"), &test_vdec_get_used_implem_null_self},
	{FN("destroy-null-is-noop"), &test_vdec_destroy_null_is_noop},
	{FN("flush-stop-null-self"), &test_vdec_flush_stop_null_self},
	{FN("get-input-buffer-pool-and-queue-null-self"),
	 &test_vdec_get_input_buffer_pool_and_queue_null_self},
	{FN("get-input-buffer-pool-real-decoder-returns-null"),
	 &test_vdec_get_input_buffer_pool_real_decoder_returns_null},
	{FN("get-input-buffer-pool-turbojpeg-returns-null"),
	 &test_vdec_get_input_buffer_pool_turbojpeg_returns_null},
	{FN("get-supported-input-formats-null-formats"),
	 &test_vdec_get_supported_input_formats_null_formats},
	{FN("get-supported-input-formats-ffmpeg"),
	 &test_vdec_get_supported_input_formats_ffmpeg},
	{FN("get-all-supported-input-formats-null-formats"),
	 &test_vdec_get_all_supported_input_formats_null_formats},
	{FN("get-all-supported-input-formats-success"),
	 &test_vdec_get_all_supported_input_formats_success},
	{FN("get-auto-implem-matches-ffmpeg"),
	 &test_vdec_get_auto_implem_matches_ffmpeg},
	{FN("get-video-dimensions-null-self"),
	 &test_vdec_get_video_dimensions_null_self},
	{FN("get-video-dimensions-not-configured"),
	 &test_vdec_get_video_dimensions_not_configured},
	{FN("get-auto-implem-by-coded-format-null-args"),
	 &test_vdec_get_auto_implem_by_coded_format_null_args},
	{FN("get-auto-implem-by-coded-format-matches"),
	 &test_vdec_get_auto_implem_by_coded_format_matches},
	{FN("get-auto-implem-by-coded-format-no-match"),
	 &test_vdec_get_auto_implem_by_coded_format_no_match},

	CU_TEST_INFO_NULL,
};

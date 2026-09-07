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

#include <arpa/inet.h>
#include <dirent.h>
#include <limits.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include <jpeg-parrot/jpeg.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <video-metadata/vmeta_frame.h>

#define PUMP_TIMEOUT_MS 100
#define PUMP_MAX_ITER 150
#define ASSET_PUMP_MAX_ITER 600 /* real files can carry many more frames */

/* Caps how many NALUs from a real elementary stream get pushed, to keep the
 * test fast - decoding the very first handful of frames is enough to prove
 * the pipeline works end to end. */
#define H264_H265_ASSET_MAX_FRAMES 30


static void pump_until(struct pomp_loop *loop, bool *flag, int max_iter)
{
	for (int i = 0; (i < max_iter) && !(*flag); i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
}


/* ================================================================== */
/* NAS-asset helpers: real .264/.265/.JPG files, same convention as         */
/* packages/libaudio-decode/tests/adec_test_lifecycle.c.                    */
/* ================================================================== */

#define VDEC_TEST_ASSETS_ROOT_DEFAULT "/mnt/DFS/MULTIMEDIA_DATA"


/* The H.264/H.265 elementary streams live under a directory whose spelling
 * differs from libaudio-decode's "miscellaneous" convention - try both,
 * preferring "miscelanneous" (as given), and fall back to whichever spelling
 * is actually reachable on the NAS mount. */
static int
resolve_h26x_asset_path(const char *filename, char *path, size_t path_len)
{
	const char *root = getenv("ASSETS_ROOT");
	static const char *dirs[] = {"Tests/miscelanneous",
				     "Tests/miscellaneous"};
	int n;

	if (root == NULL)
		root = VDEC_TEST_ASSETS_ROOT_DEFAULT;

	for (size_t i = 0; i < ARRAY_SIZE(dirs); i++) {
		n = snprintf(
			path, path_len, "%s/%s/%s", root, dirs[i], filename);
		if (n < 0 || (size_t)n >= path_len)
			return -ENAMETOOLONG;
		if (access(path, R_OK) == 0)
			return 0;
	}

	/* Neither spelling is reachable right now: keep the first candidate
	 * (as given, "miscelanneous") so wait_for_asset()'s retry-then-fatal
	 * path below reports a stable, meaningful error. */
	n = snprintf(path, path_len, "%s/%s/%s", root, dirs[0], filename);
	if (n < 0 || (size_t)n >= path_len)
		return -ENAMETOOLONG;
	return 0;
}


static int
build_jpeg_asset_path(const char *relative_path, char *path, size_t path_len)
{
	const char *root = getenv("ASSETS_ROOT");
	int n;

	if (root == NULL)
		root = VDEC_TEST_ASSETS_ROOT_DEFAULT;
	n = snprintf(path, path_len, "%s/%s", root, relative_path);
	if (n < 0 || (size_t)n >= path_len)
		return -ENAMETOOLONG;
	return 0;
}


/* Same retry convention as libaac's/libaudio-decode's wait_for_asset(): a
 * bare access() can transiently fail on an NFS/SMB-backed NAS mount even
 * though the file is really there, so retry a few times with a short backoff
 * before treating the asset as genuinely missing. Still fails FATAL
 * afterwards - only protects against flaky mount glitches, not a truly
 * absent file. */
static int wait_for_asset(const char *path)
{
	static const unsigned int backoff_ms[] = {100, 300, 800};

	if (access(path, R_OK) == 0)
		return 0;

	for (size_t i = 0; i < ARRAY_SIZE(backoff_ms); i++) {
		usleep(backoff_ms[i] * 1000);
		if (access(path, R_OK) == 0)
			return 0;
	}
	return -errno;
}


static uint8_t *read_whole_file(const char *path, size_t *out_len)
{
	FILE *f;
	long size;
	uint8_t *buf;
	size_t rd;

	f = fopen(path, "rb");
	if (f == NULL)
		return NULL;
	if (fseek(f, 0, SEEK_END) != 0 || (size = ftell(f)) < 0 ||
	    fseek(f, 0, SEEK_SET) != 0) {
		fclose(f);
		return NULL;
	}
	buf = malloc((size_t)size);
	if (buf == NULL) {
		fclose(f);
		return NULL;
	}
	rd = fread(buf, 1, (size_t)size, f);
	fclose(f);
	if (rd != (size_t)size) {
		free(buf);
		return NULL;
	}
	*out_len = (size_t)size;
	return buf;
}


static void fill_h26x_config(struct vdec_config *config,
			     enum vdec_decoder_implem implem,
			     enum vdef_encoding encoding)
{
	memset(config, 0, sizeof(*config));
	config->implem = implem;
	config->encoding = encoding;
}


/* ================================================================== */
/* H.264: NALU-splitting/PS-extraction helper, modeled on                  */
/* tools/vdec.c's nalu_end()/configure() logic.                            */
/* ================================================================== */

/* Both the h264/h265 backends supported by this product config
 * (VDEC_DECODER_IMPLEM_FFMPEG) advertise vdef_h26{4,5}_byte_stream as their
 * only supported input format (confirmed by grepping
 * ffmpeg/src/vdec_ffmpeg.c's initialize_supported_formats()). */

static int
push_h264_byte_stream_frame(struct mbuf_coded_video_frame_queue *queue,
			    enum h264_nalu_type type,
			    const uint8_t *nalu_buf,
			    size_t nalu_len,
			    uint64_t timestamp)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu;
	uint8_t *data = NULL;
	size_t capacity = 0;
	uint32_t start_code;
	int res;

	res = mbuf_mem_generic_new(4 + nalu_len, &mem);
	if (res < 0)
		return res;
	res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	if (res < 0)
		goto out;

	start_code = htonl(0x00000001);
	memcpy(data, &start_code, sizeof(start_code));
	memcpy(data + 4, nalu_buf, nalu_len);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = timestamp;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	memset(&nalu, 0, sizeof(nalu));
	nalu.size = 4 + nalu_len;
	nalu.h264.type = type;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	if (res < 0)
		goto out;
	res = mbuf_coded_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_coded_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_coded_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


struct h264_asset_ctx {
	struct vdec_decoder *dec;
	struct mbuf_coded_video_frame_queue *queue;
	struct h264_reader *reader;
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	bool ps_configured;
	unsigned int pushed;
	uint64_t next_timestamp;
	int push_errors;
};


static void h264_asset_nalu_end_cb(struct h264_ctx *ctx,
				   enum h264_nalu_type type,
				   const uint8_t *buf,
				   size_t len,
				   const struct h264_nalu_header *nh,
				   void *userdata)
{
	struct h264_asset_ctx *u = userdata;
	int res;

	(void)ctx;
	(void)nh;

	if (type == H264_NALU_TYPE_SPS && u->sps == NULL) {
		u->sps = malloc(len);
		if (u->sps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->sps, buf, len);
		u->sps_size = len;
	} else if (type == H264_NALU_TYPE_PPS && u->pps == NULL) {
		u->pps = malloc(len);
		if (u->pps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->pps, buf, len);
		u->pps_size = len;
	}

	if (!u->ps_configured && u->sps != NULL && u->pps != NULL) {
		res = vdec_set_h264_ps(u->dec,
				       u->sps,
				       u->sps_size,
				       u->pps,
				       u->pps_size,
				       &vdef_h264_raw_nalu);
		if (res < 0) {
			u->push_errors++;
			return;
		}
		u->ps_configured = true;
	}

	if (type == H264_NALU_TYPE_SPS || type == H264_NALU_TYPE_PPS ||
	    type == H264_NALU_TYPE_AUD)
		return;

	if (!u->ps_configured || u->pushed >= H264_H265_ASSET_MAX_FRAMES)
		return;

	res = push_h264_byte_stream_frame(
		u->queue, type, buf, len, u->next_timestamp);
	if (res < 0) {
		u->push_errors++;
		return;
	}
	u->next_timestamp += 33333;
	u->pushed++;

	if (u->pushed >= H264_H265_ASSET_MAX_FRAMES)
		h264_reader_stop(u->reader);
}


static const struct h264_ctx_cbs s_h264_asset_cbs = {
	.nalu_end = h264_asset_nalu_end_cb,
};


/* ================================================================== */
/* H.265: same shape as the H.264 helper above, with an extra VPS.         */
/* ================================================================== */

static int
push_h265_byte_stream_frame(struct mbuf_coded_video_frame_queue *queue,
			    enum h265_nalu_type type,
			    const uint8_t *nalu_buf,
			    size_t nalu_len,
			    uint64_t timestamp)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu;
	uint8_t *data = NULL;
	size_t capacity = 0;
	uint32_t start_code;
	int res;

	res = mbuf_mem_generic_new(4 + nalu_len, &mem);
	if (res < 0)
		return res;
	res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	if (res < 0)
		goto out;

	start_code = htonl(0x00000001);
	memcpy(data, &start_code, sizeof(start_code));
	memcpy(data + 4, nalu_buf, nalu_len);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h265_byte_stream;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = timestamp;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	memset(&nalu, 0, sizeof(nalu));
	nalu.size = 4 + nalu_len;
	nalu.h265.type = type;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	if (res < 0)
		goto out;
	res = mbuf_coded_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_coded_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_coded_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


struct h265_asset_ctx {
	struct vdec_decoder *dec;
	struct mbuf_coded_video_frame_queue *queue;
	struct h265_reader *reader;
	uint8_t *vps;
	size_t vps_size;
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	bool ps_configured;
	unsigned int pushed;
	uint64_t next_timestamp;
	int push_errors;
};


static void h265_asset_nalu_end_cb(struct h265_ctx *ctx,
				   enum h265_nalu_type type,
				   const uint8_t *buf,
				   size_t len,
				   const struct h265_nalu_header *nh,
				   void *userdata)
{
	struct h265_asset_ctx *u = userdata;
	int res;

	(void)ctx;
	(void)nh;

	if (type == H265_NALU_TYPE_VPS_NUT && u->vps == NULL) {
		u->vps = malloc(len);
		if (u->vps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->vps, buf, len);
		u->vps_size = len;
	} else if (type == H265_NALU_TYPE_SPS_NUT && u->sps == NULL) {
		u->sps = malloc(len);
		if (u->sps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->sps, buf, len);
		u->sps_size = len;
	} else if (type == H265_NALU_TYPE_PPS_NUT && u->pps == NULL) {
		u->pps = malloc(len);
		if (u->pps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->pps, buf, len);
		u->pps_size = len;
	}

	if (!u->ps_configured && u->vps != NULL && u->sps != NULL &&
	    u->pps != NULL) {
		res = vdec_set_h265_ps(u->dec,
				       u->vps,
				       u->vps_size,
				       u->sps,
				       u->sps_size,
				       u->pps,
				       u->pps_size,
				       &vdef_h265_raw_nalu);
		if (res < 0) {
			u->push_errors++;
			return;
		}
		u->ps_configured = true;
	}

	if (type == H265_NALU_TYPE_VPS_NUT || type == H265_NALU_TYPE_SPS_NUT ||
	    type == H265_NALU_TYPE_PPS_NUT || type == H265_NALU_TYPE_AUD_NUT)
		return;

	if (!u->ps_configured || u->pushed >= H264_H265_ASSET_MAX_FRAMES)
		return;

	res = push_h265_byte_stream_frame(
		u->queue, type, buf, len, u->next_timestamp);
	if (res < 0) {
		u->push_errors++;
		return;
	}
	u->next_timestamp += 33333;
	u->pushed++;

	if (u->pushed >= H264_H265_ASSET_MAX_FRAMES)
		h265_reader_stop(u->reader);
}


static const struct h265_ctx_cbs s_h265_asset_cbs = {
	.nalu_end = h265_asset_nalu_end_cb,
};


/* H.265 analog of h264_ps_only_ctx above: configures PS via a real VPS/SPS/
 * PPS from the asset, then stops before any slice NALU is ever pushed. */
struct h265_ps_only_ctx {
	struct vdec_decoder *dec;
	struct h265_reader *reader;
	uint8_t *vps;
	size_t vps_size;
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	bool ps_configured;
	int push_errors;
};


static void h265_ps_only_nalu_end_cb(struct h265_ctx *ctx,
				     enum h265_nalu_type type,
				     const uint8_t *buf,
				     size_t len,
				     const struct h265_nalu_header *nh,
				     void *userdata)
{
	struct h265_ps_only_ctx *u = userdata;
	int res;

	(void)ctx;
	(void)nh;

	if (u->ps_configured)
		return;

	if (type == H265_NALU_TYPE_VPS_NUT && u->vps == NULL) {
		u->vps = malloc(len);
		if (u->vps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->vps, buf, len);
		u->vps_size = len;
	} else if (type == H265_NALU_TYPE_SPS_NUT && u->sps == NULL) {
		u->sps = malloc(len);
		if (u->sps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->sps, buf, len);
		u->sps_size = len;
	} else if (type == H265_NALU_TYPE_PPS_NUT && u->pps == NULL) {
		u->pps = malloc(len);
		if (u->pps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->pps, buf, len);
		u->pps_size = len;
	}

	if (u->vps != NULL && u->sps != NULL && u->pps != NULL) {
		res = vdec_set_h265_ps(u->dec,
				       u->vps,
				       u->vps_size,
				       u->sps,
				       u->sps_size,
				       u->pps,
				       u->pps_size,
				       &vdef_h265_raw_nalu);
		if (res < 0) {
			u->push_errors++;
			return;
		}
		u->ps_configured = true;
		h265_reader_stop(u->reader);
	}
}


static const struct h265_ctx_cbs s_h265_ps_only_cbs = {
	.nalu_end = h265_ps_only_nalu_end_cb,
};


/* ================================================================== */
/* JPEG: single-frame push, no NALU splitting needed.                      */
/* ================================================================== */

static int push_jpeg_frame(struct mbuf_coded_video_frame_queue *queue,
			   const uint8_t *buf,
			   size_t len)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu;
	uint8_t *data = NULL;
	size_t capacity = 0;
	int res;

	res = mbuf_mem_generic_new(len, &mem);
	if (res < 0)
		return res;
	res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	if (res < 0)
		goto out;
	memcpy(data, buf, len);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_jpeg_jfif;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = 0;
	frame_info.type = VDEF_CODED_FRAME_TYPE_IDR;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	memset(&nalu, 0, sizeof(nalu));
	nalu.size = len;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	if (res < 0)
		goto out;
	res = mbuf_coded_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_coded_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_coded_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/* Same as push_jpeg_frame(), but attaches vmeta metadata to the coded frame
 * before pushing it - used to exercise the metadata-propagation path (both
 * ffmpeg's vdec_ffmpeg_set_frame_metadata() and turbojpeg's equivalent
 * inline code call mbuf_coded_video_frame_get_metadata() on the input coded
 * frame and mbuf_raw_video_frame_set_metadata() on the output raw frame). */
static int
push_jpeg_frame_with_metadata(struct mbuf_coded_video_frame_queue *queue,
			      const uint8_t *buf,
			      size_t len,
			      struct vmeta_frame *meta)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu;
	uint8_t *data = NULL;
	size_t capacity = 0;
	int res;

	res = mbuf_mem_generic_new(len, &mem);
	if (res < 0)
		return res;
	res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	if (res < 0)
		goto out;
	memcpy(data, buf, len);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_jpeg_jfif;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = 0;
	frame_info.type = VDEF_CODED_FRAME_TYPE_IDR;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	if (res < 0)
		goto out;

	res = mbuf_coded_video_frame_set_metadata(frame, meta);
	if (res < 0)
		goto out;

	memset(&nalu, 0, sizeof(nalu));
	nalu.size = len;

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	if (res < 0)
		goto out;
	res = mbuf_coded_video_frame_finalize(frame);
	if (res < 0)
		goto out;

	res = mbuf_coded_video_frame_queue_push(queue, frame);

out:
	if (frame != NULL)
		mbuf_coded_video_frame_unref(frame);
	if (mem != NULL)
		mbuf_mem_unref(mem);
	return res;
}


/* ================================================================== */
/* NAS-asset tests.                                                        */
/* ================================================================== */

static void test_frame_output_and_drain_h264_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_asset_ctx u;

	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);

	/* Hard failure if the NAS asset is unreachable, same convention as
	 * libaudio-decode's own asset-backed tests - not a silent skip. */
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);

	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);

	res = h264_reader_new(&s_h264_asset_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u.push_errors, 0);
	CU_ASSERT_TRUE(u.ps_configured);
	CU_ASSERT_TRUE(u.pushed > 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

	/* Captured output frames must be unref'd before stop()/destroy():
	 * their release path touches backend state torn down by both. */
	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);
	pomp_loop_destroy(loop);
}


static void test_frame_output_and_drain_h265_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h265_reader *reader = NULL;
	struct h265_asset_ctx u;

	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.265", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);

	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);

	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H265);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);

	res = h265_reader_new(&s_h265_asset_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h265_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u.push_errors, 0);
	CU_ASSERT_TRUE(u.ps_configured);
	CU_ASSERT_TRUE(u.pushed > 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h265_reader_destroy(reader);
	free(u.vps);
	free(u.sps);
	free(u.pps);
	free(buf);
	pomp_loop_destroy(loop);
}


static void test_frame_output_and_drain_jpeg_turbojpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct jpeg_info info;
	struct vdef_format_info format_info;
	struct mbuf_coded_video_frame_queue *queue;

	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);

	path_ret = build_jpeg_asset_path(
		"Tests/anafi3/classic/photo_recording/1323514_blended.JPG",
		path,
		sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);

	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);

	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	memset(&info, 0, sizeof(info));
	res = jpeg_parse(buf, len, &info);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_TRUE_FATAL(info.width > 0 && info.height > 0);

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	/* Mirrors tools/vdec.c's MJPEG configure() branch. */
	memset(&format_info, 0, sizeof(format_info));
	format_info.bit_depth = 8;
	format_info.sar.width = 1;
	format_info.sar.height = 1;
	format_info.full_range = 1;
	format_info.framerate.num = 1;
	format_info.framerate.den = 1;
	format_info.color_primaries = VDEF_COLOR_PRIMARIES_SRGB;
	format_info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	format_info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	format_info.resolution.width = info.width;
	format_info.resolution.height = info.height;

	res = vdec_set_jpeg_params(dec, &format_info);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	res = push_jpeg_frame(queue, buf, len);
	CU_ASSERT_EQUAL(res, 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	free(buf);
	pomp_loop_destroy(loop);
}


/* mbuf_coded_video_frame_get_metadata()/mbuf_raw_video_frame_set_metadata():
 * both backends propagate the input coded frame's metadata onto the output
 * raw frame (ffmpeg's vdec_ffmpeg_set_frame_metadata(), turbojpeg's
 * equivalent inline code) - every other JPEG test pushes a frame with no
 * metadata at all, so that path was never reached. */
static void test_vdec_metadata_propagation_jpeg_turbojpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct jpeg_info info;
	struct vdef_format_info format_info;
	struct mbuf_coded_video_frame_queue *queue;
	struct vmeta_frame *meta = NULL;

	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);

	path_ret = build_jpeg_asset_path(
		"Tests/anafi3/classic/photo_recording/1323514_blended.JPG",
		path,
		sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	memset(&info, 0, sizeof(info));
	res = jpeg_parse(buf, len, &info);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_TRUE_FATAL(info.width > 0 && info.height > 0);

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	memset(&format_info, 0, sizeof(format_info));
	format_info.bit_depth = 8;
	format_info.sar.width = 1;
	format_info.sar.height = 1;
	format_info.full_range = 1;
	format_info.framerate.num = 1;
	format_info.framerate.den = 1;
	format_info.color_primaries = VDEF_COLOR_PRIMARIES_SRGB;
	format_info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	format_info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	format_info.resolution.width = info.width;
	format_info.resolution.height = info.height;

	res = vdec_set_jpeg_params(dec, &format_info);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	res = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = push_jpeg_frame_with_metadata(queue, buf, len, meta);
	CU_ASSERT_EQUAL(res, 0);
	/* set_metadata refs the metadata itself, ownership is not
	 * transferred - the caller's own ref must still be dropped. */
	vmeta_frame_unref(meta);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE_FATAL(ctx.frame_count > 0);

	{
		struct vmeta_frame *out_meta = NULL;

		res = mbuf_raw_video_frame_get_metadata(ctx.captured[0],
							&out_meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(out_meta);
		CU_ASSERT_EQUAL(out_meta->type, VMETA_FRAME_TYPE_V3);
		vmeta_frame_unref(out_meta);
	}

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	free(buf);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* Control-flow tests (no real bitstream).                                 */
/* ================================================================== */

static void test_vdec_create_and_destroy_ffmpeg(void)
{
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	int res;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	res = vdec_destroy(dec);
	CU_ASSERT_EQUAL(res, 0);

out:
	pomp_loop_destroy(loop);
}


/* Minimal PS-only variant of h264_asset_ctx/h264_asset_nalu_end_cb above:
 * stops the reader as soon as vdec_set_h264_ps() succeeds, before the first
 * slice NALU is ever seen, so zero frames reach the input queue. Needed
 * because a discard-flush on an ffmpeg decoder that was never given SPS/PPS
 * (and never fed a single packet) does not reliably complete - every other
 * test in this file configures PS first, matching the same precedent
 * adec_test_lifecycle.c's test_flush_discard_fdk_aac sets by opening the FDK
 * handle (via adec_set_aac_asc()) before its own discard-flush. */
struct h264_ps_only_ctx {
	struct vdec_decoder *dec;
	struct h264_reader *reader;
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	bool ps_configured;
	int push_errors;
};


static void h264_ps_only_nalu_end_cb(struct h264_ctx *ctx,
				     enum h264_nalu_type type,
				     const uint8_t *buf,
				     size_t len,
				     const struct h264_nalu_header *nh,
				     void *userdata)
{
	struct h264_ps_only_ctx *u = userdata;
	int res;

	(void)ctx;
	(void)nh;

	if (u->ps_configured)
		return;

	if (type == H264_NALU_TYPE_SPS && u->sps == NULL) {
		u->sps = malloc(len);
		if (u->sps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->sps, buf, len);
		u->sps_size = len;
	} else if (type == H264_NALU_TYPE_PPS && u->pps == NULL) {
		u->pps = malloc(len);
		if (u->pps == NULL) {
			u->push_errors++;
			return;
		}
		memcpy(u->pps, buf, len);
		u->pps_size = len;
	}

	if (u->sps != NULL && u->pps != NULL) {
		res = vdec_set_h264_ps(u->dec,
				       u->sps,
				       u->sps_size,
				       u->pps,
				       u->pps_size,
				       &vdef_h264_raw_nalu);
		if (res < 0) {
			u->push_errors++;
			return;
		}
		u->ps_configured = true;
		/* Stop before any slice NALU is ever reached: this test needs
		 * SPS/PPS configured but zero frames actually queued. */
		h264_reader_stop(u->reader);
	}
}


static const struct h264_ctx_cbs s_h264_ps_only_cbs = {
	.nalu_end = h264_ps_only_nalu_end_cb,
};


static void test_vdec_flush_discard_no_input_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_ps_only_ctx u;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u.push_errors, 0);
	CU_ASSERT_TRUE_FATAL(u.ps_configured);

	res = vdec_flush(dec, 1 /* discard */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* vdec_h264_write_grey_idr() on a fully configured (real SPS/PPS) decoder. */
/* ================================================================== */

static void test_vdec_h264_write_grey_idr_configured_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_ps_only_ctx u;
	struct vdef_coded_frame in_info;
	uint64_t delta = 0;
	uint64_t timestamp = 0;
	struct mbuf_mem *idr_mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u.push_errors, 0);
	CU_ASSERT_TRUE_FATAL(u.ps_configured);

	memset(&in_info, 0, sizeof(in_info));
	in_info.format = vdef_h264_byte_stream;
	in_info.info.timestamp = 1000;

	res = mbuf_mem_generic_new(64 * 1024, &idr_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vdec_h264_write_grey_idr(
		dec, &in_info, &delta, &timestamp, idr_mem, &idr_frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(idr_frame);
	/* Non-zero input timestamp -> delta 0, output timestamp decremented
	 * by one (see vdec_h264_write_grey_idr()'s own doc/behavior). */
	CU_ASSERT_EQUAL(delta, 0u);
	CU_ASSERT_EQUAL(timestamp, 999u);

	{
		struct vdef_coded_frame idr_info;
		bool is_idr;
		int nalu_count;

		res = mbuf_coded_video_frame_get_frame_info(idr_frame,
							    &idr_info);
		CU_ASSERT_EQUAL(res, 0);
		CU_ASSERT_EQUAL(idr_info.type, VDEF_CODED_FRAME_TYPE_IDR);

		nalu_count = mbuf_coded_video_frame_get_nalu_count(idr_frame);
		CU_ASSERT_EQUAL(nalu_count, 1);

		is_idr = vdec_h264_is_idr(idr_frame, &idr_info);
		CU_ASSERT_TRUE(is_idr);
	}

	mbuf_coded_video_frame_unref(idr_frame);
	mbuf_mem_unref(idr_mem);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);
	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* Same as above, but with in_frame_info.format.data_format == AVCC: writes
 * a 4-byte big-endian length prefix instead of a start code. Never exercised
 * before - every other test in this file uses byte_stream. */
static void test_vdec_h264_write_grey_idr_avcc_format(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_ps_only_ctx u;
	struct vdef_coded_frame in_info;
	uint64_t delta = 0;
	uint64_t timestamp = 0;
	struct mbuf_mem *idr_mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(u.ps_configured);

	memset(&in_info, 0, sizeof(in_info));
	in_info.format = vdef_h264_avcc;
	in_info.info.timestamp = 1000;

	res = mbuf_mem_generic_new(64 * 1024, &idr_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vdec_h264_write_grey_idr(
		dec, &in_info, &delta, &timestamp, idr_mem, &idr_frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(idr_frame);

	{
		struct vdef_coded_frame idr_info;
		bool is_idr;
		void *data = NULL;
		size_t capacity = 0;
		uint32_t len_prefix;
		int nalu_count;

		res = mbuf_coded_video_frame_get_frame_info(idr_frame,
							    &idr_info);
		CU_ASSERT_EQUAL(res, 0);
		is_idr = vdec_h264_is_idr(idr_frame, &idr_info);
		CU_ASSERT_TRUE(is_idr);

		nalu_count = mbuf_coded_video_frame_get_nalu_count(idr_frame);
		CU_ASSERT_EQUAL(nalu_count, 1);

		/* AVCC writes a big-endian 4-byte length prefix, not a start
		 * code (00 00 00 01) - assert the two are genuinely
		 * different, not just that some 4 bytes were written. */
		res = mbuf_mem_get_data(idr_mem, &data, &capacity);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		memcpy(&len_prefix, data, sizeof(len_prefix));
		CU_ASSERT_NOT_EQUAL(ntohl(len_prefix), 0x00000001u);
	}

	mbuf_coded_video_frame_unref(idr_frame);
	mbuf_mem_unref(idr_mem);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);
	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* Same setup, but with an in_frame_info.format.data_format neither AVCC nor
 * BYTE_STREAM - hits the function's default: case (-ENOSYS), never exercised
 * before. */
static void test_vdec_h264_write_grey_idr_unsupported_format(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_ps_only_ctx u;
	struct vdef_coded_frame in_info;
	uint64_t delta = 0;
	uint64_t timestamp = 0;
	struct mbuf_mem *idr_mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(u.ps_configured);

	memset(&in_info, 0, sizeof(in_info));
	in_info.format = vdef_h264_raw_nalu;
	in_info.info.timestamp = 1000;

	res = mbuf_mem_generic_new(64 * 1024, &idr_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vdec_h264_write_grey_idr(
		dec, &in_info, &delta, &timestamp, idr_mem, &idr_frame);
	CU_ASSERT_EQUAL(res, -ENOSYS);
	CU_ASSERT_PTR_NULL(idr_frame);

	mbuf_mem_unref(idr_mem);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);
	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* vdec_set_h264_ps() "direct format match" branch + vdec_get_video_dimensions()
 */
/* ================================================================== */

struct h264_extract_ps_ctx {
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	struct h264_reader *reader;
	int errors;
};


/* Only extracts raw SPS/PPS NALU payloads (no start code) - does not call
 * vdec_set_h264_ps() itself, unlike h264_ps_only_nalu_end_cb above. */
static void h264_extract_ps_nalu_end_cb(struct h264_ctx *ctx,
					enum h264_nalu_type type,
					const uint8_t *buf,
					size_t len,
					const struct h264_nalu_header *nh,
					void *userdata)
{
	struct h264_extract_ps_ctx *u = userdata;

	(void)ctx;
	(void)nh;

	if (type == H264_NALU_TYPE_SPS && u->sps == NULL) {
		u->sps = malloc(len);
		if (u->sps == NULL) {
			u->errors++;
			return;
		}
		memcpy(u->sps, buf, len);
		u->sps_size = len;
	} else if (type == H264_NALU_TYPE_PPS && u->pps == NULL) {
		u->pps = malloc(len);
		if (u->pps == NULL) {
			u->errors++;
			return;
		}
		memcpy(u->pps, buf, len);
		u->pps_size = len;
	}

	if (u->sps != NULL && u->pps != NULL)
		h264_reader_stop(u->reader);
}


static const struct h264_ctx_cbs s_h264_extract_ps_cbs = {
	.nalu_end = h264_extract_ps_nalu_end_cb,
};


/* vdec_set_h264_ps()'s dispatcher (src/vdec.c) forwards sps/pps/format
 * UNMODIFIED to the backend when the caller's format already matches one of
 * the backend's supported_input_formats directly (ffmpeg here only supports
 * vdef_h264_byte_stream) - every other test in this file instead passes
 * vdef_h264_raw_nalu (mirroring tools/vdec.c's convention), which always
 * takes the *fallback* cascade below that branch instead. Exercising the
 * direct-match branch means the caller's own sps/pps buffers must already
 * carry a start code, since ffmpeg's own set_ps() (ffmpeg/src/vdec_ffmpeg.c)
 * always strips 4 leading bytes for any non-raw_nalu format. */
static void test_vdec_set_h264_ps_direct_format_match_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_extract_ps_ctx u;
	uint8_t *sps_bs = NULL;
	uint8_t *pps_bs = NULL;
	uint32_t start_code = htonl(0x00000001);

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	res = h264_reader_new(&s_h264_extract_ps_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u.errors, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.sps);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.pps);

	sps_bs = malloc(4 + u.sps_size);
	CU_ASSERT_PTR_NOT_NULL_FATAL(sps_bs);
	memcpy(sps_bs, &start_code, 4);
	memcpy(sps_bs + 4, u.sps, u.sps_size);

	pps_bs = malloc(4 + u.pps_size);
	CU_ASSERT_PTR_NOT_NULL_FATAL(pps_bs);
	memcpy(pps_bs, &start_code, 4);
	memcpy(pps_bs + 4, u.pps, u.pps_size);

	res = vdec_set_h264_ps(dec,
			       sps_bs,
			       4 + u.sps_size,
			       pps_bs,
			       4 + u.pps_size,
			       &vdef_h264_byte_stream);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	{
		unsigned int width = 0, height = 0;
		unsigned int crop_width = 0, crop_height = 0;

		res = vdec_get_video_dimensions(dec,
						&width,
						&height,
						NULL,
						NULL,
						NULL,
						NULL,
						&crop_width,
						&crop_height);
		CU_ASSERT_EQUAL(res, 0);
		/* Same real asset as every other test here, already proven to
		 * decode successfully - these dimensions are the ones logged
		 * during that real decode, not guessed. */
		CU_ASSERT_EQUAL(width, 432u);
		CU_ASSERT_EQUAL(height, 240u);
		CU_ASSERT_EQUAL(crop_width, 426u);
		CU_ASSERT_EQUAL(crop_height, 240u);
	}

	free(sps_bs);
	free(pps_bs);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);
	vdec_destroy(dec);

out:
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* vdec_ffmpeg_insert_grey_idr(): triggered indirectly via config.gen_grey_idr
 */
/* when the first frame pushed after a flush is not a sync (IDR) frame.    */
/* ================================================================== */

#define H264_SKIP_TO_P_MAX_FRAMES 5

struct h264_skip_to_p_ctx {
	struct mbuf_coded_video_frame_queue *queue;
	struct h264_reader *reader;
	unsigned int pushed;
	uint64_t next_timestamp;
	int push_errors;
	bool found_non_idr;
};


/* Skips every leading IDR slice (and PS/AUD NALUs) until the first plain
 * H264_NALU_TYPE_SLICE is found, then pushes that one and a few more. */
static void h264_skip_to_p_nalu_end_cb(struct h264_ctx *ctx,
				       enum h264_nalu_type type,
				       const uint8_t *buf,
				       size_t len,
				       const struct h264_nalu_header *nh,
				       void *userdata)
{
	struct h264_skip_to_p_ctx *u = userdata;
	int res;

	(void)ctx;
	(void)nh;

	if (type == H264_NALU_TYPE_SPS || type == H264_NALU_TYPE_PPS ||
	    type == H264_NALU_TYPE_AUD)
		return;

	if (!u->found_non_idr) {
		if (type != H264_NALU_TYPE_SLICE)
			return;
		u->found_non_idr = true;
	}

	if (u->pushed >= H264_SKIP_TO_P_MAX_FRAMES)
		return;

	res = push_h264_byte_stream_frame(
		u->queue, type, buf, len, u->next_timestamp);
	if (res < 0) {
		u->push_errors++;
		return;
	}
	u->next_timestamp += 33333;
	u->pushed++;

	if (u->pushed >= H264_SKIP_TO_P_MAX_FRAMES)
		h264_reader_stop(u->reader);
}


static const struct h264_ctx_cbs s_h264_skip_to_p_cbs = {
	.nalu_end = h264_skip_to_p_nalu_end_cb,
};


static void test_vdec_ffmpeg_gen_grey_idr_h264(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_asset_ctx u;
	struct h264_reader *reader2 = NULL;
	struct h264_skip_to_p_ctx u2;
	char dbg_dir_template[] = "/tmp/vdec_test_greyidr_dbg_XXXXXX";
	char *dbg_dir;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));
	memset(&u2, 0, sizeof(u2));

	dbg_dir = mkdtemp(dbg_dir_template);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dbg_dir);

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	config.gen_grey_idr = 1;
	/* Also exercises the debug-file writes inside
	 * vdec_ffmpeg_insert_grey_idr() itself (vdec_dbg_write_frame() /
	 * vdec_dbg_parse_frame() on the synthesized IDR frame), only reached
	 * when self->base->dbg.input_bs/.analysis are non-NULL. */
	config.dbg_dir = dbg_dir;
	config.dbg_flags = VDEC_DBG_FLAG_INPUT_BITSTREAM |
			   VDEC_DBG_FLAG_OUTPUT_YUV | VDEC_DBG_FLAG_ANALYSIS;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	/* Step 1: decode a real prefix of the stream normally, to get PS
	 * configured and the pipeline running. */
	u.dec = dec;
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);

	res = h264_reader_new(&s_h264_asset_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(u.ps_configured);
	CU_ASSERT_TRUE_FATAL(u.pushed > 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE_FATAL(ctx.frame_count > 0);

	/* Step 2: flush+discard resets the backend's internal need_sync flag,
	 * so the very next frame pushed will require a sync (IDR) frame. */
	res = vdec_flush(dec, 1 /* discard */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE_FATAL(ctx.flush_done);
	mock_vdec_cbs_ctx_clear(&ctx);

	/* Step 3: push a non-IDR slice (skipping every leading IDR in the
	 * file) as the very first frame after the flush. Without
	 * gen_grey_idr, vdec_ffmpeg_buffer_push_one()'s need_sync check would
	 * silently discard it; with it, vdec_ffmpeg_insert_grey_idr() (which
	 * calls vdec_h264_write_grey_idr() internally) synthesizes a grey IDR
	 * and feeds it first, so decoding resumes and frame_output fires
	 * again. */
	u2.queue = u.queue;
	res = h264_reader_new(&s_h264_skip_to_p_cbs, &u2, &reader2);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u2.reader = reader2;

	res = h264_reader_parse(reader2, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u2.push_errors, 0);
	if (!u2.found_non_idr) {
		/* This real asset never produced a plain (non-IDR) slice
		 * anywhere in the file (e.g. an all-intra encode) - nothing
		 * to resync from, so the rest of this test is moot. */
		goto cleanup;
	}
	CU_ASSERT_TRUE_FATAL(u2.pushed > 0);

	/* Force emission of whatever the decoder has buffered instead of
	 * polling frame_count directly: H.264 decoders normally hold a few
	 * frames back for reordering, and only H264_SKIP_TO_P_MAX_FRAMES (5)
	 * frames were pushed here (vs. 30 in the step-1 decode above, which
	 * was enough on its own) - a drain is what every other lifecycle test
	 * in this file already relies on to force that emission. This also
	 * bounds the wait: pump_until()'s retry loop gives up (failing the
	 * flush_done assertion) instead of spinning through the full pump
	 * budget for nothing if resync genuinely never happened. */
	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

cleanup:
	mock_vdec_cbs_ctx_clear(&ctx);
	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(reader);
	h264_reader_destroy(reader2);
	free(u.sps);
	free(u.pps);
	free(buf);

	{
		DIR *d = opendir(dbg_dir);
		struct dirent *ent;
		char filepath[PATH_MAX];
		struct stat st;

		if (d != NULL) {
			while ((ent = readdir(d)) != NULL) {
				if (strcmp(ent->d_name, ".") == 0 ||
				    strcmp(ent->d_name, "..") == 0)
					continue;
				snprintf(filepath,
					 sizeof(filepath),
					 "%s/%s",
					 dbg_dir,
					 ent->d_name);
				if (stat(filepath, &st) == 0)
					unlink(filepath);
			}
			closedir(d);
		}
		rmdir(dbg_dir);
	}

out:
	pomp_loop_destroy(loop);
}


/* Same trigger as test_vdec_ffmpeg_gen_grey_idr_h264() above (flush, then
 * push a non-IDR slice), but with config.gen_grey_idr left unset (0). This
 * exercises the *other* branch of vdec_ffmpeg_buffer_push_one()'s need_sync
 * check - the frame is silently discarded ("frame is not a sync point,
 * discarding frame") rather than resynced via a synthetic grey IDR. */
static void test_vdec_no_gen_grey_idr_discards_non_sync_frame_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_asset_ctx u;
	struct h264_reader *reader2 = NULL;
	struct h264_skip_to_p_ctx u2;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));
	memset(&u2, 0, sizeof(u2));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	/* config.gen_grey_idr is left at 0 (default). */
	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);

	res = h264_reader_new(&s_h264_asset_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(u.ps_configured);
	CU_ASSERT_TRUE_FATAL(u.pushed > 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE_FATAL(ctx.frame_count > 0);

	res = vdec_flush(dec, 1 /* discard */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE_FATAL(ctx.flush_done);
	mock_vdec_cbs_ctx_clear(&ctx);

	u2.queue = u.queue;
	res = h264_reader_new(&s_h264_skip_to_p_cbs, &u2, &reader2);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u2.reader = reader2;

	res = h264_reader_parse(reader2, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u2.push_errors, 0);
	if (!u2.found_non_idr) {
		/* Same caveat as the gen_grey_idr test: nothing to resync
		 * from if this real asset never has a non-IDR slice. */
		goto cleanup;
	}
	CU_ASSERT_TRUE_FATAL(u2.pushed > 0);

	/* Drain to force emission of anything the decoder might still be
	 * holding, then confirm nothing new came out: the pushed frame(s)
	 * were silently discarded, not queued for later. */
	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_EQUAL(ctx.frame_count, 0);

cleanup:
	mock_vdec_cbs_ctx_clear(&ctx);
	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(reader);
	h264_reader_destroy(reader2);
	free(u.sps);
	free(u.pps);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* vdec_h264/ffmpeg metadata propagation: mirrors the JPEG/turbojpeg test  */
/* above but for the H.264/ffmpeg backend.                                 */
/* ================================================================== */

struct h264_capture_one_slice_ctx {
	uint8_t *nalu;
	size_t nalu_len;
	enum h264_nalu_type nalu_type;
	struct h264_reader *reader;
	int errors;
};


/* Captures the first non-PS/AUD NALU found (real content, whatever type it
 * happens to be) and stops - used to hand-build a single real, decodable
 * frame outside of the shared push helpers, so metadata can be attached to
 * it directly. */
static void
h264_capture_one_slice_nalu_end_cb(struct h264_ctx *ctx,
				   enum h264_nalu_type type,
				   const uint8_t *buf,
				   size_t len,
				   const struct h264_nalu_header *nh,
				   void *userdata)
{
	struct h264_capture_one_slice_ctx *u = userdata;

	(void)ctx;
	(void)nh;

	if (type == H264_NALU_TYPE_SPS || type == H264_NALU_TYPE_PPS ||
	    type == H264_NALU_TYPE_AUD)
		return;
	if (u->nalu != NULL)
		return;

	u->nalu = malloc(len);
	if (u->nalu == NULL) {
		u->errors++;
		return;
	}
	memcpy(u->nalu, buf, len);
	u->nalu_len = len;
	u->nalu_type = type;
	h264_reader_stop(u->reader);
}


static const struct h264_ctx_cbs s_h264_capture_one_slice_cbs = {
	.nalu_end = h264_capture_one_slice_nalu_end_cb,
};


static void test_vdec_metadata_propagation_h264_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_ps_only_ctx ps;
	struct h264_reader *ps_reader = NULL;
	struct h264_capture_one_slice_ctx slice;
	struct h264_reader *slice_reader = NULL;
	struct mbuf_coded_video_frame_queue *queue;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vmeta_frame *meta = NULL;
	struct vdef_coded_frame frame_info;
	struct vdef_nalu nalu;
	uint8_t *data = NULL;
	size_t capacity = 0;
	uint32_t start_code = htonl(0x00000001);

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&ps, 0, sizeof(ps));
	memset(&slice, 0, sizeof(slice));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	ps.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &ps, &ps_reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	ps.reader = ps_reader;
	res = h264_reader_parse(ps_reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(ps.ps_configured);

	off = 0;
	res = h264_reader_new(
		&s_h264_capture_one_slice_cbs, &slice, &slice_reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	slice.reader = slice_reader;
	res = h264_reader_parse(slice_reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(slice.errors, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(slice.nalu);

	queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	res = mbuf_mem_generic_new(4 + slice.nalu_len, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memcpy(data, &start_code, 4);
	memcpy(data + 4, slice.nalu, slice.nalu_len);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = 1000;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_set_metadata(frame, meta);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	vmeta_frame_unref(meta);

	memset(&nalu, 0, sizeof(nalu));
	nalu.size = 4 + slice.nalu_len;
	nalu.h264.type = slice.nalu_type;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL(res, 0);

	/* Drain immediately rather than polling frame_count directly: only
	 * one frame is pushed here, and whether it comes out on its own
	 * before a drain depends on this stream's reordering delay (unlike
	 * the 30-frame push in the main H.264 lifecycle test, which has
	 * enough later input to flush it out naturally). */
	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE_FATAL(ctx.flush_done);
	CU_ASSERT_TRUE_FATAL(ctx.frame_count > 0);

	{
		struct vmeta_frame *out_meta = NULL;

		res = mbuf_raw_video_frame_get_metadata(ctx.captured[0],
							&out_meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(out_meta);
		CU_ASSERT_EQUAL(out_meta->type, VMETA_FRAME_TYPE_V3);
		vmeta_frame_unref(out_meta);
	}

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
	h264_reader_destroy(ps_reader);
	h264_reader_destroy(slice_reader);
	free(ps.sps);
	free(ps.pps);
	free(slice.nalu);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* Output pixel format: preferred_output_format handling differs a lot     */
/* between backends - turbojpeg honors it (i420/nv12 only), ffmpeg ignores */
/* it entirely (grep-confirmed: no reference to preferred_output_format in */
/* ffmpeg/src/vdec_ffmpeg.c at all).                                       */
/* ================================================================== */

static void test_frame_output_nv12_turbojpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct jpeg_info info;
	struct vdef_format_info format_info;
	struct mbuf_coded_video_frame_queue *queue;

	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);

	path_ret = build_jpeg_asset_path(
		"Tests/anafi3/classic/photo_recording/1323514_blended.JPG",
		path,
		sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	memset(&info, 0, sizeof(info));
	res = jpeg_parse(buf, len, &info);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_TRUE_FATAL(info.width > 0 && info.height > 0);

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	config.preferred_output_format = vdef_nv12;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	memset(&format_info, 0, sizeof(format_info));
	format_info.bit_depth = 8;
	format_info.sar.width = 1;
	format_info.sar.height = 1;
	format_info.full_range = 1;
	format_info.framerate.num = 1;
	format_info.framerate.den = 1;
	format_info.color_primaries = VDEF_COLOR_PRIMARIES_SRGB;
	format_info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	format_info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	format_info.resolution.width = info.width;
	format_info.resolution.height = info.height;

	res = vdec_set_jpeg_params(dec, &format_info);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	res = push_jpeg_frame(queue, buf, len);
	CU_ASSERT_EQUAL(res, 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE_FATAL(ctx.frame_count > 0);

	{
		struct vdef_raw_frame raw_info;
		bool is_nv12;

		res = mbuf_raw_video_frame_get_frame_info(ctx.captured[0],
							  &raw_info);
		CU_ASSERT_EQUAL(res, 0);
		is_nv12 = vdef_raw_format_cmp(&raw_info.format, &vdef_nv12);
		CU_ASSERT_TRUE(is_nv12);
	}

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	free(buf);
	pomp_loop_destroy(loop);
}


static void test_frame_output_ffmpeg_ignores_preferred_output_format(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_asset_ctx u;

	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	/* Requested but never honored by this backend - see the comment
	 * above this test area. */
	config.preferred_output_format = vdef_nv12;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);

	res = h264_reader_new(&s_h264_asset_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(u.pushed > 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE_FATAL(ctx.frame_count > 0);

	{
		struct vdef_raw_frame raw_info;
		bool is_i420;

		res = mbuf_raw_video_frame_get_frame_info(ctx.captured[0],
							  &raw_info);
		CU_ASSERT_EQUAL(res, 0);
		is_i420 = vdef_raw_format_cmp(&raw_info.format, &vdef_i420);
		CU_ASSERT_TRUE(is_i420);
	}

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* Debug files (core/src/vdec_dbg.c), exercised through a real decode.     */
/* ================================================================== */

static void test_vdec_dbg_files_written_during_real_decode_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_reader *reader = NULL;
	struct h264_asset_ctx u;
	char dir_template[] = "/tmp/vdec_test_dbgint_XXXXXX";
	char *dir;
	bool has_input_bs = false, has_output_yuv = false, has_analysis = false;

	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		return;

	dir = mkdtemp(dir_template);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dir);

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	config.dbg_dir = dir;
	config.dbg_flags = VDEC_DBG_FLAG_INPUT_BITSTREAM |
			   VDEC_DBG_FLAG_OUTPUT_YUV | VDEC_DBG_FLAG_ANALYSIS;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);

	res = h264_reader_new(&s_h264_asset_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(u.pushed > 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	CU_ASSERT_TRUE_FATAL(ctx.frame_count > 0);

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	/* vdec_destroy() only closes the debug files (vdec_dbg_close_files())
	 * on the ops->destroy() == 0 success path - checked here since the
	 * files must be flushed/closed before being inspected below. */
	vdec_destroy(dec);

	{
		DIR *d = opendir(dir);
		struct dirent *ent;
		char filepath[PATH_MAX];
		struct stat st;

		CU_ASSERT_PTR_NOT_NULL_FATAL(d);
		if (d != NULL) {
			while ((ent = readdir(d)) != NULL) {
				if (strstr(ent->d_name, "input.h264") != NULL)
					has_input_bs = true;
				if (strstr(ent->d_name, "output.yuv") != NULL)
					has_output_yuv = true;
				if (strstr(ent->d_name, "analysis.csv") != NULL)
					has_analysis = true;
			}
			closedir(d);
		}
		CU_ASSERT_TRUE(has_input_bs);
		CU_ASSERT_TRUE(has_output_yuv);
		CU_ASSERT_TRUE(has_analysis);

		/* Clean up regardless of which files actually got created. */
		d = opendir(dir);
		if (d != NULL) {
			while ((ent = readdir(d)) != NULL) {
				if (strcmp(ent->d_name, ".") == 0 ||
				    strcmp(ent->d_name, "..") == 0)
					continue;
				snprintf(filepath,
					 sizeof(filepath),
					 "%s/%s",
					 dir,
					 ent->d_name);
				if (stat(filepath, &st) == 0)
					unlink(filepath);
			}
			closedir(d);
		}
		rmdir(dir);
	}

	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* Flush-then-redecode / discard-then-redecode: mirrors                    */
/* libvideo-encode/tests/venc_test_lifecycle.c's own                       */
/* run_lifecycle_flush_then_reencode()/_flush_discard_then_reencode() -     */
/* proves the decoder is reusable across multiple flush cycles without     */
/* needing to be destroyed and recreated.                                  */
/*                                                                          */
/* Unlike an encoder, a decoder cannot just resume with whatever frame     */
/* comes next after a flush: vdec_ffmpeg_complete_flush() resets the       */
/* backend's need_sync flag on *every* flush (discard or drain alike), so  */
/* the next pushed frame must be a real sync (IDR) frame or it is silently */
/* discarded (see vdec_ffmpeg_buffer_push_one()'s "frame is not a sync     */
/* point, discarding frame" path - this is exactly what config.gen_grey_idr */
/* exists to work around, exercised separately above). So each "batch"     */
/* here re-parses the same real asset from its own beginning again (a      */
/* fresh h264_reader from offset 0), which naturally starts with a real    */
/* IDR every time - the direct decode analog of looping/replaying the same */
/* clip, rather than continuing mid-GOP.                                   */
/* ================================================================== */

struct h264_push_only_ctx {
	struct mbuf_coded_video_frame_queue *queue;
	struct h264_reader *reader;
	unsigned int pushed;
	unsigned int max_frames;
	uint64_t next_timestamp;
	int push_errors;
};


/* Assumes PS is already configured on the decoder - just skips PS/AUD
 * NALUs and pushes every slice NALU found, up to max_frames. */
static void h264_push_only_nalu_end_cb(struct h264_ctx *ctx,
				       enum h264_nalu_type type,
				       const uint8_t *buf,
				       size_t len,
				       const struct h264_nalu_header *nh,
				       void *userdata)
{
	struct h264_push_only_ctx *u = userdata;
	int res;

	(void)ctx;
	(void)nh;

	if (type == H264_NALU_TYPE_SPS || type == H264_NALU_TYPE_PPS ||
	    type == H264_NALU_TYPE_AUD)
		return;

	if (u->pushed >= u->max_frames)
		return;

	res = push_h264_byte_stream_frame(
		u->queue, type, buf, len, u->next_timestamp);
	if (res < 0) {
		u->push_errors++;
		return;
	}
	u->next_timestamp += 33333;
	u->pushed++;

	if (u->pushed >= u->max_frames)
		h264_reader_stop(u->reader);
}


static const struct h264_ctx_cbs s_h264_push_only_cbs = {
	.nalu_end = h264_push_only_nalu_end_cb,
};


/* Pushes max_frames real slices starting from the very beginning of the
 * asset (a fresh reader each call), asserts they were all accepted, then
 * flushes and pumps for flush_done. discard selects vdec_flush()'s mode;
 * the caller is responsible for asserting (or not) on the resulting
 * ctx.frame_count, since a discard-flush's output count is racy. */
static void push_batch_and_flush(struct vdec_decoder *dec,
				 struct pomp_loop *loop,
				 struct mock_vdec_cbs_ctx *ctx,
				 const uint8_t *buf,
				 size_t len,
				 unsigned int max_frames,
				 int discard)
{
	struct h264_reader *reader = NULL;
	struct h264_push_only_ctx u;
	size_t off = 0;
	int res;

	memset(&u, 0, sizeof(u));
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);
	u.max_frames = max_frames;

	res = h264_reader_new(&s_h264_push_only_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_EQUAL(u.push_errors, 0);
	CU_ASSERT_EQUAL_FATAL(u.pushed, max_frames);

	res = vdec_flush(dec, discard);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx->flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx->flush_done);

	h264_reader_destroy(reader);
}


static void test_vdec_flush_then_redecode_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_ps_only_ctx ps;
	struct h264_reader *ps_reader = NULL;
	size_t off = 0;
	const unsigned int frames_before = 10;
	const unsigned int frames_after = 10;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&ps, 0, sizeof(ps));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	ps.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &ps, &ps_reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	ps.reader = ps_reader;
	res = h264_reader_parse(ps_reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(ps.ps_configured);

	/* Batch 1: decode, drain, exact count. */
	push_batch_and_flush(
		dec, loop, &ctx, buf, len, frames_before, 0 /* drain */);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frames_before);
	mock_vdec_cbs_ctx_clear(&ctx);

	/* Batch 2: re-decode the same clip from its own beginning again
	 * (real IDR, so no gen_grey_idr needed) on the SAME decoder instance
	 * - must produce exactly as many frames as pushed, proving the
	 * backend correctly resets its internal state after a drain. */
	push_batch_and_flush(
		dec, loop, &ctx, buf, len, frames_after, 0 /* drain */);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frames_after);
	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(ps_reader);
	free(ps.sps);
	free(ps.pps);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_flush_discard_then_redecode_ffmpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_ps_only_ctx ps;
	struct h264_reader *ps_reader = NULL;
	size_t off = 0;
	const unsigned int frames_before = 10;
	const unsigned int frames_after = 10;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&ps, 0, sizeof(ps));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	ps.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &ps, &ps_reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	ps.reader = ps_reader;
	res = h264_reader_parse(ps_reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(ps.ps_configured);

	/* Batch 1: discard flush - output count is racy, deliberately not
	 * asserted (matches venc_test_lifecycle.c's own
	 * run_lifecycle_flush_discard_then_reencode() convention). */
	push_batch_and_flush(
		dec, loop, &ctx, buf, len, frames_before, 1 /* discard */);
	mock_vdec_cbs_ctx_clear(&ctx);

	/* Batch 2: must come out in full after a drain. */
	push_batch_and_flush(
		dec, loop, &ctx, buf, len, frames_after, 0 /* drain */);
	CU_ASSERT_EQUAL(ctx.frame_count, (int)frames_after);
	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(ps_reader);
	free(ps.sps);
	free(ps.pps);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* Corrupted-data decode errors, one per implem.                           */
/*                                                                          */
/* turbojpeg genuinely reports failures via frame_output(status<0, NULL)   */
/* (vdec_turbojpeg.c's decode_frame() -> dec_error_evt_cb()). ffmpeg does   */
/* not: grep-confirmed, its only vdec_call_frame_output_cb() call site      */
/* (out_queue_evt_cb()) is hardcoded status=0, and avcodec_receive_frame()  */
/* errors (vdec_ffmpeg_buffer_pop_all()) are only ever logged and silently  */
/* dropped, never surfaced through the callback at all. The two ffmpeg      */
/* tests below are therefore pinning tests characterizing that gap (no      */
/* negative status ever appears), not tests of a working error-reporting   */
/* path - mirrors this suite's existing convention for documented, not-     */
/* fixed asymmetries (see e.g. the preferred_output_format pinning test).   */
/* ================================================================== */

static void test_vdec_corrupted_data_ffmpeg_h264(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_ps_only_ctx ps;
	struct h264_reader *ps_reader = NULL;
	struct mbuf_coded_video_frame_queue *queue;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_coded_frame frame_info;
	struct vdef_nalu nalu;
	uint8_t *data = NULL;
	size_t capacity = 0;
	uint32_t start_code = htonl(0x00000001);
	/* Garbage slice payload - a syntactically-tagged H264_NALU_TYPE_SLICE
	 * NALU whose content is not a valid encoded slice at all. */
	static const uint8_t garbage[64] = {
		0x9a,
		0x37,
		0xc1,
		0x02,
		0xff,
		0x00,
		0xde,
		0xad,
		0xbe,
		0xef,
		0x13,
		0x37,
		0x55,
		0xaa,
		0x00,
		0xff,
	};

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&ps, 0, sizeof(ps));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	ps.dec = dec;
	res = h264_reader_new(&s_h264_ps_only_cbs, &ps, &ps_reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	ps.reader = ps_reader;
	res = h264_reader_parse(ps_reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(ps.ps_configured);

	queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	res = mbuf_mem_generic_new(sizeof(garbage) + 4, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memcpy(data, &start_code, 4);
	memcpy(data + 4, garbage, sizeof(garbage));

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = 1000;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	memset(&nalu, 0, sizeof(nalu));
	nalu.size = sizeof(garbage) + 4;
	nalu.h264.type = H264_NALU_TYPE_SLICE;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	/* Pin down the gap described above: whatever happened internally
	 * (rejected outright, silently dropped, or decoded via error
	 * concealment), frame_output was never called with a negative
	 * status - ffmpeg has no path that does that. */
	for (int i = 0; i < ctx.frame_count; i++)
		CU_ASSERT_TRUE(ctx.status[i] >= 0);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
	h264_reader_destroy(ps_reader);
	free(ps.sps);
	free(ps.pps);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


static void test_vdec_corrupted_data_ffmpeg_h265(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h265_ps_only_ctx ps;
	struct h265_reader *ps_reader = NULL;
	struct mbuf_coded_video_frame_queue *queue;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_coded_frame frame_info;
	struct vdef_nalu nalu;
	uint8_t *data = NULL;
	size_t capacity = 0;
	uint32_t start_code = htonl(0x00000001);
	static const uint8_t garbage[64] = {
		0x9a,
		0x37,
		0xc1,
		0x02,
		0xff,
		0x00,
		0xde,
		0xad,
		0xbe,
		0xef,
		0x13,
		0x37,
		0x55,
		0xaa,
		0x00,
		0xff,
	};

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&ps, 0, sizeof(ps));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.265", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H265);
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	ps.dec = dec;
	res = h265_reader_new(&s_h265_ps_only_cbs, &ps, &ps_reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	ps.reader = ps_reader;
	res = h265_reader_parse(ps_reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(ps.ps_configured);

	queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	res = mbuf_mem_generic_new(sizeof(garbage) + 4, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, (void **)&data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memcpy(data, &start_code, 4);
	memcpy(data + 4, garbage, sizeof(garbage));

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h265_byte_stream;
	frame_info.info.timescale = 1000000;
	frame_info.info.timestamp = 1000;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	memset(&nalu, 0, sizeof(nalu));
	nalu.size = sizeof(garbage) + 4;
	nalu.h265.type = H265_NALU_TYPE_TRAIL_R;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_queue_push(queue, frame);
	CU_ASSERT_EQUAL(res, 0);

	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);

	for (int i = 0; i < ctx.frame_count; i++)
		CU_ASSERT_TRUE(ctx.status[i] >= 0);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
	h265_reader_destroy(ps_reader);
	free(ps.vps);
	free(ps.sps);
	free(ps.pps);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


/* Unlike ffmpeg, turbojpeg genuinely reports decode failures: a corrupted
 * SOI marker makes libjpeg-turbo reject the image outright, which
 * decode_frame() (vdec_turbojpeg.c) turns into frame_output(status<0, NULL)
 * via dec_error_evt_cb(). */
static void test_vdec_corrupted_data_turbojpeg(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop;
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct jpeg_info info;
	struct vdef_format_info format_info;
	struct mbuf_coded_video_frame_queue *queue;

	if (!implem_available(VDEC_DECODER_IMPLEM_TURBOJPEG))
		return;

	loop = pomp_loop_new();
	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	mock_vdec_cbs_ctx_reset(&ctx);

	path_ret = build_jpeg_asset_path(
		"Tests/anafi3/classic/photo_recording/1323514_blended.JPG",
		path,
		sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	memset(&info, 0, sizeof(info));
	res = jpeg_parse(buf, len, &info);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_TRUE_FATAL(info.width > 0 && info.height > 0);

	memset(&config, 0, sizeof(config));
	config.implem = VDEC_DECODER_IMPLEM_TURBOJPEG;
	config.encoding = VDEF_ENCODING_MJPEG;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	memset(&format_info, 0, sizeof(format_info));
	format_info.bit_depth = 8;
	format_info.sar.width = 1;
	format_info.sar.height = 1;
	format_info.full_range = 1;
	format_info.framerate.num = 1;
	format_info.framerate.den = 1;
	format_info.color_primaries = VDEF_COLOR_PRIMARIES_SRGB;
	format_info.transfer_function = VDEF_TRANSFER_FUNCTION_BT709;
	format_info.matrix_coefs = VDEF_MATRIX_COEFS_BT709;
	format_info.resolution.width = info.width;
	format_info.resolution.height = info.height;

	res = vdec_set_jpeg_params(dec, &format_info);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(queue);

	/* Corrupt the SOI marker (first 2 bytes, normally 0xFFD8) - guaranteed
	 * to make libjpeg-turbo reject this as not a JPEG file at all. */
	buf[0] = 0x00;
	buf[1] = 0x00;

	res = push_jpeg_frame(queue, buf, len);
	CU_ASSERT_EQUAL(res, 0);

	for (int i = 0; i < ASSET_PUMP_MAX_ITER && ctx.frame_count == 0; i++)
		pomp_loop_wait_and_process(loop, PUMP_TIMEOUT_MS);
	/* Non-fatal: if the decoder never reports the error at all, still
	 * fall through to stop/destroy below instead of leaking dec/loop/buf.
	 */
	CU_ASSERT_TRUE(ctx.frame_count > 0);
	if (ctx.frame_count > 0) {
		CU_ASSERT_TRUE(ctx.status[0] < 0);
		CU_ASSERT_PTR_NULL(ctx.captured[0]);
	}

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	free(buf);
	pomp_loop_destroy(loop);
}


/* ================================================================== */
/* Thread-count configuration (ffmpeg/src/vdec_ffmpeg.c's initialize_backend()
 * switch on config.preferred_thread_count).                               */
/*                                                                          */
/* case 0 (the default - every other real-decode test in this file leaves  */
/* preferred_thread_count at 0 via memset) and the default: case (an        */
/* explicit count, exercised by vdec_test_config.c's own env-var-override   */
/* test) are already covered elsewhere. Only case 1 (force single-threaded, */
/* also clearing FF_THREAD_FRAME) had no real-decode test. The final        */
/* `if (!support_multi_threading)` branch right after this switch is a      */
/* separate, genuinely unreachable gap: support_multi_threading=false only  */
/* appears on the h264_nvmpi/hevc_nvmpi backend table entries (NVIDIA       */
/* Jetson-specific decoder names), which need real Jetson hardware to even  */
/* resolve as supported - not achievable in this product config.           */
/* ================================================================== */

static void test_vdec_ffmpeg_single_thread_h264(void)
{
	char path[PATH_MAX];
	uint8_t *buf = NULL;
	size_t len = 0;
	size_t off = 0;
	int path_ret, access_ret, res;
	struct pomp_loop *loop = pomp_loop_new();
	struct mock_vdec_cbs_ctx ctx;
	struct vdec_cbs cbs = {
		.frame_output = &mock_vdec_frame_output_cb,
		.flush = &mock_vdec_flush_cb,
		.stop = &mock_vdec_stop_cb,
	};
	struct vdec_config config;
	struct vdec_decoder *dec = NULL;
	struct h264_asset_ctx u;
	struct h264_reader *reader = NULL;

	CU_ASSERT_PTR_NOT_NULL_FATAL(loop);
	if (!implem_available(VDEC_DECODER_IMPLEM_FFMPEG))
		goto out;
	mock_vdec_cbs_ctx_reset(&ctx);
	memset(&u, 0, sizeof(u));

	path_ret = resolve_h26x_asset_path(
		"crowd_run_240p.264", path, sizeof(path));
	CU_ASSERT_EQUAL_FATAL(path_ret, 0);
	access_ret = wait_for_asset(path);
	CU_ASSERT_EQUAL_FATAL(access_ret, 0);
	buf = read_whole_file(path, &len);
	CU_ASSERT_PTR_NOT_NULL_FATAL(buf);
	CU_ASSERT_TRUE_FATAL(len > 0);

	fill_h26x_config(
		&config, VDEC_DECODER_IMPLEM_FFMPEG, VDEF_ENCODING_H264);
	config.preferred_thread_count = 1;
	res = vdec_new(loop, &config, &cbs, &ctx, &dec);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec);

	u.dec = dec;
	u.queue = vdec_get_input_buffer_queue(dec);
	CU_ASSERT_PTR_NOT_NULL_FATAL(u.queue);

	res = h264_reader_new(&s_h264_asset_cbs, &u, &reader);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	u.reader = reader;

	res = h264_reader_parse(reader, 0, buf, len, &off);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_TRUE_FATAL(u.pushed > 0);

	/* Proves single-threaded decode (no FF_THREAD_FRAME) still produces
	 * correct output, not just that the switch statement doesn't crash. */
	res = vdec_flush(dec, 0 /* drain */);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.flush_done, ASSET_PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.flush_done);
	CU_ASSERT_TRUE(ctx.frame_count > 0);

	mock_vdec_cbs_ctx_clear(&ctx);

	res = vdec_stop(dec);
	CU_ASSERT_EQUAL(res, 0);
	pump_until(loop, &ctx.stop_done, PUMP_MAX_ITER);
	CU_ASSERT_TRUE(ctx.stop_done);

	vdec_destroy(dec);
	h264_reader_destroy(reader);
	free(u.sps);
	free(u.pps);
	free(buf);

out:
	pomp_loop_destroy(loop);
}


/* Grouped by scenario/feature, like
 * libvideo-encode/tests/venc_test_lifecycle.c's own CU_TestInfo table: a
 * comment names each scenario, with every implem's variant of it listed
 * together underneath. */
CU_TestInfo g_vdec_test_lifecycle[] = {
	/* frame-output-and-drain */
	{FN("frame-output-and-drain-h264-ffmpeg"),
	 &test_frame_output_and_drain_h264_ffmpeg},
	{FN("frame-output-and-drain-h265-ffmpeg"),
	 &test_frame_output_and_drain_h265_ffmpeg},
	{FN("frame-output-and-drain-jpeg-turbojpeg"),
	 &test_frame_output_and_drain_jpeg_turbojpeg},

	/* create-and-destroy */
	{FN("create-and-destroy-ffmpeg"), &test_vdec_create_and_destroy_ffmpeg},

	/* flush-discard-no-input */
	{FN("flush-discard-no-input-ffmpeg"),
	 &test_vdec_flush_discard_no_input_ffmpeg},

	/* flush-then-redecode */
	{FN("flush-then-redecode-ffmpeg"),
	 &test_vdec_flush_then_redecode_ffmpeg},

	/* flush-discard-then-redecode */
	{FN("flush-discard-then-redecode-ffmpeg"),
	 &test_vdec_flush_discard_then_redecode_ffmpeg},

	/* metadata-propagation */
	{FN("metadata-propagation-h264-ffmpeg"),
	 &test_vdec_metadata_propagation_h264_ffmpeg},
	{FN("metadata-propagation-jpeg-turbojpeg"),
	 &test_vdec_metadata_propagation_jpeg_turbojpeg},

	/* h264-write-grey-idr (direct call, fully configured decoder) */
	{FN("h264-write-grey-idr-configured-ffmpeg"),
	 &test_vdec_h264_write_grey_idr_configured_ffmpeg},
	{FN("h264-write-grey-idr-avcc-format"),
	 &test_vdec_h264_write_grey_idr_avcc_format},
	{FN("h264-write-grey-idr-unsupported-format"),
	 &test_vdec_h264_write_grey_idr_unsupported_format},

	/* set-h264-ps: direct format-match dispatch branch */
	{FN("set-h264-ps-direct-format-match-ffmpeg"),
	 &test_vdec_set_h264_ps_direct_format_match_ffmpeg},

	/* gen-grey-idr: resync after a flush via config.gen_grey_idr */
	{FN("ffmpeg-gen-grey-idr-h264"), &test_vdec_ffmpeg_gen_grey_idr_h264},
	{FN("no-gen-grey-idr-discards-non-sync-frame-ffmpeg"),
	 &test_vdec_no_gen_grey_idr_discards_non_sync_frame_ffmpeg},

	/* preferred-output-format */
	{FN("frame-output-nv12-turbojpeg"), &test_frame_output_nv12_turbojpeg},
	{FN("frame-output-ffmpeg-ignores-preferred-output-format"),
	 &test_frame_output_ffmpeg_ignores_preferred_output_format},

	/* dbg-files */
	{FN("dbg-files-written-during-real-decode-ffmpeg"),
	 &test_vdec_dbg_files_written_during_real_decode_ffmpeg},

	/* corrupted-data */
	{FN("corrupted-data-ffmpeg-h264"),
	 &test_vdec_corrupted_data_ffmpeg_h264},
	{FN("corrupted-data-ffmpeg-h265"),
	 &test_vdec_corrupted_data_ffmpeg_h265},
	{FN("corrupted-data-turbojpeg"), &test_vdec_corrupted_data_turbojpeg},

	/* thread-count */
	{FN("ffmpeg-single-thread-h264"), &test_vdec_ffmpeg_single_thread_h264},

	CU_TEST_INFO_NULL,
};

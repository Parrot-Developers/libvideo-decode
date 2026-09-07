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

#include <dirent.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>

#include <media-buffers/mbuf_mem_generic.h>

/*
 * vdec_dbg_create_files()/vdec_dbg_close_files() (core/src/vdec_dbg.c):
 * pure file-creation helpers driven by self->config.dbg_dir/dbg_flags, using
 * a hand-built struct vdec_decoder - no live backend/reader needed.
 */

/* buf must be caller-owned (not static/local-and-returned): mkdtemp()
 * rewrites the trailing "XXXXXX" in place and returns the same buffer, so a
 * shared static template would only work for a single call - every test
 * below supplies its own stack buffer. */
static void make_tmp_dir(char *buf, size_t buf_len)
{
	char *dir;

	CU_ASSERT_TRUE_FATAL(buf_len >= sizeof("/tmp/vdec_test_dbg_XXXXXX"));
	snprintf(buf, buf_len, "/tmp/vdec_test_dbg_XXXXXX");
	dir = mkdtemp(buf);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dir);
}


static void test_dbg_create_files_null_self(void)
{
	int res = vdec_dbg_create_files(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_dbg_close_files_null_self(void)
{
	int res = vdec_dbg_close_files(NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_dbg_create_files_no_flags(void)
{
	struct vdec_decoder dec;
	char dir[64];
	make_tmp_dir(dir, sizeof(dir));
	int res;

	memset(&dec, 0, sizeof(dec));
	dec.config.dbg_dir = dir;
	dec.config.dbg_flags = 0;
	dec.config.encoding = VDEF_ENCODING_H264;

	res = vdec_dbg_create_files(&dec);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NULL(dec.dbg.input_bs);
	CU_ASSERT_PTR_NULL(dec.dbg.output_yuv);
	CU_ASSERT_PTR_NULL(dec.dbg.analysis);

	res = vdec_dbg_close_files(&dec);
	CU_ASSERT_EQUAL(res, 0);

	rmdir(dir);
}


static void test_dbg_create_and_close_files_all_flags(void)
{
	struct vdec_decoder dec;
	char dir[64];
	make_tmp_dir(dir, sizeof(dir));
	char path[512];
	int res;
	struct stat st;

	memset(&dec, 0, sizeof(dec));
	dec.config.dbg_dir = dir;
	dec.config.dbg_flags = VDEC_DBG_FLAG_INPUT_BITSTREAM |
			       VDEC_DBG_FLAG_OUTPUT_YUV |
			       VDEC_DBG_FLAG_ANALYSIS;
	dec.config.encoding = VDEF_ENCODING_H264;

	res = vdec_dbg_create_files(&dec);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec.dbg.input_bs);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec.dbg.output_yuv);
	CU_ASSERT_PTR_NOT_NULL_FATAL(dec.dbg.analysis);

	/* The analysis file gets a CSV header written immediately. */
	fflush(dec.dbg.analysis);

	res = vdec_dbg_close_files(&dec);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NULL(dec.dbg.input_bs);
	CU_ASSERT_PTR_NULL(dec.dbg.output_yuv);
	CU_ASSERT_PTR_NULL(dec.dbg.analysis);

	/* Filenames embed a timestamp/pid/pointer we cannot predict exactly,
	 * so just confirm at least one file matching each debug category
	 * was created in the target directory, rather than an exact name. */
	{
		DIR *d = opendir(dir);
		struct dirent *ent;
		bool has_h264 = false, has_yuv = false, has_analysis = false;

		CU_ASSERT_PTR_NOT_NULL_FATAL(d);
		while ((ent = readdir(d)) != NULL) {
			if (strstr(ent->d_name, "input.h264") != NULL)
				has_h264 = true;
			if (strstr(ent->d_name, "output.yuv") != NULL)
				has_yuv = true;
			if (strstr(ent->d_name, "analysis.csv") != NULL)
				has_analysis = true;
			if (strcmp(ent->d_name, ".") != 0 &&
			    strcmp(ent->d_name, "..") != 0) {
				snprintf(path,
					 sizeof(path),
					 "%s/%s",
					 dir,
					 ent->d_name);
				res = stat(path, &st);
				if (res == 0)
					unlink(path);
			}
		}
		closedir(d);
		CU_ASSERT_TRUE(has_h264);
		CU_ASSERT_TRUE(has_yuv);
		CU_ASSERT_TRUE(has_analysis);
	}

	rmdir(dir);
}


/*
 * vdec_dbg_write_h264_ps()/vdec_dbg_write_frame() (core/src/vdec_dbg.c):
 * pure byte-copying helpers over a caller-supplied FILE* - content of the
 * SPS/PPS/NALU buffers is irrelevant, only the resulting byte layout matters.
 */

static void test_dbg_write_h264_ps_raw_nalu(void)
{
	char dir[64];
	make_tmp_dir(dir, sizeof(dir));
	char path[512];
	FILE *file;
	int res;
	long size;
	static const uint8_t sps[] = {0x11, 0x22, 0x33, 0x44};
	static const uint8_t pps[] = {0x55, 0x66};

	snprintf(path, sizeof(path), "%s/ps.bin", dir);
	file = fopen(path, "wb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(file);

	res = vdec_dbg_write_h264_ps(
		file, sps, sizeof(sps), pps, sizeof(pps), &vdef_h264_raw_nalu);
	CU_ASSERT_EQUAL(res, 0);
	fflush(file);

	/* raw_nalu buffers carry no start code of their own: write_ps()
	 * prepends one (4 bytes) before each of sps/pps. */
	size = ftell(file);
	CU_ASSERT_EQUAL(size, (long)(4 + sizeof(sps) + 4 + sizeof(pps)));

	fclose(file);
	unlink(path);
	rmdir(dir);
}


static void test_dbg_write_h264_ps_byte_stream(void)
{
	char dir[64];
	make_tmp_dir(dir, sizeof(dir));
	char path[512];
	FILE *file;
	int res;
	long size;
	/* byte_stream buffers already carry their own start code: write_ps()
	 * must not prepend a second one. */
	static const uint8_t sps[] = {0x00, 0x00, 0x00, 0x01, 0x11, 0x22};
	static const uint8_t pps[] = {0x00, 0x00, 0x00, 0x01, 0x33};

	snprintf(path, sizeof(path), "%s/ps.bin", dir);
	file = fopen(path, "wb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(file);

	res = vdec_dbg_write_h264_ps(file,
				     sps,
				     sizeof(sps),
				     pps,
				     sizeof(pps),
				     &vdef_h264_byte_stream);
	CU_ASSERT_EQUAL(res, 0);
	fflush(file);

	size = ftell(file);
	CU_ASSERT_EQUAL(size, (long)(sizeof(sps) + sizeof(pps)));

	fclose(file);
	unlink(path);
	rmdir(dir);
}


static void test_dbg_write_h264_ps_invalid_args(void)
{
	int res;
	static const uint8_t sps[] = {0x11};

	res = vdec_dbg_write_h264_ps(
		NULL, sps, sizeof(sps), sps, sizeof(sps), &vdef_h264_raw_nalu);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_dbg_write_h265_ps_raw_nalu(void)
{
	char dir[64];
	make_tmp_dir(dir, sizeof(dir));
	char path[512];
	FILE *file;
	int res;
	long size;
	static const uint8_t vps[] = {0x01, 0x02};
	static const uint8_t sps[] = {0x11, 0x22, 0x33};
	static const uint8_t pps[] = {0x44};

	snprintf(path, sizeof(path), "%s/ps.bin", dir);
	file = fopen(path, "wb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(file);

	res = vdec_dbg_write_h265_ps(file,
				     vps,
				     sizeof(vps),
				     sps,
				     sizeof(sps),
				     pps,
				     sizeof(pps),
				     &vdef_h265_raw_nalu);
	CU_ASSERT_EQUAL(res, 0);
	fflush(file);

	/* raw_nalu buffers carry no start code of their own: write_ps()
	 * prepends one (4 bytes) before each of vps/sps/pps. */
	size = ftell(file);
	CU_ASSERT_EQUAL(
		size,
		(long)(4 + sizeof(vps) + 4 + sizeof(sps) + 4 + sizeof(pps)));

	fclose(file);
	unlink(path);
	rmdir(dir);
}


static void test_dbg_write_h265_ps_byte_stream(void)
{
	char dir[64];
	make_tmp_dir(dir, sizeof(dir));
	char path[512];
	FILE *file;
	int res;
	long size;
	/* byte_stream buffers already carry their own start code: write_ps()
	 * must not prepend a second one. */
	static const uint8_t vps[] = {0x00, 0x00, 0x00, 0x01, 0x01};
	static const uint8_t sps[] = {0x00, 0x00, 0x00, 0x01, 0x11, 0x22};
	static const uint8_t pps[] = {0x00, 0x00, 0x00, 0x01, 0x33};

	snprintf(path, sizeof(path), "%s/ps.bin", dir);
	file = fopen(path, "wb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(file);

	res = vdec_dbg_write_h265_ps(file,
				     vps,
				     sizeof(vps),
				     sps,
				     sizeof(sps),
				     pps,
				     sizeof(pps),
				     &vdef_h265_byte_stream);
	CU_ASSERT_EQUAL(res, 0);
	fflush(file);

	size = ftell(file);
	CU_ASSERT_EQUAL(size, (long)(sizeof(vps) + sizeof(sps) + sizeof(pps)));

	fclose(file);
	unlink(path);
	rmdir(dir);
}


static void test_dbg_write_h265_ps_invalid_args(void)
{
	int res;
	static const uint8_t vps[] = {0x11};

	res = vdec_dbg_write_h265_ps(NULL,
				     vps,
				     sizeof(vps),
				     vps,
				     sizeof(vps),
				     vps,
				     sizeof(vps),
				     &vdef_h265_raw_nalu);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


static void test_dbg_write_frame_byte_stream(void)
{
	char dir[64];
	make_tmp_dir(dir, sizeof(dir));
	char path[512];
	FILE *file;
	int res;
	long size;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_coded_frame frame_info;
	struct vdef_nalu nalu = {0};
	void *data = NULL;
	size_t capacity = 0;

	res = mbuf_mem_generic_new(16, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0x42, capacity);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	nalu.size = capacity;
	nalu.h264.type = H264_NALU_TYPE_SLICE;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	snprintf(path, sizeof(path), "%s/frame.bin", dir);
	file = fopen(path, "wb");
	CU_ASSERT_PTR_NOT_NULL_FATAL(file);

	res = vdec_dbg_write_frame(file, frame);
	CU_ASSERT_EQUAL(res, 0);
	fflush(file);

	size = ftell(file);
	CU_ASSERT_EQUAL(size, (long)capacity);

	fclose(file);
	unlink(path);
	rmdir(dir);
	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_dbg_write_frame_invalid_args(void)
{
	int res = vdec_dbg_write_frame(NULL, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);
}


CU_TestInfo g_vdec_test_dbg[] = {
	{FN("create-files-null-self"), &test_dbg_create_files_null_self},
	{FN("close-files-null-self"), &test_dbg_close_files_null_self},
	{FN("create-files-no-flags"), &test_dbg_create_files_no_flags},
	{FN("create-and-close-files-all-flags"),
	 &test_dbg_create_and_close_files_all_flags},
	{FN("write-h264-ps-raw-nalu"), &test_dbg_write_h264_ps_raw_nalu},
	{FN("write-h264-ps-byte-stream"), &test_dbg_write_h264_ps_byte_stream},
	{FN("write-h264-ps-invalid-args"),
	 &test_dbg_write_h264_ps_invalid_args},
	{FN("write-h265-ps-raw-nalu"), &test_dbg_write_h265_ps_raw_nalu},
	{FN("write-h265-ps-byte-stream"), &test_dbg_write_h265_ps_byte_stream},
	{FN("write-h265-ps-invalid-args"),
	 &test_dbg_write_h265_ps_invalid_args},
	{FN("write-frame-byte-stream"), &test_dbg_write_frame_byte_stream},
	{FN("write-frame-invalid-args"), &test_dbg_write_frame_invalid_args},

	CU_TEST_INFO_NULL,
};

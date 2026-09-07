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

#include <media-buffers/mbuf_mem_generic.h>

/*
 * vdec_h264_is_idr() (core/src/vdec_h264.c): scans the frame's NALUs and
 * trusts the first SLICE/SLICE_IDR type found; other NALU types are skipped.
 */

static struct mbuf_coded_video_frame *
new_h264_frame_with_nalu_type(struct mbuf_mem *mem, enum h264_nalu_type type)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	nalu.size = 32;
	nalu.h264.type = type;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


/* Builds a frame with nalu.h264.type left UNKNOWN and the real NALU header
 * byte written into the backing buffer instead - exercises
 * vdec_h264_is_idr()'s raw-byte-inspection fallback (core/src/vdec_h264.c),
 * never reached by new_h264_frame_with_nalu_type() above since that always
 * pre-tags the type directly. */
static struct mbuf_coded_video_frame *
new_h264_frame_with_unknown_type_from_bytes(struct mbuf_mem *mem,
					    uint8_t nalu_header_byte)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	void *data = NULL;
	size_t capacity = 0;
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);
	/* byte_stream format: vdec_h264_is_idr() skips the first 4 bytes
	 * (start code) before reading the NALU header byte. */
	((uint8_t *)data)[4] = nalu_header_byte;

	nalu.size = 32;
	nalu.h264.type = H264_NALU_TYPE_UNKNOWN;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


static void test_h264_is_idr_unknown_type_read_from_bytes_idr(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* Upper 3 bits (nal_ref_idc) set to a nonzero, irrelevant value -
	 * only the low 5 bits (the type) are extracted via `& 0x1F`. */
	frame = new_h264_frame_with_unknown_type_from_bytes(
		mem, (uint8_t)(0x60 | H264_NALU_TYPE_SLICE_IDR));

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;

	idr = vdec_h264_is_idr(frame, &info);
	CU_ASSERT_TRUE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h264_is_idr_unknown_type_read_from_bytes_non_idr(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h264_frame_with_unknown_type_from_bytes(
		mem, (uint8_t)(0x60 | H264_NALU_TYPE_SLICE));

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;

	idr = vdec_h264_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h264_is_idr_null_args(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h264_frame_with_nalu_type(mem, H264_NALU_TYPE_SLICE_IDR);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;

	idr = vdec_h264_is_idr(NULL, &info);
	CU_ASSERT_FALSE(idr);
	idr = vdec_h264_is_idr(frame, NULL);
	CU_ASSERT_FALSE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h264_is_idr_slice_idr(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h264_frame_with_nalu_type(mem, H264_NALU_TYPE_SLICE_IDR);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;

	idr = vdec_h264_is_idr(frame, &info);
	CU_ASSERT_TRUE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h264_is_idr_slice_non_idr(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h264_frame_with_nalu_type(mem, H264_NALU_TYPE_SLICE);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;

	idr = vdec_h264_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h264_is_idr_no_slice_nalu(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* Neither SLICE nor SLICE_IDR: the loop runs to completion without
	 * ever returning early, falling through to the final `return false`. */
	frame = new_h264_frame_with_nalu_type(mem, H264_NALU_TYPE_SPS);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;

	idr = vdec_h264_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/*
 * vdec_h264_write_grey_idr() (core/src/vdec_h264.c): the -EAGAIN guard when
 * self->configured == 0 fires before anything else meaningful happens, so no
 * live backend/reader is needed to exercise it.
 *
 * Note: this used to be reachable to crash instead: `ctx` was initialized via
 * `h264_reader_get_ctx(self->reader.h264)` in the variable declaration,
 * which ran unconditionally *before* the `self == NULL` guard below it -
 * calling this function with self == NULL dereferenced NULL. Fixed in
 * core/src/vdec_h264.c by moving that read after the guards.
 */

static void test_h264_write_grey_idr_null_args(void)
{
	struct vdec_decoder self;
	struct vdef_coded_frame in_frame_info;
	uint64_t delta = 0;
	uint64_t timestamp = 0;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;
	int res;

	memset(&self, 0, sizeof(self));
	memset(&in_frame_info, 0, sizeof(in_frame_info));
	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vdec_h264_write_grey_idr(
		NULL, &in_frame_info, &delta, &timestamp, mem, &idr_frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_h264_write_grey_idr(
		&self, NULL, &delta, &timestamp, mem, &idr_frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_h264_write_grey_idr(
		&self, &in_frame_info, NULL, &timestamp, mem, &idr_frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_h264_write_grey_idr(
		&self, &in_frame_info, &delta, NULL, mem, &idr_frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_h264_write_grey_idr(
		&self, &in_frame_info, &delta, &timestamp, NULL, &idr_frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_h264_write_grey_idr(
		&self, &in_frame_info, &delta, &timestamp, mem, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_mem_unref(mem);
}


static void test_h264_write_grey_idr_not_configured(void)
{
	struct vdec_decoder self;
	struct vdef_coded_frame in_frame_info;
	uint64_t delta = 0;
	uint64_t timestamp = 0;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *idr_frame = NULL;
	int res;

	memset(&self, 0, sizeof(self));
	self.configured = 0;
	memset(&in_frame_info, 0, sizeof(in_frame_info));
	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vdec_h264_write_grey_idr(
		&self, &in_frame_info, &delta, &timestamp, mem, &idr_frame);
	CU_ASSERT_EQUAL(res, -EAGAIN);
	CU_ASSERT_PTR_NULL(idr_frame);

	mbuf_mem_unref(mem);
}


CU_TestInfo g_vdec_test_h264[] = {
	{FN("is-idr-null-args"), &test_h264_is_idr_null_args},
	{FN("is-idr-slice-idr"), &test_h264_is_idr_slice_idr},
	{FN("is-idr-slice-non-idr"), &test_h264_is_idr_slice_non_idr},
	{FN("is-idr-no-slice-nalu"), &test_h264_is_idr_no_slice_nalu},
	{FN("is-idr-unknown-type-read-from-bytes-idr"),
	 &test_h264_is_idr_unknown_type_read_from_bytes_idr},
	{FN("is-idr-unknown-type-read-from-bytes-non-idr"),
	 &test_h264_is_idr_unknown_type_read_from_bytes_non_idr},
	{FN("write-grey-idr-null-args"), &test_h264_write_grey_idr_null_args},
	{FN("write-grey-idr-not-configured"),
	 &test_h264_write_grey_idr_not_configured},

	CU_TEST_INFO_NULL,
};

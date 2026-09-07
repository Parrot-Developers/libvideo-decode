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
 * vdec_h265_is_idr() (core/src/vdec_h265.c): scans the frame's NALUs;
 * IDR_W_RADL/IDR_N_LP -> true, the BLA and CRA_NUT types -> false, anything
 * else is skipped (falls through to the final "return false" after the loop).
 */

static struct mbuf_coded_video_frame *
new_h265_frame_with_nalu_type(struct mbuf_mem *mem, enum h265_nalu_type type)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h265_byte_stream;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	nalu.size = 32;
	nalu.h265.type = type;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


/* Builds a frame with nalu.h265.type left UNKNOWN and the real NALU header
 * byte written into the backing buffer instead - exercises
 * vdec_h265_is_idr()'s raw-byte-inspection fallback (core/src/vdec_h265.c),
 * never reached by new_h265_frame_with_nalu_type() above since that always
 * pre-tags the type directly. H.265 NALU header: forbidden_zero_bit(1) +
 * type(6) + layer_id_high(1), so the type sits at bits 1-6 of the first
 * byte: `(*raw_nalu & 0x3E) >> 1`. */
static struct mbuf_coded_video_frame *
new_h265_frame_with_unknown_type_from_bytes(struct mbuf_mem *mem,
					    enum h265_nalu_type type)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	void *data = NULL;
	size_t capacity = 0;
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h265_byte_stream;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);
	/* byte_stream format: vdec_h265_is_idr() skips the first 4 bytes
	 * (start code) before reading the NALU header byte. */
	((uint8_t *)data)[4] = (uint8_t)((type << 1) & 0x3E);

	nalu.size = 32;
	nalu.h265.type = H265_NALU_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


static void test_h265_is_idr_unknown_type_read_from_bytes_idr(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h265_frame_with_unknown_type_from_bytes(
		mem, H265_NALU_TYPE_IDR_W_RADL);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;

	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_TRUE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h265_is_idr_unknown_type_read_from_bytes_non_idr(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h265_frame_with_unknown_type_from_bytes(
		mem, H265_NALU_TYPE_TRAIL_R);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;

	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h265_is_idr_null_args(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_IDR_W_RADL);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;

	idr = vdec_h265_is_idr(NULL, &info);
	CU_ASSERT_FALSE(idr);
	idr = vdec_h265_is_idr(frame, NULL);
	CU_ASSERT_FALSE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h265_is_idr_idr_w_radl(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_IDR_W_RADL);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;

	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_TRUE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h265_is_idr_idr_n_lp(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_IDR_N_LP);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;

	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_TRUE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_h265_is_idr_bla_and_cra_are_not_idr(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_BLA_W_LP);
	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);
	mbuf_coded_video_frame_unref(frame);

	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_BLA_W_RADL);
	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);
	mbuf_coded_video_frame_unref(frame);

	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_BLA_N_LP);
	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);
	mbuf_coded_video_frame_unref(frame);

	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_CRA_NUT);
	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);
	mbuf_coded_video_frame_unref(frame);

	mbuf_mem_unref(mem);
}


static void test_h265_is_idr_unrelated_nalu_type(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool idr;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* Hits the switch's `default: break` case, falls through the loop to
	 * the final `return false`. */
	frame = new_h265_frame_with_nalu_type(mem, H265_NALU_TYPE_TRAIL_R);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;

	idr = vdec_h265_is_idr(frame, &info);
	CU_ASSERT_FALSE(idr);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


CU_TestInfo g_vdec_test_h265[] = {
	{FN("is-idr-null-args"), &test_h265_is_idr_null_args},
	{FN("is-idr-idr-w-radl"), &test_h265_is_idr_idr_w_radl},
	{FN("is-idr-idr-n-lp"), &test_h265_is_idr_idr_n_lp},
	{FN("is-idr-bla-and-cra-are-not-idr"),
	 &test_h265_is_idr_bla_and_cra_are_not_idr},
	{FN("is-idr-unrelated-nalu-type"),
	 &test_h265_is_idr_unrelated_nalu_type},
	{FN("is-idr-unknown-type-read-from-bytes-idr"),
	 &test_h265_is_idr_unknown_type_read_from_bytes_idr},
	{FN("is-idr-unknown-type-read-from-bytes-non-idr"),
	 &test_h265_is_idr_unknown_type_read_from_bytes_non_idr},

	CU_TEST_INFO_NULL,
};

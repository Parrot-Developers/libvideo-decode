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

#include <media-buffers/mbuf_mem_generic.h>
#include <video-metadata/vmeta_frame.h>

/*
 * vdec_default_input_filter_internal() (core/src/vdec_format.c): pure
 * function operating on a caller-supplied vdef_coded_frame + formats array.
 * It never dereferences the `frame` argument itself, so NULL can be passed
 * for it directly.
 */

static void test_internal_filter_accepts_matching_format_and_timestamp(void)
{
	struct vdec_decoder dec;
	struct vdef_coded_frame frame_info;
	const struct vdef_coded_format fmts[] = {vdef_h264_byte_stream};
	bool accepted;

	memset(&dec, 0, sizeof(dec));
	dec.last_timestamp = UINT64_MAX;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.info.timestamp = 1000;

	accepted = vdec_default_input_filter_internal(
		&dec, NULL, &frame_info, fmts, ARRAY_SIZE(fmts));
	CU_ASSERT_TRUE(accepted);
}


static void test_internal_filter_rejects_unsupported_format(void)
{
	struct vdec_decoder dec;
	struct vdef_coded_frame frame_info;
	const struct vdef_coded_format fmts[] = {vdef_h264_byte_stream};
	bool accepted;

	memset(&dec, 0, sizeof(dec));
	dec.last_timestamp = UINT64_MAX;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h265_byte_stream;
	frame_info.info.timestamp = 1000;

	accepted = vdec_default_input_filter_internal(
		&dec, NULL, &frame_info, fmts, ARRAY_SIZE(fmts));
	CU_ASSERT_FALSE(accepted);
}


static void test_internal_filter_rejects_non_monotonic_timestamp(void)
{
	struct vdec_decoder dec;
	struct vdef_coded_frame frame_info;
	const struct vdef_coded_format fmts[] = {vdef_h264_byte_stream};
	bool accepted;

	memset(&dec, 0, sizeof(dec));
	dec.last_timestamp = 1000;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;

	/* Same timestamp as the current last_timestamp -> rejected */
	frame_info.info.timestamp = 1000;
	accepted = vdec_default_input_filter_internal(
		&dec, NULL, &frame_info, fmts, ARRAY_SIZE(fmts));
	CU_ASSERT_FALSE(accepted);

	/* Strictly lower -> rejected too */
	frame_info.info.timestamp = 500;
	accepted = vdec_default_input_filter_internal(
		&dec, NULL, &frame_info, fmts, ARRAY_SIZE(fmts));
	CU_ASSERT_FALSE(accepted);

	/* Strictly higher -> accepted */
	frame_info.info.timestamp = 1001;
	accepted = vdec_default_input_filter_internal(
		&dec, NULL, &frame_info, fmts, ARRAY_SIZE(fmts));
	CU_ASSERT_TRUE(accepted);
}


/*
 * vdec_default_input_filter() (core/src/vdec_format.c): the public wrapper
 * needs a struct vdec_decoder with a working ops->get_supported_input_formats
 * and a real frame, since it calls mbuf_coded_video_frame_get_frame_info().
 */

static int
fake_get_supported_input_formats(const struct vdef_coded_format **formats)
{
	*formats = &vdef_h264_byte_stream;
	return 1;
}


static const struct vdec_ops s_fake_filter_ops = {
	.get_supported_input_formats = fake_get_supported_input_formats,
};


static struct mbuf_coded_video_frame *
new_test_coded_frame_fmt_ts(struct mbuf_mem *mem,
			    struct vdef_coded_format format,
			    uint64_t timestamp)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = format;
	frame_info.info.timestamp = timestamp;
	frame_info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	nalu.size = 64;
	nalu.h264.type = H264_NALU_TYPE_SLICE;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


static void test_default_input_filter_accepts_matching_frame(void)
{
	struct vdec_decoder dec;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;
	bool accepted;

	memset(&dec, 0, sizeof(dec));
	dec.ops = &s_fake_filter_ops;
	dec.last_timestamp = UINT64_MAX;

	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_coded_frame_fmt_ts(mem, vdef_h264_byte_stream, 1000);

	accepted = vdec_default_input_filter(frame, &dec);
	CU_ASSERT_TRUE(accepted);
	CU_ASSERT_EQUAL(dec.counters.in, 1u);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_default_input_filter_invalid_args(void)
{
	struct vdec_decoder dec;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;
	bool accepted;

	memset(&dec, 0, sizeof(dec));
	dec.ops = &s_fake_filter_ops;

	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_coded_frame_fmt_ts(mem, vdef_h264_byte_stream, 1000);

	accepted = vdec_default_input_filter(NULL, &dec);
	CU_ASSERT_FALSE(accepted);
	accepted = vdec_default_input_filter(frame, NULL);
	CU_ASSERT_FALSE(accepted);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_default_input_filter_rejects_unsupported_format(void)
{
	struct vdec_decoder dec;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;
	bool accepted;

	memset(&dec, 0, sizeof(dec));
	dec.ops = &s_fake_filter_ops;
	dec.last_timestamp = UINT64_MAX;

	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* s_fake_filter_ops only advertises vdef_h264_byte_stream */
	frame = new_test_coded_frame_fmt_ts(mem, vdef_h265_byte_stream, 1000);

	accepted = vdec_default_input_filter(frame, &dec);
	CU_ASSERT_FALSE(accepted);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/*
 * vdec_default_input_filter_internal_confirm_frame() (core/src/vdec_format.c):
 * updates last_timestamp/counters.in and adds the
 * VDEC_ANCILLARY_KEY_INPUT_TIME ancillary data to the frame.
 */

static void test_confirm_frame_sets_ancillary_and_counters(void)
{
	struct vdec_decoder dec;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame frame_info;
	struct mbuf_ancillary_data *anc = NULL;
	int res;

	memset(&dec, 0, sizeof(dec));
	dec.last_timestamp = UINT64_MAX;

	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_coded_frame_fmt_ts(mem, vdef_h264_byte_stream, 1000);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.info.timestamp = 1000;

	vdec_default_input_filter_internal_confirm_frame(
		&dec, frame, &frame_info);
	CU_ASSERT_EQUAL(dec.last_timestamp, 1000u);
	CU_ASSERT_EQUAL(dec.counters.in, 1u);

	res = mbuf_coded_video_frame_get_ancillary_data(
		frame, VDEC_ANCILLARY_KEY_INPUT_TIME, &anc);
	CU_ASSERT_EQUAL(res, 0);
	if (res == 0)
		mbuf_ancillary_data_unref(anc);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/* Adding VDEC_ANCILLARY_KEY_INPUT_TIME twice on the same frame hits
 * mbuf_coded_video_frame_add_ancillary_buffer()'s documented -EEXIST path -
 * the confirm helper only logs it and otherwise continues (last_timestamp is
 * still updated), it never propagates the error since its return type is
 * void. */
static void test_confirm_frame_duplicate_ancillary_logs_and_continues(void)
{
	struct vdec_decoder dec;
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame frame_info;
	int res;

	memset(&dec, 0, sizeof(dec));
	dec.last_timestamp = UINT64_MAX;

	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_test_coded_frame_fmt_ts(mem, vdef_h264_byte_stream, 1000);

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.info.timestamp = 1000;

	vdec_default_input_filter_internal_confirm_frame(
		&dec, frame, &frame_info);
	CU_ASSERT_EQUAL(dec.last_timestamp, 1000u);

	/* Second call on the same frame: the ancillary key already exists. */
	vdec_default_input_filter_internal_confirm_frame(
		&dec, frame, &frame_info);
	CU_ASSERT_EQUAL(dec.last_timestamp, 1000u);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/*
 * vdec_is_sync_frame() (core/src/vdec_format.c): dispatches to
 * vdec_h264_is_idr()/vdec_h265_is_idr() only when info->type is UNKNOWN;
 * otherwise it trusts info->type directly. Note the function guards
 * frame == NULL even on paths that never dereference frame, so a real
 * (non-NULL) frame is always required.
 */

static struct mbuf_coded_video_frame *
new_h264_frame(struct mbuf_mem *mem,
	       enum h264_nalu_type type,
	       enum vdef_coded_frame_type frame_type)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h264_byte_stream;
	frame_info.type = frame_type;

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


static struct mbuf_coded_video_frame *
new_h265_frame(struct mbuf_mem *mem,
	       enum h265_nalu_type type,
	       enum vdef_coded_frame_type frame_type)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = vdef_h265_byte_stream;
	frame_info.type = frame_type;

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


static void test_is_sync_frame_null_args(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool sync;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_h264_frame(
		mem, H264_NALU_TYPE_SLICE_IDR, VDEF_CODED_FRAME_TYPE_UNKNOWN);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;

	sync = vdec_is_sync_frame(NULL, &info);
	CU_ASSERT_FALSE(sync);
	sync = vdec_is_sync_frame(frame, NULL);
	CU_ASSERT_FALSE(sync);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_is_sync_frame_declared_type_is_trusted(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool sync;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* The NALU content (a plain SLICE, not IDR) is irrelevant: a
	 * non-UNKNOWN declared frame type is trusted without inspecting the
	 * frame at all. */
	frame = new_h264_frame(
		mem, H264_NALU_TYPE_SLICE, VDEF_CODED_FRAME_TYPE_IDR);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;
	info.type = VDEF_CODED_FRAME_TYPE_IDR;
	sync = vdec_is_sync_frame(frame, &info);
	CU_ASSERT_TRUE(sync);

	info.type = VDEF_CODED_FRAME_TYPE_P;
	sync = vdec_is_sync_frame(frame, &info);
	CU_ASSERT_FALSE(sync);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_is_sync_frame_unknown_dispatches_to_h264(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool sync;

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;
	info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = new_h264_frame(
		mem, H264_NALU_TYPE_SLICE_IDR, VDEF_CODED_FRAME_TYPE_UNKNOWN);
	sync = vdec_is_sync_frame(frame, &info);
	CU_ASSERT_TRUE(sync);
	mbuf_coded_video_frame_unref(frame);

	frame = new_h264_frame(
		mem, H264_NALU_TYPE_SLICE, VDEF_CODED_FRAME_TYPE_UNKNOWN);
	sync = vdec_is_sync_frame(frame, &info);
	CU_ASSERT_FALSE(sync);
	mbuf_coded_video_frame_unref(frame);

	mbuf_mem_unref(mem);
}


static void test_is_sync_frame_unknown_dispatches_to_h265(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool sync;

	memset(&info, 0, sizeof(info));
	info.format = vdef_h265_byte_stream;
	info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	frame = new_h265_frame(
		mem, H265_NALU_TYPE_IDR_W_RADL, VDEF_CODED_FRAME_TYPE_UNKNOWN);
	sync = vdec_is_sync_frame(frame, &info);
	CU_ASSERT_TRUE(sync);
	mbuf_coded_video_frame_unref(frame);

	frame = new_h265_frame(
		mem, H265_NALU_TYPE_BLA_W_LP, VDEF_CODED_FRAME_TYPE_UNKNOWN);
	sync = vdec_is_sync_frame(frame, &info);
	CU_ASSERT_FALSE(sync);
	mbuf_coded_video_frame_unref(frame);

	mbuf_mem_unref(mem);
}


static void test_is_sync_frame_unsupported_encoding(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct vdef_coded_frame info;
	int res;
	bool sync;

	res = mbuf_mem_generic_new(32, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	/* NALU content is irrelevant here: the switch on info->format.encoding
	 * falls to its default case before ever inspecting the frame. */
	frame = new_h264_frame(
		mem, H264_NALU_TYPE_SLICE, VDEF_CODED_FRAME_TYPE_UNKNOWN);

	memset(&info, 0, sizeof(info));
	info.format.encoding = VDEF_ENCODING_MJPEG;
	info.type = VDEF_CODED_FRAME_TYPE_UNKNOWN;

	sync = vdec_is_sync_frame(frame, &info);
	CU_ASSERT_FALSE(sync);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/*
 * vdec_format_convert() (core/src/vdec_format.c): byte-stream <-> AVCC/HVCC
 * conversion for H.264/H.265; everything else is -ENOSYS.
 */

static struct mbuf_coded_video_frame *
new_convert_test_frame(struct mbuf_mem *mem, struct vdef_coded_format format)
{
	struct vdef_coded_frame frame_info;
	struct mbuf_coded_video_frame *frame = NULL;
	struct vdef_nalu nalu = {0};
	void *data = NULL;
	size_t capacity = 0;
	int res;

	memset(&frame_info, 0, sizeof(frame_info));
	frame_info.format = format;

	res = mbuf_coded_video_frame_new(&frame_info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);
	if (format.data_format == VDEF_CODED_DATA_FORMAT_BYTE_STREAM) {
		/* Start code 00 00 00 01 */
		((uint8_t *)data)[3] = 1;
	} else if (format.data_format == VDEF_CODED_DATA_FORMAT_AVCC) {
		uint32_t sc = htonl((uint32_t)(capacity - 4));
		memcpy(data, &sc, sizeof(sc));
	}

	nalu.size = capacity;
	if (format.encoding == VDEF_ENCODING_H264) {
		nalu.h264.type = H264_NALU_TYPE_SLICE;
		nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	} else if (format.encoding == VDEF_ENCODING_H265) {
		nalu.h265.type = H265_NALU_TYPE_TRAIL_R;
	}

	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	return frame;
}


static void test_format_convert_invalid_args(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h264_byte_stream);

	res = vdec_format_convert(NULL, &vdef_h264_avcc);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_format_convert(frame, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_format_convert_no_op_same_format(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h264_byte_stream);

	res = vdec_format_convert(frame, &vdef_h264_byte_stream);
	CU_ASSERT_EQUAL(res, 0);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_format_convert_h264_byte_stream_to_avcc(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h264_byte_stream);

	res = vdec_format_convert(frame, &vdef_h264_avcc);
	CU_ASSERT_EQUAL(res, 0);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_format_convert_h264_avcc_to_byte_stream(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h264_avcc);

	res = vdec_format_convert(frame, &vdef_h264_byte_stream);
	CU_ASSERT_EQUAL(res, 0);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_format_convert_h265_byte_stream_to_hvcc(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h265_byte_stream);

	res = vdec_format_convert(frame, &vdef_h265_hvcc);
	CU_ASSERT_EQUAL(res, 0);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_format_convert_raw_nalu_unsupported(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h264_raw_nalu);

	res = vdec_format_convert(frame, &vdef_h264_avcc);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_format_convert_encoding_mismatch(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h264_byte_stream);

	res = vdec_format_convert(frame, &vdef_h265_hvcc);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


static void test_format_convert_unsupported_encoding(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	/* Must differ from vdef_jpeg_jfif (below), otherwise
	 * vdef_coded_format_cmp() short-circuits vdec_format_convert() with a
	 * "no conversion needed" 0 before the encoding is ever checked. */
	struct vdef_coded_format jpeg_target = {
		.encoding = VDEF_ENCODING_MJPEG,
		.data_format = VDEF_CODED_DATA_FORMAT_UNKNOWN,
	};
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_jpeg_jfif);

	res = vdec_format_convert(frame, &jpeg_target);
	CU_ASSERT_EQUAL(res, -ENOSYS);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


/*
 * vdec_copy_coded_frame_as_metadata() (core/src/vdec_format.c): copies
 * frame_info + ancillary data + metadata without copying NALU byte content
 * (the copied frame gets a single zero-size placeholder NALU).
 */

static void test_copy_coded_frame_as_metadata(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_mem *meta_mem = NULL;
	struct mbuf_coded_video_frame *frame = NULL;
	struct mbuf_coded_video_frame *meta_frame = NULL;
	struct vdef_coded_frame info;
	struct vdef_nalu nalu = {0};
	void *data = NULL;
	size_t capacity = 0;
	int res;
	int nalu_count;
	const void *nalu_data;
	struct vdef_nalu meta_nalu;

	res = mbuf_mem_generic_new(64, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	res = mbuf_mem_generic_new(16, &meta_mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	memset(&info, 0, sizeof(info));
	info.format = vdef_h264_byte_stream;
	info.info.timestamp = 42;

	res = mbuf_coded_video_frame_new(&info, &frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_mem_get_data(mem, &data, &capacity);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	memset(data, 0, capacity);

	nalu.size = capacity;
	nalu.h264.type = H264_NALU_TYPE_SLICE;
	nalu.h264.slice_type = H264_SLICE_TYPE_UNKNOWN;
	res = mbuf_coded_video_frame_add_nalu(frame, mem, 0, &nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = mbuf_coded_video_frame_add_ancillary_buffer(
		frame, "test-key", "abc", 4);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	{
		struct vmeta_frame *meta = NULL;
		res = vmeta_frame_new(VMETA_FRAME_TYPE_V3, &meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		res = mbuf_coded_video_frame_set_metadata(frame, meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		/* set_metadata refs the metadata itself, ownership is not
		 * transferred - the caller's own ref must still be dropped. */
		vmeta_frame_unref(meta);
	}

	res = mbuf_coded_video_frame_finalize(frame);
	CU_ASSERT_EQUAL_FATAL(res, 0);

	res = vdec_copy_coded_frame_as_metadata(frame, meta_mem, &meta_frame);
	CU_ASSERT_EQUAL(res, 0);
	CU_ASSERT_PTR_NOT_NULL_FATAL(meta_frame);

	nalu_count = mbuf_coded_video_frame_get_nalu_count(meta_frame);
	CU_ASSERT_EQUAL(nalu_count, 1);

	res = mbuf_coded_video_frame_get_nalu(
		meta_frame, 0, &nalu_data, &meta_nalu);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	CU_ASSERT_EQUAL(meta_nalu.size, 0u);
	mbuf_coded_video_frame_release_nalu(meta_frame, 0, nalu_data);

	{
		struct mbuf_ancillary_data *anc = NULL;
		res = mbuf_coded_video_frame_get_ancillary_data(
			meta_frame, "test-key", &anc);
		CU_ASSERT_EQUAL(res, 0);
		if (res == 0)
			mbuf_ancillary_data_unref(anc);
	}

	{
		struct vmeta_frame *got_meta = NULL;
		res = mbuf_coded_video_frame_get_metadata(meta_frame,
							  &got_meta);
		CU_ASSERT_EQUAL_FATAL(res, 0);
		CU_ASSERT_PTR_NOT_NULL_FATAL(got_meta);
		CU_ASSERT_EQUAL(got_meta->type, VMETA_FRAME_TYPE_V3);
		vmeta_frame_unref(got_meta);
	}

	mbuf_coded_video_frame_unref(frame);
	mbuf_coded_video_frame_unref(meta_frame);
	mbuf_mem_unref(mem);
	mbuf_mem_unref(meta_mem);
}


static void test_copy_coded_frame_as_metadata_invalid_args(void)
{
	struct mbuf_mem *mem = NULL;
	struct mbuf_coded_video_frame *frame;
	struct mbuf_coded_video_frame *meta_frame = NULL;
	int res;

	res = mbuf_mem_generic_new(8, &mem);
	CU_ASSERT_EQUAL_FATAL(res, 0);
	frame = new_convert_test_frame(mem, vdef_h264_byte_stream);

	res = vdec_copy_coded_frame_as_metadata(NULL, mem, &meta_frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_copy_coded_frame_as_metadata(frame, NULL, &meta_frame);
	CU_ASSERT_EQUAL(res, -EINVAL);
	res = vdec_copy_coded_frame_as_metadata(frame, mem, NULL);
	CU_ASSERT_EQUAL(res, -EINVAL);

	mbuf_coded_video_frame_unref(frame);
	mbuf_mem_unref(mem);
}


CU_TestInfo g_vdec_test_format[] = {
	{FN("internal-filter-accepts-matching-format-and-timestamp"),
	 &test_internal_filter_accepts_matching_format_and_timestamp},
	{FN("internal-filter-rejects-unsupported-format"),
	 &test_internal_filter_rejects_unsupported_format},
	{FN("internal-filter-rejects-non-monotonic-timestamp"),
	 &test_internal_filter_rejects_non_monotonic_timestamp},
	{FN("default-input-filter-accepts-matching-frame"),
	 &test_default_input_filter_accepts_matching_frame},
	{FN("default-input-filter-invalid-args"),
	 &test_default_input_filter_invalid_args},
	{FN("default-input-filter-rejects-unsupported-format"),
	 &test_default_input_filter_rejects_unsupported_format},
	{FN("confirm-frame-sets-ancillary-and-counters"),
	 &test_confirm_frame_sets_ancillary_and_counters},
	{FN("confirm-frame-duplicate-ancillary-logs-and-continues"),
	 &test_confirm_frame_duplicate_ancillary_logs_and_continues},
	{FN("is-sync-frame-null-args"), &test_is_sync_frame_null_args},
	{FN("is-sync-frame-declared-type-is-trusted"),
	 &test_is_sync_frame_declared_type_is_trusted},
	{FN("is-sync-frame-unknown-dispatches-to-h264"),
	 &test_is_sync_frame_unknown_dispatches_to_h264},
	{FN("is-sync-frame-unknown-dispatches-to-h265"),
	 &test_is_sync_frame_unknown_dispatches_to_h265},
	{FN("is-sync-frame-unsupported-encoding"),
	 &test_is_sync_frame_unsupported_encoding},
	{FN("format-convert-invalid-args"), &test_format_convert_invalid_args},
	{FN("format-convert-no-op-same-format"),
	 &test_format_convert_no_op_same_format},
	{FN("format-convert-h264-byte-stream-to-avcc"),
	 &test_format_convert_h264_byte_stream_to_avcc},
	{FN("format-convert-h264-avcc-to-byte-stream"),
	 &test_format_convert_h264_avcc_to_byte_stream},
	{FN("format-convert-h265-byte-stream-to-hvcc"),
	 &test_format_convert_h265_byte_stream_to_hvcc},
	{FN("format-convert-raw-nalu-unsupported"),
	 &test_format_convert_raw_nalu_unsupported},
	{FN("format-convert-encoding-mismatch"),
	 &test_format_convert_encoding_mismatch},
	{FN("format-convert-unsupported-encoding"),
	 &test_format_convert_unsupported_encoding},
	{FN("copy-coded-frame-as-metadata"),
	 &test_copy_coded_frame_as_metadata},
	{FN("copy-coded-frame-as-metadata-invalid-args"),
	 &test_copy_coded_frame_as_metadata_invalid_args},

	CU_TEST_INFO_NULL,
};

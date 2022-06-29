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

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifdef _WIN32
#	include <winsock2.h>
#	include <windows.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#	include <sys/mman.h>
#endif /* !_WIN32 */

#include <futils/futils.h>
#include <h264/h264.h>
#include <h265/h265.h>
#include <libpomp.h>
#include <media-buffers/mbuf_coded_video_frame.h>
#include <media-buffers/mbuf_mem_generic.h>
#include <media-buffers/mbuf_raw_video_frame.h>
#include <photo_metadata.h>
#include <video-decode/vdec.h>
#include <video-raw/vraw.h>
#define ULOG_TAG vdec_prog
#include <ulog.h>
ULOG_DECLARE_TAG(vdec_prog);

#ifdef RASPI
#	ifdef TOSTRING
#		undef TOSTRING /* already defined in futils */
#	endif /* TOSTRING */
#	include <bcm_host.h>
#endif /* RASPI */

/* Win32 stubs */
#ifdef _WIN32
static inline const char *strsignal(int signum)
{
	return "??";
}
#endif /* _WIN32 */


#define DEFAULT_IN_BUF_COUNT 10
#define DEFAULT_IN_BUF_CAPACITY (3840 * 2160 * 3 / 4)
#define DEFAULT_MJPEG_IN_BUF_CAPACITY (8000 * 6000 * 3 / 4)
#define DEFAULT_TS_INC 33333


union nalu_type {
	enum h264_nalu_type h264;
	enum h265_nalu_type h265;
};


struct vdec_prog {
	char *input_file;
#ifdef _WIN32
	HANDLE in_file;
	HANDLE in_file_map;
#else
	int in_fd;
#endif
	void *in_data;
	size_t in_len;
	size_t in_off;
	struct pomp_loop *loop;
	union {
		struct h264_reader *h264;
		struct h265_reader *h265;
	} reader;
	struct vdec_decoder *decoder;
	struct vdec_config config;
	int configured;
	int finishing;
	int stopped;
	int last_in_frame;
	int first_out_frame;
	int input_finished;
	int output_finished;
	unsigned int input_count;
	unsigned int output_count;
	unsigned int au_index;
	unsigned int start_index;
	unsigned int max_count;
	size_t total_bytes;
	uint8_t *vps;
	size_t vps_size;
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	unsigned int decimation;
	struct vdef_frac framerate;
	uint64_t ts_inc;
	struct vdef_coded_frame in_info;
	struct mbuf_pool *in_pool;
	int in_pool_allocated;
	struct mbuf_coded_video_frame_queue *in_queue;
	struct mbuf_mem *in_mem;
	size_t in_mem_offset;
	struct mbuf_coded_video_frame *in_frame;
	struct vraw_writer *writer;
	struct vraw_writer_config writer_cfg;
	char *output_file;
	uint8_t *pending_nalu;
	size_t pending_nalu_len;
	union nalu_type pending_nalu_type;
	struct {
		struct vraw_reader *reader;
		struct vdef_raw_format src_format;
		unsigned int start_index;
		char *src;
		uint8_t *src_data;
		size_t src_data_len;
		char *csv;
		FILE *csv_file;
		double psnr_sum[3];
	} psnr;
	struct pmeta_data metadata;
};


static struct vdec_prog *s_self;
static int s_stopping;


static void au_parse_idle(void *userdata);
static void pool_event_cb(struct pomp_evt *evt, void *userdata);


static void unmap_file(struct vdec_prog *self)
{
#ifdef _WIN32
	if (self->in_data != NULL)
		UnmapViewOfFile(self->in_data);
	self->in_data = NULL;
	if (self->in_file_map != INVALID_HANDLE_VALUE)
		CloseHandle(self->in_file_map);
	self->in_file_map = INVALID_HANDLE_VALUE;
	if (self->in_file != INVALID_HANDLE_VALUE)
		CloseHandle(self->in_file);
	self->in_file = INVALID_HANDLE_VALUE;
#else
	if (self->in_fd >= 0) {
		if (self->in_data != NULL)
			munmap(self->in_data, self->in_len);
		self->in_data = NULL;
		close(self->in_fd);
		self->in_fd = -1;
	}
#endif
}


static int map_file(struct vdec_prog *self)
{
	int res;

#ifdef _WIN32
	BOOL ret;
	LARGE_INTEGER filesize;

	self->in_file = CreateFileA(self->input_file,
				    GENERIC_READ,
				    0,
				    NULL,
				    OPEN_EXISTING,
				    FILE_ATTRIBUTE_NORMAL,
				    NULL);
	if (self->in_file == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileA('%s')", -res, self->input_file);
		goto error;
	}

	self->in_file_map = CreateFileMapping(
		self->in_file, NULL, PAGE_READONLY, 0, 0, NULL);
	if (self->in_file_map == INVALID_HANDLE_VALUE) {
		res = -EIO;
		ULOG_ERRNO("CreateFileMapping('%s')", -res, self->input_file);
		goto error;
	}

	ret = GetFileSizeEx(self->in_file, &filesize);
	if (ret == FALSE) {
		res = -EIO;
		ULOG_ERRNO("GetFileSizeEx('%s')", -res, self->input_file);
		goto error;
	}
	self->in_len = filesize.QuadPart;

	self->in_data =
		MapViewOfFile(self->in_file_map, FILE_MAP_READ, 0, 0, 0);
	if (self->in_data == NULL) {
		res = -EIO;
		ULOG_ERRNO("MapViewOfFile('%s')", -res, self->input_file);
		goto error;
	}
#else
	/* Try to open input file */
	self->in_fd = open(self->input_file, O_RDONLY);
	if (self->in_fd < 0) {
		res = -errno;
		ULOG_ERRNO("open('%s')", -res, self->input_file);
		goto error;
	}

	/* Get size and map it */
	self->in_len = lseek(self->in_fd, 0, SEEK_END);
	if (self->in_len == (size_t)-1) {
		res = -errno;
		ULOG_ERRNO("lseek", -res);
		goto error;
	}

	self->in_data = mmap(
		NULL, self->in_len, PROT_READ, MAP_PRIVATE, self->in_fd, 0);
	if (self->in_data == MAP_FAILED) {
		res = -errno;
		ULOG_ERRNO("mmap", -res);
		goto error;
	}
#endif

	return 0;

error:
	unmap_file(self);
	return res;
}


static int configure(struct vdec_prog *self)
{
	int res, buf_count;
	size_t mem_size;

	switch (self->config.encoding) {
	case VDEF_ENCODING_H264: {
		res = vdec_set_h264_ps(self->decoder,
				       self->sps,
				       self->sps_size,
				       self->pps,
				       self->pps_size,
				       &vdef_h264_raw_nalu);
		if (res < 0) {
			ULOG_ERRNO("vdec_set_h264_ps", -res);
			return res;
		}
		struct h264_info info;
		res = h264_get_info(self->sps,
				    self->sps_size,
				    self->pps,
				    self->pps_size,
				    &info);
		if (res < 0) {
			ULOG_ERRNO("h264_get_info", -res);
			return res;
		}
		self->framerate.num = info.framerate_num;
		self->framerate.den = info.framerate_den * self->decimation;
		break;
	}
	case VDEF_ENCODING_H265: {
		res = vdec_set_h265_ps(self->decoder,
				       self->vps,
				       self->vps_size,
				       self->sps,
				       self->sps_size,
				       self->pps,
				       self->pps_size,
				       &vdef_h265_raw_nalu);
		if (res < 0) {
			ULOG_ERRNO("vec_set_h265_ps", -res);
			return res;
		}
		struct h265_info info;
		res = h265_get_info(self->vps,
				    self->vps_size,
				    self->sps,
				    self->sps_size,
				    self->pps,
				    self->pps_size,
				    &info);

		if (res < 0) {
			ULOG_ERRNO("h265_get_info", -res);
			return res;
		}
		self->framerate.num = info.framerate_num;
		self->framerate.den = info.framerate_den * self->decimation;
		break;
	}
	case VDEF_ENCODING_MJPEG: {
		/* As it is not possible yet to handle a stream of JPEG
		 * photo, let's consider that only one photo can be handled
		 * and force framerate to 1 fps */
		self->framerate.num = 1;
		self->framerate.den = 1;

		/* clang-format off */
		struct vdef_format_info format_info = {
			.bit_depth = 8,
			.sar = {1, 1},
			.full_range = 1,
			.framerate = self->framerate,
			.color_primaries = VDEF_COLOR_PRIMARIES_SRGB,
			.transfer_function = VDEF_TRANSFER_FUNCTION_BT709,
			.matrix_coefs = VDEF_MATRIX_COEFS_SRGB,
			.resolution = {
				.width = self->metadata.width,
				.height = self->metadata.height,
			},
		};
		/* clang-format on */

		res = vdec_set_jpeg_params(self->decoder, &format_info);
		if (res < 0) {
			ULOG_ERRNO("vdec_set_jpeg_params", -res);
			return res;
		}
		break;
	}
	default:
		break;
	}

	/* Input buffer pool */
	self->in_pool = vdec_get_input_buffer_pool(self->decoder);
	if (self->in_pool == NULL) {
		mem_size = self->config.encoding == VDEF_ENCODING_MJPEG
				   ? DEFAULT_MJPEG_IN_BUF_CAPACITY
				   : DEFAULT_IN_BUF_CAPACITY;
		buf_count = (self->config.preferred_thread_count + 1 >
			     DEFAULT_IN_BUF_COUNT)
				    ? self->config.preferred_thread_count + 1
				    : DEFAULT_IN_BUF_COUNT;
		res = mbuf_pool_new(mbuf_mem_generic_impl,
				    mem_size,
				    0,
				    MBUF_POOL_SMART_GROW,
				    buf_count,
				    "vdec_default_pool",
				    &self->in_pool);
		if (res < 0) {
			ULOG_ERRNO("mbuf_pool_new:input", -res);
			return res;
		}
		self->in_pool_allocated = 1;
	}

	/* Input buffer queue */
	self->in_queue = vdec_get_input_buffer_queue(self->decoder);
	if (self->in_queue == NULL) {
		res = -EPROTO;
		ULOG_ERRNO("vdec_get_input_buffer_queue", -res);
		return res;
	}

	self->configured = 1;
	return 0;
}


static int au_decode(struct vdec_prog *self)
{
	int res = 0, err;
	ssize_t len;

	if (self->in_frame == NULL)
		return 0;

	if (((self->max_count > 0) && (self->input_count >= self->max_count)) ||
	    (self->finishing))
		goto cleanup;


	res = mbuf_coded_video_frame_finalize(self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize:input", -res);
		goto cleanup;
	}

	res = mbuf_coded_video_frame_queue_push(self->in_queue, self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push:input", -res);
		goto cleanup;
	}
	len = mbuf_coded_video_frame_get_packed_size(self->in_frame);
	if (len < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_size", (int)-len);
	else
		self->total_bytes += len;
	self->input_count++;
	self->output_finished = 0;

cleanup:
	err = mbuf_coded_video_frame_unref(self->in_frame);
	if (err < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_unref:input", -err);
	self->in_frame = NULL;
	err = mbuf_mem_unref(self->in_mem);
	if (err < 0)
		ULOG_ERRNO("mbuf_mem_unref:input", -err);
	self->in_mem = NULL;
	self->in_info.info.index++;
	self->in_info.info.timestamp += self->ts_inc;

	return res;
}


static int append_to_frame(struct vdec_prog *self,
			   struct mbuf_coded_video_frame *frame,
			   struct mbuf_mem *mem,
			   const uint8_t *data,
			   size_t len,
			   union nalu_type type)
{
	int res;
	size_t au_offset, capacity;
	size_t nalu_offset;
	uint8_t *au_data, *nalu_data;
	uint32_t start;

	ULOG_ERRNO_RETURN_ERR_IF(frame == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(mem == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);

	switch (self->in_info.format.data_format) {
	case VDEF_CODED_DATA_FORMAT_AVCC:
	case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		nalu_offset = 4;
		break;
	case VDEF_CODED_DATA_FORMAT_RAW_NALU:
	case VDEF_CODED_DATA_FORMAT_JFIF:
	default:
		nalu_offset = 0;
		break;
	}

	au_offset = self->in_mem_offset;
	res = mbuf_mem_get_data(mem, (void **)&au_data, &capacity);
	if (res < 0) {
		ULOG_ERRNO("mbuf_mem_get_data", -res);
		return res;
	}
	if (capacity < au_offset + nalu_offset + len) {
		ULOGE("memory too small for frame");
		return -ENOBUFS;
	}
	if (au_data == NULL) {
		ULOG_ERRNO("mbuf_mem_get_data", EPROTO);
		return -EPROTO;
	}
	nalu_data = au_data + au_offset;
	switch (self->in_info.format.data_format) {
	case VDEF_CODED_DATA_FORMAT_BYTE_STREAM:
		start = htonl(0x00000001);
		memcpy(nalu_data, &start, sizeof(uint32_t));
		break;
	case VDEF_CODED_DATA_FORMAT_AVCC:
		start = htonl(len);
		memcpy(nalu_data, &start, sizeof(uint32_t));
		break;
	default:
		/* Nothing to do otherwise */
		break;
	}
	memcpy(nalu_data + nalu_offset, data, len);
	self->in_mem_offset = au_offset + nalu_offset + len;

	struct vdef_nalu nalu = {
		.size = len + nalu_offset,
	};
	switch (self->in_info.format.encoding) {
	case VDEF_ENCODING_H264:
		nalu.h264.type = type.h264;
		break;
	case VDEF_ENCODING_H265:
		nalu.h265.type = type.h265;
		break;
	case VDEF_ENCODING_MJPEG:
		/* nothing to do */
		break;
	default:
		ULOGE("unsupported encoding %s",
		      vdef_encoding_to_str(self->in_info.format.encoding));
		return -EPROTO;
	}
	res = mbuf_coded_video_frame_add_nalu(frame, mem, au_offset, &nalu);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_add_nalu", -res);
		return res;
	}

	return 0;
}


static int au_end(struct vdec_prog *self)
{
	int res = 0;

	if ((self->in_frame != NULL) && (self->au_index >= self->start_index)) {
		res = au_decode(self);
		if (res < 0)
			ULOG_ERRNO("au_decode", -res);
	}
	self->au_index++;

	return res;
}


static void h264_au_end_cb(struct h264_ctx *ctx, void *userdata)
{
	struct vdec_prog *self = userdata;
	int res = au_end(userdata);
	if (res < 0) {
		res = h264_reader_stop(self->reader.h264);
		if (res < 0)
			ULOG_ERRNO("h264_reader_stop", -res);
	}
}


static void h265_au_end_cb(struct h265_ctx *ctx, void *userdata)
{
	struct vdec_prog *self = userdata;
	int res = au_end(userdata);
	if (res < 0) {
		res = h265_reader_stop(self->reader.h265);
		if (res < 0)
			ULOG_ERRNO("h265_reader_stop", -res);
	}
}


static void stop_reader(struct vdec_prog *self)
{
	int res;
	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		res = h264_reader_stop(self->reader.h264);
		if (res < 0)
			ULOG_ERRNO("h264_reader_stop", -res);
		break;
	case VDEF_ENCODING_H265:
		res = h265_reader_stop(self->reader.h265);
		if (res < 0)
			ULOG_ERRNO("h265_reader_stop", -res);
		break;
	default:
		break;
	}
}


static void nalu_end(struct vdec_prog *self,
		     union nalu_type type,
		     const uint8_t *buf,
		     size_t len)
{
	int res;

	if (self->finishing) {
		stop_reader(self);
		return;
	}

	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		if ((type.h264 == H264_NALU_TYPE_SPS) && (self->sps == NULL)) {
			self->sps_size = len;
			self->sps = malloc(self->sps_size);
			if (self->sps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->sps, buf, len);
			ULOGI("SPS found");
		} else if ((type.h264 == H264_NALU_TYPE_PPS) &&
			   (self->pps == NULL)) {
			self->pps_size = len;
			self->pps = malloc(self->pps_size);
			if (self->pps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->pps, buf, len);
			ULOGI("PPS found");
		}
		break;
	case VDEF_ENCODING_H265:
		if ((type.h265 == H265_NALU_TYPE_VPS_NUT) &&
		    self->vps == NULL) {
			self->vps_size = len;
			self->vps = malloc(self->vps_size);
			if (self->vps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->vps, buf, len);
			ULOGI("VPS found");
		} else if ((type.h265 == H265_NALU_TYPE_SPS_NUT) &&
			   self->sps == NULL) {
			self->sps_size = len;
			self->sps = malloc(self->sps_size);
			if (self->sps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->sps, buf, len);
			ULOGI("SPS found");
		} else if ((type.h265 == H265_NALU_TYPE_PPS_NUT) &&
			   self->pps == NULL) {
			self->pps_size = len;
			self->pps = malloc(self->pps_size);
			if (self->pps == NULL) {
				ULOG_ERRNO("malloc", ENOMEM);
				return;
			}
			memcpy(self->pps, buf, len);
			ULOGI("PPS found");
		}
		break;
	default:
		break;
	}

	int ps_ready = 0;
	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		ps_ready = (self->sps != NULL) && (self->pps != NULL);
		break;
	case VDEF_ENCODING_H265:
		ps_ready = (self->vps != NULL) && (self->sps != NULL) &&
			   (self->pps != NULL);
		break;
	default:
		break;
	}

	/* Configure the decoder */
	if ((!self->configured) && ps_ready) {
		res = configure(self);
		if (res < 0) {
			ULOG_ERRNO("configure", -res);
			return;
		}
	}

	int is_ps = 0;
	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		is_ps = (type.h264 == H264_NALU_TYPE_AUD) ||
			(type.h264 == H264_NALU_TYPE_SPS) ||
			(type.h264 == H264_NALU_TYPE_PPS);
		break;
	case VDEF_ENCODING_H265:
		is_ps = (type.h265 == H265_NALU_TYPE_AUD_NUT) ||
			(type.h265 == H265_NALU_TYPE_VPS_NUT) ||
			(type.h265 == H265_NALU_TYPE_SPS_NUT) ||
			(type.h265 == H265_NALU_TYPE_PPS_NUT);
		break;
	default:
		break;
	}

	/* Start decoding at start_index */
	if (self->au_index < self->start_index)
		return;

	/* Filter out VPS, SPS, PPS and AUD */
	if (is_ps)
		return;

	/* Get an input buffer (non-blocking) */
	if ((self->in_pool != NULL) && (self->in_mem == NULL)) {
		res = mbuf_pool_get(self->in_pool, &self->in_mem);
		if (res < 0) {
			if (res != -EAGAIN)
				ULOG_ERRNO("mbuf_pool_get:input", -res);

			/* Stop the parser */
			stop_reader(self);

			/* Copy data in a buffer */
			uint8_t *nalu = realloc(self->pending_nalu, len);
			if (!nalu)
				return;
			self->pending_nalu = nalu;
			self->pending_nalu_len = len;
			self->pending_nalu_type = type;
			memcpy(self->pending_nalu, buf, len);
			return;
		}
		self->in_mem_offset = 0;
	}
	if (self->in_mem == NULL)
		return;

	/* Create the frame */
	if (self->in_frame == NULL) {
		res = mbuf_coded_video_frame_new(&self->in_info,
						 &self->in_frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_new:input", -res);
			return;
		}
	}

	/* Add the NALU to the input frame */
	res = append_to_frame(
		self, self->in_frame, self->in_mem, buf, len, type);
	if (res < 0)
		ULOG_ERRNO("append_to_frame", -res);
}


static void h264_nalu_end_cb(struct h264_ctx *ctx,
			     enum h264_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h264_nalu_header *nh,
			     void *userdata)
{
	nalu_end(userdata, (union nalu_type){.h264 = type}, buf, len);
}


static void h265_nalu_end_cb(struct h265_ctx *ctx,
			     enum h265_nalu_type type,
			     const uint8_t *buf,
			     size_t len,
			     const struct h265_nalu_header *nh,
			     void *userdata)
{
	nalu_end(userdata, (union nalu_type){.h265 = type}, buf, len);
}


static const struct h264_ctx_cbs h264_cbs = {
	.au_end = h264_au_end_cb,
	.nalu_end = h264_nalu_end_cb,
};


static const struct h265_ctx_cbs h265_cbs = {
	.au_end = h265_au_end_cb,
	.nalu_end = h265_nalu_end_cb,
};


static int compute_psnr(struct vdec_prog *self,
			struct mbuf_raw_video_frame *frame,
			struct vdef_raw_frame *info,
			double psnr[4])
{
	int res;
	unsigned int plane_count;
	struct vraw_frame dec_frame;
	struct vraw_frame src_frame;

	if (self->psnr.src == NULL)
		return 0;

	memset(&src_frame, 0, sizeof(src_frame));
	memset(&dec_frame, 0, sizeof(dec_frame));

	/* Initialize the reader for the source file */
	if (self->psnr.reader == NULL) {
		struct vraw_reader_config reader_cfg = {0};
		reader_cfg.start_index = self->psnr.start_index;
		reader_cfg.format = self->psnr.src_format;
		vdef_frame_to_format_info(&info->info, &reader_cfg.info);
		reader_cfg.info.framerate = self->framerate;
		if ((strlen(self->psnr.src) > 4) &&
		    (strcmp(self->psnr.src + strlen(self->psnr.src) - 4,
			    ".y4m") == 0))
			reader_cfg.y4m = 1;

		res = vraw_reader_new(
			self->psnr.src, &reader_cfg, &self->psnr.reader);
		if (res < 0) {
			ULOG_ERRNO("vraw_reader_new", -res);
			return res;
		}

		res = vraw_reader_get_config(self->psnr.reader, &reader_cfg);
		if (res < 0) {
			ULOG_ERRNO("vraw_reader_get_config", -res);
			return res;
		}

		self->psnr.src_data_len = reader_cfg.info.resolution.width *
					  reader_cfg.info.resolution.height *
					  3 / 2 *
					  (reader_cfg.format.data_size / 8);
	}

	if (self->psnr.src_data == NULL) {
		self->psnr.src_data = malloc(self->psnr.src_data_len);
		if (self->psnr.src_data == NULL) {
			res = -ENOMEM;
			ULOG_ERRNO("malloc", -res);
			return res;
		}
	}

	/* Open CSV file */
	if ((self->psnr.csv != NULL) && (self->psnr.csv_file == NULL)) {
		self->psnr.csv_file = fopen(self->psnr.csv, "w");
		if (self->psnr.csv_file == NULL) {
			res = -errno;
			ULOG_ERRNO("fopen", -res);
			/* try later */
		}
	}

	/* Read the source frame */
	do {
		res = vraw_reader_frame_read(self->psnr.reader,
					     self->psnr.src_data,
					     self->psnr.src_data_len,
					     &src_frame);
		if (res < 0) {
			ULOG_ERRNO("vraw_reader_frame_read", -res);
			return res;
		}
	} while ((src_frame.frame.info.index % self->decimation) ||
		 (src_frame.frame.info.index / self->decimation !=
		  info->info.index));

	/* Prepare decoded frame */
	plane_count = vdef_get_raw_frame_plane_count(&info->format);
	for (unsigned int i = 0; i < plane_count; i++) {
		size_t len;
		res = mbuf_raw_video_frame_get_plane(
			frame, i, (const void **)&dec_frame.cdata[i], &len);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_plane", -res);
			goto out;
		}
	}
	dec_frame.frame = *info;

	/* Compute the PSNR */
	res = vraw_compute_psnr(&src_frame, &dec_frame, psnr);
	if (res < 0) {
		ULOG_ERRNO("vraw_compute_psnr", -res);
		return res;
	}

	if (self->psnr.csv_file != NULL) {
		fprintf(self->psnr.csv_file,
			"%u %.3f %.3f %.3f\n",
			info->info.index,
			psnr[0],
			psnr[1],
			psnr[2]);
	}

	for (int i = 0; i < 3; i++)
		self->psnr.psnr_sum[i] += psnr[i];
out:
	for (unsigned int i = 0; i < plane_count; i++) {
		if (dec_frame.cdata[i] == NULL)
			continue;
		int err = mbuf_raw_video_frame_release_plane(
			frame, i, dec_frame.cdata[i]);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_release_plane", -err);
	}
	return 0;
}


static int yuv_output(struct vdec_prog *self,
		      struct mbuf_raw_video_frame *out_frame)
{
	int res;
	struct vraw_frame frame;
	struct vdef_raw_frame info;
	unsigned int plane_count;

	if (out_frame == NULL)
		return -EINVAL;

	if (self->output_file == NULL)
		return 0;

	res = mbuf_raw_video_frame_get_frame_info(out_frame, &info);
	if (res < 0) {
		ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);
		return res;
	}

	if (self->writer == NULL) {
		/* Initialize the writer on first frame */
		self->writer_cfg.format = info.format;
		vdef_frame_to_format_info(&info.info, &self->writer_cfg.info);
		self->writer_cfg.info.framerate = self->framerate;
		res = vraw_writer_new(
			self->output_file, &self->writer_cfg, &self->writer);
		if (res < 0) {
			ULOG_ERRNO("vraw_writer_new", -res);
			return res;
		}
		char *fmt = vdef_raw_format_to_str(&self->writer_cfg.format);
		ULOGI("YUV output file format is %s", fmt);
		free(fmt);
	}

	plane_count = vdef_get_raw_frame_plane_count(&info.format);
	for (unsigned int i = 0; i < plane_count; i++) {
		size_t len;
		res = mbuf_raw_video_frame_get_plane(
			out_frame, i, (const void **)&frame.cdata[i], &len);
		if (res < 0) {
			ULOG_ERRNO("mbuf_raw_video_frame_get_plane", -res);
			goto out;
		}
	}
	frame.frame = info;

	/* Write the frame */
	res = vraw_writer_frame_write(self->writer, &frame);
	if (res < 0) {
		ULOG_ERRNO("vraw_writer_frame_write", -res);
		goto out;
	}

out:
	for (unsigned int i = 0; i < plane_count; i++) {
		if (frame.cdata[i] == NULL)
			continue;
		int err = mbuf_raw_video_frame_release_plane(
			out_frame, i, frame.cdata[i]);
		if (err < 0)
			ULOG_ERRNO("mbuf_raw_video_frame_release_plane", -err);
	}
	return res;
}


static uint64_t get_timestamp(struct mbuf_raw_video_frame *frame,
			      const char *key)
{
	int res;
	struct mbuf_ancillary_data *data;
	uint64_t ts = 0;
	const void *raw_data;
	size_t len;

	res = mbuf_raw_video_frame_get_ancillary_data(frame, key, &data);
	if (res < 0)
		return 0;

	raw_data = mbuf_ancillary_data_get_buffer(data, &len);
	if (!raw_data || len != sizeof(ts))
		goto out;
	memcpy(&ts, raw_data, sizeof(ts));

out:
	mbuf_ancillary_data_unref(data);
	return ts;
}


static int frame_output(struct vdec_prog *self,
			struct mbuf_raw_video_frame *out_frame)
{
	int res = 0;
	double psnr[4] = {0};
	uint64_t input_time, dequeue_time, output_time;
	struct vdef_raw_frame info;

	res = yuv_output(self, out_frame);
	if (res < 0)
		ULOG_ERRNO("yuv_output", -res);

	input_time = get_timestamp(out_frame, VDEC_ANCILLARY_KEY_INPUT_TIME);
	dequeue_time =
		get_timestamp(out_frame, VDEC_ANCILLARY_KEY_DEQUEUE_TIME);
	output_time = get_timestamp(out_frame, VDEC_ANCILLARY_KEY_OUTPUT_TIME);

	res = mbuf_raw_video_frame_get_frame_info(out_frame, &info);
	if (res < 0)
		ULOG_ERRNO("mbuf_raw_video_frame_get_frame_info", -res);

	res = compute_psnr(self, out_frame, &info, psnr);
	if (res < 0)
		ULOG_ERRNO("compute_psnr", -res);

	ULOGI("decoded frame #%d, dequeue: %.2fms, decode: %.2fms, "
	      "overall: %.2fms, PSNR: Y=%.3fdB U=%.3fdB Y=%.3fdB",
	      info.info.index,
	      (float)(dequeue_time - input_time) / 1000.,
	      (float)(output_time - dequeue_time) / 1000.,
	      (float)(output_time - input_time) / 1000.,
	      psnr[0],
	      psnr[1],
	      psnr[2]);

	self->first_out_frame = 0;

	return 0;
}


static void frame_output_cb(struct vdec_decoder *dec,
			    int status,
			    struct mbuf_raw_video_frame *out_frame,
			    void *userdata)
{
	int res;
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(out_frame == NULL, EINVAL);

	if (status != 0) {
		ULOGE("decoder error, resync required");
		return;
	}

	res = frame_output(self, out_frame);
	if (res < 0)
		ULOG_ERRNO("frame_output", -res);

	self->output_count++;

	if ((self->input_finished) &&
	    (self->output_count == self->input_count)) {
		ULOGI("decoding is finished (output, count=%d)",
		      self->output_count);
		self->output_finished = 1;
		return;
	}

	/* TODO: This should be triggered by an event in the in_pool signaling
	 * that a memory is available */
	if (self->pending_nalu_len && self->in_pool && !self->in_mem) {
		res = mbuf_pool_get(self->in_pool, &self->in_mem);
		if (res < 0)
			return;
		self->in_mem_offset = 0;

		/* Create the frame */
		res = mbuf_coded_video_frame_new(&self->in_info,
						 &self->in_frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_new:input", -res);
			mbuf_mem_unref(self->in_mem);
			self->in_mem = NULL;
			return;
		}

		/* Add the NALU to the input frame */
		res = append_to_frame(self,
				      self->in_frame,
				      self->in_mem,
				      self->pending_nalu,
				      self->pending_nalu_len,
				      self->pending_nalu_type);
		if (res < 0) {
			ULOG_ERRNO("append_to_frame", -res);
			mbuf_coded_video_frame_unref(self->in_frame);
			mbuf_mem_unref(self->in_mem);
			self->in_frame = NULL;
			self->in_mem = NULL;
			return;
		}

		res = pomp_loop_idle_add(self->loop, &au_parse_idle, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
		self->pending_nalu_len = 0;
	}
}


static void flush_cb(struct vdec_decoder *dec, void *userdata)
{
	int res;
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("decoder is flushed");

	/* Stop the decoder */
	res = vdec_stop(self->decoder);
	if (res < 0)
		ULOG_ERRNO("vdec_stop", -res);
}


static void stop_cb(struct vdec_decoder *dec, void *userdata)
{
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("decoder is stopped");
	self->stopped = 1;

	pomp_loop_wakeup(self->loop);
}


static const struct vdec_cbs vdec_cbs = {
	.frame_output = frame_output_cb,
	.flush = flush_cb,
	.stop = stop_cb,
};


static void finish_idle(void *userdata)
{
	int res;
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	if (self->finishing)
		return;

	if ((s_stopping) || (self->input_finished)) {
		self->finishing = 1;

		/* Stop the parser */
		stop_reader(self);

		/* Flush the decoder */
		res = vdec_flush(self->decoder, (s_stopping) ? 1 : 0);
		if (res < 0)
			ULOG_ERRNO("vdec_flush", -res);
	}
}


static void jpeg_parse_idle(void *userdata)
{
	int res;
	ssize_t len;
	struct vdec_prog *self = userdata;

	/* TODO need a JPEG reader,
	 * to handle the case of a stream that
	 * contains several JPEG file
	 * by now only one frame can be handled */

	res = configure(self);
	if (res < 0) {
		ULOG_ERRNO("configure", -res);
		goto cleanup;
	}

	if ((self->in_pool != NULL) && (self->in_mem == NULL)) {
		res = mbuf_pool_get(self->in_pool, &self->in_mem);
		if ((res < 0) && (res != -EAGAIN))
			ULOG_ERRNO("mbuf_pool_get:input", -res);

		if (self->in_mem == NULL)
			goto cleanup;
	}

	/* Create the frame */
	if (self->in_frame == NULL) {
		res = mbuf_coded_video_frame_new(&self->in_info,
						 &self->in_frame);
		if (res < 0) {
			ULOG_ERRNO("mbuf_coded_video_frame_new:input", -res);
			mbuf_mem_unref(self->in_mem);
			self->in_mem = NULL;
			goto cleanup;
		}
	}

	/* Take note that self->pending_nalu_type that is useless here for
	 * JPEG/MJPEG decoding */
	res = append_to_frame(self,
			      self->in_frame,
			      self->in_mem,
			      self->in_data,
			      self->in_len,
			      self->pending_nalu_type);
	if (res < 0) {
		ULOG_ERRNO("append_to_frame", -res);
		goto cleanup;
	}

	res = mbuf_coded_video_frame_finalize(self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_finalize:input", -res);
		goto cleanup;
	}

	res = mbuf_coded_video_frame_queue_push(self->in_queue, self->in_frame);
	if (res < 0) {
		ULOG_ERRNO("mbuf_coded_video_frame_queue_push:input", -res);
		goto cleanup;
	}
	len = mbuf_coded_video_frame_get_packed_size(self->in_frame);
	if (len < 0)
		ULOG_ERRNO("mbuf_coded_video_frame_get_packed_size", (int)-len);
	else
		self->total_bytes += len;
	self->input_count++;
	self->output_finished = 0;

cleanup:
	if (self->in_frame) {
		res = mbuf_coded_video_frame_unref(self->in_frame);
		if (res < 0)
			ULOG_ERRNO("mbuf_coded_video_frame_unref:input", -res);
		self->in_frame = NULL;
	}

	if (self->in_mem) {
		res = mbuf_mem_unref(self->in_mem);
		if (res < 0)
			ULOG_ERRNO("mbuf_mem_unref:input", -res);
		self->in_mem = NULL;
	}
	self->in_info.info.index++;
	self->in_info.info.timestamp += self->ts_inc;

	/* TODO: find a better way to end decoding for MJPEG stream */
	self->input_finished = 1;
	res = pomp_loop_idle_add(self->loop, &finish_idle, self);
	if (res < 0)
		ULOG_ERRNO("pomp_loop_idle_add", -res);
}

static void au_parse_idle(void *userdata)
{
	int res;
	size_t off = 0;
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	/* Waiting for input memory buffer */
	if (self->pending_nalu_len && self->in_pool && !self->in_mem)
		return;

	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		res = h264_reader_parse(self->reader.h264,
					0,
					(uint8_t *)self->in_data + self->in_off,
					self->in_len - self->in_off,
					&off);
		if (res < 0) {
			ULOG_ERRNO("h264_reader_parse", -res);
			return;
		}
		break;
	case VDEF_ENCODING_H265:
		res = h265_reader_parse(self->reader.h265,
					0,
					(uint8_t *)self->in_data + self->in_off,
					self->in_len - self->in_off,
					&off);
		if (res < 0) {
			ULOG_ERRNO("h265_reader_parse", -res);
			return;
		}
		break;
	default:
		break;
	}

	self->in_off += off;
	if (((self->in_off >= self->in_len) ||
	     ((self->max_count > 0) &&
	      (self->input_count >= self->max_count))) &&
	    self->in_frame != NULL) {
		/* Decode the last AU in the file */
		au_end(self);

		ULOGI("decoding is finished (input, count=%d)",
		      self->input_count);
		self->input_finished = 1;

		/* Stop the parser now, no point continuing */
		stop_reader(self);

		res = pomp_loop_idle_add(self->loop, &finish_idle, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
	}

	if (!self->input_finished && !self->finishing) {
		res = pomp_loop_idle_add(self->loop, &au_parse_idle, self);
		if (res < 0)
			ULOG_ERRNO("pomp_loop_idle_add", -res);
	}
}


static void sig_handler(int signum)
{
	int res;

	ULOGI("signal %d(%s) received", signum, strsignal(signum));
	printf("Stopping...\n");

	s_stopping = 1;
	signal(SIGINT, SIG_DFL);

	if (s_self == NULL)
		return;

	res = pomp_loop_idle_add(s_self->loop, &finish_idle, s_self);
	if (res < 0)
		ULOG_ERRNO("pomp_loop_idle_add", -res);
}


enum args_id {
	ARGS_ID_PSNR_SRC = 256,
	ARGS_ID_PSNR_SRC_FORMAT,
	ARGS_ID_PSNR_SRC_START,
	ARGS_ID_PSNR_CSV,
};


static const char short_options[] = "hi:o:s:n:lj:d:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"infile", required_argument, NULL, 'i'},
	{"outfile", required_argument, NULL, 'o'},
	{"start", required_argument, NULL, 's'},
	{"count", required_argument, NULL, 'n'},
	{"low-delay", no_argument, NULL, 'l'},
	{"threads", required_argument, NULL, 'j'},
	{"decimation", required_argument, NULL, 'd'},
	{"psnr-src", required_argument, NULL, ARGS_ID_PSNR_SRC},
	{"psnr-src-format", required_argument, NULL, ARGS_ID_PSNR_SRC_FORMAT},
	{"psnr-src-start", required_argument, NULL, ARGS_ID_PSNR_SRC_START},
	{"psnr-csv", required_argument, NULL, ARGS_ID_PSNR_CSV},
	{0, 0, 0, 0},
};


static void welcome(char *prog_name)
{
	printf("\n%s - Video decoding program\n"
	       "Copyright (c) 2017 Parrot Drones SAS\n\n",
	       prog_name);
}


static void usage(char *prog_name)
{
	printf("Usage: %s [options]\n"
	       "Options:\n"
	       "  -h | --help                        "
	       "Print this message\n"
	       "  -i | --infile <file_name>          "
	       "H.264 Annex B byte stream input file (.264/.h264)\n"
	       "  -o | --outfile <file_name>         "
	       "YUV output file\n"
	       "  -s | --start <i>                   "
	       "Start decoding at frame index i\n"
	       "  -n | --count <n>                   "
	       "Decode at most n frames\n"
	       "  -l | --low-delay                   "
	       "Favor low delay decoding\n"
	       "  -j | --threads <thread_count>      "
	       "Preferred decoding thread count (0 = default,\n"
	       "                                     "
	       "1 = no multi-threading, >1 = multi-threading)\n"
	       "  -d | --decimation <n>              "
	       "Known decimation factor, applied to bitrate computation "
	       "and PSNR source file (default is 1)\n"
	       "       --psnr-src <yuv_file>         "
	       "Source YUV file for PSNR computation\n"
	       "       --psnr-src-format <format>    "
	       "Source YUV file data format (\"I420\", \"YV12\", "
	       "\"NV12\", \"NV21\"...)\n"
	       "       --psnr-src-start <i>          "
	       "Source YUV file start index for PSNR computation\n"
	       "       --psnr-csv <file>             "
	       "Output the PSNR results to a CSV file\n"
	       "\n",
	       prog_name);
}


static int is_suffix(const char *suffix, const char *s)
{
	size_t suffix_len = strlen(suffix);
	size_t s_len = strlen(s);

	return s_len >= suffix_len &&
	       (strcasecmp(suffix, &s[s_len - suffix_len]) == 0);
}


int main(int argc, char **argv)
{
	int res = 0, status = EXIT_SUCCESS;
	int idx, c;
	struct vdec_prog *self = NULL;
	struct timespec cur_ts = {0, 0};
	uint64_t start_time = 0, end_time = 0;
	double bitrate = 0.;

	s_self = NULL;
	s_stopping = 0;

	struct vdef_coded_format byte_stream = {
		.data_format = VDEF_CODED_DATA_FORMAT_BYTE_STREAM,
	};
	struct vdef_coded_format avcc = {
		.data_format = VDEF_CODED_DATA_FORMAT_AVCC,
	};

	welcome(argv[0]);

#ifdef RASPI
	bcm_host_init();
#endif /* RASPI */

	/* Context allocation */
	self = calloc(1, sizeof(*self));
	if (self == NULL) {
		ULOG_ERRNO("calloc", ENOMEM);
		status = EXIT_FAILURE;
		goto out;
	}
	s_self = self;
#ifdef _WIN32
	self->in_file = INVALID_HANDLE_VALUE;
	self->in_file_map = INVALID_HANDLE_VALUE;
#else
	self->in_fd = -1;
#endif
	self->first_out_frame = 1;
	self->ts_inc = DEFAULT_TS_INC;
	self->decimation = 1;

	/* Command-line parameters */
	while ((c = getopt_long(
			argc, argv, short_options, long_options, &idx)) != -1) {
		switch (c) {
		case 0:
			break;

		case 'h':
			usage(argv[0]);
			status = EXIT_SUCCESS;
			goto out;

		case 'i':
			self->input_file = optarg;
			if (is_suffix(".h265", self->input_file) ||
			    is_suffix(".265", self->input_file) ||
			    is_suffix(".hvc", self->input_file)) {
				self->config.encoding = VDEF_ENCODING_H265;
			} else if (is_suffix(".h264", self->input_file) ||
				   is_suffix(".264", self->input_file)) {
				self->config.encoding = VDEF_ENCODING_H264;
			} else {
				self->config.encoding = VDEF_ENCODING_MJPEG;
			}
			break;

		case 'o':
			self->output_file = optarg;
			if (is_suffix(".y4m", self->output_file))
				self->writer_cfg.y4m = 1;
			break;

		case 's':
			self->start_index = atoi(optarg);
			break;

		case 'n':
			self->max_count = atoi(optarg);
			break;

		case 'l':
			self->config.low_delay = 1;
			break;

		case 'j':
			self->config.preferred_thread_count = atoi(optarg);
			break;

		case 'd':
			self->decimation = atoi(optarg);
			break;

		case ARGS_ID_PSNR_SRC:
			self->psnr.src = optarg;
			break;

		case ARGS_ID_PSNR_SRC_FORMAT:
			res = vdef_raw_format_from_str(optarg,
						       &self->psnr.src_format);
			if (res < 0) {
				ULOG_ERRNO("vdef_raw_format_from_str", -res);
				status = EXIT_FAILURE;
				goto out;
			}
			break;

		case ARGS_ID_PSNR_SRC_START:
			self->psnr.start_index = atoi(optarg);
			break;

		case ARGS_ID_PSNR_CSV:
			self->psnr.csv = optarg;
			break;

		default:
			usage(argv[0]);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	/* Check the parameters */
	if (self->input_file == NULL) {
		ULOGE("invalid input file");
		usage(argv[0]);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Setup signal handlers */
	signal(SIGINT, &sig_handler);
	signal(SIGTERM, &sig_handler);
#ifndef _WIN32
	signal(SIGPIPE, SIG_IGN);
#endif

	/* Loop */
	self->loop = pomp_loop_new();
	if (self->loop == NULL) {
		res = -ENOMEM;
		ULOG_ERRNO("pomp_loop_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Map the input file */
	res = map_file(self);
	if (res < 0) {
		status = EXIT_FAILURE;
		goto out;
	}

	/* Create reader */
	switch (self->config.encoding) {
	case VDEF_ENCODING_H264:
		res = h264_reader_new(&h264_cbs, self, &self->reader.h264);
		if (res < 0) {
			ULOG_ERRNO("h264_reader_new", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		break;

	case VDEF_ENCODING_H265:
		res = h265_reader_new(&h265_cbs, self, &self->reader.h265);
		if (res < 0) {
			ULOG_ERRNO("h265_reader_new", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		break;
	case VDEF_ENCODING_MJPEG:
		/* TODO: make a version that work with JPEG image that does
		 * not contains metadata (APP1) */
		res = pmeta_data_parse(self->input_file, &self->metadata);
		if (res < 0 && res == -ENOSYS) {
			ULOG_ERRNO(
				"unable to parse JPEG/JIFF without EXIF and "
				"XMP metadata",
				-res);
			status = EXIT_FAILURE;
			goto out;
		}

		if (res < 0) {
			ULOG_ERRNO("cannot parse metadata", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		break;
	default:
		break;
	}

	if (self->config.encoding != VDEF_ENCODING_MJPEG) {
		byte_stream.encoding = avcc.encoding = self->config.encoding;
		self->config.implem =
			vdec_get_auto_implem_by_coded_format(&byte_stream);
		self->in_info.format = byte_stream;
		if (self->config.implem == VDEC_DECODER_IMPLEM_AUTO) {
			self->config.implem =
				vdec_get_auto_implem_by_coded_format(&avcc);
			self->in_info.format = avcc;
		}

		if (self->config.implem == VDEC_DECODER_IMPLEM_AUTO) {
			ULOGE("unsupported video encoding");
			status = EXIT_FAILURE;
			goto out;
		}

		self->config.gen_grey_idr = 1;
	} else {
		self->in_info.format = vdef_jpeg_jfif;
		self->config.implem = vdec_get_auto_implem_by_coded_format(
			&self->in_info.format);
		if (self->config.implem == VDEC_DECODER_IMPLEM_AUTO) {
			ULOGE("unsupported photo encoding");
			status = EXIT_FAILURE;
			goto out;
		}
	}

	/* Create the decoder */
	res = vdec_new(
		self->loop, &self->config, &vdec_cbs, self, &self->decoder);
	if (res < 0) {
		ULOG_ERRNO("vdec_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Start */
	res = pomp_loop_idle_add(self->loop,
				 (self->config.encoding == VDEF_ENCODING_MJPEG)
					 ? jpeg_parse_idle
					 : au_parse_idle,
				 self);
	if (res < 0) {
		ULOG_ERRNO("pomp_loop_idle_add", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &start_time);

	/* Main loop */
	while (!self->stopped)
		pomp_loop_wait_and_process(self->loop, -1);

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &end_time);
	printf("\nTotal frames: input=%u output=%u\n",
	       self->input_count,
	       self->output_count);
	printf("Overall time: %.2fs\n",
	       (float)(end_time - start_time) / 1000000.);
	if ((self->framerate.num != 0) && (self->framerate.den != 0) &&
	    (self->total_bytes > 0) && (self->output_count > 0) &&
	    (self->input_count == self->output_count)) {
		double bitrate_scaled = bitrate =
			(double)self->total_bytes * 8 / self->output_count *
			self->framerate.num / self->framerate.den;
		char *bitrate_str = "";
		if (bitrate_scaled > 1000.) {
			bitrate_scaled /= 1000.;
			bitrate_str = "K";
		}
		if (bitrate_scaled > 1000.) {
			bitrate_scaled /= 1000.;
			bitrate_str = "M";
		}
		printf("Framerate: %u/%u, bitrate: %.1f%sbit/s\n",
		       self->framerate.num,
		       self->framerate.den,
		       bitrate_scaled,
		       bitrate_str);
	}

	if (self->psnr.reader != NULL && self->output_count != 0) {
		if (self->psnr.csv_file != NULL) {
			fprintf(self->psnr.csv_file,
				"#overall %.1f %.3f %.3f %.3f\n",
				bitrate,
				self->psnr.psnr_sum[0] / self->output_count,
				self->psnr.psnr_sum[1] / self->output_count,
				self->psnr.psnr_sum[2] / self->output_count);
		}

		printf("PSNR: Y=%.3fdB U=%.3fdB V=%.3fdB\n",
		       self->psnr.psnr_sum[0] / self->output_count,
		       self->psnr.psnr_sum[1] / self->output_count,
		       self->psnr.psnr_sum[2] / self->output_count);
	}

out:
	/* Cleanup and exit */
	if (self != NULL) {
		if (self->loop) {
			res = pomp_loop_idle_remove(
				self->loop, &finish_idle, self);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_idle_remove", -res);
			res = pomp_loop_idle_remove(
				self->loop, &au_parse_idle, self);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_idle_remove", -res);
		}
		unmap_file(self);
		res = vraw_writer_destroy(self->writer);
		if (res < 0)
			ULOG_ERRNO("vraw_writer_destroy", -res);
		if (self->in_frame != NULL) {
			res = mbuf_coded_video_frame_unref(self->in_frame);
			if (res < 0)
				ULOG_ERRNO("mbuf_coded_video_frame_unref:input",
					   -res);
		}
		if (self->in_mem != NULL) {
			res = mbuf_mem_unref(self->in_mem);
			if (res < 0)
				ULOG_ERRNO("mbuf_mem_unref:input", -res);
		}

		switch (self->config.encoding) {
		case VDEF_ENCODING_H264:
			if (self->reader.h264 != NULL) {
				res = h264_reader_destroy(self->reader.h264);
				if (res < 0)
					ULOG_ERRNO("h264_reader_destroy", -res);
			}
			break;
		case VDEF_ENCODING_H265:
			if (self->reader.h265 != NULL) {
				res = h265_reader_destroy(self->reader.h265);
				if (res < 0)
					ULOG_ERRNO("h265_reader_destroy", -res);
			}
			break;
		default:
			break;
		}

		if (self->decoder != NULL) {
			res = vdec_destroy(self->decoder);
			if (res < 0)
				ULOG_ERRNO("vdec_destroy", -res);
		}
		if (self->in_pool_allocated) {
			res = mbuf_pool_destroy(self->in_pool);
			if (res < 0)
				ULOG_ERRNO("mbuf_pool_destroy:input", -res);
		}
		if (self->loop) {
			res = pomp_loop_destroy(self->loop);
			if (res < 0)
				ULOG_ERRNO("pomp_loop_destroy", -res);
		}
		free(self->pending_nalu);
		if (self->psnr.csv_file != NULL)
			fclose(self->psnr.csv_file);
		if (self->psnr.reader) {
			status = vraw_reader_destroy(self->psnr.reader);
			if (status < 0)
				ULOG_ERRNO("vraw_reader_destroy", -status);
		}
		free(self->psnr.src_data);
		free(self->vps);
		free(self->sps);
		free(self->pps);
		free(self);
	}

	printf("\n%s\n", (status == EXIT_SUCCESS) ? "Finished!" : "Failed!");
	exit(status);
}

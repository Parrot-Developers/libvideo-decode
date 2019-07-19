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
#include <video-buffers/vbuf.h>
#include <video-buffers/vbuf_generic.h>
#include <video-decode/vdec.h>
#define ULOG_TAG vdec_prog
#include <ulog.h>
ULOG_DECLARE_TAG(vdec_prog);

#ifdef RASPI
#	ifdef TOSTRING
#		undef TOSTRING /* already defined in futils */
#	endif /* TOSTRING */
#	include <bcm_host.h>
#endif /* RASPI */


#define DEFAULT_IN_BUF_COUNT 10
#define DEFAULT_IN_BUF_CAPACITY (3840 * 2160 * 3 / 4)
#define DEFAULT_TS_INC 33333


struct vdec_prog {
	char *input_file;
#ifdef _WIN32
	HANDLE in_file;
	HANDLE in_file_map;
#else
	int in_fd;
#endif
	void *in_data;
	size_t in_size;
	struct h264_reader *reader;
	struct vdec_decoder *decoder;
	struct vdec_config config;
	int configured;
	int first_in_frame;
	int first_out_frame;
	int flushed;
	int stopped;
	unsigned int input_count;
	unsigned int max_count;
	uint8_t *sps;
	size_t sps_size;
	uint8_t *pps;
	size_t pps_size;
	uint64_t ts_inc;
	int sps_frame_mbs_only_flag;
	uint32_t sps_pic_order_cnt_type;
	struct h264_nalu_header prev_slice_nalu_header;
	struct h264_slice_header prev_slice_header;
	int first_vcl_nalu;
	int prev_vcl_nalu;
	struct vdec_input_metadata in_meta;
	struct vbuf_pool *in_pool;
	int in_pool_allocated;
	struct vbuf_queue *in_queue;
	struct vbuf_buffer *in_buf;
	char *output_file;
	FILE *out_file;
	int output_file_y4m;
	pthread_mutex_t mutex;
	pthread_cond_t cond;
};


static const char short_options[] = "hi:o:n:slj:";


static const struct option long_options[] = {
	{"help", no_argument, NULL, 'h'},
	{"infile", required_argument, NULL, 'i'},
	{"outfile", required_argument, NULL, 'o'},
	{"count", required_argument, NULL, 'n'},
	{"sync", no_argument, NULL, 's'},
	{"low-delay", no_argument, NULL, 'l'},
	{"threads", required_argument, NULL, 'j'},
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
	       "  -n | --count <n>                   "
	       "Decode at most n frames\n"
	       "  -s | --sync                        "
	       "Use the synchronized decoding mode\n"
	       "  -l | --low-delay                   "
	       "Favor low delay decoding\n"
	       "  -j | --threads <thread_count>      "
	       "Preferred decoding thread count (0 = default,\n"
	       "                                     "
	       "1 = no multi-threading, >1 = multi-threading)\n"
	       "\n",
	       prog_name);
}


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
			munmap(self->in_data, self->in_size);
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

	res = GetFileSizeEx(self->in_file, &filesize);
	if (res == 0) {
		res = -EIO;
		ULOG_ERRNO("GetFileSizeEx('%s')", -res, self->input_file);
		goto error;
	}
	self->in_size = filesize.QuadPart;

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
	self->in_size = lseek(self->in_fd, 0, SEEK_END);
	if (self->in_size == (size_t)-1) {
		res = -errno;
		ULOG_ERRNO("lseek", -res);
		goto error;
	}

	self->in_data = mmap(
		NULL, self->in_size, PROT_READ, MAP_PRIVATE, self->in_fd, 0);
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
	struct vbuf_cbs buf_cbs;

	res = vdec_set_sps_pps(self->decoder,
			       self->sps,
			       self->sps_size,
			       self->pps,
			       self->pps_size,
			       VDEC_INPUT_FORMAT_RAW_NALU);
	if (res < 0) {
		ULOG_ERRNO("vdec_set_sps_pps", -res);
		return res;
	}

	/* Input buffer pool */
	self->in_pool = vdec_get_input_buffer_pool(self->decoder);
	if (self->in_pool == NULL) {
		res = vbuf_generic_get_cbs(&buf_cbs);
		if (res < 0) {
			ULOG_ERRNO("vbuf_generic_get_cbs", -res);
			return res;
		}
		buf_count = (self->config.preferred_thread_count + 1 >
			     DEFAULT_IN_BUF_COUNT)
				    ? self->config.preferred_thread_count + 1
				    : DEFAULT_IN_BUF_COUNT;
		res = vbuf_pool_new(buf_count,
				    DEFAULT_IN_BUF_CAPACITY,
				    0,
				    &buf_cbs,
				    &self->in_pool);
		if (res < 0) {
			ULOG_ERRNO("vbuf_pool_new:input", -res);
			return res;
		}
		self->in_pool_allocated = 1;
	}

	/* Input buffer queue (async decoding only) */
	if (!self->config.sync_decoding) {
		self->in_queue = vdec_get_input_buffer_queue(self->decoder);
		if (self->in_queue == NULL) {
			ULOG_ERRNO("vdec_get_input_buffer_queue", EPROTO);
			return res;
		}
	}

	self->configured = 1;
	return 0;
}


static int yuv_output(struct vdec_prog *self,
		      struct vbuf_buffer *out_buf,
		      struct vdec_output_metadata *out_meta)
{
	int res = 0, chroma_idx;
	size_t stride, res1;
	const uint8_t *data, *ptr;
	unsigned int i, j;

	if (self->out_file == NULL)
		return 0;

	if ((out_buf == NULL) || (out_meta == NULL))
		return -EINVAL;

	data = vbuf_get_cdata(out_buf);
	if (data == NULL) {
		res = -EPROTO;
		ULOG_ERRNO("vbuf_get_cdata:output", -res);
		return res;
	}

	/* Write YUV4MPEG2 file and frame headers */
	if (self->output_file_y4m) {
		if (self->first_out_frame) {
			fprintf(self->out_file,
				"YUV4MPEG2 W%d H%d F30:1 Ip A1:1\n",
				out_meta->crop_width,
				out_meta->crop_height);
		}
		fprintf(self->out_file, "FRAME\n");
	}

	/* Write to output file in "I420" format */
	switch (out_meta->format) {
	case VDEC_OUTPUT_FORMAT_I420:
	case VDEC_OUTPUT_FORMAT_YV12:
		/* Y */
		ptr = data + out_meta->plane_offset[0];
		stride = out_meta->plane_stride[0];
		ptr += out_meta->crop_top * stride + out_meta->crop_left;
		for (i = 0; i < out_meta->crop_height; i++) {
			res1 = fwrite(
				ptr, out_meta->crop_width, 1, self->out_file);
			if (res1 != 1) {
				res = -EIO;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += stride;
		}

		/* U */
		chroma_idx =
			(out_meta->format == VDEC_OUTPUT_FORMAT_I420) ? 1 : 2;
		ptr = data + out_meta->plane_offset[chroma_idx];
		stride = out_meta->plane_stride[chroma_idx];
		ptr += out_meta->crop_top / 2 * stride +
		       out_meta->crop_left / 2;
		for (i = 0; i < out_meta->crop_height / 2; i++) {
			res1 = fwrite(ptr,
				      out_meta->crop_width / 2,
				      1,
				      self->out_file);
			if (res1 != 1) {
				res = -EIO;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += stride;
		}

		/* V */
		chroma_idx =
			(out_meta->format == VDEC_OUTPUT_FORMAT_I420) ? 2 : 1;
		ptr = data + out_meta->plane_offset[chroma_idx];
		stride = out_meta->plane_stride[chroma_idx];
		ptr += out_meta->crop_top / 2 * stride +
		       out_meta->crop_left / 2;
		for (i = 0; i < out_meta->crop_height / 2; i++) {
			res1 = fwrite(ptr,
				      out_meta->crop_width / 2,
				      1,
				      self->out_file);
			if (res1 != 1) {
				res = -EIO;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += stride;
		}
		break;
	case VDEC_OUTPUT_FORMAT_NV12:
	case VDEC_OUTPUT_FORMAT_NV21:
		/* Y */
		ptr = data + out_meta->plane_offset[0];
		stride = out_meta->plane_stride[0];
		ptr += out_meta->crop_top * stride + out_meta->crop_left;
		for (i = 0; i < out_meta->crop_height; i++) {
			res1 = fwrite(
				ptr, out_meta->crop_width, 1, self->out_file);
			if (res1 != 1) {
				res = -EIO;
				ULOG_ERRNO("fwrite", -res);
				return res;
			}
			ptr += stride;
		}

		/* U */
		chroma_idx =
			(out_meta->format == VDEC_OUTPUT_FORMAT_NV12) ? 0 : 1;
		ptr = data + out_meta->plane_offset[1];
		stride = out_meta->plane_stride[1];
		ptr += out_meta->crop_top / 2 * stride + out_meta->crop_left;
		for (i = 0; i < out_meta->crop_height / 2; i++) {
			for (j = 0; j < out_meta->crop_width / 2; j++) {
				res1 = fwrite(&ptr[j * 2 + chroma_idx],
					      1,
					      1,
					      self->out_file);
				if (res1 != 1) {
					res = -EIO;
					ULOG_ERRNO("fwrite", -res);
					return res;
				}
			}
			ptr += stride;
		}

		/* V */
		chroma_idx =
			(out_meta->format == VDEC_OUTPUT_FORMAT_NV12) ? 1 : 0;
		ptr = data + out_meta->plane_offset[1];
		stride = out_meta->plane_stride[1];
		ptr += out_meta->crop_top / 2 * stride + out_meta->crop_left;
		for (i = 0; i < out_meta->crop_height / 2; i++) {
			for (j = 0; j < out_meta->crop_width / 2; j++) {
				res1 = fwrite(&ptr[j * 2 + chroma_idx],
					      1,
					      1,
					      self->out_file);
				if (res1 != 1) {
					res = -EIO;
					ULOG_ERRNO("fwrite", -res);
					return res;
				}
			}
			ptr += stride;
		}
		break;
	default:
		ULOGE("unsupported YUV format");
		res = -ENOSYS;
		break;
	}

	return res;
}


static int frame_output(struct vdec_prog *self, struct vbuf_buffer *out_buf)
{
	int res = 0;
	struct vdec_output_metadata *out_meta;
	size_t len = 0;

	res = vbuf_metadata_get(
		out_buf, self->decoder, NULL, &len, (uint8_t **)&out_meta);
	if ((res < 0) || (out_meta == NULL) || (len != sizeof(*out_meta))) {
		ULOG_ERRNO("vbuf_metadata_get", -res);
		return res;
	}

	res = yuv_output(self, out_buf, out_meta);
	if (res < 0)
		ULOG_ERRNO("yuv_output", -res);

	ULOGI("decoded frame #%d (dequeue: %.2fms, decode: %.2fms, "
	      "overall: %.2fms)",
	      out_meta->index,
	      (float)(out_meta->dequeue_time - out_meta->input_time) / 1000.,
	      (float)(out_meta->output_time - out_meta->dequeue_time) / 1000.,
	      (float)(out_meta->output_time - out_meta->input_time) / 1000.);

	self->first_out_frame = 0;

	return 0;
}


static void frame_output_cb(struct vdec_decoder *dec,
			    int status,
			    struct vbuf_buffer *out_buf,
			    void *userdata)
{
	int res;
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(out_buf == NULL, EINVAL);

	if (status != 0) {
		ULOGE("decoder error, resync required");
		return;
	}

	res = frame_output(self, out_buf);
	if (res < 0)
		ULOG_ERRNO("frame_output", -res);
}


static int au_decode(struct vdec_prog *self)
{
	int res = 0, err;
	uint8_t *in_meta;
	struct timespec cur_ts = {0, 0};
	struct vbuf_buffer *out_buf = NULL;

	if (self->in_buf == NULL)
		return 0;

	if ((self->max_count > 0) && (self->input_count >= self->max_count))
		goto cleanup;

	if (self->configured) {
		time_get_monotonic(&cur_ts);
		time_timespec_to_us(&cur_ts, &self->in_meta.input_time);

		res = vbuf_metadata_add(self->in_buf,
					self->decoder,
					1,
					sizeof(self->in_meta),
					(uint8_t **)&in_meta);
		if (res < 0) {
			ULOG_ERRNO("vbuf_metadata_add:input", -res);
			goto cleanup;
		}
		memcpy(in_meta, &self->in_meta, sizeof(self->in_meta));

		if (self->config.sync_decoding) {
			res = vdec_sync_decode(
				self->decoder, self->in_buf, &out_buf);
			if (res < 0) {
				ULOG_ERRNO("vdec_sync_decode", -res);
				goto cleanup;
			}

			res = frame_output(self, out_buf);
			if (res < 0)
				ULOG_ERRNO("frame_output", -res);

			res = vbuf_unref(out_buf);
			if (res < 0) {
				ULOG_ERRNO("vbuf_unref:output", -res);
				goto cleanup;
			}
		} else {
			res = vbuf_queue_push(self->in_queue, self->in_buf);
			if (res < 0) {
				ULOG_ERRNO("vbuf_queue_push:input", -res);
				goto cleanup;
			}
		}
		self->input_count++;
	}

cleanup:
	err = vbuf_unref(self->in_buf);
	if (err < 0)
		ULOG_ERRNO("vbuf_unref:input", -err);
	self->in_buf = NULL;
	self->in_meta.index++;
	self->in_meta.timestamp += self->ts_inc;
	return res;
}


static int append_to_buffer(struct vbuf_buffer *buf,
			    const uint8_t *data,
			    size_t len,
			    enum vdec_input_format format)
{
	int res;
	ssize_t res1;
	size_t au_offset, capacity;
	size_t nalu_offset = (format == VDEC_INPUT_FORMAT_RAW_NALU) ? 0 : 4;
	uint8_t *au_data, *nalu_data;
	uint32_t start;

	ULOG_ERRNO_RETURN_ERR_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(data == NULL, EINVAL);
	ULOG_ERRNO_RETURN_ERR_IF(len == 0, EINVAL);

	res1 = vbuf_get_size(buf);
	if (res1 < 0) {
		ULOG_ERRNO("vbuf_get_size", (int)-res1);
		return (int)res1;
	}
	au_offset = res1;
	res1 = vbuf_get_capacity(buf);
	if (res1 < 0) {
		ULOG_ERRNO("vbuf_get_capacity", (int)-res1);
		return (int)res1;
	}
	capacity = res1;
	if (capacity < au_offset + nalu_offset + len) {
		res1 = vbuf_set_capacity(buf, au_offset + nalu_offset + len);
		if (res1 < 0) {
			ULOG_ERRNO("vbuf_set_capacity", (int)-res1);
			return (int)res1;
		}
	}
	au_data = vbuf_get_data(buf);
	if (au_data == NULL) {
		ULOG_ERRNO("vbuf_get_data", EPROTO);
		return -EPROTO;
	}
	nalu_data = au_data + au_offset;
	if (format == VDEC_INPUT_FORMAT_BYTE_STREAM) {
		start = htonl(0x00000001);
		memcpy(nalu_data, &start, sizeof(uint32_t));
	} else if (format == VDEC_INPUT_FORMAT_AVCC) {
		start = htonl(len);
		memcpy(nalu_data, &start, sizeof(uint32_t));
	}
	memcpy(nalu_data + nalu_offset, data, len);
	res = vbuf_set_size(buf, au_offset + nalu_offset + len);
	if (res < 0) {
		ULOG_ERRNO("vbuf_set_size", -res);
		return res;
	}

	return 0;
}


static void nalu_end_cb(struct h264_ctx *ctx,
			enum h264_nalu_type type,
			const uint8_t *buf,
			size_t len,
			void *userdata)
{
	int res;
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);

	/* Access unit change detection
	 * see rec. ITU-T H.264 chap. 7.4.1.2.4 */
	if ((self->in_buf != NULL) &&
	    (self->prev_slice_header.slice_type !=
	     (unsigned)H264_SLICE_TYPE_UNKNOWN) &&
	    ((type == H264_NALU_TYPE_AUD) || (type == H264_NALU_TYPE_SPS) ||
	     (type == H264_NALU_TYPE_PPS) || (type == H264_NALU_TYPE_SEI) ||
	     (((unsigned)type >= 14) && ((unsigned)type <= 18)) ||
	     (self->first_vcl_nalu && self->prev_vcl_nalu))) {
		self->prev_vcl_nalu = 0;
		res = au_decode(self);
		if (res < 0)
			ULOG_ERRNO("au_decode", -res);
	}

	if ((type != H264_NALU_TYPE_SLICE) &&
	    (type != H264_NALU_TYPE_SLICE_IDR)) {
		self->prev_slice_header.slice_type =
			(unsigned)H264_SLICE_TYPE_UNKNOWN;
	} else {
		self->prev_vcl_nalu = 1;
	}

	/* Save the SPS and PPS */
	if ((type == H264_NALU_TYPE_SPS) && (self->sps == NULL)) {
		self->sps_size = len;
		self->sps = malloc(self->sps_size);
		if (self->sps == NULL) {
			ULOG_ERRNO("malloc", ENOMEM);
			return;
		}
		memcpy(self->sps, buf, len);
		ULOGI("SPS found");
	} else if ((type == H264_NALU_TYPE_PPS) && (self->pps == NULL)) {
		self->pps_size = len;
		self->pps = malloc(self->pps_size);
		if (self->pps == NULL) {
			ULOG_ERRNO("malloc", ENOMEM);
			return;
		}
		memcpy(self->pps, buf, len);
		ULOGI("PPS found");
	}

	/* Configure the decoder */
	if ((!self->configured) && (self->sps != NULL) && (self->pps != NULL)) {
		res = configure(self);
		if (res < 0) {
			ULOG_ERRNO("configure", -res);
			return;
		}
	}

	/* Get an input buffer (blocking) */
	if ((self->in_pool != NULL) && (self->in_buf == NULL)) {
		res = vbuf_pool_get(self->in_pool, -1, &self->in_buf);
		if (res < 0) {
			ULOG_ERRNO("vbuf_pool_get:input", -res);
			return;
		}
	}
	if (self->in_buf == NULL)
		return;

	/* Filter out SPS, PPS and AUD */
	if ((type == H264_NALU_TYPE_AUD) || (type == H264_NALU_TYPE_SPS) ||
	    (type == H264_NALU_TYPE_PPS))
		return;

	/* Add SPS/PPS for the first frame */
	if (self->first_in_frame) {
		res = append_to_buffer(self->in_buf,
				       self->sps,
				       self->sps_size,
				       self->in_meta.format);
		if (res < 0) {
			ULOG_ERRNO("append_to_buffer", -res);
			return;
		}
		res = append_to_buffer(self->in_buf,
				       self->pps,
				       self->pps_size,
				       self->in_meta.format);
		if (res < 0) {
			ULOG_ERRNO("append_to_buffer", -res);
			return;
		}
		self->first_in_frame = 0;
	}

	/* Add the NALU to the input buffer */
	res = append_to_buffer(self->in_buf, buf, len, self->in_meta.format);
	if (res < 0) {
		ULOG_ERRNO("append_to_buffer", -res);
		return;
	}
}


static void slice_cb(struct h264_ctx *ctx,
		     const uint8_t *buf,
		     size_t len,
		     const struct h264_slice_header *sh,
		     void *userdata)
{
	int res;
	struct vdec_prog *self = userdata;
	struct h264_nalu_header nh;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);
	ULOG_ERRNO_RETURN_IF(sh == NULL, EINVAL);

	self->first_vcl_nalu = 0;

	memset(&nh, 0, sizeof(nh));
	res = h264_parse_nalu_header(buf, len, &nh);
	if (res < 0) {
		ULOG_ERRNO("h264_parse_nalu_header", -res);
		goto out;
	}

	if (self->prev_slice_header.slice_type ==
	    (unsigned)H264_SLICE_TYPE_UNKNOWN) {
		self->first_vcl_nalu = 1;
		goto out;
	}

	/* Detection of the first VCL NAL unit of a primary coded picture
	 * see rec. ITU-T H.264 chap. 7.4.1.2.4 */
	if ((sh->pic_parameter_set_id !=
	     self->prev_slice_header.pic_parameter_set_id) ||
	    (sh->field_pic_flag != self->prev_slice_header.field_pic_flag) ||
	    (nh.nal_ref_idc != self->prev_slice_nalu_header.nal_ref_idc)) {
		self->first_vcl_nalu = 1;
		goto out;
	}
	if (((nh.nal_unit_type == H264_NALU_TYPE_SLICE_IDR) ||
	     (self->prev_slice_nalu_header.nal_unit_type ==
	      H264_NALU_TYPE_SLICE_IDR)) &&
	    ((nh.nal_unit_type != self->prev_slice_nalu_header.nal_unit_type) ||
	     (sh->idr_pic_id != self->prev_slice_header.idr_pic_id))) {
		self->first_vcl_nalu = 1;
		goto out;
	}
	if ((self->sps_pic_order_cnt_type == 0) &&
	    ((sh->pic_order_cnt_lsb !=
	      self->prev_slice_header.pic_order_cnt_lsb) ||
	     (sh->delta_pic_order_cnt_bottom !=
	      self->prev_slice_header.delta_pic_order_cnt_bottom))) {
		self->first_vcl_nalu = 1;
		goto out;
	}
	if ((self->sps_pic_order_cnt_type == 1) &&
	    ((sh->delta_pic_order_cnt[0] !=
	      self->prev_slice_header.delta_pic_order_cnt[0]) ||
	     (sh->delta_pic_order_cnt[1] !=
	      self->prev_slice_header.delta_pic_order_cnt[1]))) {
		self->first_vcl_nalu = 1;
		goto out;
	}
	if ((!self->sps_frame_mbs_only_flag) && (sh->field_pic_flag) &&
	    (self->prev_slice_header.field_pic_flag) &&
	    (sh->bottom_field_flag !=
	     self->prev_slice_header.bottom_field_flag)) {
		self->first_vcl_nalu = 1;
		goto out;
	}

out:
	self->prev_slice_nalu_header = nh;
	self->prev_slice_header = *sh;
}


static void sps_cb(struct h264_ctx *ctx,
		   const uint8_t *buf,
		   size_t len,
		   const struct h264_sps *sps,
		   void *userdata)
{
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(ctx == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(buf == NULL, EINVAL);
	ULOG_ERRNO_RETURN_IF(len == 0, EINVAL);
	ULOG_ERRNO_RETURN_IF(sps == NULL, EINVAL);

	self->sps_frame_mbs_only_flag = sps->frame_mbs_only_flag;
	self->sps_pic_order_cnt_type = sps->pic_order_cnt_type;
}


static void flush_cb(struct vdec_decoder *dec, void *userdata)
{
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("decoder is flushed");
	self->flushed = 1;
	pthread_cond_signal(&self->cond);
}


static void stop_cb(struct vdec_decoder *dec, void *userdata)
{
	struct vdec_prog *self = userdata;

	ULOG_ERRNO_RETURN_IF(self == NULL, EINVAL);

	ULOGI("decoder is stopped");
	self->stopped = 1;
	pthread_cond_signal(&self->cond);
}


static const struct h264_ctx_cbs h264_cbs = {
	.nalu_end = &nalu_end_cb,
	.slice = &slice_cb,
	.sps = &sps_cb,
};


static const struct vdec_cbs vdec_cbs = {
	.frame_output = &frame_output_cb,
	.flush = &flush_cb,
	.stop = &stop_cb,
};


int main(int argc, char **argv)
{
	int res = 0, status = EXIT_SUCCESS;
	int idx, c, mutex_init = 0, cond_init = 0;
	uint32_t supported_input_format;
	struct vdec_prog *self = NULL;
	struct timespec cur_ts = {0, 0};
	uint64_t start_time = 0, end_time = 0;
	size_t off = 0;

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
#ifdef _WIN32
	self->in_file = INVALID_HANDLE_VALUE;
	self->in_file_map = INVALID_HANDLE_VALUE;
#else
	self->in_fd = -1;
#endif
	self->first_in_frame = 1;
	self->first_out_frame = 1;
	self->ts_inc = DEFAULT_TS_INC;
	self->in_meta.complete = 1;
	self->in_meta.ref = 1;
	self->prev_slice_header.slice_type = (unsigned)H264_SLICE_TYPE_UNKNOWN;
	res = pthread_mutex_init(&self->mutex, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_mutex_init", res);
		status = EXIT_FAILURE;
		goto out;
	}
	mutex_init = 1;
	res = pthread_cond_init(&self->cond, NULL);
	if (res != 0) {
		ULOG_ERRNO("pthread_cond_init", res);
		status = EXIT_FAILURE;
		goto out;
	}
	cond_init = 1;

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
			break;

		case 'o':
			self->output_file = optarg;
			if ((strlen(self->output_file) > 4) &&
			    (!strncasecmp(self->output_file +
						  strlen(self->output_file) - 4,
					  ".y4m",
					  4)))
				self->output_file_y4m = 1;
			break;

		case 'n':
			self->max_count = atoi(optarg);
			break;

		case 's':
			self->config.sync_decoding = 1;
			break;

		case 'l':
			self->config.low_delay = 1;
			break;

		case 'j':
			self->config.preferred_thread_count = atoi(optarg);
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

	/* Map the input file */
	res = map_file(self);
	if (res < 0) {
		status = EXIT_FAILURE;
		goto out;
	}

	/* Output file */
	if (self->output_file) {
		self->out_file = fopen(self->output_file, "wb");
		if (self->out_file == NULL) {
			ULOG_ERRNO("fopen", errno);
			status = EXIT_FAILURE;
			goto out;
		}
	}

	/* Create the H.264 reader */
	res = h264_reader_new(&h264_cbs, self, &self->reader);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	supported_input_format =
		vdec_get_supported_input_format(VDEC_DECODER_IMPLEM_AUTO);
	if (supported_input_format & VDEC_INPUT_FORMAT_BYTE_STREAM) {
		self->in_meta.format = VDEC_INPUT_FORMAT_BYTE_STREAM;
	} else if (supported_input_format & VDEC_INPUT_FORMAT_AVCC) {
		self->in_meta.format = VDEC_INPUT_FORMAT_AVCC;
	} else {
		ULOGE("unsuppoted input format");
		status = EXIT_FAILURE;
		goto out;
	}

	/* Create the decoder */
	res = vdec_new(&self->config, &vdec_cbs, self, &self->decoder);
	if (res < 0) {
		ULOG_ERRNO("vdec_new", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &start_time);

	/* Parse the input file */
	res = h264_reader_parse(
		self->reader, 0, self->in_data, self->in_size, &off);
	if (res < 0) {
		ULOG_ERRNO("h264_reader_parse", -res);
		status = EXIT_FAILURE;
		goto out;
	}

	/* Decode the last frame */
	if (self->in_buf != NULL) {
		res = au_decode(self);
		if (res < 0)
			ULOG_ERRNO("au_decode", -res);
	}

	/* Flush and stop the decoder (async decoding only) */
	if (!self->config.sync_decoding) {
		pthread_mutex_lock(&self->mutex);
		res = vdec_flush(self->decoder, 0);
		if (res < 0) {
			ULOG_ERRNO("vdec_flush", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		while (!self->flushed)
			pthread_cond_wait(&self->cond, &self->mutex);
		res = vdec_stop(self->decoder);
		if (res < 0) {
			ULOG_ERRNO("vdec_stop", -res);
			status = EXIT_FAILURE;
			goto out;
		}
		while (!self->stopped)
			pthread_cond_wait(&self->cond, &self->mutex);
		pthread_mutex_unlock(&self->mutex);
	}

	time_get_monotonic(&cur_ts);
	time_timespec_to_us(&cur_ts, &end_time);
	printf("\nOverall time: %.2fs\n",
	       (float)(end_time - start_time) / 1000000.);

out:
	/* Cleanup and exit */
	if (self != NULL) {
		unmap_file(self);
		if (self->out_file != NULL)
			fclose(self->out_file);
		if (self->in_buf != NULL) {
			res = vbuf_unref(self->in_buf);
			if (res < 0)
				ULOG_ERRNO("vbuf_unref:input", -res);
		}
		if (self->reader != NULL) {
			res = h264_reader_destroy(self->reader);
			if (res < 0)
				ULOG_ERRNO("h264_reader_destroy", -res);
		}
		if (self->decoder != NULL) {
			res = vdec_destroy(self->decoder);
			if (res < 0)
				ULOG_ERRNO("vdec_destroy", -res);
		}
		if (self->in_pool_allocated) {
			res = vbuf_pool_destroy(self->in_pool);
			if (res < 0)
				ULOG_ERRNO("vbuf_pool_destroy:input", -res);
		}
		if (cond_init) {
			res = pthread_cond_destroy(&self->cond);
			if (res != 0)
				ULOG_ERRNO("pthread_cond_destroy", res);
		}
		if (mutex_init) {
			res = pthread_mutex_destroy(&self->mutex);
			if (res != 0)
				ULOG_ERRNO("pthread_mutex_destroy", res);
		}
		free(self->sps);
		free(self->pps);
		free(self);
	}

	printf("\n%s\n", (status == EXIT_SUCCESS) ? "Finished!" : "Failed!");
	exit(status);
}

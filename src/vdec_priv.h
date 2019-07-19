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

#ifndef _VDEC_PRIV_H_
#define _VDEC_PRIV_H_

#define _GNU_SOURCE
#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <ulog.h>

#ifdef _WIN32
#	include <winsock2.h>
#else /* !_WIN32 */
#	include <arpa/inet.h>
#endif /* !_WIN32 */

#include <futils/futils.h>
#include <h264/h264.h>
#include <video-buffers/vbuf.h>
#include <video-decode/vdec.h>

#include "vdec_ffmpeg.h"
#include "vdec_mediacodec.h"
#include "vdec_videocoremmal.h"
#include "vdec_videotoolbox.h"


struct vdec_derived;


struct vdec_ops {
	uint32_t (*get_supported_input_format)(void);

	int (*new)(struct vdec_decoder *base);

	int (*flush)(struct vdec_decoder *base, int discard);

	int (*stop)(struct vdec_decoder *base);

	int (*destroy)(struct vdec_decoder *base);

	int (*set_sps_pps)(struct vdec_decoder *base,
			   const uint8_t *sps,
			   size_t sps_size,
			   const uint8_t *pps,
			   size_t pps_size,
			   enum vdec_input_format format);

	struct vbuf_pool *(*get_input_buffer_pool)(struct vdec_decoder *base);

	struct vbuf_queue *(*get_input_buffer_queue)(struct vdec_decoder *base);

	int (*sync_decode)(struct vdec_decoder *base,
			   struct vbuf_buffer *in_buf,
			   struct vbuf_buffer **out_buf);
};


struct vdec_decoder {
	struct vdec_derived *derived;
	struct vdec_ops ops;
	struct vdec_cbs cbs;
	void *userdata;
	struct vdec_config config;
	int configured;
	unsigned int width;
	unsigned int height;
	unsigned int sar_width;
	unsigned int sar_height;
	unsigned int crop_left;
	unsigned int crop_top;
	unsigned int crop_width;
	unsigned int crop_height;
	int full_range;
	unsigned int num_units_in_tick;
	unsigned int time_scale;
	unsigned int nal_hrd_bitrate;
	unsigned int nal_hrd_cpb_size;
	unsigned int vcl_hrd_bitrate;
	unsigned int vcl_hrd_cpb_size;
	struct {
		char *dir;
		struct h264_reader *h264_reader;
		unsigned int h264_frame_index;
		FILE *input_bs;
		FILE *output_yuv;
		FILE *h264_analysis;
	} dbg;
};


int vdec_h264_get_video_info(struct vdec_decoder *self,
			     const uint8_t *sps,
			     size_t sps_size,
			     const uint8_t *pps,
			     size_t pps_size,
			     enum vdec_input_format format);


int vdec_h264_format_convert(struct vbuf_buffer *buf,
			     enum vdec_input_format current_format,
			     enum vdec_input_format target_format);


int vdec_dbg_create_files(struct vdec_decoder *vdec);


int vdec_dbg_close_files(struct vdec_decoder *vdec);


int vdec_dbg_write_h264_sps_pps(FILE *file,
				const uint8_t *sps,
				size_t sps_size,
				const uint8_t *pps,
				size_t pps_size,
				enum vdec_input_format format);


int vdec_dbg_write_h264_frame(FILE *file,
			      struct vbuf_buffer *buf,
			      enum vdec_input_format format);


int vdec_dbg_parse_h264_sps_pps(struct vdec_decoder *vdec,
				const uint8_t *sps,
				size_t sps_size,
				const uint8_t *pps,
				size_t pps_size,
				enum vdec_input_format format);


int vdec_dbg_parse_h264_frame(struct vdec_decoder *vdec,
			      struct vbuf_buffer *buf,
			      enum vdec_input_format format);


int vdec_dbg_write_yuv_frame(FILE *file,
			     struct vbuf_buffer *buf,
			     struct vdec_output_metadata *meta);


static inline void xfree(void **ptr)
{
	if (ptr) {
		free(*ptr);
		*ptr = NULL;
	}
}


static inline char *xstrdup(const char *s)
{
	return s == NULL ? NULL : strdup(s);
}


#endif /* !_VDEC_PRIV_H_ */

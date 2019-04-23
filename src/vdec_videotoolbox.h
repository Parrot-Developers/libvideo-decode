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

#ifndef _VDEC_VIDEOTOOLBOX_H_
#define _VDEC_VIDEOTOOLBOX_H_


struct vdec_videotoolbox;


#define VDEC_OPS_VIDEOTOOLBOX                                                  \
	{                                                                      \
		.get_supported_input_format =                                  \
			&vdec_videotoolbox_get_supported_input_format,         \
		.new = &vdec_videotoolbox_new,                                 \
		.flush = &vdec_videotoolbox_flush,                             \
		.stop = &vdec_videotoolbox_stop,                               \
		.destroy = &vdec_videotoolbox_destroy,                         \
		.set_sps_pps = &vdec_videotoolbox_set_sps_pps,                 \
		.get_input_buffer_pool =                                       \
			&vdec_videotoolbox_get_input_buffer_pool,              \
		.get_input_buffer_queue =                                      \
			&vdec_videotoolbox_get_input_buffer_queue,             \
		.sync_decode = &vdec_videotoolbox_sync_decode,                 \
	}


uint32_t vdec_videotoolbox_get_supported_input_format(void);


int vdec_videotoolbox_new(struct vdec_decoder *base);


int vdec_videotoolbox_flush(struct vdec_decoder *base, int discard);


int vdec_videotoolbox_stop(struct vdec_decoder *base);


int vdec_videotoolbox_destroy(struct vdec_decoder *base);


int vdec_videotoolbox_set_sps_pps(struct vdec_decoder *base,
				  const uint8_t *sps,
				  size_t sps_size,
				  const uint8_t *pps,
				  size_t pps_size,
				  enum vdec_input_format format);


struct vbuf_pool *
vdec_videotoolbox_get_input_buffer_pool(struct vdec_decoder *base);


struct vbuf_queue *
vdec_videotoolbox_get_input_buffer_queue(struct vdec_decoder *base);


int vdec_videotoolbox_sync_decode(struct vdec_decoder *base,
				  struct vbuf_buffer *in_buf,
				  struct vbuf_buffer **out_buf);


#endif /* !_VDEC_VIDEOTOOLBOX_H_ */

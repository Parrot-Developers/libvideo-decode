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
#include <h265/h265.h>
#include <video-decode/vdec.h>
#include <video-decode/vdec_internal.h>

#ifdef BUILD_LIBVIDEO_DECODE_FFMPEG
#	include <video-decode/vdec_ffmpeg.h>
#endif

#ifdef BUILD_LIBVIDEO_DECODE_MEDIACODEC
#	include <video-decode/vdec_mediacodec.h>
#endif

#ifdef BUILD_LIBVIDEO_DECODE_VIDEOCOREMMAL
#	include <video-decode/vdec_videocoremmal.h>
#endif

#ifdef BUILD_LIBVIDEO_DECODE_VIDEOTOOLBOX
#	include <video-decode/vdec_videotoolbox.h>
#endif

#ifdef BUILD_LIBVIDEO_DECODE_HISI
#	include <video-decode/vdec_hisi.h>
#endif

#ifdef BUILD_LIBVIDEO_DECODE_AML
#	include <video-decode/vdec_aml.h>
#endif

#ifdef BUILD_LIBVIDEO_DECODE_TURBOJPEG
#	include <video-decode/vdec_turbojpeg.h>
#endif


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

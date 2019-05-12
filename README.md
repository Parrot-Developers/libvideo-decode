# libvideo-decode - Video decoding library

_libvideo-decode_ is a C library to handle the decoding of H.264 video on
various platforms with a common API.

The library uses hardware-accelerated decoding when available.

## Implementations

The following implementations are available:

* FFmpeg (CPU or cuvid acceleration on Nvidia GPU if available)
* MediaCodec on Android 4.2+ (through the NDK API on Android 5.0+
or the Java API through JNI)
* VideoToolbox on iOS 8+ and MacOS X
* MMAL on Broadcom VideoCore IV (Raspberry Pi platforms)

The application can force using a specific implementation or let the library
decide according to what is supported by the platform.

## Dependencies

The library depends on the following Alchemy modules:

* libulog
* libfutils
* libvideo-buffers
* libh264
* (optional) ffmpeg-libav (for FFmpeg support)
* (optional) libvideo-buffers-generic (for FFmpeg support)
* (optional) libmediacodec-wrapper (for Android MediaCodec support)
* (optional) mmal (for Broadcom VideoCore IV)

The library also depends on the following frameworks for iOS and MacOS only:

* Foundation
* CoreMedia
* CoreVideo
* VideoToolbox

## Building

Building is activated by enabling _libvideo-decode_ in the Alchemy build
configuration.

## Operation

Operations are asynchronous: the application pushes buffers to decode in the
input queue and is notified of decoded frames through a callback function.

Some decoders need the input buffers to be originating from its own buffer
pool; when the input buffer pool returned by the library is not _NULL_ it must
be used and input buffers cannot be shared with other video pipeline elements.

### Threading model

The library is designed to run on a _libpomp_ event loop (_pomp_loop_, see
_libpomp_ documentation). All API functions must be called from the _pomp_loop_
thread. All callback functions (frame_output, flush or stop) are called from
the _pomp_loop_ thread.

## Testing

The library can be tested using the provided _vdec_ command-line tool which
takes as input a raw H.264 (annex B byte stream) file and can optionally
output a decoded YUV file.

To build the tool, enable _vdec_ in the Alchemy build configuration.

For a list of available options, run

    $ vdec -h

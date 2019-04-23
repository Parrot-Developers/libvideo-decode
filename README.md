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

Two modes of operation can be used for decoding: asynchronous and synchronous.

Some decoders need the input buffers to be originating from its own buffer
pool; when the input buffer pool returned by the library is not _NULL_ it must
be used and input buffers cannot be shared with other video pipeline elements.

### Asynchronous decoding mode

In asynchronous mode the application pushes buffers to decode in the input
queue and is notified of decoded frames through a callback function.

Asynchronous decoding mode support is mandatory for decoding implementations
and is always available.

### Synchronous decoding mode

In synchronous mode the application calls the synchronized decoding function
with an input buffer to decode and a decoded output buffer is returned. The
function is blocking during the processing time.

Synchronous decoding mode support is optional for decoding implementations
and is not always available.

### Threading model

The API functions are not thread safe and should always be called from the
same thread, or it is the caller's resposibility to synchronize calls if
multiple threads are used.

In asynchronous mode, the callback functions (frame output, flush or stop)
are generally called from a library internal thread and the application has
the responsibility of synchronization with its thread(s).

## Testing

The library can be tested using the provided _vdec_ command-line tool which
takes as input a raw H.264 (annex B byte stream) file and can optionally
output a decoded YUV file.

To build the tool, enable _vdec_ in the Alchemy build configuration.

For a list of available options, run

    $ vdec -h

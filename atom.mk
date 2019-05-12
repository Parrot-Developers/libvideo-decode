
LOCAL_PATH := $(call my-dir)

# API library. This is the library that most programs should use.
include $(CLEAR_VARS)
LOCAL_MODULE := libvideo-decode
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video decoding library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DVDEC_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/vdec.c
LOCAL_LIBRARIES := \
	libh264 \
	libpomp \
	libh265 \
	libulog \
	libvideo-decode-core \
	libvideo-defs
LOCAL_CONFIG_FILES := config.in
$(call load-config)
LOCAL_CONDITIONAL_LIBRARIES := \
	CONFIG_VDEC_FFMPEG:libvideo-decode-ffmpeg \
	CONFIG_VDEC_MEDIACODEC:libvideo-decode-mediacodec \
	CONFIG_VDEC_VIDEOCOREMMAL:libvideo-decode-videocoremmal \
	CONFIG_VDEC_VIDEOTOOLBOX:libvideo-decode-videotoolbox \
	CONFIG_VDEC_HISI:libvideo-decode-hisi \
	CONFIG_VDEC_AML:libvideo-decode-aml
LOCAL_EXPORT_LDLIBS := -lvideo-decode-core
ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

include $(CLEAR_VARS)

# Core library, common code for all implementations and structures definitions.
# Used by implementations.
LOCAL_MODULE := libvideo-decode-core
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video decoding library: core files
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/core/include
LOCAL_CFLAGS := -DVDEC_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	core/src/vdec_dbg.c \
	core/src/vdec_enums.c \
	core/src/vdec_format.c \
	core/src/vdec_h264.c \
	core/src/vdec_h265.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libulog \
	libvideo-defs \
	libvideo-metadata
ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

# ffmpeg implementation. can be enabled in the product configuration
LOCAL_MODULE := libvideo-decode-ffmpeg
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video decoding library: ffmpeg implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/ffmpeg/include
LOCAL_CFLAGS := -DVDEC_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	ffmpeg/src/vdec_ffmpeg.c
LOCAL_LIBRARIES := \
	ffmpeg-libav \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog \
	libvideo-decode-core \
	libvideo-defs \
	libvideo-metadata
ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)","linux-android")

include $(CLEAR_VARS)

# MediaCodec (Android) implementation.
# can be enabled in the product configuration
LOCAL_MODULE := libvideo-decode-mediacodec
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video decoding library: mediacodec implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/mediacodec/include
LOCAL_CFLAGS := -DVDEC_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_LDLIBS := -lmediandk
LOCAL_SRC_FILES := \
	mediacodec/src/vdec_mediacodec.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog \
	libvideo-decode-core \
	libvideo-defs \
	libvideo-metadata

include $(BUILD_LIBRARY)

endif

include $(CLEAR_VARS)

# VideoCore MMAL (RaspberryPi) implementation.
# can be enabled in the product configuration
LOCAL_MODULE := libvideo-decode-videocoremmal
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video decoding library: videocoremmal implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/videocoremmal/include
LOCAL_CFLAGS := -DVDEC_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	videocoremmal/src/vdec_videocoremmal.c
LOCAL_LIBRARIES := \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libpomp \
	libulog \
	libvideo-decode-core \
	libvideo-defs \
	mmal
ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)

ifeq ("${TARGET_OS}", "darwin")
include $(CLEAR_VARS)

# VideoToolbox (iOS/macOS) implementation.
# can be enabled in the product configuration
LOCAL_MODULE := libvideo-decode-videotoolbox
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video decoding library: videotoolbox implementation
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/videotoolbox/include
LOCAL_CFLAGS := -DVDEC_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	videotoolbox/src/vdec_videotoolbox.c
LOCAL_LIBRARIES := \
	libfutils \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog \
	libvideo-decode-core \
	libvideo-defs \
	libvideo-metadata
LOCAL_LDLIBS += \
	-framework Foundation \
	-framework CoreMedia \
	-framework CoreVideo \
	-framework VideoToolbox

include $(BUILD_LIBRARY)
endif


include $(CLEAR_VARS)

LOCAL_MODULE := vdec
LOCAL_DESCRIPTION := Video decoding program
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tools/vdec.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libh265 \
	libmedia-buffers \
	libmedia-buffers-memory \
	libmedia-buffers-memory-generic \
	libpomp \
	libulog \
	libvideo-decode \
	libvideo-defs \
	libvideo-raw
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:mmal

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)-$(TARGET_PRODUCT_VARIANT)","linux-generic-raspi")
  LOCAL_LDLIBS += -lbcm_host -lvchiq_arm
else ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif


include $(BUILD_EXECUTABLE)

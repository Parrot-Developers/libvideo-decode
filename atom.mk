
LOCAL_PATH := $(call my-dir)

include $(CLEAR_VARS)
LOCAL_MODULE := libvideo-decode
LOCAL_CATEGORY_PATH := libs
LOCAL_DESCRIPTION := Video decoding library
LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_CFLAGS := -DVDEC_API_EXPORTS -fvisibility=hidden -std=gnu99
LOCAL_SRC_FILES := \
	src/vdec.c \
	src/vdec_dbg.c \
	src/vdec_ffmpeg.c \
	src/vdec_h264.c \
	src/vdec_mediacodec.c \
	src/vdec_videocoremmal.c \
	src/vdec_videotoolbox.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libulog \
	libvideo-buffers
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:ffmpeg-libav \
	OPTIONAL:libmediacodec-wrapper \
	OPTIONAL:libvideo-buffers-generic \
	OPTIONAL:mmal

ifeq ("$(TARGET_OS)","darwin")
  LOCAL_CFLAGS += -DUSE_VIDEOTOOLBOX
  LOCAL_LDLIBS += \
	-framework Foundation \
	-framework CoreMedia \
	-framework CoreVideo \
	-framework VideoToolbox
else ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_LIBRARY)


include $(CLEAR_VARS)

LOCAL_MODULE := vdec
LOCAL_DESCRIPTION := Video decoding program
LOCAL_CATEGORY_PATH := multimedia
LOCAL_SRC_FILES := tools/vdec.c
LOCAL_LIBRARIES := \
	libfutils \
	libh264 \
	libulog \
	libvideo-buffers \
	libvideo-buffers-generic \
	libvideo-decode
LOCAL_CONDITIONAL_LIBRARIES := \
	OPTIONAL:mmal

ifeq ("$(TARGET_OS)-$(TARGET_OS_FLAVOUR)-$(TARGET_PRODUCT_VARIANT)","linux-generic-raspi")
  LOCAL_LDLIBS += -lbcm_host -lvchiq_arm
else ifeq ("$(TARGET_OS)","windows")
  LOCAL_LDLIBS += -lws2_32
endif

include $(BUILD_EXECUTABLE)

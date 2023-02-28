cmake_minimum_required(VERSION 3.15)
project(libvideo-decode VERSION 1.0)

# video-decode
set(LIB_SOURCES
        src/vdec.c
)

add_library(video-decode ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# message(" SOME PRINT OUTS : ")
# message($<INSTALL_INTERFACE:include>)
# message($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)


target_include_directories(video-decode PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(video-decode PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(video-decode PRIVATE "_GNU_SOURCE")
target_compile_options(video-decode PRIVATE "-fvisibility=hidden")
target_compile_options(video-decode PRIVATE "-std=gnu99")

if(WIN32)
  target_link_libraries(video-decode ws2_32)
endif()

target_link_libraries(video-decode h264 pomp h265 ulog video-decode-core video-defs)
# if option TODO
# target_link_libraries(video-decode video-decode-ffmpeg)

# video-decode-core
set(LIB_SOURCES
    core/src/vdec_dbg.c
    core/src/vdec_enums.c
    core/src/vdec_format.c
    core/src/vdec_h264.c
    core/src/vdec_h265.c
)

add_library(video-decode-core ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(video-decode-core PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(video-decode-core PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(video-decode-core PRIVATE "_GNU_SOURCE")
target_compile_options(video-decode-core PRIVATE "-fvisibility=hidden")
target_compile_options(video-decode-core PRIVATE "-std=gnu99")

if(WIN32)
  target_link_libraries(video-decode-core ws2_32)
endif()

target_link_libraries(video-decode-core 
                        futils 
                        h264 
                        h265 
                        media-buffers 
                        media-buffers-memory 
                        ulog 
                        video-defs 
                        video-metadata
)

# video-decode-ffmpeg
set(LIB_SOURCES
      ffmpeg/src/vdec_ffmpeg.c
)

add_library(video-decode-ffmpeg ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(video-decode-ffmpeg PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ffmpeg/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(video-decode-ffmpeg PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(video-decode-ffmpeg PRIVATE "_GNU_SOURCE")
target_compile_options(video-decode-ffmpeg PRIVATE "-fvisibility=hidden")
target_compile_options(video-decode-ffmpeg PRIVATE "-std=gnu99")

if(WIN32)
  target_link_libraries(video-decode-ffmpeg ws2_32)
endif()

# find_package(FFMPEG REQUIRED)
set(FFMPEG_LIBRARIES
    avcodec 
    avdevice 
    avfilter 
    avformat 
    avutil 
    swresample 
    swscale)

set(FFMPEG_INCLUDE_DIRS
    /se/sw/contribs/centos7-gcc11-x86_64/ffmpeg/n4.1-contrib20200311.01/include
)

set(FFMPEG_LIBRARY_DIRS
    /se/sw/contribs/centos7-gcc11-x86_64/ffmpeg/n4.1-contrib20200311.01/lib
)

target_include_directories(video-decode-ffmpeg PRIVATE ${FFMPEG_INCLUDE_DIRS})
target_link_directories(video-decode-ffmpeg PRIVATE ${FFMPEG_LIBRARY_DIRS})
target_link_libraries(video-decode-ffmpeg ${FFMPEG_LIBRARIES})

target_link_libraries(video-decode-ffmpeg
                        futils 
                        media-buffers 
                        media-buffers-memory 
                        media-buffers-memory-generic
                        pomp
                        ulog 
                        video-decode-core
                        video-defs 
                        video-metadata
)

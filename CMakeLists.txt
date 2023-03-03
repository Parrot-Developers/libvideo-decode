cmake_minimum_required(VERSION 3.15)
project(video-decode VERSION 1.0)


# ${PROJECT_NAME}
set(LIB_SOURCES
        src/vdec.c
)

add_library(${PROJECT_NAME} ${LIB_SOURCES})

# checks if set up rpath exists for install
if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

set_target_properties(${PROJECT_NAME} PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

# -- PROJECT

target_include_directories(${PROJECT_NAME} PUBLIC
                            # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ffmpeg/include>
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME} PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME} PRIVATE "BUILD_LIBVIDEO_DECODE_FFMPEG")
target_compile_definitions(${PROJECT_NAME} PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME} PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME} PRIVATE "-std=gnu99")


#  -- CORE

set(LIB_SOURCES
    core/src/vdec_dbg.c
    core/src/vdec_enums.c
    core/src/vdec_format.c
    core/src/vdec_h264.c
    core/src/vdec_h265.c
)

add_library(${PROJECT_NAME}-core ${LIB_SOURCES})

set_target_properties(video-decode-core PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(${PROJECT_NAME}-core PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}-core PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME}-core PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME}-core PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME}-core PRIVATE "-std=gnu99")
target_compile_definitions(${PROJECT_NAME}-core PRIVATE "BUILD_LIBVIDEO_DECODE_FFMPEG")


# -- ffmpeg
set(LIB_SOURCES
      ffmpeg/src/vdec_ffmpeg.c
)

add_library(${PROJECT_NAME}-ffmpeg ${LIB_SOURCES})

set_target_properties(video-decode-ffmpeg PROPERTIES
                    POSITION_INDEPENDENT_CODE ${BUILD_SHARED_LIBS}
)

target_include_directories(${PROJECT_NAME}-ffmpeg PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ffmpeg/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}-ffmpeg PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME}-ffmpeg PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME}-ffmpeg PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME}-ffmpeg PRIVATE "-std=gnu99")

find_package(FFMPEG)
set(FFMPEG_LIBRARIES
    avcodec 
    avdevice 
    avfilter 
    avformat 
    avutil 
    swresample 
    swscale
)



if((NOT DEFINED FFMPEG_INCLUDE_DIRS OR NOT DEFINED FFMPEG_LIBRARY_DIRS))
    message(FATAL_ERROR "Need to set FFMPEG_INCLUDE_DIRS and FFMPEG_LIBRARY_DIRS!")
else()

    message(INFO " FFMPEG SOURCES: ") 
    message(INFO " 1. FFMPEG INCLUDE DIRS: ${FFMPEG_INCLUDE_DIRS}") 
    message(INFO " 2. FFMPEG LIBRARY DIRS: ${FFMPEG_LIBRARY_DIRS}") 

    target_include_directories(${PROJECT_NAME}-ffmpeg PRIVATE ${FFMPEG_INCLUDE_DIRS})
    target_link_directories(${PROJECT_NAME}-ffmpeg PRIVATE ${FFMPEG_LIBRARY_DIRS})

    if(WIN32)

        target_link_libraries(${PROJECT_NAME} 
                              PRIVATE 
                              ws2_32
                              media-buffers
                              futils
                              h264 
                              pomp 
                              h265 
                              ulog 
                              ${PROJECT_NAME}-core 
                              ${PROJECT_NAME}-ffmpeg 
                              video-defs
                              )

        target_link_libraries(${PROJECT_NAME}-ffmpeg 
                               PRIVATE 
                               ws2_32
                               ${FFMPEG_LIBRARIES}
                               futils 
                               media-buffers 
                               media-buffers-memory 
                               media-buffers-memory-generic
                               pomp
                               ulog 
                               ${PROJECT_NAME}-core
                               video-defs 
                               video-metadata
                               )

	file(GLOB ffmpeg_dll "${FFMPEG_LIBRARY_DIRS}/../bin/*.dll")
	message(${ffmpeg_dll})
        install(FILES ${ffmpeg_dll} DESTINATION bin)



        target_link_libraries(${PROJECT_NAME}-core 
                              PRIVATE 
                              ws2_32
                              pomp
                              futils 
                              h264 
                              h265 
                              media-buffers 
                              media-buffers-memory 
                              ulog 
                              video-defs 
                              video-metadata
                              )


    else()

        target_link_libraries(${PROJECT_NAME} 
                    PRIVATE
                    media-buffers
                    futils
                    h264 
                    pomp 
                    h265 
                    ulog 
                    ${PROJECT_NAME}-core 
                    ${PROJECT_NAME}-ffmpeg 
                    video-defs
                    )

        target_link_libraries(${PROJECT_NAME}-core 
                               PRIVATE
                               pomp
                               futils 
                               h264 
                               h265 
                               media-buffers 
                               media-buffers-memory 
                               ulog 
                               video-defs 
                               video-metadata
                            )

        target_link_libraries(${PROJECT_NAME}-ffmpeg
                                PRIVATE
                                ${FFMPEG_LIBRARIES}
                                futils 
                                media-buffers 
                                media-buffers-memory 
                                media-buffers-memory-generic
                                pomp
                                ulog 
                                ${PROJECT_NAME}-core
                                video-defs 
                                video-metadata
        )

    endif()

    install(TARGETS ${PROJECT_NAME}
            EXPORT ${PROJECT_NAME}-targets
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )

    install(TARGETS ${PROJECT_NAME}-ffmpeg
            EXPORT ${PROJECT_NAME}-ffmpeg-targets
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )

    install(TARGETS ${PROJECT_NAME}-core
            EXPORT ${PROJECT_NAME}-core-targets
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )
endif()

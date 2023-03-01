cmake_minimum_required(VERSION 3.15)
project(video-decode VERSION 1.0)

# checks if set up rpath exists for install
if(COMMAND set_up_rpath)
    set_up_rpath()
else()
    message("Set up rpath not defined!")
endif()

# ${PROJECT_NAME}
set(LIB_SOURCES
        src/vdec.c
)

add_library(${PROJECT_NAME} ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

# message(" SOME PRINT OUTS : ")
# message($<INSTALL_INTERFACE:include>)
# message($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

# -- PROJECT

target_include_directories(${PROJECT_NAME} PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                            $<INSTALL_INTERFACE:include>
)


target_compile_definitions(${PROJECT_NAME} PRIVATE "VDEC_API_EXPORTS")
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

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(${PROJECT_NAME}-core PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}-core PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME}-core PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME}-core PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME}-core PRIVATE "-std=gnu99")


# -- ffmpeg
set(LIB_SOURCES
      ffmpeg/src/vdec_ffmpeg.c
)

add_library(${PROJECT_NAME}-ffmpeg ${LIB_SOURCES})

option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

target_include_directories(${PROJECT_NAME}-ffmpeg PUBLIC
                            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ffmpeg/include>
                            $<INSTALL_INTERFACE:include>
)

target_compile_definitions(${PROJECT_NAME}-ffmpeg PRIVATE "VDEC_API_EXPORTS")
target_compile_definitions(${PROJECT_NAME}-ffmpeg PRIVATE "_GNU_SOURCE")
target_compile_options(${PROJECT_NAME}-ffmpeg PRIVATE "-fvisibility=hidden")
target_compile_options(${PROJECT_NAME}-ffmpeg PRIVATE "-std=gnu99")

# find_package(FFMPEG REQUIRED)
set(FFMPEG_LIBRARIES
    avcodec 
    avdevice 
    avfilter 
    avformat 
    avutil 
    swresample 
    swscale
)



if(NOT DEFINED FFMPEG_INCLUDE_DIRS AND NOT DEFINED FFMPEG_LIBRARY_DIRS)
    message(FATAL_ERROR "Need to set FFMPEG_INCLUDE_DIRS and FFMPEG_LIBRARY_DIRS!")
else()

    message("FFMPEG INCLUDE DIRS: ") 
    message(${FFMPEG_INCLUDE_DIRS}) 
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
                              video-defs
                              )


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

    install(TARGETS ${PROJECT_NAME}-core
            EXPORT ${PROJECT_NAME}-core-targets
            ARCHIVE DESTINATION lib
            LIBRARY DESTINATION lib
            RUNTIME DESTINATION bin
            )
endif()

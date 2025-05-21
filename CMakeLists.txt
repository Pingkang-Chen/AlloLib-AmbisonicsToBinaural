cmake_minimum_required(VERSION 3.16)
project(AlloLib-AmbisonicsToiBnaural)
set(APP_NAME AmbisonicsToiBnaural)

# cmake scripts are kept in the cmake directory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# set compiler/platform specific flags
if(MSVC)
  set(CMAKE_BUILD_PARALLEL_LEVEL $ENV{NUMBER_OF_PROCESSORS})
  # add_compile_options(/utf-8)
  # set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /STACK:4194304")
endif()

# add source files to executable
add_executable(${APP_NAME} main.cpp)

# add allolib to project & link
add_subdirectory(allolib)
target_link_libraries(${APP_NAME} PRIVATE al)

# Add and configure Spatial Audio Framework
# Enable SOFA reader module which includes libmysofa
set(SAF_PERFORMANCE_LIB SAF_USE_APPLE_ACCELERATE CACHE STRING "Performance library to use")
set(SAF_ENABLE_SOFA_READER_MODULE 1 CACHE BOOL "Enable SOFA reader module")
set(SAF_BUILD_EXAMPLES 0 CACHE BOOL "Don't build SAF examples")

# Add SAF as a subdirectory
add_subdirectory(Spatial_Audio_Framework)

# Link against SAF
target_link_libraries(${APP_NAME} PRIVATE saf)

# Include SAF headers
target_include_directories(${APP_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/include)
target_include_directories(${APP_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_hrir)
target_include_directories(${APP_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_hoa)
target_include_directories(${APP_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_vbap)
target_include_directories(${APP_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_utilities)
target_include_directories(${APP_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_sofa_reader)
target_include_directories(${APP_NAME} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_sofa_reader/libmysofa/include)

# add al_ext to project & link if it exists
if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/al_ext)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/al_ext)
  get_target_property(AL_EXT_LIBRARIES al_ext AL_EXT_LIBRARIES)
  target_link_libraries(${APP_NAME} PRIVATE ${AL_EXT_LIBRARIES})
endif()

# Platform-specific settings
if(APPLE)
  # MacOS specific settings
  target_link_libraries(${APP_NAME} PRIVATE "-framework CoreAudio" "-framework CoreMIDI" "-framework CoreFoundation")
elseif(UNIX AND NOT APPLE)
  # Linux specific settings
  find_package(ALSA QUIET)
  if(ALSA_FOUND)
    target_link_libraries(${APP_NAME} PRIVATE ${ALSA_LIBRARIES})
  endif()
  
  find_package(PulseAudio QUIET)
  if(PULSAUDIO_FOUND)
    target_link_libraries(${APP_NAME} PRIVATE ${PULSAUDIO_LIBRARIES})
  endif()
elseif(WIN32)
  # Windows specific settings
  target_link_libraries(${APP_NAME} PRIVATE winmm)
endif()

# Suppress warnings for now
target_compile_options(${APP_NAME} PRIVATE -w)

# binaries are put into the ./bin directory by default
set_target_properties(${APP_NAME} PROPERTIES
  CXX_STANDARD 14
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR}/debug
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR}/bin
  # ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/lib
  # LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin
)
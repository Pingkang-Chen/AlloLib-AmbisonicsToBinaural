cmake_minimum_required(VERSION 3.22)
project(AlloLib-AmbisonicsToBinaural)
set(APP_NAME AmbisonicsToBinaural)

# cmake scripts are kept in the cmake directory
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

# Set C++ standard (matching your working project)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set compiler/platform specific flags
if(MSVC)
  set(CMAKE_BUILD_PARALLEL_LEVEL $ENV{NUMBER_OF_PROCESSORS})
endif()

# ========================
# JUCE Setup - Using Working Approach from Your Successful Project
# ========================

# Add JUCE as subdirectory (this is the key - no FetchContent, no building from source)
add_subdirectory(JUCE)

# Create a static library for JUCE convolution functionality
# This approach avoids the plugin system but uses JUCE's build system properly
add_library(juce_convolution_lib STATIC
    # Empty - we'll link the JUCE modules we need
)

# Link only the JUCE modules we need (avoiding GUI modules that cause issues)
target_link_libraries(juce_convolution_lib
    PUBLIC
        juce::juce_core
        juce::juce_audio_basics
        juce::juce_dsp
        juce::juce_audio_formats
        juce::juce_recommended_config_flags
)

# Set compile definitions for JUCE (matching your working project)
target_compile_definitions(juce_convolution_lib
    PUBLIC
        JUCE_WEB_BROWSER=0
        JUCE_USE_CURL=0
        JUCE_STANDALONE_APPLICATION=1
        JUCE_DISABLE_JUCE_VERSION_PRINTING=1
        # Disable GUI modules that cause macOS 15.1 issues
        JUCE_MODULE_AVAILABLE_juce_gui_basics=0
        JUCE_MODULE_AVAILABLE_juce_gui_extra=0
        JUCE_MODULE_AVAILABLE_juce_opengl=0
        JUCE_MODULE_AVAILABLE_juce_graphics=0
        JUCE_MODULE_AVAILABLE_juce_events=0
)

# ========================
# Header-Only Structure with Ambisonic Reverb Support
# ========================

# Main source file (contains most of the code)
add_executable(${APP_NAME} src/main.cpp)

# Include directories for our header-only modules including new reverb processor
target_include_directories(${APP_NAME} PRIVATE
    src
    src/audio
    ${CMAKE_CURRENT_SOURCE_DIR}/src/audio
)

# add allolib to project & link
add_subdirectory(allolib)
target_link_libraries(${APP_NAME} PRIVATE al)

# Add and configure Spatial Audio Framework
set(SAF_PERFORMANCE_LIB SAF_USE_APPLE_ACCELERATE CACHE STRING "Performance library to use")
set(SAF_ENABLE_SOFA_READER_MODULE 1 CACHE BOOL "Enable SOFA reader module")
set(SAF_BUILD_EXAMPLES 0 CACHE BOOL "Don't build SAF examples")

# Add SAF as a subdirectory
add_subdirectory(Spatial_Audio_Framework)

# ADD THIS LINE - Native File Dialog Extended
add_subdirectory(nfd)

# Link against SAF
target_link_libraries(${APP_NAME} PRIVATE saf)

# Include SAF headers
target_include_directories(${APP_NAME} PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/include
    ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_hrir
    ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_hoa
    ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_vbap
    ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_utilities
    ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_sofa_reader
    ${CMAKE_CURRENT_LIST_DIR}/Spatial_Audio_Framework/framework/modules/saf_sofa_reader/libmysofa/include
    ${CMAKE_CURRENT_LIST_DIR}/al_ext/assets3d
)

# Link our JUCE convolution library (using the working approach)
target_link_libraries(${APP_NAME} PRIVATE juce_convolution_lib)

target_link_libraries(${APP_NAME} PRIVATE nfd)

# add al_ext to project & link if it exists
if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/al_ext)
  add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/al_ext)
  get_target_property(AL_EXT_LIBRARIES al_ext AL_EXT_LIBRARIES)
  target_link_libraries(${APP_NAME} PRIVATE ${AL_EXT_LIBRARIES})
endif()

target_link_libraries(${APP_NAME} PRIVATE al_assets3d)

# Platform-specific settings for your application
if(APPLE)
  # MacOS specific settings
  target_link_libraries(${APP_NAME} PRIVATE 
    "-framework CoreAudio" 
    "-framework CoreMIDI" 
    "-framework CoreFoundation"
  )
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

# Suppress warnings
target_compile_options(${APP_NAME} PRIVATE -w)

# binaries are put into the ./bin directory by default
set_target_properties(${APP_NAME} PROPERTIES
  CXX_STANDARD 20
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin
  RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_LIST_DIR}/debug
  RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_CURRENT_LIST_DIR}/bin
)
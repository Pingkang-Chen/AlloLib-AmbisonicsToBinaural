#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Gamma" for configuration "Release"
set_property(TARGET Gamma APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Gamma PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libGamma.a"
  )

list(APPEND _cmake_import_check_targets Gamma )
list(APPEND _cmake_import_check_files_for_Gamma "${_IMPORT_PREFIX}/lib/libGamma.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

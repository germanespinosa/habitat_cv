#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "maze_cv_stitching" for configuration "Release"
set_property(TARGET maze_cv_stitching APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(maze_cv_stitching PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libmaze_cv_stitching.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS maze_cv_stitching )
list(APPEND _IMPORT_CHECK_FILES_FOR_maze_cv_stitching "${_IMPORT_PREFIX}/lib/libmaze_cv_stitching.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

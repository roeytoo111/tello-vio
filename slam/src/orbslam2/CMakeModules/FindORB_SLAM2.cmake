#
# Find ORB_SLAM2 library and headers.
#
# This repo's install script (`scripts/orbslam.sh`) exports ORB_SLAM2_ROOT_DIR.
# We use that as the primary hint, but also fall back to common prefixes.
#
# Sets:
#   ORB_SLAM2_FOUND
#   ORB_SLAM2_INCLUDE_DIRS
#   ORB_SLAM2_LIBRARIES
#

include(FindPackageHandleStandardArgs)

set(_ORB_SLAM2_HINTS "")

if(DEFINED ORB_SLAM2_ROOT_DIR AND NOT ORB_SLAM2_ROOT_DIR STREQUAL "")
  list(APPEND _ORB_SLAM2_HINTS "${ORB_SLAM2_ROOT_DIR}")
endif()

if(DEFINED ENV{ORB_SLAM2_ROOT_DIR} AND NOT "$ENV{ORB_SLAM2_ROOT_DIR}" STREQUAL "")
  list(APPEND _ORB_SLAM2_HINTS "$ENV{ORB_SLAM2_ROOT_DIR}")
endif()

list(APPEND _ORB_SLAM2_HINTS
  "/usr/local"
  "/usr"
  "/opt"
)

find_path(ORB_SLAM2_INCLUDE_DIRS
  NAMES ORB_SLAM2/System.h
  HINTS ${_ORB_SLAM2_HINTS}
  PATH_SUFFIXES include Include
)

find_library(ORB_SLAM2_LIBRARIES
  NAMES ORB_SLAM2 libORB_SLAM2 orbslam2 liborbslam2
  HINTS ${_ORB_SLAM2_HINTS}
  PATH_SUFFIXES lib lib64 build/lib build/lib64
)

find_package_handle_standard_args(ORB_SLAM2
  REQUIRED_VARS ORB_SLAM2_INCLUDE_DIRS ORB_SLAM2_LIBRARIES
)

mark_as_advanced(ORB_SLAM2_INCLUDE_DIRS ORB_SLAM2_LIBRARIES)


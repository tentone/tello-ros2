# - Try to find ORB_SLAM2
# Set alternative paths to search for using ORB_SLAM2_DIR
# Once done this will define
#  ORB_SLAM2_FOUND - System has ORB_SLAM2
#  ORB_SLAM2_INCLUDE_DIRS - The ORB_SLAM2 include directories
#  ORB_SLAM2_LIBRARIES - The libraries needed to use ORB_SLAM2
#  ORB_SLAM2_DEFINITIONS - Compiler switches required for using ORB_SLAM2
#
# To help the search ORB_SLAM2_ROOT_DIR environment variable as the path to ORB_SLAM2 root folder
# e.g. `export ORB_SLAM2_ROOT_DIR=~/ORB_SLAM2`

# Default location of ORB_SLAM2
set(_ORB_SLAM2_SEARCHES /usr/local /usr)

if (ORB_SLAM2_DIR)
    set(_ORB_SLAM2_SEARCHES ${ORB_SLAM2_DIR} ${_ORB_SLAM2_SEARCHES})
endif()

if (DEFINED ENV{ORB_SLAM2_ROOT_DIR})
    get_filename_component(ORB_SLAM2_PARENT_DIR $ENV{ORB_SLAM2_ROOT_DIR} DIRECTORY)
    set(_ORB_SLAM2_SEARCHES $ENV{ORB_SLAM2_ROOT_DIR} ${ORB_SLAM2_PARENT_DIR} ${_ORB_SLAM2_SEARCHES})
endif()

# Find ORB_SLAM2
find_path(ORB_SLAM2_INCLUDE_DIR System.h
          PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES include ORB_SLAM2 include/ORB_SLAM2)

find_library(ORB_SLAM2_LIBRARY NAMES ORB_SLAM2 libORB_SLAM2
             PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES lib)

# Find built-in DBoW2
find_path(DBoW2_INCLUDE_DIR Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES include)
find_library(DBoW2_LIBRARY NAMES DBoW2 
             PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES lib Thirdparty/DBoW2/lib)

# Find built-in g2o
find_library(g2o_LIBRARY NAMES g2o 
             PATHS ${_ORB_SLAM2_SEARCHES} PATH_SUFFIXES lib Thirdparty/g2o/lib)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM2_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM2  DEFAULT_MSG
                                  ORB_SLAM2_LIBRARY ORB_SLAM2_INCLUDE_DIR DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)

mark_as_advanced(ORB_SLAM2_INCLUDE_DIR ORB_SLAM2_LIBRARY )

set(ORB_SLAM2_LIBRARIES ${ORB_SLAM2_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})
set(ORB_SLAM2_INCLUDE_DIRS ${ORB_SLAM2_INCLUDE_DIR} ${DBoW2_INCLUDE_DIR})

# FindSDL2.cmake
#
# This module defines:
#  SDL2_FOUND        - True if SDL2 was found.
#  SDL2_INCLUDE_DIRS - SDL2 include directories.
#  SDL2_LIBRARIES    - SDL2 libraries to link against.
#  SDL2_VERSION      - The version of SDL2 found.

# Try to find SDL2 headers and libraries
find_path(SDL2_INCLUDE_DIR SDL.h
          HINTS ${SDL2_ROOT_DIR}/include/SDL2
                $ENV{SDL2_ROOT_DIR}/include/SDL2
                /usr/include/SDL2
                /usr/local/include/SDL2
                /opt/local/include/SDL2)

find_library(SDL2_LIBRARY SDL2
             HINTS ${SDL2_ROOT_DIR}/lib
                   $ENV{SDL2_ROOT_DIR}/lib
                   /usr/lib
                   /usr/local/lib
                   /opt/local/lib)

# Handle the REQUIRED argument and set SDL2_FOUND
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SDL2
                                  REQUIRED_VARS SDL2_LIBRARY SDL2_INCLUDE_DIR
                                  VERSION_VAR SDL2_VERSION)

# Set the include directories and libraries
if(SDL2_FOUND)
    set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIR})
    set(SDL2_LIBRARIES ${SDL2_LIBRARY})
endif()

# Mark variables as advanced
mark_as_advanced(SDL2_INCLUDE_DIR SDL2_LIBRARY)

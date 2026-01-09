# FindV4L2.cmake - Find Video4Linux2 (V4L2) development libraries and headers
#
# This module defines the following variables:
#   V4L2_FOUND       - True if V4L2 is found
#   V4L2_INCLUDE_DIRS - Directory containing V4L2 headers
#   V4L2_LIBRARIES    - V4L2 libraries to link against
#   V4L2_DEFINITIONS  - Compiler definitions required for V4L2
#
# This module will also set the following imported targets if available:
#   V4L2::V4L2 - The V4L2 library target
#
# Typical usage in CMakeLists.txt:
#   find_package(V4L2 REQUIRED)
#   target_link_libraries(my_target V4L2::V4L2)

# Find the header file
find_path(V4L2_INCLUDE_DIR
    NAMES linux/videodev2.h
    PATHS /usr/include
          /usr/local/include
          /opt/local/include
    DOC "Path to V4L2 development headers"
)

# Find the library
find_library(V4L2_LIBRARY
    NAMES v4l2 libv4l2.so.0 libv4l2.so
    PATHS /usr/lib/x86_64-linux-gnu
          /usr/lib
          /usr/local/lib
          /opt/local/lib
    DOC "Path to V4L2 library"
)

# Handle the QUIET and REQUIRED arguments and set V4L2_FOUND
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(V4L2
    FOUND_VAR V4L2_FOUND
    REQUIRED_VARS V4L2_LIBRARY V4L2_INCLUDE_DIR
    FAIL_MESSAGE "V4L2 (Video4Linux2) not found. Please install libv4l-dev (Debian/Ubuntu) or v4l-utils (Fedora/RHEL)"
)

# Set the standard variables
if(V4L2_FOUND)
    set(V4L2_LIBRARIES ${V4L2_LIBRARY})
    set(V4L2_INCLUDE_DIRS ${V4L2_INCLUDE_DIR})
    set(V4L2_DEFINITIONS "")
    
    # Create imported target if not already created
    if(NOT TARGET V4L2::V4L2)
        add_library(V4L2::V4L2 UNKNOWN IMPORTED)
        set_target_properties(V4L2::V4L2 PROPERTIES
            IMPORTED_LOCATION "${V4L2_LIBRARY}"
            INTERFACE_INCLUDE_DIRECTORIES "${V4L2_INCLUDE_DIR}"
            INTERFACE_COMPILE_DEFINITIONS "HAVE_V4L2=1"
        )
    endif()
endif()

# Mark advanced variables to clean up CMake UIs
mark_as_advanced(V4L2_INCLUDE_DIR V4L2_LIBRARY)
# Print status message
if(V4L2_FOUND AND NOT V4L2_FIND_QUIETLY)
    message(STATUS "Found V4L2:")
    message(STATUS "  Include dir: ${V4L2_INCLUDE_DIR}")
    message(STATUS "  Library: ${V4L2_LIBRARY}")
endif()
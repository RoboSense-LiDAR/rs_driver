cmake_minimum_required(VERSION 3.16.3)
project(UVC-AC VERSION 0.0.7 LANGUAGES C CXX)

if(EXTRA_CMAKE)
    include("${EXTRA_CMAKE}")
    message(STATUS "Including extra cmake from ${EXTRA_CMAKE}")
endif()

# Used to configure generated files.
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

# Basic Information.
MESSAGE(STATUS "CMake version: " ${CMAKE_VERSION})
MESSAGE(STATUS "CMake system name: " ${CMAKE_SYSTEM_NAME})
MESSAGE(STATUS "CMake system processor: " ${CMAKE_SYSTEM_PROCESSOR})

# Specifying a search path for CMake modules to be loaded by the the include() or find_package() commands before checking the default modules that come with CMake.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Macro to provide an option dependent on other options.
include(CMakeDependentOption)

# Provides a macro to check if a list of one or more header files can be included together in C.
include(CheckIncludeFiles)

# Check if a C function can be linked.
include(CheckFunctionExists)

# Check if a symbol exists as a function, variable, or macro.
include(CheckSymbolExists)

# Provides install directory variables as defined by the GNU Coding Standards.
# GNU Coding Standards Reference 'https://www.gnu.org/prep/standards/html_node/Directory-Variables.html'.
include(GNUInstallDirs)

# Check whether the CXX compiler supports a given flag.
include(CheckCXXCompilerFlag)

# Check whether the C compiler supports a given flag.
include(CheckCCompilerFlag)

if((NOT CMAKE_INSTALL_RPATH) AND CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_FULL_LIBDIR})
    message(STATUS "Setting install rpath to ${CMAKE_INSTALL_RPATH}")
endif()

include(cmake/CMakeUtils.cmake)

# Provides an option that the user can optionally select.
option(ENABLE_SHARED_UVC "Build shared library." OFF)
option(ENABLE_STATIC_UVC "Build static library." ON)
option(ENABLE_STRICT "Build with strict compile options." ON)
option(ENABLE_TESTS_UVC "Enable compilation of test programs." OFF)
option(ENABLE_DEBUG "Enable any functionality related to debugging." OFF)
option(ENABLE_DEBUG_LOG "Turn on or off debug level logs." ON)
option(ENABLE_CPPCHECK "Turn on or off static analysis." OFF)
option(ENABLE_DOC "Enable documentation generation with Doxygen." OFF)
option(ENABLE_PIC "Enable position-independent code." ON)

if(ENABLE_DEBUG)
    set(ENABLE_DEBUG_LOG ON)
endif()

if(ENABLE_PIC)
    check_c_compiler_flag("-fpic" SUPPORT_fpic)
    if(SUPPORT_fpic)
        add_compile_options("-fpic")
    endif()
    check_c_compiler_flag("-fPIC" SUPPORT_fPIC)
    if(SUPPORT_fPIC)
        add_compile_options("-fPIC")
    endif()
endif()

if((NOT ENABLE_SHARED_UVC) AND (NOT ENABLE_STATIC_UVC))
    message(FATAL_ERROR "Please enable shared or static library.")
endif()

# Strict options, reference from Belledonne Communications, Grenoble France.
set(STRICT_OPTIONS_CPP "")
set(STRICT_OPTIONS_C "")
set(STRICT_OPTIONS_CXX "")
list(APPEND STRICT_OPTIONS_CPP
    "-Wall"
    "-Wcast-align"
    "-Wconversion"
    "-Werror=return-type"
    "-Wfloat-equal"
    "-Winit-self"
    "-Wno-error=deprecated-declarations"
    "-Wpointer-arith"
    "-Wuninitialized"
    "-Wunused"
)
list(APPEND STRICT_OPTIONS_CXX
    "-Wnon-virtual-dtor"
    "-Woverloaded-virtual"
)
CHECK_CXX_COMPILER_FLAG("-Wsuggest-override" SUGGEST_OVERRIDE)
if(SUGGEST_OVERRIDE)
    list(APPEND STRICT_OPTIONS_CXX "-Wsuggest-override" "-Wno-error=suggest-override")
endif()
list(APPEND STRICT_OPTIONS_C "-Wstrict-prototypes" "-Werror=strict-prototypes")
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
    list(APPEND STRICT_OPTIONS_C "-fno-inline-small-functions")
endif()
if(CMAKE_C_COMPILER_ID MATCHES "Clang")
    list(APPEND STRICT_OPTIONS_CPP "-Qunused-arguments" "-Wno-array-bounds")
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    list(APPEND STRICT_OPTIONS_CXX "-x c++")
endif()
if(ENABLE_STRICT)
    list(APPEND STRICT_OPTIONS_C "-Werror" "-Wextra" "-Wno-unused-parameter" "-Wno-missing-field-initializers" "-fno-strict-aliasing")
    list(APPEND STRICT_OPTIONS_CPP "-Werror" "-Wextra" "-Wno-unused-parameter" "-Wno-missing-field-initializers" "-fno-strict-aliasing")
endif()
if(STRICT_OPTIONS_CPP)
    list(REMOVE_DUPLICATES STRICT_OPTIONS_CPP)
endif()
if(STRICT_OPTIONS_C)
    list(REMOVE_DUPLICATES STRICT_OPTIONS_C)
endif()

if(ENABLE_DEBUG)
    add_definitions(-DENABLE_DEBUG=1)
endif()

if(ENABLE_DEBUG_LOG)
    add_definitions(-DENABLE_DEBUG_LOG=1)
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/libuvc/libuvc_config.h.in ${CMAKE_CURRENT_BINARY_DIR}/libuvc/libuvc_config.h)
# Source files can have properties that affect how they are built.
#  GENERATED - Is this source file generated as part of the build process.
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/libuvc/libuvc_config.h PROPERTIES GENERATED ON)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    include
    ../libusb/libusb
)

set(COMMON_LIBS "")
set(COMMON_INCLUDE_DIRS "")

if(MSVC)
    add_definitions(-DPLATFORM_WINDOWS_MSVC=1)
    list(APPEND COMMON_LIBS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pthread_win/lib/pthread_static.lib)
    list(APPEND COMMON_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pthread_win/include)
endif()

# find_package(USB)
# if(NOT USB_FOUND)
#     message(FATAL_ERROR "Missing USB, please compile it first.")
# endif()
# list(APPEND COMMON_LIBS "${USB_LIBRARIES}")
# list(APPEND COMMON_INCLUDE_DIRS "${USB_INCLUDE_DIRS}")

list(APPEND COMMON_LIBS usb-ac-static)

list(REMOVE_DUPLICATES COMMON_LIBS)
list(REMOVE_DUPLICATES COMMON_INCLUDE_DIRS)

add_subdirectory(src)
add_subdirectory(include)

if(ENABLE_TESTS_UVC)
    add_definitions(-DENABLE_TESTS=1)
    add_subdirectory(tester)
endif()

# set(CONFIG_SIGNATURE "${PROJECT_NAME_UPPER}")
# configure_file(cmake/Config.cmake.in "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)

# set(CONFIG_PACKAGE_LOCATION "${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake")
# install(EXPORT ${PROJECT_NAME}Targets
#     FILE ${PROJECT_NAME}Targets.cmake
#     DESTINATION ${CONFIG_PACKAGE_LOCATION}
# )
# install(FILES
#     "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
#     DESTINATION ${CONFIG_PACKAGE_LOCATION}
# )

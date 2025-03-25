cmake_minimum_required(VERSION 3.16.3)
project(USB-AC VERSION 0.0.1 LANGUAGES C CXX)

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
#include(cmake/Version.cmake)

# Provides an option that the user can optionally select.
option(ENABLE_SHARED_USB "Build shared library." OFF)
option(ENABLE_STATIC_USB "Build static library." ON)
option(ENABLE_STRICT "Build with strict compile options." ON)
option(ENABLE_TESTS_USB "Enable compilation of test programs." OFF)
option(ENABLE_DEBUG "Enable any functionality related to debugging." OFF)
option(ENABLE_PIC "Enable position-independent code." ON)

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

if((NOT ENABLE_SHARED_USB) AND (NOT ENABLE_STATIC_USB))
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

if(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(PLATFORM_WINDOWS 1)
    add_definitions(-D_WIN32_WINNT=_WIN32_WINNT_VISTA)
else()
    set(PLATFORM_POSIX 1)
endif()
set(ENABLE_LOGGING 1)

if(ENABLE_DEBUG)
    set(ENABLE_DEBUG_LOGGING 1)
endif()

utils_check_symbol_exists("clock_gettime" "time.h" HAVE_CLOCK_GETTIME)
utils_check_symbol_exists("eventfd" "sys/eventfd.h" HAVE_EVENTFD)
utils_check_symbol_exists("pthread_setname_np" "pthread.h" HAVE_PTHREAD_SETNAME_NP) # TODO: Require _GNU_SOURCE

utils_check_type_exists("nfds_t" "poll.h" HAVE_NFDS_T)

check_include_files("inttypes.h" HAVE_INTTYPES_H)

if(MSVC)
    file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/msvc/config.h DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/)
else()
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake ${CMAKE_CURRENT_BINARY_DIR}/config.h)
endif()
# Source files can have properties that affect how they are built.
#  GENERATED - Is this source file generated as part of the build process.
set_source_files_properties(${CMAKE_CURRENT_BINARY_DIR}/config.h PROPERTIES GENERATED ON)
add_definitions(-DHAVE_CONFIG_H=1)

include_directories(
    ${CMAKE_CURRENT_BINARY_DIR}
    libusb
)

set(COMMON_LIBS "")
set(COMMON_INCLUDE_DIRS "")

find_package(Threads)
if(CMAKE_THREAD_LIBS_INIT)
    list(APPEND COMMON_LIBS "${CMAKE_THREAD_LIBS_INIT}")
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_package(UDev)
    if(UDEV_FOUND)
        set(HAVE_LIBUDEV 1)
    endif()

    if(UDEV_FOUND)
        list(APPEND COMMON_LIBS "${UDEV_LIBRARIES}")
        list(APPEND COMMON_INCLUDE_DIRS "${UDEV_INCLUDE_DIRS}")
    endif()

    list(REMOVE_DUPLICATES COMMON_LIBS)
    list(REMOVE_DUPLICATES COMMON_INCLUDE_DIRS)
endif()

add_subdirectory(libusb)

if(ENABLE_TESTS_USB)
    add_subdirectory(examples)
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

set(LIBRARY_NAME "${PROJECT_NAME_LOWER}")

set(SOURCE_FILES_C
    core.c
    descriptor.c
    hotplug.c
    io.c
    strerror.c
    sync.c
)

if(PLATFORM_WINDOWS)
    list(APPEND SOURCE_FILES_C
        "os/events_windows.c"
        "os/threads_windows.c"
        "os/windows_common.c"
        "os/windows_winusb.c"
        "os/windows_usbdk.c"
    )
endif()

if(PLATFORM_POSIX)
    list(APPEND SOURCE_FILES_C
        "os/events_posix.c"
        "os/threads_posix.c"
    )
endif()

if(UDEV_FOUND)
    list(APPEND SOURCE_FILES_C "os/linux_udev.c")
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND SOURCE_FILES_C
        "os/linux_netlink.c"
        "os/linux_usbfs.c"
    )
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    find_library(COREFOUNDATION CoreFoundation)
    find_library(IOKIT IOKit)
    find_library(SECURITY Security)
    list(APPEND COMMON_LIBS ${COREFOUNDATION} ${IOKIT} ${SECURITY})
    list(APPEND SOURCE_FILES_C
        "os/darwin_usb.c"
    )
endif()

set(LIBS
    ${COMMON_LIBS}
)

set(INCLUDE_DIRS
    ${COMMON_INCLUDE_DIRS}
)

#utils_apply_compile_flags(SOURCE_FILES_C STRICT_OPTIONS_CPP STRICT_OPTIONS_C)

if(ENABLE_STATIC_USB)
    add_library(${LIBRARY_NAME}-static STATIC ${SOURCE_FILES_C})
    target_link_libraries(${LIBRARY_NAME}-static PUBLIC ${LIBS})
    target_include_directories(${LIBRARY_NAME}-static PUBLIC ${INCLUDE_DIRS})
    # install(TARGETS ${LIBRARY_NAME}-static EXPORT ${PROJECT_NAME}Targets
    #     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #     PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    # )
endif()
if(ENABLE_SHARED_USB)
    add_library(${LIBRARY_NAME} SHARED ${SOURCE_FILES_C})
    target_link_libraries(${LIBRARY_NAME} PUBLIC ${LIBS})
    target_include_directories(${LIBRARY_NAME} PUBLIC ${INCLUDE_DIRS})
    # install(TARGETS ${LIBRARY_NAME} EXPORT ${PROJECT_NAME}Targets
    #     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    #     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #     ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    #     PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
    # )
endif()

set(LIBRARY_NAME "${LIBRARY_NAME}" PARENT_SCOPE)

# install(FILES libusb.h
#     DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME_LOWER}
#     PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ WORLD_READ
# )

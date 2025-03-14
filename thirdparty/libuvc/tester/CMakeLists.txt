set(EXECUTABLE_NAME "tester_${PROJECT_NAME_LOWER}")

set(SOURCE_FILES_C
    example.c
)

if(ENABLE_SHARED_UVC)
    set(LIBS
        ${LIBRARY_NAME}
    )
else()
    set(LIBS
        ${LIBRARY_NAME}-static
    )
endif()

list(APPEND LIBS
    ${COMMON_LIBS}
)

set(INCLUDE_DIRS
    ${COMMON_INCLUDE_DIRS}
)

find_package(SDL2)
if(NOT SDL2_FOUND)
    message(WARNING "Missing SDL2, disable the renderer.")
else()
    list(APPEND LIBS "${SDL2_LIBRARIES}")
    list(APPEND INCLUDE_DIRS "${SDL2_INCLUDE_DIRS}")
    add_definitions(-DSDL2_FOUND=1)
endif()

#utils_apply_compile_flags(SOURCE_FILES_C STRICT_OPTIONS_CPP STRICT_OPTIONS_C)

add_executable(${EXECUTABLE_NAME} ${SOURCE_FILES_C})
target_link_libraries(${EXECUTABLE_NAME} ${LIBS})
target_include_directories(${EXECUTABLE_NAME} PRIVATE ${INCLUDE_DIRS})

install(TARGETS ${EXECUTABLE_NAME}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

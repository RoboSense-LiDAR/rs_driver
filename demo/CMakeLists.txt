cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


project(rs_driver_demos)

message(=============================================================)
message("-- Ready to compile demos")
message(=============================================================)

message("CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
message("CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")

add_subdirectory(
    ${CMAKE_SOURCE_DIR}/thirdparty/glfw
    ${CMAKE_BINARY_DIR}/glfw_build
)

find_package(OpenGL REQUIRED)


set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/imgui)
set(IMGUIFD_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/ImGuiFileDialog)


if(WIN32)
    set(IMGUI_BACKEND
        ${IMGUI_DIR}/backends/imgui_impl_win32.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
    )
else()
    set(IMGUI_BACKEND
        ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl2.cpp
    )
endif()


if (${ENABLE_PCL_POINTCLOUD})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

endif (${ENABLE_PCL_POINTCLOUD})

include_directories(${DRIVER_INCLUDE_DIRS})

add_executable(demo_online
    demo_online.cpp
    simple_viewer_glfw.cpp

    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp

    ${IMGUI_BACKEND}

    ${IMGUIFD_DIR}/ImGuiFileDialog.cpp
    
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_draw.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_tables.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_widgets.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_glfw.cpp

#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_win32.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl2.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/ImGuiFileDialog/ImGuiFileDialog.cpp
)

#target_link_libraries(demo_online
#    glfw
#    ${EXTERNAL_LIBS}
#)

#if(WIN32)
#    target_link_libraries(demo_online opengl32 glu32)
#endif()

target_link_libraries(demo_online
    glfw
    OpenGL::GL
    OpenGL::GLU
    ${EXTERNAL_LIBS}
)

add_executable(demo_online_multi_lidars
              demo_online_multi_lidars.cpp)

target_link_libraries(demo_online_multi_lidars
                    ${EXTERNAL_LIBS})

target_include_directories(demo_online PRIVATE
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends
    ${CMAKE_SOURCE_DIR}/thirdparty/ImGuiFileDialog
)

if(NOT ${DISABLE_PCAP_PARSE})


#add_executable(demo_pcap
#    demo_pcap.cpp
#    simple_viewer_glfw.cpp
#    
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_draw.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_tables.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/imgui_widgets.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_glfw.cpp
#
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_win32.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl2.cpp
#    ${CMAKE_SOURCE_DIR}/thirdparty/ImGuiFileDialog/ImGuiFileDialog.cpp
#)


add_executable(demo_pcap
    demo_pcap.cpp
    simple_viewer_glfw.cpp
    
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp

    ${IMGUI_BACKEND}

    ${IMGUIFD_DIR}/ImGuiFileDialog.cpp
)


target_include_directories(demo_pcap PRIVATE
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui
    ${CMAKE_SOURCE_DIR}/thirdparty/imgui/backends
    ${CMAKE_SOURCE_DIR}/thirdparty/ImGuiFileDialog
)



target_link_libraries(demo_pcap
    glfw
    OpenGL::GL
    OpenGL::GLU
    ${EXTERNAL_LIBS}
)


#target_link_libraries(demo_pcap
#    glfw
#    ${EXTERNAL_LIBS}
#)

#if(WIN32)
#    target_link_libraries(demo_pcap opengl32 glu32)
#endif()

endif(NOT ${DISABLE_PCAP_PARSE})

        

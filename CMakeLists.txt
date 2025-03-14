cmake_minimum_required(VERSION 3.16)

if(NOT DEFINED ENV{VCPKG_ROOT})
    set(VCPKG_ROOT "D:/vcpkg")
else()
    set(VCPKG_ROOT $ENV{VCPKG_ROOT})
endif()

message(STATUS "VCPKG_ROOT: $ENV{VCPKG_ROOT}")
set(CMAKE_TOOLCHAIN_FILE "${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(example LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)

# ###############################################################################
# CMake Options
# ###############################################################################

# Specify a custom install prefix path
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install CACHE STRING "Install directory used by install().")

# Generates a `compile_commands.json` that can be used for autocompletion
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE BOOL "Enable/Disable output of compile commands during generation.")

# Use ccache to speed up compilation of repeated builds
find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    message(STATUS "Enabling Ccache support")
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM} CACHE STRING "")
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM} CACHE STRING "")
endif()

if(MSVC)
    add_compile_options(/utf-8)

    # Disable verbose include file output during compilation
    add_compile_options(/nologo /MP)
endif()

# Find 3rd party packages
find_package(Eigen3 REQUIRED)
find_package(libigl CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(imguizmo CONFIG REQUIRED)
find_package(OpenCASCADE REQUIRED)
find_package(nfd CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(Netgen CONFIG REQUIRED)

add_library(imguiext
    STATIC
    imgui/ImGuiPlugin.cpp
    imgui/ImGuiMenu.cpp
    imgui/ImGuizmoWidget.cpp
    imgui/SelectionWidget.cpp)

target_link_libraries(
    imguiext PUBLIC
    imgui::imgui
    igl::igl_glfw
    igl::igl_opengl
    nfd::nfd
    spdlog::spdlog
    imguizmo::imguizmo)

add_executable(${PROJECT_NAME}
    src/IMesher.cpp
    src/OCCMesher.cpp
    src/NetgenMesher.cpp
    src/CADWidget.cpp
    src/MeshInspector.cpp
    src/main.cpp)

# After buiding, copy examples to the build directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory
    $<TARGET_FILE_DIR:${PROJECT_NAME}>/examples
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/examples
    $<TARGET_FILE_DIR:${PROJECT_NAME}>/examples
)

# Link igl (and the glfw module) to your project
target_link_libraries(${PROJECT_NAME} PUBLIC
    imguiext

    # # Other modules you could link to
    # igl::embree
    # igl::stb
    # igl::predicates
    # igl::xml
    igl_copyleft::igl_copyleft_cgal

    # igl_copyleft::comiso
    # igl_copyleft::core
    # igl_copyleft::cork
    # igl_copyleft::tetgen
    # igl_restricted::matlab
    # igl_restricted::mosek
    # igl_restricted::triangle
    ${OpenCASCADE_LIBRARIES}
    nglib
    ngcore
)

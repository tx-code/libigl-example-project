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

# Always add colored output (e.g. when using Ninja)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    add_compile_options(-fdiagnostics-color=always) # GCC
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-fcolor-diagnostics) # Clang
    add_compile_options(-ftime-trace) # Clang
endif()

if(MSVC)
    add_compile_options(/utf-8)
endif()

add_subdirectory(deps/libigl)

igl_include(glfw)
igl_include(imgui)
igl_include(opengl)
igl_include(copyleft cgal)

# Add your project files
file(GLOB SRC_FILES *.cpp)
add_executable(${PROJECT_NAME} ${SRC_FILES})

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
    igl::glfw

    # # Other modules you could link to
    # igl::embree
    igl::imgui
    igl::opengl

    # igl::stb
    # igl::predicates
    # igl::xml
    igl_copyleft::cgal

    # igl_copyleft::comiso
    # igl_copyleft::core
    # igl_copyleft::cork
    # igl_copyleft::tetgen
    # igl_restricted::matlab
    # igl_restricted::mosek
    # igl_restricted::triangle
)

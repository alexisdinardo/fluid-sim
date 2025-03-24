cmake_minimum_required(VERSION 3.10)

# Set C and C++ compilers explicitly for macOS
set(CMAKE_C_COMPILER "/usr/bin/clang")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

project(FluidSimulation LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(OpenGL REQUIRED)

# Check for CUDA support
option(USE_CUDA "Build with CUDA support" OFF)
if(USE_CUDA)
    enable_language(CUDA)
    find_package(CUDA)
    if(CUDA_FOUND)
        add_definitions(-DUSE_CUDA)
        set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -std=c++14)
        message(STATUS "CUDA support enabled")
    else()
        message(STATUS "CUDA not found, building CPU version")
        set(USE_CUDA OFF)
    endif()
endif()

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/external/glfw/include
    ${CMAKE_SOURCE_DIR}/external/glad/include
    ${CMAKE_SOURCE_DIR}/external/glm
)

# Source files
set(SOURCES
    src/main.cpp
    src/particle_system.cpp
    external/glad/src/glad.c
)

# Add CUDA files if enabled
if(USE_CUDA)
    set(SOURCES ${SOURCES} src/cuda_kernels.cu)
endif()

# Configure GLFW
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_SHARED_LIBRARY OFF CACHE BOOL "" FORCE)

# Add external libraries
add_subdirectory(external/glfw)

# Create executable
add_executable(FluidSimulation ${SOURCES})

# Link libraries
target_link_libraries(FluidSimulation
    glfw
    ${OPENGL_LIBRARIES}
)

if(USE_CUDA)
    target_link_libraries(FluidSimulation ${CUDA_LIBRARIES})
endif()

# On macOS, we need to link against Cocoa framework
if(APPLE)
    target_link_libraries(FluidSimulation "-framework Cocoa")
endif()

# Copy shader files to build directory
file(COPY ${CMAKE_SOURCE_DIR}/shaders DESTINATION ${CMAKE_BINARY_DIR})
cmake_minimum_required(VERSION 3.28)
project(EasyAssign VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(LLVM_MINGW_PATH "C:/Program Files/llvm-mingw")

message(STATUS "Building ${PROJECT_NAME} version ${PROJECT_VERSION} ${CMAKE_BUILD_TYPE}")

# Set optimization flags for release builds
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options(-O3 -march=native -flto) #--ffast-math
        add_link_options(-flto)
    endif()
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    add_compile_options(-Wno-ignored-attributes)
    add_compile_options(-Wno-unknown-pragmas)
endif()

# Ensure BUILD_SHARED_LIBS is ON for jsoncpp
set(BUILD_SHARED_LIBS ON CACHE BOOL "Build shared libraries" FORCE)

# Add external dependencies
add_subdirectory(libs/jsoncpp)
add_subdirectory(libs/zlib)

# Add libraries
add_subdirectory(libs/ConfReader)
add_subdirectory(libs/DatabaseConnection)
add_subdirectory(libs/Logger)
add_subdirectory(libs/Stopwatch)

# Add the main application
add_subdirectory(src/EasyAssign)

# Configure a header file to pass the version information to the source code
configure_file(
        "${CMAKE_SOURCE_DIR}/src/Version.h.in"
        "${CMAKE_BINARY_DIR}/generated/Version.h"
)

# Include the generated header file in your project
target_include_directories(EasyAssign PRIVATE "${CMAKE_BINARY_DIR}/generated")

# Configure the resource file to include version information
configure_file(
        "${CMAKE_SOURCE_DIR}/src/version.rc.in"
        "${CMAKE_BINARY_DIR}/generated/version.rc"
        @ONLY
)

# Add the resource file to your executable
target_sources(EasyAssign PRIVATE "${CMAKE_BINARY_DIR}/generated/version.rc")

# Access the project version
message(STATUS "Building ${PROJECT_NAME} version ${PROJECT_VERSION}")
message(STATUS "Version: ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")
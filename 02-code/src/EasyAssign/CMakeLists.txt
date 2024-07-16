cmake_minimum_required(VERSION 3.28)

project(EasyAssign)

# Define the Python paths manually
set(PYTHON_INCLUDE_DIR "C:/Users/piter/AppData/Local/Programs/Python/Python312/include")
set(PYTHON_LIBRARIES "C:/Users/piter/AppData/Local/Programs/Python/Python312/libs/python312.lib")

# Define the source files
set(SOURCES
        src/EasyAssign.cpp
        src/Assignment.cpp
        src/Matrix.cpp
        src/Network.cpp
)

# Specify the path to libpq.a (ensure this is the static version)
set(LIBPQ_LIB_PATH "${CMAKE_SOURCE_DIR}/libs/libpq/libpq.lib")
# Specify the path to PostgreSQL include directory if needed
set(POSTGRESQL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libs/libpq/include")

# Include directories
set(INCLUDE_DIRS
        include
        ../../libs/DatabaseConnection/include
        ../../libs/Logger/include
        ../../libs/Stopwatch/include
        ../../libs/boost
        ${POSTGRESQL_INCLUDE_DIR}
        ../../libs/zlib
        ${PYTHON_INCLUDE_DIR}
)

# Ensure 64-bit architecture
set(CMAKE_GENERATOR_PLATFORM x64)

# Enable static runtime linking for LLVM/Clang
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static -static-libstdc++ -static-libgcc")
endif()

# Add the executable target
add_executable(EasyAssign ${SOURCES})

# Add the shared library target for DLL
add_library(EasyAssignLib SHARED ${SOURCES})

# Add pybind11 module
# Note: `pybind11` is already added in the toplevel CMakeLists.txt

# Define preprocessor macro for static linking and building DLL
target_compile_definitions(EasyAssignLib PRIVATE BUILDING_DLL)

# Apply include directories to both targets
target_include_directories(EasyAssign PRIVATE ${INCLUDE_DIRS})
target_include_directories(EasyAssignLib PRIVATE ${INCLUDE_DIRS})

# Define preprocessor macro for static linking
add_definitions(-DPQ_STATIC)

# Link against libraries for both targets
set(LINK_LIBS
        jsoncpp_lib
        "${LIBPQ_LIB_PATH}"
        DatabaseConnection
        Logger
        Stopwatch
        zlib
        ${PYTHON_LIBRARIES}
)

target_link_libraries(EasyAssign PRIVATE ${LINK_LIBS})
target_link_libraries(EasyAssignLib PRIVATE ${LINK_LIBS})

# Define the build output directory based on the build type
set(BIN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/../../bin/${CMAKE_BUILD_TYPE})
set(TEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/../../../04-test/bbma_mib)

# Ensure the output directory exists
file(MAKE_DIRECTORY ${BIN_OUTPUT_DIR})

# Function to add post-build commands
function(add_post_build_commands target)
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:jsoncpp_lib>
            ${BIN_OUTPUT_DIR})
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:zlib>
            ${BIN_OUTPUT_DIR})
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            ${LLVM_MINGW_PATH}/bin/libc++.dll
            ${BIN_OUTPUT_DIR})
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            ${LLVM_MINGW_PATH}/bin/libunwind.dll
            ${BIN_OUTPUT_DIR})
    add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy
            $<TARGET_FILE:${target}>
            ${BIN_OUTPUT_DIR})
endfunction()

# Add post-build commands to both targets
add_post_build_commands(EasyAssign)
add_post_build_commands(EasyAssignLib)

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Function to copy binaries after build
    function(add_copy_directory_command target destination)
        add_custom_command(TARGET ${target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${BIN_OUTPUT_DIR}"
                "${CMAKE_SOURCE_DIR}/${destination}"
                COMMENT "Copying binaries to ${CMAKE_SOURCE_DIR}/${destination}"
        )
    endfunction()

    # Add copy commands for release
    add_copy_directory_command(EasyAssign "../04-test/mobspec")
    add_copy_directory_command(EasyAssign "../04-test/bbma_mib")
    add_copy_directory_command(EasyAssignLib "../04-test/mobspec")
    add_copy_directory_command(EasyAssignLib "../04-test/bbma_mib")
endif()


#project(EasyAssign)
#
#add_executable(EasyAssign
#        src/EasyAssign.cpp
#        src/Assignment.cpp
#        src/Matrix.cpp
#        src/Network.cpp
#)
#
## Specify the path to libpq.lib
#set(LIBPQ_LIB_PATH "${CMAKE_SOURCE_DIR}/libs/libpq/libpq.lib")
## Specify the path to PostgreSQL include directory if needed
#set(POSTGRESQL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libs/libpq/include")
#
## Include directories
#target_include_directories(EasyAssign PRIVATE include)
#target_include_directories(EasyAssign PRIVATE ../../libs/DatabaseConnection/include)
#target_include_directories(EasyAssign PRIVATE ../../libs/Logger/include)
#target_include_directories(EasyAssign PRIVATE ../../libs/Stopwatch/include)
#target_include_directories(EasyAssign PRIVATE ../../libs/boost)
#target_include_directories(EasyAssign PRIVATE ${POSTGRESQL_INCLUDE_DIR})
#target_include_directories(EasyAssign PRIVATE ../../libs/zlib)
#
## Link against libraries
#target_link_libraries(EasyAssign PRIVATE jsoncpp_lib)
#target_link_libraries(EasyAssign PRIVATE "${LIBPQ_LIB_PATH}")
#target_link_libraries(EasyAssign PRIVATE DatabaseConnection)
#target_link_libraries(EasyAssign PRIVATE Logger)
#target_link_libraries(EasyAssign PRIVATE Stopwatch)
#target_link_libraries(EasyAssign PRIVATE zlib)
#
## Define the build output directory based on the build type
#set(BIN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/../../bin/${CMAKE_BUILD_TYPE})
#set(TEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/../../../04-test/bbma_mib)
#
## Ensure the output directory exists
#file(MAKE_DIRECTORY ${BIN_OUTPUT_DIR})
#
## Copy necessary DLLs to the output directory after build
#add_custom_command(TARGET EasyAssign POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy_if_different
#        $<TARGET_FILE:jsoncpp_lib>
#        ${BIN_OUTPUT_DIR})
#
## Copy the zlib DLL to the output directory
#add_custom_command(TARGET EasyAssign POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#        $<TARGET_FILE:zlib>
#        ${BIN_OUTPUT_DIR})
#
#add_custom_command(TARGET EasyAssign POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#        ${LLVM_MINGW_PATH}/bin/libc++.dll
#        ${BIN_OUTPUT_DIR})
#
#add_custom_command(TARGET EasyAssign POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#        ${LLVM_MINGW_PATH}/bin/libunwind.dll
#        ${BIN_OUTPUT_DIR})
#
## Copy the main application executable to the output directory
#add_custom_command(TARGET EasyAssign POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#        $<TARGET_FILE:EasyAssign>
#        ${BIN_OUTPUT_DIR})
#
#if(CMAKE_BUILD_TYPE STREQUAL "Release")
#    # Custom command to copy binaries after build
#    add_custom_command(TARGET EasyAssign POST_BUILD
#            COMMAND ${CMAKE_COMMAND} -E copy_directory
#            "${BIN_OUTPUT_DIR}"
#            "${CMAKE_SOURCE_DIR}/../04-test/mobspec"
#            COMMENT "Copying binaries to ${CMAKE_SOURCE_DIR}/../04-test/mobspec"
#    )
#    add_custom_command(TARGET EasyAssign POST_BUILD
#            COMMAND ${CMAKE_COMMAND} -E copy_directory
#            "${BIN_OUTPUT_DIR}"
#            "${CMAKE_SOURCE_DIR}/../04-test/bbma_mib"
#            COMMENT "Copying binaries to ${CMAKE_SOURCE_DIR}/../04-test/bbma_mib"
#    )
#endif()
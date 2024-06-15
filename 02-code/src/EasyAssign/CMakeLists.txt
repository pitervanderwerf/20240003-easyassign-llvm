project(EasyAssign)

#add_executable(EasyAssign src/EasyAssign.cpp)

add_executable(EasyAssign
        src/EasyAssign.cpp
        src/Assignment.cpp
        src/Matrix.cpp
        src/Network.cpp
)


# Specify the path to libpq.lib
set(LIBPQ_LIB_PATH "${CMAKE_SOURCE_DIR}/libs/libpq/libpq.lib")
# Specify the path to PostgreSQL include directory if needed
set(POSTGRESQL_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/libs/libpq/include")

# Include directories
target_include_directories(EasyAssign PRIVATE include)
target_include_directories(EasyAssign PRIVATE ../../libs/DatabaseConnection/include)
target_include_directories(EasyAssign PRIVATE ../../libs/Logger/include)
target_include_directories(EasyAssign PRIVATE ../../libs/Stopwatch/include)
target_include_directories(EasyAssign PRIVATE ../../libs/boost)
target_include_directories(EasyAssign PRIVATE ${POSTGRESQL_INCLUDE_DIR})
target_include_directories(EasyAssign PRIVATE ../../libs/zlib)

# Link against libraries
target_link_libraries(EasyAssign PRIVATE jsoncpp_lib)
target_link_libraries(EasyAssign PRIVATE "${LIBPQ_LIB_PATH}")
target_link_libraries(EasyAssign PRIVATE DatabaseConnection)
target_link_libraries(EasyAssign PRIVATE Logger)
target_link_libraries(EasyAssign PRIVATE Stopwatch)
target_link_libraries(EasyAssign PRIVATE zlib)

# Define the build output directory based on the build type
set(BIN_OUTPUT_DIR ${CMAKE_BINARY_DIR}/../../bin/${CMAKE_BUILD_TYPE})
set(TEST_OUTPUT_DIR ${CMAKE_BINARY_DIR}/../../../04-test/bbma_mib)

# Ensure the output directory exists
file(MAKE_DIRECTORY ${BIN_OUTPUT_DIR})

# Copy necessary DLLs to the output directory after build
add_custom_command(TARGET EasyAssign POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:jsoncpp_lib>
        ${BIN_OUTPUT_DIR})

# Copy the zlib DLL to the output directory
add_custom_command(TARGET EasyAssign POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:zlib>
        ${BIN_OUTPUT_DIR})

add_custom_command(TARGET EasyAssign POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${LLVM_MINGW_PATH}/bin/libc++.dll
        ${BIN_OUTPUT_DIR})

add_custom_command(TARGET EasyAssign POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        ${LLVM_MINGW_PATH}/bin/libunwind.dll
        ${BIN_OUTPUT_DIR})

# Copy the main application executable to the output directory
add_custom_command(TARGET EasyAssign POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:EasyAssign>
        ${BIN_OUTPUT_DIR})

if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Custom command to copy binaries after build
    add_custom_command(TARGET EasyAssign POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
            "${BIN_OUTPUT_DIR}"
            "${CMAKE_SOURCE_DIR}/../04-test/mobspec"
            COMMENT "Copying binaries to ${CMAKE_SOURCE_DIR}/../04-test/bbma_mib"
    )
endif()
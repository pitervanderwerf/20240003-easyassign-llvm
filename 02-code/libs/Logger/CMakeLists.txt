project(Logger)

add_library(Logger STATIC
        src/Logger.cpp
)

target_include_directories(Logger PUBLIC include)
target_include_directories(Logger PUBLIC ../Config/include)
target_include_directories(Logger PUBLIC ../ConfReader/include)

target_link_libraries(Logger PUBLIC Config)
target_link_libraries(Logger PUBLIC ConfReader)
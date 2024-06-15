project(Stopwatch)

add_library(Stopwatch STATIC
        src/Stopwatch.cpp
)

target_include_directories(Stopwatch PUBLIC include)
target_include_directories(Stopwatch PUBLIC ../Logger/include)
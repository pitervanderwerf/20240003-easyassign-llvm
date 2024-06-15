project(Config)

add_library(Config STATIC
        src/Config.cpp
)

add_library(ConfReader STATIC
        src/ConfReader.cpp
)

target_include_directories(Config PUBLIC include)
target_include_directories(Config PUBLIC ../jsoncpp/include)
target_include_directories(Config PUBLIC ../Logger/include)
target_include_directories(ConfReader PUBLIC include)
target_include_directories(ConfReader PUBLIC ../jsoncpp/include)

target_link_libraries(Config PRIVATE jsoncpp_lib)
target_link_libraries(Config PRIVATE Logger)
target_link_libraries(ConfReader PRIVATE jsoncpp_lib)
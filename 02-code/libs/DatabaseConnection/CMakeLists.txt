project(DatabaseConnection)

add_library(DatabaseConnection STATIC
        src/DatabaseConnection.cpp
)

target_include_directories(DatabaseConnection PUBLIC include)
target_include_directories(DatabaseConnection PUBLIC ../Config/include)
target_include_directories(DatabaseConnection PUBLIC ../ConfReader/include)
target_include_directories(DatabaseConnection PUBLIC ../libpq/include)
target_include_directories(DatabaseConnection PUBLIC ../Logger/include)

target_link_libraries(DatabaseConnection PUBLIC Config)
target_link_libraries(DatabaseConnection PUBLIC ConfReader)
target_link_libraries(DatabaseConnection PUBLIC Logger)
#pragma once

#include <string>
#include "libpq-fe.h"
#include "ConfReader.h"

class DatabaseConnection {
public:
    DatabaseConnection();
    DatabaseConnection(const ConfReader& conf_reader);
    ~DatabaseConnection();

    bool connect();
    bool isConnected() const;
    PGconn* getConnection() const;
    PGresult* executeQuery(const char* query);
    bool executeFile(const std::string& file_path);

    bool checkAndConnect();
    void printPGresult(PGresult* result);

    void exampleUsage();

private:
    PGconn* connection;
    const ConfReader& conf_reader;
    std::string connection_string;
};
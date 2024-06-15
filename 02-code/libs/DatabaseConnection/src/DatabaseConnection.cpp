#include <format>
#include <fstream>

#include "Config.h"
#include "ConfReader.h"
#include "DatabaseConnection.h"
#include "Logger.h"


DatabaseConnection::DatabaseConnection() : conf_reader(conf_reader) {
    connection_string = std::format("user={} password={} dbname={} port={}",
        Config::PGUSER, Config::PGPASSWORD, Config::PGDATABASE, Config::PGPORT);
    connection = nullptr;
    connect();
}

DatabaseConnection::DatabaseConnection(const ConfReader& conf_reader) : conf_reader(conf_reader) {
    // Read connection parameters from the configuration file
    const Json::Value& config = conf_reader.getConfig();
    const Json::Value& cp = config["pg_connection_params"];

    connection_string = std::format("user={} password={} dbname={} port={}",
        cp["PGUSER"].asString(), cp["PGPASSWORD"].asString(),
        cp["PGDATABASE"].asString(), cp["PGPORT"].asString());
    connection = nullptr;
    connect();
}

DatabaseConnection::~DatabaseConnection() {
    if (connection != nullptr) {
        PQfinish(connection);
    }
}

bool DatabaseConnection::connect() {
    connection = PQconnectdb(connection_string.c_str());
    return (PQstatus(connection) == CONNECTION_OK);
}

bool DatabaseConnection::isConnected() const {
    return (connection != nullptr) && (PQstatus(connection) == CONNECTION_OK);
}

PGconn* DatabaseConnection::getConnection() const {
    return connection;
}

PGresult* DatabaseConnection::executeQuery(const char* query) {
    if (!isConnected()) {
        Logger::log("Connection to database not established");
        return nullptr;
    }

    return PQexec(connection, query);
}

bool DatabaseConnection::executeFile(const std::string& file_path) {
    if (!isConnected()) {
        Logger::log("Connection to database not established");
        return false;
    }

    std::ifstream fileStream(file_path);
    if (!fileStream.is_open()) {
        Logger::log("Failed to open SQL file : " + file_path);
        return false;
    }

    std::string sqlContent((std::istreambuf_iterator<char>(fileStream)),
        std::istreambuf_iterator<char>());

    PGresult* result = PQexec(connection, sqlContent.c_str());
    ExecStatusType status = PQresultStatus(result);

    if (status != PGRES_COMMAND_OK && status != PGRES_TUPLES_OK) {
        Logger::log("SQL execution failed: " + std::string(PQerrorMessage(connection)));
        PQclear(result);
        return false;
    }

    PQclear(result);
    return true;
}

bool DatabaseConnection::checkAndConnect() {
    if (!connect()) {
        Logger::log("Failed to connect to the database");
        return false;
    }
    return true;
}

void DatabaseConnection::printPGresult(PGresult* result) {
    if (result == nullptr) {
        Logger::log("Query execution failed");
        return;
    }

    ExecStatusType status = PQresultStatus(result);

    if (status == PGRES_TUPLES_OK) { // Check if the query returned rows
        int numRows = PQntuples(result);
        int numColumns = PQnfields(result);

        for (int row = 0; row < numRows; ++row) {
            for (int col = 0; col < numColumns; ++col) {
                const char* value = PQgetvalue(result, row, col);
                Logger::log(std::string(value) + "\t");
            }
            Logger::log("");
        }
    }
    else if (status == PGRES_COMMAND_OK) { // Check if the query was a command (e.g., INSERT, UPDATE)
        Logger::log("Query executed successfully");
    }
    else {
        Logger::log("Query execution failed with status: " + std::string(PQresStatus(status)));
    }

    PQclear(result); // Free the memory associated with the result
}

void DatabaseConnection::exampleUsage() {
    if (this->checkAndConnect()) {
        Logger::log("PG connected successfully!");
    }
    PGresult* result = this->executeQuery("SELECT version();");
    this->printPGresult(result);
}
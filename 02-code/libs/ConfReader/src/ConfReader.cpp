#include <fstream>
#include <iostream>
#include <string>

#include "json/json.h"

#include "ConfReader.h"

ConfReader::ConfReader() {
    std::string filename = "config.json";
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    try {
        file >> config_file;
    }
    catch (const Json::Exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return;
    }
}

ConfReader::ConfReader(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    try {
        file >> config_file;
    }
    catch (const Json::Exception& e) {
        std::cerr << "Error parsing JSON: " << e.what() << std::endl;
        return;
    }
}

const Json::Value& ConfReader::getConfig() const {
    return config_file;
}

const Json::Value& ConfReader::getPgConnectionParams() const {
    return config_file["pg_connection_params"];
}

const Json::Value& ConfReader::operator[](const std::string& key) const {
    if (config_file.isMember(key)) {
        const Json::Value& value = config_file[key];
        return value;
    }
    else {
        static const Json::Value emptyJson;
        return emptyJson;
    }
}
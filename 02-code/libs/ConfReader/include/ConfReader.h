#pragma once
#include <string>
#include <json/json.h>

class ConfReader {
public:
    ConfReader();
    ConfReader(const std::string& filename);

    const Json::Value& getConfig() const;
    const Json::Value& getPgConnectionParams() const;
    const Json::Value& operator[](const std::string& key) const;

private:
    Json::Value config_file;
};

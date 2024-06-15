#include "Config.h"

#include <iomanip>

#include "json/json.h"

#include "ConfReader.h"
#include "Logger.h"


// READ CONFIGURATION FILE
ConfReader confReader;
const Json::Value& config = confReader.getConfig();
const Json::Value& project_settings = config["project_settings"];
const Json::Value& db_settings = config["db_settings"];
const Json::Value& system_settings = config["system_settings"];
const Json::Value& model_settings = config["model_settings"];
const Json::Value& assignment_settings = config["assignment_settings"];

// SET PROJECT SETTINGS
const bool Config::LOG = project_settings["LOG"].asBool();

// SET DB SETTINGS
const std::string Config::PGDATABASE = db_settings["PGDATABASE"].asString();
const std::string Config::PGHOST = db_settings["PGHOST"].asString();
const std::string Config::PGPORT = db_settings["PGPORT"].asString();
const std::string Config::PGUSER = db_settings["PGUSER"].asString();
const std::string Config::PGPASSWORD = db_settings["PGPASSWORD"].asString();

// SET SYSTEM SETTINGS
const int Config::MAX_THREADS = system_settings["MAX_THREADS"].asInt();

// SET MODEL SETTINGS
const int Config::NUMBER_OF_CENTROIDS = model_settings["NUMBER_OF_CENTROIDS"].asInt();
const double Config::INSIGNIFICANT_OD_PAIRS_THRESHOLD = model_settings["INSIGNIFICANT_OD_PAIRS_THRESHOLD"].asDouble();
const double Config::VALUE_OF_TIME = model_settings["VALUE_OF_TIME"].asDouble();

// SET ASSIGNMENT SETTINGS
const std::string Config::ASSIGNMENT_METHOD = assignment_settings["ASSIGNMENT_METHOD"].asString();
const std::string Config::LEVEL_OF_DETAIL = assignment_settings["LEVEL_OF_DETAIL"].asString();
const int Config::NUMBER_OF_ITERATIONS = assignment_settings["NUMBER_OF_ITERATIONS"].asInt();
const bool Config::JUNCTION_MODELLING = assignment_settings["JUNCTION_MODELLING"].asBool();
const double Config::EQ_JUNCTION_COST = assignment_settings["EQ_JUNCTION_COST"].asDouble();
const bool Config::ASSIGN_FLOWS = assignment_settings["ASSIGN_FLOWS"].asBool();
const bool Config::CREATE_SKIMS = assignment_settings["CREATE_SKIMS"].asBool();
const bool Config::PRINT_PATHS = assignment_settings["PRINT_PATHS"].asBool();


// Function template to left-align and pad any type to a specified width
template<typename T>
std::string alignLeft(const std::string& label, int width, const T& value) {
    std::ostringstream oss;
    oss << std::left << std::setw(width) << label << value;
    return oss.str();
}

void Config::printConfigValues() {
    Logger::log("-----------------------------------------------------------------------------------------------------------");
    Logger::log("--- CONFIGURATION ///");
    Logger::log("-----------------------------------------------------------------------------------------------------------");

    constexpr int columnWidth = 40; // Adjust the width as needed
    Logger::log("\n\t// Project settings");
    Logger::log(alignLeft("\tLOG:", columnWidth, Config::LOG ? "true" : "false"));

    Logger::log("\n\t// Database settings");
    Logger::log(alignLeft("\tPGDATABASE:", columnWidth, Config::PGDATABASE));
    Logger::log(alignLeft("\tPGHOST:", columnWidth, Config::PGHOST));
    Logger::log(alignLeft("\tPGPORT:", columnWidth, Config::PGPORT));
    Logger::log(alignLeft("\tPGUSER:", columnWidth, Config::PGUSER));
    Logger::log(alignLeft("\tPGPASSWORD:", columnWidth, Config::PGPASSWORD));

    Logger::log("\n\t// System settings");
    Logger::log(alignLeft("\tMAX_THREADS:", columnWidth, std::to_string(Config::MAX_THREADS)));

    Logger::log("\n\t// Model settings");
    Logger::log(alignLeft("\tNUMBER_OF_CENTROIDS:", columnWidth, std::to_string(Config::NUMBER_OF_CENTROIDS)));
    Logger::log(alignLeft("\tINSIGNIFICANT_OD_PAIRS_THRESHOLD:", columnWidth, std::to_string(Config::INSIGNIFICANT_OD_PAIRS_THRESHOLD)));
    Logger::log(alignLeft("\tVALUE_OF_TIME:", columnWidth, std::to_string(Config::VALUE_OF_TIME)));

    Logger::log("\n\t// Assignment settings");
    Logger::log(alignLeft("\tASSIGNMENT_METHOD:", columnWidth, Config::ASSIGNMENT_METHOD));
    Logger::log(alignLeft("\tLEVEL_OF_DETAIL:", columnWidth, Config::LEVEL_OF_DETAIL));
    Logger::log(alignLeft("\tNUMBER_OF_ITERATIONS:", columnWidth, std::to_string(Config::NUMBER_OF_ITERATIONS)));
    Logger::log(alignLeft("\tJUNCTION_MODELLING:", columnWidth, Config::JUNCTION_MODELLING ? "true" : "false"));
    Logger::log(alignLeft("\tEQ_JUNCTION_COST:", columnWidth, std::to_string(Config::EQ_JUNCTION_COST)));
    Logger::log(alignLeft("\tASSIGN_FLOWS:", columnWidth, Config::ASSIGN_FLOWS ? "true" : "false"));
    Logger::log(alignLeft("\tCREATE_SKIMS:", columnWidth, Config::CREATE_SKIMS ? "true" : "false"));
    Logger::log(alignLeft("\tPRINT_PATHS:", columnWidth, Config::PRINT_PATHS ? "true" : "false"));

    Logger::log("");
}

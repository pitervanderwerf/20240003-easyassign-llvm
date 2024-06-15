#include "Logger.h"

#include <filesystem>

#include "Config.h"

Logger& Logger::getInstance() {
    static Logger instance; // Singleton pattern
    return instance;
}

void Logger::log(const std::string& message) {
    Logger& logger = getInstance();

    std::lock_guard<std::mutex> lock(logger.mutex); 
    std::cout << message << "\n";

    if (logger.writeToLogFile) {
        logger.logToFile(message);
    }
}

Logger::Logger() : writeToLogFile(Config::LOG) {
    std::filesystem::create_directory("results");
    (Config::LOG) ? logFile.open("results/easyassign.log") : void();
}

Logger::~Logger() {
    if (writeToLogFile) {
        logFile.close();
    }
}

void Logger::logToFile(const std::string& message) {
    if (logFile.is_open()) {
        logFile << message << std::endl;
    }
}

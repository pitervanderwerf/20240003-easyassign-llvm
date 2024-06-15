// Logger.h
#pragma once

#include <iostream>
#include <fstream>
#include <mutex>

class Logger {
public:
    static void log(const std::string& message);

private:
    static Logger& getInstance();

    Logger();
    ~Logger();

    void logToFile(const std::string& message);

private:
    bool writeToLogFile;
    std::ofstream logFile;
    std::mutex mutex;
};

#include "Stopwatch.h"
#include "Logger.h"

#include <ctime>
#include <iomanip>
#include <chrono>
#include <sstream>

Stopwatch::Stopwatch(const std::string& process_name) : process_name(process_name) {}

void Stopwatch::start() {
    start_time = std::chrono::high_resolution_clock::now();
    last_checkpoint = start_time;

    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::tm tm_buffer;
    localtime_s(&tm_buffer, &now);

    Logger::log("-----------------------------------------------------------------------------------------------------------");
    Logger::log("--- " + process_name + " STARTED /// " + convertTimeToString(&tm_buffer));
    Logger::log("-----------------------------------------------------------------------------------------------------------");
}

void Stopwatch::checkpoint(const std::string& checkpointName) {
    auto now = std::chrono::high_resolution_clock::now();
    double elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_checkpoint).count() / 1000.0;
    last_checkpoint = now;

    Logger::log("-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -  ");
    Logger::log("--- Checkpoint [" + checkpointName + "]: " + std::to_string(elapsed_time) + " seconds");
    Logger::log("-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -  ");
}

void Stopwatch::finish() {
    auto end_time = std::chrono::high_resolution_clock::now();
    std::string total_time = doubleToStringWithPrecision(std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0, 2);

    auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

    std::tm tm_buffer;
    localtime_s(&tm_buffer, &now);

    Logger::log("-----------------------------------------------------------------------------------------------------------");
    Logger::log("--- " + process_name + " FINISHED IN " + total_time + " seconds /// " + convertTimeToString(&tm_buffer));
    Logger::log("-----------------------------------------------------------------------------------------------------------");
}

std::string Stopwatch::convertTimeToString(const std::tm* timeInfo) {
    std::stringstream ss;
    ss << std::put_time(timeInfo, "%d-%m-%Y %H:%M:%S");
    return ss.str();
}

std::string Stopwatch::doubleToStringWithPrecision(double value, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}
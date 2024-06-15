#pragma once

#include <chrono>
#include <string>

class Stopwatch {
public:
    Stopwatch(const std::string& processName);

    void start();
    void checkpoint(const std::string& checkpointName);
    void finish();

    std::string convertTimeToString(const std::tm* timeInfo);
    std::string doubleToStringWithPrecision(double value, int precision);

private:
    std::string process_name;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_checkpoint;
};


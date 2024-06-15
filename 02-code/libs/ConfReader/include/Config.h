#pragma once

#include <string>

namespace Config {
	// Database settings
	extern const std::string PGDATABASE;
	extern const std::string PGHOST;
	extern const std::string PGPORT;
	extern const std::string PGUSER;
	extern const std::string PGPASSWORD;

	// System settings
	extern const int MAX_THREADS;

	// Model settings
	extern const int NUMBER_OF_CENTROIDS;
	extern const double INSIGNIFICANT_OD_PAIRS_THRESHOLD;
    extern const double SIGNIFICANT_SOURCE_PRODUCTION;
	extern const double VALUE_OF_TIME;

	// Assignment settings
	extern const std::string ASSIGNMENT_METHOD;
	extern const std::string LEVEL_OF_DETAIL;
	extern const int NUMBER_OF_ITERATIONS;
	extern const bool JUNCTION_MODELLING;
	extern const double EQ_JUNCTION_COST; // In minutes, so: 0.05 minute == 3 seconds
	extern const bool ASSIGN_FLOWS;
	extern const bool CREATE_SKIMS;
	extern const bool PRINT_PATHS;
	
	// Project settings
	extern const bool LOG;

	void printConfigValues();
}
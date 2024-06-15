#include <iostream>

#include "Assignment.h"
#include "Config.h"
#include "DatabaseConnection.h"
#include "Network.h"
#include "Matrix.h"
#include "Stopwatch.h"

int main()
{
    Config::printConfigValues();

    // Test Assignment
    Assignment my_assignment({ 1,11,1,1,{ 1,2,3 },1 }); // Mobspec
//    Assignment my_assignment({ 1,2,1,4,{ 1,2,3 },1 }); // BBMA MiB

    if (Config::LEVEL_OF_DETAIL == "C2C") {
        my_assignment.execute();
    }
//    else if (Config::LEVEL_OF_DETAIL == "PC5") {
//        my_assignment.executePc5(35357);
//    }
//    else if (Config::LEVEL_OF_DETAIL == "PC6") {
//        my_assignment.executePc6(462369);
//    }


    //// Test Network
    //Network network;
    //network.fetchLinksFromDatabase();
    //network.printNetwork();
    //network.printLinksToFile("network_links2.txt");

    //// Test Config
    //std::cout << Config::PGDATABASE << std::endl;
    //std::cout << Config::NUMBER_OF_CENTROIDS << std::endl;
    //
    //DatabaseConnection dbConnection;
    //dbConnection.exampleUsage();

    //// Read Matrix.data from OD-list in DB
    //Matrix myBaseMatrix({ 1, 11, 1, 1, 1, 1 }, "odm");
    //Matrix myBaseMatrix({ 1, 2, 3, 371, 1, 1 }, "odm");
    //myBaseMatrix.getOdMatrixFromDatabaseList();
    //myBaseMatrix.writeToBinaryFile();
    //myBaseMatrix.writeMatrixToTextFile("testmat.txt");

    //myBaseMatrix.getOdMatrix();
    //myBaseMatrix.writeToBinaryFile();
    //myBaseMatrix.readFromBinaryFile();
    //myBaseMatrix.writeMatrixToTextFile(myBaseMatrix.generateMatrixFileName("txt").c_str(), myBaseMatrix.getData(), myBaseMatrix.getRows(), myBaseMatrix.getCols());

    //// Test Matrix
    //Stopwatch stopwatch("Exporting matrix");
    //stopwatch.start();

    //Matrix myBaseMatrix({ 1, 11, 1, 1 });
    //myBaseMatrix.exampleUsage();

    //stopwatch.checkpoint("Setting up filled matrix");

    //myBaseMatrix.writeToBinaryFile();

    //stopwatch.checkpoint("Matrix written to compressed binary");

    //myBaseMatrix.writeMatrixToTextFile(myBaseMatrix.generateMatrixFileName("txt").c_str(), myBaseMatrix.readFromBinaryFile(), myBaseMatrix.getRows(), myBaseMatrix.getCols());

    //stopwatch.finish();

    std::cout << "\n\n" << "Press any key to close...";
    std::cin.get();

    return 0;
}


//#include "Config.h"
//#include "DatabaseConnection.h"
//#include "Logger.h"
//#include "Matrix.h"
//#include "Version.h"
//
//#include <string>
//
//int main() {
//    Config::printConfigValues();
////    DatabaseConnection connection = DatabaseConnection();
////    connection.checkAndConnect();
////    PGresult * res = connection.executeQuery("SELECT * FROM link_flows LIMIT 10");
////    connection.printPGresult(res);
//
//    Logger::log("App version: " + std::string(EASYASSIGN_VERSION));
//
//    Matrix myBaseMatrix({ 1, 2, 3, 371, 1, 1 }, "odm");
//    myBaseMatrix.getOdMatrixFromDatabaseList();
//    myBaseMatrix.writeToBinaryFile();
//    myBaseMatrix.writeMatrixToTextFile("testmat.txt");
//
//    return 0;
//}

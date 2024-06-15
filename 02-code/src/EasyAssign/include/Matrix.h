#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>

#include "DatabaseConnection.h"

class Matrix {
public:
    // Constructor
    Matrix(const std::vector<int>& pmturi, std::string matrixType);

    // Getter functions
    int getRows() const;
    int getCols() const;
    double getMatrixSum() const;
    const std::vector<int>& getPmturi() const;
    const std::vector<double>& getData() const;
    const std::string getMatrixPrettyName() const;

    // Accessor and Mutator functions
    double& at(int row, int col);
    const double& at(int row, int col) const;
    void setPmturi(const std::vector<int>& newPmturi);
    void setMatrixSum(double value);
    void incrementMatrixSum(double value);
    void resetMatrix();

    double sumRow(int row) const;
    void createSumRowMap();
    void printAllRowSums() const;

    // Write to file
    [[nodiscard]] std::string generateMatrixFileName(std::string ext) const;
    int writeToBinaryFile() const;
    void writeMatrixToTextFile(const char* filename) const;

    std::vector<double>  readFromBinaryFile() const;

    // Helper function to read OD-list from Database
    PGresult* getOdListFromDatabase();
    void getOdMatrixFromDatabaseList();
    void setOdMatrixFromBinary();

    // Print function
    void print() const;
    void exampleUsage();

    void printMatrixSum() const;
    std::map<int, double> row_sum_map;

private:
    int rows;
    int cols;
    long double matrix_sum;

    std::vector<double> data;
    std::vector<int> pmturi;
    std::string matrix_type;
};
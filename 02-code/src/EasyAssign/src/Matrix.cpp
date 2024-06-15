#include <algorithm>
#include <iostream>
#include <malloc.h>
#include <cstdio>
#include <vector>

#include <zlib.h>

#include "Config.h"
#include "DatabaseConnection.h"
#include "Logger.h"
#include "Matrix.h"
#include "Stopwatch.h"

// Constructor
Matrix::Matrix(const std::vector<int>& pmturi, std::string matrix_type)
    : rows(Config::NUMBER_OF_CENTROIDS),
    cols(Config::NUMBER_OF_CENTROIDS),
    matrix_sum(0),
    pmturi(pmturi),
    matrix_type(matrix_type),
    data(rows* cols) {} // Single vector to hold all elements

// Getter functions
int Matrix::getRows() const {
    return rows;
}

int Matrix::getCols() const {
    return cols;
}

double Matrix::getMatrixSum() const {
    return matrix_sum;
}

const std::vector<int>& Matrix::getPmturi() const {
    return pmturi;
}

const std::vector<double>& Matrix::getData() const {
    return data;
}


const std::string Matrix::getMatrixPrettyName() const {
    // Convert matrix_type to uppercase
    std::string uppercaseType = matrix_type;
    std::transform(uppercaseType.begin(), uppercaseType.end(), uppercaseType.begin(), [](unsigned char c) {
        return std::toupper(c);
        });

    // Format pmturi as [p, m, t, u, r, i]
    std::ostringstream oss;
    oss << uppercaseType << " [";
    for (size_t i = 0; i < pmturi.size(); ++i) {
        oss << pmturi[i];
        if (i < pmturi.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";

    return oss.str();
}

// Accessor and Mutator functions
double& Matrix::at(int row, int col) {
    return data[row * cols + col]; // Calculate index for 1D vector
}

// Const version of at
const double& Matrix::at(int row, int col) const {
    return data[row * cols + col];
}

void Matrix::setPmturi(const std::vector<int>& newPmturi) {
    pmturi = newPmturi;
}

void Matrix::setMatrixSum(double value) {
    matrix_sum = value;
}

void Matrix::incrementMatrixSum(double value) {
    matrix_sum += value;
}

double Matrix::sumRow(int row) const {
    if (row < 0 || row >= rows) {
        throw std::out_of_range("Row index out of range");
    }
    double sum = 0.0;
    for (int col = 0; col < cols; ++col) {
        sum += at(row, col);
    }
    return sum;
}

void Matrix::printAllRowSums() const {
    for (int row = 0; row < rows; ++row) {
        Logger::log(std::to_string(row) + ": " + std::to_string(sumRow(row)));
    }
}

void Matrix::resetMatrix() {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            //data[i][j] = 0.0;
            this->at(i, j) = 0.0;
        }
    }
}

// Write to file functions
std::string Matrix::generateMatrixFileName(std::string ext) const {
    if (matrix_type == "odm") {
        return "matrix_" + std::to_string(pmturi[0]) + "_" + std::to_string(pmturi[1]) + "_" +
            std::to_string(pmturi[2]) + "_" + std::to_string(pmturi[3]) + "." + ext;
    }
    if (matrix_type == "skm") {
        return "matrix_" + std::to_string(pmturi[0]) + "_" + std::to_string(pmturi[1]) + "_" +
            std::to_string(pmturi[2]) + "_" + std::to_string(pmturi[3]) + "_" + std::to_string(pmturi[4]) + "_" + std::to_string(pmturi[5]) + "." + ext;
    }
}

int Matrix::writeToBinaryFile() const {
    Stopwatch stopwatch("WRITING " + getMatrixPrettyName() + " MATRIX TO BINARY FILE");
    stopwatch.start();

    // Use a flat list to represent the matrix
    double* matrix = (double*)malloc(rows * cols * sizeof(double));

    if (matrix == nullptr) {
        Logger::log("Memory allocation for matrix failed");
        return 1;
    }

    // Initialize the matrix with values
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            matrix[i * cols + j] = this->at(i, j);
        }
    }

    // Placeholder for the compressed version of the matrix
    unsigned char* compressedMatrix = nullptr;
    uLong compressedSize = 0;

    // Compress the matrix
    z_stream defstream;
    defstream.zalloc = Z_NULL;
    defstream.zfree = Z_NULL;
    defstream.opaque = Z_NULL;

    // Input: matrix, Output: compressedMatrix
    defstream.avail_in = rows * cols * sizeof(double);
    defstream.next_in = (Bytef*)matrix;

    // Calculate the maximum possible compressed size and allocate the buffer
    uLong maxCompressedSize = compressBound(defstream.avail_in);
    compressedMatrix = (unsigned char*)malloc(maxCompressedSize);

    if (compressedMatrix == nullptr) {
        Logger::log("Memory allocation for compressedMatrix failed");
        free(matrix);
        return 1;
    }

    defstream.avail_out = maxCompressedSize;
    defstream.next_out = compressedMatrix;

    deflateInit(&defstream, Z_BEST_SPEED);
    deflate(&defstream, Z_FINISH);
    compressedSize = defstream.total_out;
    deflateEnd(&defstream);

    // Write the compressed matrix to a binary file
    FILE* compressedFile;
    if (fopen_s(&compressedFile, generateMatrixFileName(matrix_type).c_str(), "wb") == 0) {
        fwrite(compressedMatrix, 1, compressedSize, compressedFile);
        fclose(compressedFile);
    }
    else {
        Logger::log("Failed to open the binary file for writing");
        free(compressedMatrix);
        free(matrix);
        return 1;
    }

    free(compressedMatrix);
    free(matrix);

    stopwatch.finish();
    return 0;
}

void Matrix::writeMatrixToTextFile(const char* filename) const {
    Stopwatch stopwatch("WRITING " + getMatrixPrettyName() + " TO ASCII TEXT FILE");
    stopwatch.start();

    FILE* file;
    if (fopen_s(&file, filename, "w") == 0) {
        for (int i = 0; i < getRows(); i++) {
            for (int j = 0; j < getCols(); j++) {
                fprintf(file, "%f;", this->at(i, j));
            }
            fprintf(file, "\n");
        }
        fclose(file);
        Logger::log("Matrix data written to " + std::string(filename));
    }
    else {
        Logger::log("Failed to open the text file for writing");
    }

    stopwatch.finish();
}

// Read functions
std::vector<double> Matrix::readFromBinaryFile() const {
    Stopwatch stopwatch("READING " + getMatrixPrettyName() + " FROM BINARY FILE");
    stopwatch.start();

    std::vector<double> decompressedMatrix(rows * cols, 0.0); // Flat structure

    FILE* decompressedFile;
    if (fopen_s(&decompressedFile, generateMatrixFileName(matrix_type).c_str(), "rb") == 0) {
        fseek(decompressedFile, 0, SEEK_END);
        long fileSize = ftell(decompressedFile);
        fseek(decompressedFile, 0, SEEK_SET);

        unsigned char* compressedMatrix = (unsigned char*)malloc(fileSize);
        if (compressedMatrix == nullptr) {
            Logger::log("Memory allocation for compressedMatrix failed");
            fclose(decompressedFile);
            return decompressedMatrix;
        }

        fread(compressedMatrix, 1, fileSize, decompressedFile);
        fclose(decompressedFile);

        z_stream infstream;
        infstream.zalloc = Z_NULL;
        infstream.zfree = Z_NULL;
        infstream.opaque = Z_NULL;
        infstream.avail_in = fileSize;
        infstream.next_in = compressedMatrix;
        infstream.avail_out = rows * cols * sizeof(double);
        infstream.next_out = reinterpret_cast<Bytef*>(decompressedMatrix.data());

        inflateInit(&infstream);
        inflate(&infstream, Z_NO_FLUSH);
        inflateEnd(&infstream);

        free(compressedMatrix);
    }
    else {
        Logger::log("Failed to open the binary file for reading");
    }

    stopwatch.finish();
    return decompressedMatrix;
}


// Helper functions to fetch OD-list from Database
PGresult* Matrix::getOdListFromDatabase() {
    DatabaseConnection conn;
    Logger::log("Reading matrixdata...");
    std::string sql = "SELECT from_zone, to_zone, trips FROM odm_1_2_3_371 ORDER BY from_zone ASC;";
    //std::string sql = "SELECT from_zone, to_zone, trips FROM odm.odm_1_11_1_1 ORDER BY from_zone ASC;";
    //std::string sql = "SELECT from_zone_gid, to_zone_gid, ritten FROM public.pc5_odm_input ORDER BY from_zone_gid, to_zone_gid ASC;";
    PGresult* qres = conn.executeQuery(sql.c_str());
    return qres;
}

// Set the ODM data
void Matrix::setOdMatrixFromBinary() {
    Stopwatch stopwatch("READING " + getMatrixPrettyName() + " FROM BINARY FILE");
    stopwatch.start();

    FILE* decompressedFile;
    if (fopen_s(&decompressedFile, generateMatrixFileName(matrix_type).c_str(), "rb") == 0) {
        fseek(decompressedFile, 0, SEEK_END);
        long fileSize = ftell(decompressedFile);
        fseek(decompressedFile, 0, SEEK_SET);

        unsigned char* compressedMatrix = (unsigned char*)malloc(fileSize);

        if (compressedMatrix == nullptr) {
            Logger::log("Memory allocation for compressedMatrix failed");
            fclose(decompressedFile);
            return;  // Return without setting data member
        }

        fread(compressedMatrix, 1, fileSize, decompressedFile);
        fclose(decompressedFile);

        z_stream infstream;
        infstream.zalloc = Z_NULL;
        infstream.zfree = Z_NULL;
        infstream.opaque = Z_NULL;

        // Allocate memory for the decompressed matrix
        double* tempDecompressedMatrix = (double*)malloc(rows * cols * sizeof(double));

        if (tempDecompressedMatrix == nullptr) {
            Logger::log("Memory allocation for tempDecompressedMatrix failed");
            free(compressedMatrix);
            return;  // Return without setting data member
        }

        // Input: compressedMatrix, Output: tempDecompressedMatrix
        infstream.avail_in = fileSize;
        infstream.next_in = compressedMatrix;
        infstream.avail_out = rows * cols * sizeof(double);
        infstream.next_out = (Bytef*)tempDecompressedMatrix;

        inflateInit(&infstream);
        inflate(&infstream, Z_NO_FLUSH);
        inflateEnd(&infstream);

        // Copy the flat decompressed matrix to the vector of vectors
        //data.resize(rows, std::vector<double>(cols, 0.0));  // Resize data member
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                this->at(i, j) = tempDecompressedMatrix[i * cols + j];
            }
        }

        free(tempDecompressedMatrix);
        free(compressedMatrix);
    }
    else {
        Logger::log("Failed to open the binary file for reading");
        return;  // Return without setting data member
    }

    stopwatch.finish();
}

void Matrix::getOdMatrixFromDatabaseList() {
    Stopwatch stopwatch("PREPARING MATRICES");
    stopwatch.start();
    PGresult* qres = this->getOdListFromDatabase();
    stopwatch.checkpoint("READING MATRIXDATA");

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            //this->data[i][j] = 0.0;
            this->at(i, j) = 0.0;
        }
    }

    // Populate the OD-Matrix with the trips values
    Logger::log("Parsing matrixdata...");
    for (int row = 0; row < PQntuples(qres); ++row) {
        int from_zone = std::stoi(PQgetvalue(qres, row, 0));
        int to_zone = std::stoi(PQgetvalue(qres, row, 1));
        double trips = std::stod(PQgetvalue(qres, row, 2));

        //this->data[from_zone - 1][to_zone - 1] = trips;
        this->at(from_zone - 1, to_zone - 1) = trips;
    }
    PQclear(qres);
    stopwatch.finish();
}


// Print function
void Matrix::print() const {
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            std::cout << this->at(i, j) << " ";
        }
        std::cout << std::endl;
    }
}

void Matrix::printMatrixSum() const {
    Logger::log("Sum of " + getMatrixPrettyName() + ": " + std::to_string(matrix_sum));
}

// Example usage
void Matrix::exampleUsage() {
    // Set values
    for (int i = 0; i < this->getRows(); ++i) {
        for (int j = 0; j < this->getCols(); ++j) {
            this->at(i, j) = (j + 0.1);
        }
    }

    this->print();
}
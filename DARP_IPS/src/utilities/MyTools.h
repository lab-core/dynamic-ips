//
// Created by Ella on 2021-09-07.
//

#ifndef MY_TOOLS_H
#define MY_TOOLS_H

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <chrono>
#include "Eigen/Dense"
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include "Tools.h"
#include <algorithm>
#include <valarray>
#include <bitset>
#include <boost/dynamic_bitset.hpp>
#include <unordered_map>
#include "json.hpp"
#include "Types.h"

using json = nlohmann::json;
namespace eu = enum_utils;
using namespace eu;
using namespace constants;

using std::string;
using std::vector;
using std::chrono::high_resolution_clock;
extern std::vector<std::vector<float>> durationMatrix_;
//-----------------------------------------------------------------------------
//  Useful tools and data types
//-----------------------------------------------------------------------------

// Function template to concatenate two vectors of the same type
template <typename T>
std::vector<T> concatenateVectors(const std::vector<T>& vec1, const std::vector<T>& vec2) {
    std::vector<T> result;
    result.reserve(vec1.size() + vec2.size()); // Reserve memory to optimize performance

    // Insert elements from the first vector
    result.insert(result.end(), vec1.begin(), vec1.end());

    // Insert elements from the second vector
    result.insert(result.end(), vec2.begin(), vec2.end());

    return result;
}




namespace myTools {
    // class for defining exception errors
    class myException : public std::exception {
    public:
        // Constructor with just the line number
        explicit myException(const char* Msg, int Line = 0) noexcept {
            initialize(Msg, "", Line);
        }

        // Constructor with both file name and line number
        myException(const char* Msg, const char* File, int Line) noexcept {
            initialize(Msg, File, Line);
        }

        // Copy constructor
        myException(const myException& other) noexcept
                : msg(other.msg) { }

        // Move constructor
        myException(myException&& other) noexcept
                : msg(std::move(other.msg)) { }

        // Copy assignment operator
        myException& operator=(const myException& other) noexcept {
            if (this != &other) {
                msg = other.msg;
            }
            return *this;
        }

        // Move assignment operator
        myException& operator=(myException&& other) noexcept {
            if (this != &other) {
                msg = std::move(other.msg);
            }
            return *this;
        }

        // Destructor
        ~myException() noexcept override = default;

        // Returns a pointer to the (constant) error description
        const char* what() const noexcept override {
            return msg.c_str();
        }

        // Throw an exception with the input message
        static void throwException(const char *exceptionMsg) {
            try {
                throw myException(exceptionMsg);
            }
            catch (std::exception& e) {
                printf("Exception caught: %s\n", e.what());
                throw;
            }
        }

        static void throwException(const std::string &strMsg) {
            throwException(strMsg.c_str());
        }

        static void throwError(const std::string &strMsg) {
            try {
                throw strMsg;
            }
            catch (std::string& Msg) {
                printf("Error caught: %s\n", Msg.c_str());
                throw;
            }
        }

        static void throwError(const char *exceptionMsg) {
            throwError(std::string(exceptionMsg));
        }

    private:
        std::string msg;

        void initialize(const char* Msg, const char* File, int Line) {
            std::ostringstream oss;
            oss << Msg;
            if (File && File[0] != '\0') {
                oss << "\nError occurred at line " << Line << " in file " << File;
            } else {
                oss << "\nError occurred at line " << Line;
            }
            msg = oss.str();
        }
    };


    // functions to create node IDs
    std::string createNodeID(unsigned int requestID, NodeType type);
    std::string createSourceID(int vehicleID, NodeType type);

    // function to compare two val_array
    bool isLess_equal(const std::valarray<int> &rhs, const std::valarray<int> &lhs);



    //-----------------------------------------------------------------------------
    //  TIMER CLASS
    //-----------------------------------------------------------------------------
    class Timer {
    private:
        high_resolution_clock::time_point cpuInit_;
        std::chrono::duration<double> cpuSinceStart_;
        std::chrono::duration<double> cpuSinceInit_;

        int coStop_;	        //number of times the timer was stopped
        bool isInit_;
        bool isStarted_;
        bool isStopped_;

        // Constructor and Destructor
    public:
        Timer();
        virtual ~Timer();


        void init();        // function fo initialize the timer
        bool isInit() const;      // function to check initialization status
        void start();       // function to start the timer
        void stop();        // function to stop the timer
        void addTime(double sec);

        // get the time spent since the initialization of the timer and since the last
        // time it was started
        std::chrono::duration<float> dSinceInit() const;
        std::chrono::duration<float> dSinceStart();
    };

    //-----------------------------------------------------------------------------
    //  SHARED VECTOR CLASS
    //  a vector to be shared in multithreading
    //-----------------------------------------------------------------------------
    template<class T>
    class SharedVector{
    private:
        std::vector<std::vector<T>> shared_vector;
        mutable std::mutex mutex_;
    public:
        void push_data(std::vector<T> data) {
            std::lock_guard<std::mutex> lock(mutex_);
            shared_vector.push_back(std::move(data));
        }

        std::vector<T> pop_data() {
            std::lock_guard<std::mutex> lock(mutex_);
            std::vector<T> data = shared_vector.back();
            shared_vector.pop_back();
            return data;
        }

        void defineSize(int size) {
            shared_vector.resize(size);
        }

        int getSize(){
            return shared_vector[0].size();
        }

        void clear() {
            shared_vector.clear();  // Clears the internal vector
        }
    };

    // Define a RAII guard to manage std::cout redirection
    class CoutRedirector {
    public:
        CoutRedirector(const std::string &logFilePath, const std::string& model)
            : logFile_(logFilePath, std::ofstream::app),
              originalBuffer_(std::cout.rdbuf()) {
            logFile_ << "----------------------- " << model << " ------------------------" << std::endl;
            std::cout.rdbuf(logFile_.rdbuf());
        }
        ~CoutRedirector() {
            std::cout.rdbuf(originalBuffer_);
        }
    private:
        std::ofstream logFile_;
        std::streambuf *originalBuffer_;
    };


} // myTools namespace


#endif //MY_TOOLS_H

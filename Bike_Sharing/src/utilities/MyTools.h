//
// Created by Ella on 2021-09-07.
//

#ifndef MY_TOOLS_H
#define MY_TOOLS_H

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <map>
#include <unordered_map>
#include <map>
#include <string>
#include <climits>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <cstdio>
#include "Tools.h"
#include <set>
#include <queue>
#include <algorithm>
#include <valarray>
#include <numeric>
#include <bitset>
#include <ilcplex/ilocplex.h>
#include "types.h"
#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/prettywriter.h" // For pretty print

using std::string;
using std::vector;
using std::chrono::high_resolution_clock;
//-----------------------------------------------------------------------------
//  Definition of useful tools and data types
//-----------------------------------------------------------------------------

#define LARGE_CONSTANT 9999999
const int MAX_BIT_SIZE = 1000;
const int MAX_ZONE = 350;

static const int sentenceSize = 47;
static const int TimePerBike = 6;
static const int TimeToPark = 120;


// Definition of useful types
template<class T> using vector2D = std::vector<std::vector<T>>;

typedef IloArray<IloNumVarArray> IloNumVar2D;		// 2-dim array of variables
typedef IloArray<IloNumVar2D> IloNumVar3D;          // 3-dim array of variables
typedef IloArray<IloNumArray> IloNum2D;		        // 2-dim array of variables


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
        std::chrono::duration<double> dSinceInit();
        std::chrono::duration<double> dSinceStart();
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

        void getSize(){
            std::cout << shared_vector[0].size() << std::endl;
        }
    };


} // myTools namespace


#endif //MY_TOOLS_H

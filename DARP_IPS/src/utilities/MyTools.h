//
// Created by Ella on 2021-09-07.
//

#ifndef _MYTOOLS_H
#define _MYTOOLS_H

#define NOMINMAX

#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include <map>
#include <string>
#include <limits.h>
#include <chrono>
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
#include <json/json.h>
#include <curl/curl.h>



using std::string;
using std::vector;
using std::chrono::high_resolution_clock;
//-----------------------------------------------------------------------------
//  Definition of useful tools and data types
//-----------------------------------------------------------------------------

// useful types
class Instance;
typedef std::shared_ptr<Instance> PInstance;
class Request;
typedef std::shared_ptr<Request> PRequest;
class Vehicle;
typedef std::shared_ptr<Vehicle> PVehicle;
class Graph;
typedef std::shared_ptr<Graph> PGraph;
class Node;
typedef std::shared_ptr<Node> PNode;
class Route;
typedef std::shared_ptr<Route> PRoute;
class ReducedProblem;
typedef std::shared_ptr<ReducedProblem> PReducedProblem;
class ComplementPro;
typedef std::shared_ptr<ComplementPro> PComplementPro;
class TravelTime;
typedef std::shared_ptr<TravelTime> PTravelTime;
class Label;
typedef std::shared_ptr<Label> PLabel;
// extern PTravelTime travelMat;


// SubProblem solution status
enum SubSolveStatus { FULL = 0, RESTRICTED = 1, EXACT = 2, H1 = 3, H2 = 4, H1H2 = 5};

#define MAXReachTime 9999999

static const int DECIMALS = 3;          // precision when printing floats
// the constant 275 calculated by excel just to convert distance in mile to travel time in sec
static const float TimePerMile = 10;   // travel time per mile distance
static const float alphaParam = 1.5;
static const float betaParam = 440;
static const float deltaPram = 820;
static const int epochLength = 50;
static const int sentenceSize = 45;

// Definition of useful types
template<class T> using vector2D = std::vector<std::vector<T>>;
template<class T> using vector3D = std::vector<vector2D<T>>;
extern vector2D<float> durationMatrix_;

class PCompare {
public:
    template<typename T>
    bool operator () (std::shared_ptr<T> a, std::shared_ptr<T> b) {
        return (*a) < (*b);
    }
};

// Different node types and their names
enum NodeType { SOURCE, SINK, PICKUP, DROPOFF };
/*static const std::vector<std::string> nodeTypeName = {
        "SOURCE ",
        "SINK   ",
        "PICKUP ",
        "DROPOFF"
};*/
static const char *NodeTypeStr[] = {
        "SOURCE ",
        "SINK   ",
        "PICKUP ",
        "DROPOFF"
};

namespace Tools {
    // class for defining exception errors

    // class for defining exception errors
    class myException : public std::exception
    {
    public:
        // Constructor
        myException(const char* Msg, int Line)
        {
            std::ostringstream oss;
            oss << "Error line" << Line << " : " << Msg;
            this->msg = oss.str();
        }

        // Destructor
        virtual ~myException() throw() {}

        // Returns a pointer to the (constant) error description
        /* The underlying memory is in possession of the Except object.
         * Callers must not attempt to free the memory */
        virtual  const char* what() const throw() { return this->msg.c_str();}
    private:
        std::string msg;
    };

    // Throw an exception with the input message
    struct INMsgException: std::exception {
        INMsgException(const char* what): std::exception(), what_(what) {}
        const char* what() const throw() {
            return what_;
        }
    private:
        const char* what_;
    };

    void throwException(const char* exceptionMsg);
    void throwException(const std::string& strMsg);
    void throwError(const std::string& strMsg);
    void throwError(const char* exceptionMsg);

    //-----------------------------------------------------------------------------
    //  Functions for calculating the shortest spherical
    //  distance between to points on earth surface
    //-----------------------------------------------------------------------------

    // function to convert degrees to radians
    double toRadians(const double degree);

    // function to calculate the distance
    double calcDistance(double lat1, double long1, double lat2, double long2);

    // function to calculate travel time between two coordinate
    double calcTravelTime(double lat1, double long1, double lat2, double long2);

    // function to create node ID based on request ID
    std::string createNodeID(int requestID, NodeType type);

    // Appends the values of v2 vector to at the end of v1 vector
    template <typename T>
    std::vector<T> appendVectors(std::vector<T> & v1, std::vector<T> & V2);

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
        bool isInit();      // function to check initialization status
        void start();       // function to start the timer
        void stop();        // function to stop the timer

        // get the time spent since the initialization of the timer and since the last
        // time it was started
        const std::chrono::duration<double> dSinceInit();
        const std::chrono::duration<double> dSinceStart();
    };

    // function for reading data from url
    static int writer(char *data, size_t size, size_t nmemb, std::string *writerData);

    // function to query the fastest route between coordinates
    float queryTravelTime(double lat1, double long1, double lat2, double long2);

    // function to get data from a http url
    std::string queryHTTPData(const std::string &url);


}; // Tools namespace


#endif //_MYTOOLS_H

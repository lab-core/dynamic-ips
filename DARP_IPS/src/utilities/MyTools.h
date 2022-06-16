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
#include <unordered_map>
#include <map>
#include <string>
#include <limits.h>
#include <chrono>
#include <iomanip>
#include <stdlib.h>
#include <stdio.h>
/*#include <json/json.h>
#include <curl/curl.h>*/
#include "Eigen/Dense"
#include <set>
#include <queue>
#include <algorithm>
#include <valarray>

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
class ZoomReducedProblem;
typedef std::shared_ptr<ZoomReducedProblem> PZoomReducedProblem;
class TravelTime;
typedef std::shared_ptr<TravelTime> PTravelTime;
class Label;
typedef std::shared_ptr<Label> PLabel;
struct Parameters;
typedef std::shared_ptr<Parameters> PParameters;
struct solverOption;
typedef std::shared_ptr<solverOption> PSolverOption;
class GreedyLabel;
typedef std::shared_ptr<GreedyLabel> PGreedyLabel;
class LinkedGreedyLabels;
typedef std::shared_ptr<LinkedGreedyLabels> PLinkedGreedyLabels;
class GreedyModeler;
typedef std::shared_ptr<GreedyModeler> PGreedyModeler;
struct insertPosition;
typedef std::shared_ptr<insertPosition> PInsertPosition;
// extern PTravelTime travelMat;


// SubProblem solution status
enum SubProSolveStart {NOT_RESTRICTED = 0, TIME_RESTRICTED = 1, NUM_PICK_RESTRICTED = 2};
enum LabelingStrategy { PUSHING = 0, PULLING = 1};
enum subproblemAlgorithm { CPLEX = 0, LABEL_SETTING = 1};
enum MainAlgorithm {GREEDY = 0, MIP_CPLEX = 1, CG_CPLEX = 2, CG_ISUD = 3};
static const std::vector<std::string> LabelingStrategyName = {
        "PUSHING",
        "PULLING" };

static const std::vector<std::string> subAlgorithmName = {
        "CPLEX        ",
        "LABEL_SETTING" };

static const std::vector<std::string> mainAlgorithmName = {
        "GREEDY    ",
        "MIP_CPLEX ",
        "CG_CPLEX  ",
        "CG_ISUD   "};
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

#define MAXReachTime 9999999

static const int DECIMALS = 3;          // precision when printing floats
static const float TimePerMile = 10;   // travel time per mile distance
static const int sentenceSize = 45;

/*static const float alphaParam_ = 1.5;
static const float betaParam = 440;
static const float deltaPram = 820;
static const int epochLength = 50;*/

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

    // functions to create node IDs
    std::string createNodeID(int requestID, NodeType type);
    std::string createSourceID(int vehicleID, NodeType type);

    // function to compare two valarray
    bool isLess_equal(const std::valarray<int> &rhs, const std::valarray<int> &lhs);



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
    //float queryTravelTime(double lat1, double long1, double lat2, double long2);

    // function to get data from a http url
    //std::string queryHTTPData(const std::string &url);

}; // Tools namespace

struct Parameters {
public:
    // model Parameters
    float alphaParam_;
    float betaParam_;
    float deltaPram_;
    int epochLength_;

    bool emptyStart_;

    MainAlgorithm mainAlgorithm_;

    // label setting strategies
    bool isTruncated_;
    int MaxLabel_;
    bool isDominanceReleased_;
    bool isSuccessorsLimited_;
    SubProSolveStart SubproSolveStartState_;
    LabelingStrategy LabelingStrategy_;
    subproblemAlgorithm subAlgorithm_;


    //CPLEX Parameters
    int bigM_;
    int solveTimeLimit_;
    int populateTimeLimit_;

    // Constructor and Destructor
    Parameters();
    Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, bool emptyStart,
               MainAlgorithm mainAlgorithm, bool isTruncated, int maxLebel, bool isSuccessorsLimited, bool isDominanceReleased,
               SubProSolveStart subproSolveStartState, LabelingStrategy LabelingStrategy,
               subproblemAlgorithm subAlgorithm, int bigM, int solveTimeLimit, int populateTimeLimit);

    virtual ~Parameters();

    // Display function
    std::string toString() const;
};


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------
struct solverOption {
    float maxReachTime_;
    int maxPickup_;

    bool isTruncated_;
    bool isDominanceReleased_;
    bool isSuccessorsLimited_;
    LabelingStrategy LabelingStrategy_;
    int MaxLabel_;

    // Constructor and Destructor
    solverOption(float maxReachTime, int maxPickup, bool isTruncated, int maxLabel, bool isDominanceReleased,
                 bool isSuccessorsLimited, LabelingStrategy labelingStrategy);

    solverOption(float maxReachTime, int maxPickup, const PParameters MainParams);

    virtual ~solverOption();
    void disableHeuristics();
    bool areHeuristicsDisabled();
};
#endif //_MYTOOLS_H

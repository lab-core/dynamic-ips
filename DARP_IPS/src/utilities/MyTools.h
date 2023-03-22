//
// Created by Ella on 2021-09-07.
//

#ifndef _MYTOOLS_H
#define _MYTOOLS_H

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
#include "Eigen/Dense"
#include "Tools.h"
#include <set>
#include <queue>
#include <algorithm>
#include <valarray>
#include <numeric>
#include <bitset>


using std::string;
using std::vector;
using std::chrono::high_resolution_clock;
extern std::vector<std::vector<float>> durationMatrix_;
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
class Label;
typedef std::shared_ptr<Label> PLabel;
struct Parameters;
typedef std::shared_ptr<Parameters> PParameters;
struct solverOption;
typedef std::shared_ptr<solverOption> PSolverOption;
class StopLabel;
typedef std::shared_ptr<StopLabel> PStopLabel;
class GreedyRoute;
typedef std::shared_ptr<GreedyRoute> PGreedyRoute;
class GreedyModeler;
typedef std::shared_ptr<GreedyModeler> PGreedyModeler;
struct insertPosition;
typedef std::shared_ptr<insertPosition> PInsertPosition;
// extern PTravelTime travelMat;

// SubProblem solution status
enum SubProSolveMode {NOT_RESTRICTED = 0, TIME_RESTRICTED = 1, NUM_PICK_RESTRICTED = 2};
enum LabelingStrategy { PUSHING = 0, PULLING = 1};
enum subproblemAlgorithm { CPLEX = 0, LABEL_SETTING = 1};
enum MainAlgorithm {GREEDY = 0, MIP_CPLEX = 1, CG_CPLEX = 2, CG_ISUD = 3};
enum SolutionMode {STATIC = 0, DYNAMIC = 1, ANYTIME = 2};
enum warmStart {GREEDY_START = 0, PRE_SOLUTION = 1, EMPTY_ROUTES = 2};
enum InitialDual {LAST_CP = 0, PENALTIES = 1};
static const std::vector<std::string> LabelingStrategyName = {
        "PUSHING",
        "PULLING" };

static const std::vector<std::string> subAlgorithmName = {
        "CPLEX        ",
        "LABEL_SETTING" };

static const std::vector<std::string> mainAlgorithmName = {
        "GREEDY",
        "MIP_CPLEX",
        "CG_CPLEX",
        "CG_ISUD"};

static const std::vector<std::string> warmStartName = {
        "GREEDY_START     ",
        "PREVIOUS_SOLUTION",
        "EMPTY_START      "};

static const std::vector<std::string> solutionModeName = {
        "STATIC",
        "DYNAMIC",
        "ANYTIME"};

static const std::vector<std::string> InitialDualName = {
        "LAST_CP  ",
        "PENALTIES"};
static const std::vector<std::string> SubProSolveStartName = {
        "NOT_RESTRICTED     ",
        "TIME_RESTRICTED    ",
        "NUM_PICK_RESTRICTED"};

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
const int MAX_SIZE = 2000;

static const int DECIMALS = 3;          // precision when printing floats
static const float TimePerMile = 10;   // travel time per mile distance
static const int sentenceSize = 47;
static const int ServiceTime = 30;

// Definition of useful types
template<class T> using vector2D = std::vector<std::vector<T>>;
template<class T> using vector3D = std::vector<vector2D<T>>;


namespace myTools {
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
        ~myException() noexcept override = default;

        // Returns a pointer to the (constant) error description
        /* The underlying memory is in possession of the Except object.
         * Callers must not attempt to free the memory */
         const char* what() const noexcept override { return this->msg.c_str();}
    private:
        std::string msg;
    };

    // Throw an exception with the input message
    struct INMsgException: std::exception {
        explicit INMsgException(const char* what): std::exception(), what_(what) {}
        const char* what() const noexcept override {
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
    double toRadians(double degree);

    // function to calculate the distance
    double calcDistance(double lat1, double long1, double lat2, double long2);

    // function to calculate travel time between two coordinate
    double calcTravelTime(double lat1, double long1, double lat2, double long2);

    // functions to create node IDs
    std::string createNodeID(unsigned int requestID, NodeType type);
    std::string createSourceID(int vehicleID, NodeType type);

    // function to compare two val_array
    bool isLess_equal(const std::valarray<int> &rhs, const std::valarray<int> &lhs);


    // Appends the values of v2 vector to at the end of v1 vector

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
    //  SET CLASS WITH BIT ARRAY
    //  Functions for defining sets with bit array
    //-----------------------------------------------------------------------------

#include <iostream>

    class BitVector {
    private:
        const int max_elem_;    // maximum element that can be stored in the set
        const int array_size_;  // size of the bit array
        unsigned long long* bit_array_;  // the bit array to represent the set
        static const int WORD_SIZE = sizeof(unsigned long long) * 8;  // number of bits in a word

        int getArrayIndex(int elem) const { return elem /WORD_SIZE; }
        int getBitIndex(int elem) const { return elem % WORD_SIZE; }


    public:
        // Constructor
        BitVector(int max_elem);

        // Copy constructor
        BitVector(const BitVector& other);

        // Destructor
        virtual ~BitVector();

        // Add an element to the set
        void add(int x);

        // Remove an element from the set
        void remove(int x);

        // Check if an element is in the set
        bool contains(int x) const;

        // Check if one set is a subset of another set
        bool isSubset(const BitVector& other) const;

        // Check if one set is equal to another set
        bool operator==(const BitVector& other) const;
        bool isEqual(const BitVector& other) const;

        void copyValues(const BitVector& other) const;

        // Check if the intersection of two sets is empty
        bool isIntersectionEmpty(const BitVector& other) const;
        void AddSet(const BitVector& other);

        // function to return the number of elements in the set
        int numElements() const;

        const int getMaxElem() const;

        const int getArraySize() const;

        unsigned long long int *getBitArray() const;

        // Display function
        std::string toString() const;
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


#endif //_MYTOOLS_H

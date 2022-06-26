//
// Created by Ella on 2021-09-07.
//
#define _USE_MATH_DEFINES

#include "MyTools.h"
#include <cmath>
#include <chrono>

//-----------------------------------------------------------------------------
//  Definition of useful tools and data types
//-----------------------------------------------------------------------------

namespace Tools {

    // Throw an exception with the input message
    void throwException(const char *exceptionMsg) {
        try {
            throw INMsgException(exceptionMsg);
        }
        catch (std::exception& e) {
            printf("Exception caught: %s\n", e.what());
            throw e;
        }
    }

    void throwException(const std::string &strMsg) {
        throwException(strMsg.c_str());
    }


    void throwError(const std::string &strMsg) {
        try {
            throw strMsg;
        }
        catch (std::string& Msg) {
            printf("Error caught: %s\n", Msg.c_str());
            throw Msg;
        }
    }

    void throwError(const char *exceptionMsg) {
        throwError(std::string(exceptionMsg));
    }

    //-----------------------------------------------------------------------------
    //  Functions for calculating the shortest distance between to points on earth
    //-----------------------------------------------------------------------------

    // function to convert degrees to radians
    double toRadians(double degree) {
        double oneDeg = (M_PI)/180;
        return (oneDeg * degree);
    }

    // function to calculate the distance
    double calcDistance(double lat1, double long1, double lat2, double long2) {

        // convert the latitudes and longitudes from degree to radians
        lat1 = toRadians(lat1);
        long1 = toRadians(long1);
        lat2 = toRadians(lat2);
        long2 = toRadians(long2);

        // Haversine Formula
        double dLong = long2 - long1;
        double dLat = lat2 - lat1;

        double dist = pow(sin(dLat/2),2) + cos(lat1) * cos(lat2) * pow(sin(dLong/2),2);
        dist = 2 * asin(sqrt(dist));

        // Radius of the Earth is 6371 km and 3956 miles
        double Radius = 3956;

        return dist * Radius;
    }
    // function to calculate travel time
    double calcTravelTime(double lat1, double long1, double lat2, double long2) {
        double dist = calcDistance(lat1, long1, lat2, long2);
        return dist * TimePerMile;
    }

    // functions to create node IDs
    std::string createNodeID(unsigned int requestID, NodeType type) {
        std::string ID;
        switch(type) {
            case SOURCE :
                ID = "SO_" + std::to_string(requestID);
                break;
            case SINK :
                ID = "SI_" + std::to_string(requestID);
                break;
            case PICKUP :
                ID = "PI_" + std::to_string(requestID);
                break;
            case DROPOFF :
                ID = "DR_" + std::to_string(requestID);
                break;
        }
        return ID;
    }

    std::string createSourceID(int vehicleID, NodeType type) {
        std::string ID;
        if (type == SOURCE)
            ID = "SO_" + std::to_string(vehicleID);
        else if (type == SINK)
            ID = "SI_" + std::to_string(vehicleID);
        return ID;
    }

    // function to compare two val_array
    bool isLess_equal(const std::valarray<int> &rhs, const std::valarray<int> &lhs) {
        std::valarray<bool> result = rhs <= lhs;
        return std::all_of(begin(result), end(result), [](bool v) { return v; });
    }

    // Appends the values of v2 vector to at the end of v1 vector
    template<typename T>
    vector<T> appendVectors(vector<T> &v1, vector<T> &v2) {
        std::vector<T> ANS;
        for (int i = 0; i < v1.size(); ++i) ANS.push_back(v1[i]);
        for (int i = 0; i < v2.size(); ++i) ANS.push_back(v2[i]);
        return ANS;
    }

    //************************************************************************
    //                     FUNCTIONS FOR TIMER CLASS
    //************************************************************************

    Timer::Timer() : isInit_(false), isStarted_(false), isStopped_(false) {
        this->init();
    }
    Timer::~Timer() = default;

    // function fo initialize the timer and set it in stop mode
    void Timer::init() {
        coStop_ = 0;

        cpuSinceInit_ = high_resolution_clock::duration::zero();
        cpuSinceStart_= high_resolution_clock::duration::zero();

        isInit_ = true;
        isStarted_ = false;
        isStopped_ = true;
    }

    // function to check initialization status
    bool Timer::isInit() const {
        return isInit_;
    }

    // function to start the timer
    void Timer::start() {
        if (isStarted_)
            throwError("Trying to start an already started timer!");
        cpuInit_ = high_resolution_clock::now();
        cpuSinceStart_= high_resolution_clock::duration::zero();
        isStarted_ = true;
        isStopped_ = false;
    } // end start

    // function to stop the timer
    void Timer::stop() {
        if (isStopped_)
            throwError("Trying to stop an already stopped timer!");
        high_resolution_clock::time_point cpuNow;
        cpuNow = high_resolution_clock::now();

        cpuSinceStart_ = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);
        cpuSinceInit_ = cpuSinceInit_ + cpuSinceStart_;
        isStarted_ = false;
        isStopped_ = true;
        coStop_ ++;
    } // end stop

    // get the time spent since the initialization of the timer and since the last
    // time it was started
    std::chrono::duration<double> Timer::dSinceInit() {
        std::chrono::duration<double> cpuCurrent;
        if (isStarted_) {
            high_resolution_clock::time_point cpuNow;
            cpuNow = high_resolution_clock::now();

            std::chrono::duration<double> cpuTmp;
            cpuTmp = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);


            cpuCurrent = cpuTmp + cpuSinceInit_;
            return cpuCurrent;
        }
        else if (isStopped_) {
            return cpuSinceInit_;
        }
        else
            throwError("Trying to get the value of an uninitialized timer!");
        return high_resolution_clock::duration::zero();
    } // end dSinceInit

    // Get the time spent since the last start of the timer without stopping it
    std::chrono::duration<double> Timer::dSinceStart() {
        if (!isStarted_ && !isStopped_)
            throwError("Trying to get the value of an uninitialized timer!");
        else if (isStarted_) {
            high_resolution_clock::time_point cpuNow;
            cpuNow = high_resolution_clock::now();
            cpuSinceStart_ = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);

        }
        return cpuSinceStart_;
    } //end dSinceStart

    //************************************************************************
    //                FUNCTIONS RELATED TO QUERY INFO FROM A URL
    //************************************************************************



} // end namespace

//************************************************************************
//                     FUNCTIONS FOR PARAMETERS CLASS
//************************************************************************

// Constructor and Destructor
Parameters::Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, bool emptyStart,
                       MainAlgorithm mainAlgorithm, bool isTruncated, int maxLabel, bool isSuccessorsLimited, bool isDominanceReleased,
                       SubProSolveStart subproSolveStartState, LabelingStrategy LabelingStrategy,
                       subproblemAlgorithm subAlgorithm, int bigM, int solveTimeLimit, int populateTimeLimit) :
        alphaParam_(alphaParam), betaParam_(betaParam), deltaPram_(deltaPram), epochLength_(epochLength),
        emptyStart_(emptyStart), mainAlgorithm_(mainAlgorithm), isTruncated_(isTruncated),
        MaxLabel_(maxLabel), isSuccessorsLimited_(isSuccessorsLimited),
        isDominanceReleased_(isDominanceReleased), SubproSolveStartState_(subproSolveStartState),
        LabelingStrategy_(LabelingStrategy), subAlgorithm_(subAlgorithm), bigM_(bigM),
        solveTimeLimit_(solveTimeLimit), populateTimeLimit_(populateTimeLimit) {
}

Parameters::~Parameters() = default;

// Display function
std::string Parameters::toString() const {
    std::stringstream repStr;
    repStr << "# MODEL PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    int setwLength = 30;
    repStr << std::left << std::fixed << std::setprecision(1) << std::boolalpha;
    repStr << std::setw(setwLength) << "# alpha Parameter " << " = " << alphaParam_ << std::endl;
    repStr << std::setw(setwLength) << "# beta Parameter " << " = " << betaParam_ << std::endl;
    repStr << std::setw(setwLength) << "# delta Parameter" << " = " << deltaPram_ << std::endl;
    repStr << std::setw(setwLength) << "# epoch Length " << " = " << epochLength_ << std::endl;
    repStr << std::setw(setwLength) << "# empty route at each epoch " << " = " << emptyStart_ << std::endl;
    repStr << std::setw(setwLength) << "# Main algorithm " << " = " << mainAlgorithmName[mainAlgorithm_] << std::endl;
    repStr << std::endl;

    repStr << "# LABEL SETTING STRATEGIES" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# Use Truncated Labeling " << " = " << isTruncated_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxLabel in Truncating " << " = " << MaxLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# Release Dominance Rule " << " = " << isDominanceReleased_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict outgoing arcs " << " = " << isSuccessorsLimited_ << std::endl;
    repStr << std::setw(setwLength) << "# Restrict Route Length " << " = " << SubproSolveStartState_ << std::endl;
    repStr << std::setw(setwLength) << "# Labeling Strategy " << " = " << LabelingStrategyName[LabelingStrategy_] << std::endl;
    repStr << std::setw(setwLength) << "# SubProblem solution Method " << " = " << subAlgorithmName[subAlgorithm_] << std::endl;
    repStr << std::endl;

    repStr << "# CPLEX PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(0);
    repStr << std::setw(setwLength) << "# BigM value " << " = " << bigM_ << std::endl;
    repStr << std::setw(setwLength) << "# solve Time Limit " << " = " << solveTimeLimit_ << std::endl;
    repStr << std::setw(setwLength) << "# populate Time Limit " << " = " << populateTimeLimit_ << std::endl;

    return repStr.str();

}


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------

solverOption::solverOption(float maxReachTime, int maxPickup, bool isTruncated, int maxLabel, bool isDominanceReleased,
                           bool isSuccessorsLimited, LabelingStrategy labelingStrategy) : maxReachTime_(maxReachTime),
                           maxPickup_(maxPickup), isTruncated_(isTruncated), MaxLabel_(maxLabel),
                           isSuccessorsLimited_(isSuccessorsLimited), isDominanceReleased_(isDominanceReleased),
                           LabelingStrategy_(labelingStrategy) {}

solverOption::~solverOption() = default;

void solverOption::disableHeuristics() {
    isTruncated_ = false;
    isDominanceReleased_ = false;
    isSuccessorsLimited_ = false;
}

solverOption::solverOption(float maxReachTime, int maxPickup, PParameters &MainParams): maxReachTime_(maxReachTime),
                                                                                             maxPickup_(maxPickup) {
    isTruncated_ = MainParams->isTruncated_;
    MaxLabel_ = MainParams->MaxLabel_;
    isDominanceReleased_ = MainParams->isDominanceReleased_;
    LabelingStrategy_ = MainParams->LabelingStrategy_;
    isSuccessorsLimited_ = MainParams->isSuccessorsLimited_;
}

bool solverOption::areHeuristicsDisabled() const {
    if (isTruncated_ || isDominanceReleased_||isSuccessorsLimited_)
        return false;
    else
        return true;
}



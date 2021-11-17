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
    double toRadians(const double degree) {
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
    float calcTravelTime(double lat1, double long1, double lat2, double long2) {
        double dist = calcDistance(lat1, long1, lat2, long2);
        return dist * TimePerMile;
    }

    // function to create node ID based on request ID
    std::string createNodeID(int requestID, NodeType type) {
        std::string ID;
        switch(type) {
            case SOURCE :
                ID = "SO_" + std::to_string(requestID);
                break; //optional
            case SINK :
                ID = "SI_" + std::to_string(requestID);
                break; //optional
            case PICKUP :
                ID = "PI_" + std::to_string(requestID);
                break; //optional
            case DROPOFF :
                ID = "DR_" + std::to_string(requestID);
                break; //optional
        }
        return ID;
    }

    // Appends the values of v2 vector to at the end of v1 vector
    template<typename T>
    vector<T> appendVectors(vector<T> &v1, vector<T> &v2) {
        std::vector<T> ANS;
        for (int i = 0; i < v1.size(); ++i) ANS.push_back(v1[i]);
        for (int i = 0; i < v2.size(); ++i) ANS.push_back(v2[i]);
        return ANS;
    }

    Timer::Timer() : isInit_(0), isStarted_(0), isStopped_(0) {
        this->init();
    }
    Timer::~Timer() {}

    // function fo initialize the timer and set it in stop mode
    void Timer::init() {
        coStop_ = 0;

        cpuSinceInit_ = high_resolution_clock::duration::zero();
        cpuSinceStart_= high_resolution_clock::duration::zero();

        isInit_ = 1;
        isStarted_ = 0;
        isStopped_ = 1;
    }

    // function to check initialization status
    bool Timer::isInit() {
        return isInit_;
    }

    // function to start the timer
    void Timer::start() {
        if (isStarted_)
            throwError("Trying to start an already started timer!");
        cpuInit_ = high_resolution_clock::now();
        cpuSinceStart_= high_resolution_clock::duration::zero();
        isStarted_ = 1;
        isStopped_ = 0;
    } // end start

    // function to stop the timer
    void Timer::stop() {
        if (isStopped_)
            throwError("Trying to stop an already stopped timer!");
        high_resolution_clock::time_point cpuNow;
        cpuNow = high_resolution_clock::now();

        cpuSinceStart_ = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);
        cpuSinceInit_ = cpuSinceInit_ + cpuSinceStart_;
        isStarted_ = 0;
        isStopped_ = 1;
        coStop_ ++;
    } // end stop

    // get the time spent since the initialization of the timer and since the last
    // time it was started
    const std::chrono::duration<double> Timer::dSinceInit() {
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
    const std::chrono::duration<double> Timer::dSinceStart() {
        if (!isStarted_ && !isStopped_)
            throwError("Trying to get the value of an uninitialized timer!");
        else if (isStarted_) {
            high_resolution_clock::time_point cpuNow;
            cpuNow = high_resolution_clock::now();
            cpuSinceStart_ = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);

        }
        return cpuSinceStart_;
    } //end dSinceStart

} // end namespace


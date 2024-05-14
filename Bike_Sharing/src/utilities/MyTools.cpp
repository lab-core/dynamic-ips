//
// Created by Ella on 2021-09-07.
//

#include "MyTools.h"
#include <chrono>


//-----------------------------------------------------------------------------
//  Definition of useful tools and data types
//-----------------------------------------------------------------------------

namespace myTools {


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
            myException::throwError("Trying to start an already started timer!");
        cpuInit_ = high_resolution_clock::now();
        cpuSinceStart_= high_resolution_clock::duration::zero();
        isStarted_ = true;
        isStopped_ = false;
    } // end start

    // function to stop the timer
    void Timer::stop() {
        if (isStopped_)
            myException::throwError("Trying to stop an already stopped timer!");
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
            myException::throwError("Trying to get the value of an uninitialized timer!");
        return high_resolution_clock::duration::zero();
    } // end dSinceInit

    // Get the time spent since the last start of the timer without stopping it
    std::chrono::duration<double> Timer::dSinceStart() {
        if (!isStarted_ && !isStopped_)
            myException::throwError("Trying to get the value of an uninitialized timer!");
        else if (isStarted_) {
            high_resolution_clock::time_point cpuNow;
            cpuNow = high_resolution_clock::now();
            cpuSinceStart_ = std::chrono::duration_cast<std::chrono::duration<double>>(cpuNow - cpuInit_);

        }
        return cpuSinceStart_;
    }

    void Timer::addTime(double sec) {
        auto timeToAdd = std::chrono::duration<double>(sec);
        cpuSinceInit_ = cpuSinceInit_ + timeToAdd;
    }
    //end dSinceStart

} // end namespace




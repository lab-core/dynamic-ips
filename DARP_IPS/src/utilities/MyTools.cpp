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

namespace myTools {

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
    //                     FUNCTIONS FOR BitVector CLASS
    //************************************************************************
    BitVector::BitVector(int max_elem) : max_elem_(max_elem),
                                         array_size_((max_elem + WORD_SIZE - 1) / WORD_SIZE),
                                         bit_array_(new unsigned long long[array_size_]) {
        // initialize all bits to 0
        for (int i = 0; i < array_size_; i++) {
            bit_array_[i] = 0;
        }
    }

    // Copy constructor
    BitVector::BitVector(const BitVector &other) : max_elem_(other.max_elem_), array_size_(other.array_size_),
                                                   bit_array_(new unsigned long long[array_size_]){
        for (int i = 0; i < array_size_; i++) {
            bit_array_[i] = other.bit_array_[i];
        }
    }

    BitVector::~BitVector() {
        delete[] bit_array_;
    }

    // Add an element to the set
    void BitVector::add(int x) {
        if (x > max_elem_) return;
        /*int array_index = getArrayIndex(x);
        int bit_index = getBitIndex(x);
        bit_array_[array_index] |= (1u << bit_index);*/
        bit_array_[x / WORD_SIZE] |= (1ull << (x % WORD_SIZE));
    }

    // Remove an element from the set
    void BitVector::remove(int x) {
        if (x > max_elem_) return;
        int array_index = getArrayIndex(x);
        int bit_index = getBitIndex(x);
        bit_array_[array_index] &= ~(1u << bit_index);
    }

    // Check if an element is in the set
    bool BitVector::contains(int x) const {
        if (x > max_elem_) return false;
        /*int array_index = getArrayIndex(x);
        int bit_index = getBitIndex(x);
        return (bit_array_[array_index] & (1u << bit_index)) != 0;*/
        return (bit_array_[x / WORD_SIZE] & (1ull << (x % WORD_SIZE))) != 0;
    }

    // Check if one set is a subset of another set
    bool BitVector::isSubset(const BitVector &other) const {
        if (max_elem_ != other.max_elem_) {
            return false;
        }
        for (int i = 0; i < array_size_; i++) {
            if ((bit_array_[i] & other.bit_array_[i]) != bit_array_[i]) {
                return false;
            }
        }
        return true;
    }

    // Check if one set is equal to another set
    bool BitVector::operator==(const BitVector &other) const {
        if (max_elem_ != other.max_elem_) {
            return false;
        }
        for (int i = 0; i < array_size_; i++) {
            if (bit_array_[i] != other.bit_array_[i]) {
                return false;
            }
        }
        return true;
    }

    bool BitVector::isEqual(const BitVector &other) const {
        if (max_elem_ != other.max_elem_) {
            return false;
        }
        for (int i = 0; i < array_size_; i++) {
            if (bit_array_[i] != other.bit_array_[i]) {
                return false;
            }
        }
        return true;
    }

    void BitVector::copyValues(const BitVector &other) const {
        for (int i = 0; i < array_size_; i++) {
            bit_array_[i] = other.bit_array_[i];
        }
    }

    bool BitVector::isIntersectionEmpty(const BitVector &other) const {
        if (max_elem_ != other.max_elem_) {
            return true;
        }
        for (int i = 0; i < array_size_; i++) {
            if ((bit_array_[i] & other.bit_array_[i]) != 0) {
                return false;
            }
        }
        return true;
    }

    const int BitVector::getMaxElem() const {
        return max_elem_;
    }

    std::string BitVector::toString() const {
        std::stringstream repStr;
        for (int j = 0; j < array_size_; j++) {
            for (int i = WORD_SIZE; i>= 0; i--){
                if (bit_array_[j] & (1 << i)) {
                    repStr << "1";
                } else {
                    repStr << "0";
                }
            }
        }
        repStr << std::endl;
        return repStr.str();
    }

    const int BitVector::getArraySize() const {
        return array_size_;
    }

    unsigned long long int *BitVector::getBitArray() const {
        return bit_array_;
    }

    void BitVector::AddSet(const BitVector &other) {
        for (int i = 0; i < array_size_; i++) {
            bit_array_[i] =  bit_array_[i] || other.bit_array_[i];
        }
    }

    int BitVector::numElements() const {
        int count = 0;
        for (int j = 0; j < array_size_; j++) {
            for (int i = WORD_SIZE; i>= 0; i--){
                if (bit_array_[j]) {
                    count++;
                }
            }
        }
        return count;
    }


} // end namespace




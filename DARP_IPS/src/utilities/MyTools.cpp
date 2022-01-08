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
    double calcTravelTime(double lat1, double long1, double lat2, double long2) {
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

    // function for reading data from url
    int writer(char *data, size_t size, size_t nmemb, std::string *writerData) {
        if (writerData == NULL)
            return 0;

        writerData->append(data, size*nmemb);

        return size * nmemb;
    }


    // function to query the fastest route between coordinates
    float queryTravelTime(double lat1, double long1, double lat2, double long2) {
        float duration = 0;
        const std::string url = "http://206.12.92.28/ny/route/v1/driving/" +
                                std::to_string(long1) + "," + std::to_string(lat1) + ";" +
                                std::to_string(long2) + "," + std::to_string(lat2) +
                                "?overview=false&generate_hints=false";

        std::string content = queryHTTPData(url);
        Json::Value jsonData;
        Json::CharReaderBuilder builder;
        JSONCPP_STRING errs;

        const std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
        // parse the results and return duration time
        bool parsingSuccessful = reader->parse(content.c_str(), content.c_str() + content.size(), &jsonData,
                                               &errs);
        if (parsingSuccessful) {
            // check the route code
            if (jsonData["code"] == "Ok") {

                duration = jsonData["routes"][0]["duration"].asDouble();
            }
            else if (jsonData["code"] == "NoRoute")
                std::cout << "No route found between input locations" << std::endl;
            else if (jsonData["code"] == "TooBig")
                std::cout << "Too many coordinates" << std::endl;
        }
        return duration;

    }

    // function to get data from a http url
    std::string queryHTTPData(const std::string &url) {
        CURL *curl = curl_easy_init();
        std::string content;

        // Set remote URL.
        curl_easy_setopt(curl, CURLOPT_URL, url.c_str());

        // Don't bother trying IPv6, which would increase DNS resolution time.
        curl_easy_setopt(curl, CURLOPT_IPRESOLVE, CURL_IPRESOLVE_V4);

        // Don't wait forever, time out after 10 seconds.
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10);

        // Follow HTTP redirects if necessary.
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);

        // Response information.
        int httpCode(0);

        // Hook up data handling function.
//        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, callback);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writer);

        // Hook up data container (will be passed as the last parameter to the
        // callback handling function).  Can be any pointer type, since it will
        // internally be passed as a void pointer.
//        curl_easy_setopt(curl, CURLOPT_WRITEDATA, httpData.get());
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &content);

        // Run our HTTP GET command, capture the HTTP response code, and clean up.
        curl_easy_perform(curl);
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &httpCode);
        curl_easy_cleanup(curl);

        return content;
    }





} // end namespace


//
// Created by Ella on 2021-09-07.
//
#define _USE_MATH_DEFINES

#include "MyTools.h"
#include <cmath>

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


}


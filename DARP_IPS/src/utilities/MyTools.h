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
#include <string>
#include <limits.h>


using std::string;
using std::vector;
//-----------------------------------------------------------------------------
//  Definition of useful tools and data types
//-----------------------------------------------------------------------------

static const int DECIMALS = 3;  // precision when printing floats

// Definition of useful types
template<class T> using vector2D = std::vector<std::vector<T>>;
template<class T> using vector3D = std::vector<vector2D<T>>;

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

}; // Tools namespace



#endif //_MYTOOLS_H

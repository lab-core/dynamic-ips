//
// Created by Ella on 2021-09-08.
//

#ifndef _VEHICLE_H
#define _VEHICLE_H

#include "utilities/MyTools.h"

//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

class Vehicle {
public:
    const int vehicleID_;       // vehicle ID
    int startTime_;             // vehicle start time
    int endTime_;               // vehicle end time
    int capacity_;              // vehicle capacity
    int numPassengers_;         // number of passengers in the vehicle
    int departTime_;            // time the vehicle arrives at its departing stop for the epoch

    // Constructor and Destructor
    Vehicle(const int vehicleId, int capacity, int startTime, int endTime);

    virtual ~Vehicle();

    // Setters
    void setNumPassengers(int numPassengers);
    void setDepartTime(int departTime);

    // Display function
    std::string toString() const;
};
typedef std::shared_ptr<Vehicle> PVehicle;


#endif //_VEHICLE_H

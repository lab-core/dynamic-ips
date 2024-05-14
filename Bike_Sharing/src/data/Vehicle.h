//
// Created by Ella on 2021-09-08.
//

#ifndef VEHICLE_H
#define VEHICLE_H

#include "data/Route.h"


//-----------------------------------------------------------------------------
//  vehicle class
//  The ``Vehicle`` class mostly serves as a structure for storing basic
//        information about the vehicles.
//-----------------------------------------------------------------------------

class Vehicle {
public:
    int vehicleID_;                         // vehicle ID
    std::string name_;
    float readyTime_;                       // vehicle start time
    float endTime_;                         // vehicle end time
    int departIndex_;
    int capacity_;                          // vehicle capacity
    int bikeLoad_;                          // number of bikes on the vehicle
    PNode departNode_;                      // vehicle departing stop

    PRoutePlan currentRoute_;               // current vehicle plan
    PRoute solutionRoute_;                  // actual vehicle plan (performed plan)
    double dual_;
    double bestReducedCost_;                // best reduce cost of the routes generated after solving its subproblem


    // Constructor and Destructor
    Vehicle(int vehicleId, const string &name, float readyTime, float endTime, int departIndex, int capacity,
            int bikeLoad, PNode &departNode);

    virtual ~Vehicle();

    void setCurrentRoute(PRoute &currentRoute);
    void setEmptyRoute();
};



#endif //VEHICLE_H

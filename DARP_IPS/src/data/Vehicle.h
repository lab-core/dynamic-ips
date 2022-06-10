//
// Created by Ella on 2021-09-08.
//

#ifndef _VEHICLE_H
#define _VEHICLE_H

#include "data/Route.h"


//-----------------------------------------------------------------------------
//  vehicle class
//  contains the vehicle info like working time, capacity and passengers
//-----------------------------------------------------------------------------

class Vehicle {
public:
    int vehicleID_;                         // vehicle ID
    float startTime_;                       // vehicle start time
    float endTime_;                         // vehicle end time
    int capacity_;                          // vehicle capacity
    int numPassengers_;                     // number of passengers in the vehicle
    float departTime_;                      // time the vehicle arrives at its departing stop for the epoch
    std::vector<std::string> onboards_;     // list of nodeIDs of the drop-off points for the onboard passengers
    std::vector<PRoute> generatedRoutes_;   // list of generated routes
    std::string departID_;
    std::string sinkID_;
    PRoute currentRoute_;
    PRoute solutionRoute_;
    PRoute emptyRoute_;
    double dual_;
    double bestReducedCost_;
    double idleTime_;


    // Constructor and Destructor
    Vehicle(int vehicleId, int capacity, float departTime, float endTime, std::string departID, std::string sinkID);

    virtual ~Vehicle();

    // Setters
    void setDepartTime(float departTime);
    void setEmptyRoute(PInstance &pInst);
    void setCurrentRoute(PRoute currentRoute);

    void setEndTime(float endTime);

    // Display function
    std::string toString() const;

    // function to update vehicle depart time at each time and
    // update the situation of nodes and ride requests

    void updateState(int epoch, int &epochLength);
    void updateStateTime(float stopTime);
};



#endif //_VEHICLE_H

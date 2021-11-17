//
// Created by Ella on 2021-09-08.
//

#ifndef _VEHICLE_H
#define _VEHICLE_H

#include "utilities/MyTools.h"
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
    PRoute* currentRoute_;


    // Constructor and Destructor
    Vehicle(int vehicleId, int capacity, float startTime, float endTime);

    virtual ~Vehicle();

    // Setters
    void setDepartTime(float departTime);

    // Display function
    std::string toString() const;

    // function to update vehicle depart time at each time and
    // update the situation of nodes and ride requests
    void updateStatus(int epoch);
};



#endif //_VEHICLE_H

//
// Created by Ella on 2021-09-08.
//

#ifndef VEHICLE_H
#define VEHICLE_H

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
    PNode departNode_;                      // vehicle departing stop
    PNode sinkNode_;                        // vehicle sink node
    std::vector<std::string> onboards_;     // list of nodeIDs of the drop-off points for the onboard passengers

    PRoute currentRoute_;                   // current vehicle plan
    PRoute solutionRoute_;                  // actual vehicle plan (performed plan)
    PRoute emptyRoute_;                     // empty route which may contain drop of points
    double dual_;
    double InitialDual_;                    // when in parameters we use penalties as duals we save previous duals in it
    double bestReducedCost_;                // best reduce cost of the routes generated after solving its subproblem
    float idleTime_;                        // idle time of the vehicle
    float score_;                           // calculated based on earliest possible pickup (for selection vehicle portion)
    bool idle_;
    int vehicleIndex_;                      // used for considering a part of vehicle constraints in master problems
    std::bitset<MAX_BIT_SIZE> graphRequests_;// is not used now (help in selecting column disjoint columns to insert)


    // Constructor and Destructor
    Vehicle(int vehicleId, int capacity, float departTime, float endTime, PNode &departNode, PNode & sinkNode);

    virtual ~Vehicle();

    // Setters
    void setDepartTime(float departTime);
    void setEmptyRoute(PInstance &pInst);
    void setSolutionRoute();
    void setCurrentRoute(PRoute &currentRoute);

    // Display function
    std::string toString() const;

    // function to update vehicle depart time at each time and
    // update the situation of nodes and ride requests
    void updateState(int epoch, int &epochLength, float simulationStart);
    void updateStateTime(float elapsedTime, float &epochLength, float simulationStart);
    void updateCurrentRoute(float elapsedTime);

    // this function is called at the end of algorithm to set the final stos of the solution based on final epoch
    void finalizeSolutionRoutes() const;
    void updateDepartTime(float departTime);
};



#endif //VEHICLE_H

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
    float departTimeActual_;                      // time the vehicle arrives at its departing stop for the epoch
    PNode departNodeActual_;
    std::vector<std::string> onboards_;     // list of nodeIDs of the drop-off points for the onboard passengers
    PNode departNode_;
    PNode sinkNode_;
//    std::string sinkNode_;
    PRoute currentRoute_;
    PRoute solutionRoute_;
    PRoute emptyRoute_;
    double dual_;
    double InitialDual_;
    double bestReducedCost_;
    float idleTime_;
    float score_;
    int zoneID_;
    bool selected_;                         // this variable indicates which subProblem is selected to be solved
    bool idle_;
    int vehicleIndex_;
    std::bitset<MAX_SIZE> graphRequests_;


    // Constructor and Destructor
    Vehicle(int vehicleId, int capacity, float departTime, float endTime, PNode &departNode, PNode & sinkNode);
    Vehicle(int vehicleId, int capacity, float departTime, float endTime, PNode &departNode, PNode & sinkNode, int zoneID);

    virtual ~Vehicle();

    // Setters
    void setDepartTime(float departTime);
    void setEmptyRoute(PInstance &pInst);
    void setSolutionRoute();
    void setCurrentRoute(PRoute &currentRoute);

    // this function set bestReducedCost_ to infinity
    void resetBestReducedCost();

    // Display function
    std::string toString() const;

    // function to update vehicle depart time at each time and
    // update the situation of nodes and ride requests
    void updateState(int epoch, int &epochLength, float simulationStart);
    void updateStateTime(float elapsedTime, float &epochLength, float simulationStart);
    void updateCurrentRoute(float elapsedTime);

    // this function is called at the end of algorithm to set the final stos of the solution based on final epoch
    void finalizeSolutionRoutes(PInstance & pInst) const;
    void updateDepartTime(float departTime);
};



#endif //VEHICLE_H

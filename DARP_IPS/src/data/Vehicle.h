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
    std::vector<std::string> onboards_;     // list of the nodeIDs for drop-off points for the onboard passengers

    PRoute currentRoute_;                   // current vehicle plan
    PRoute greedyRoute_;                   // current vehicle plan
    PRoute solutionRoute_;                  // actual vehicle plan (performed plan)
    PRoute emptyRoute_;                     // empty route which may contain a drop of points
    float dual_;
    float InitialDual_;                    // used to dave duals when we use penalties as the duals
    float bestReducedCost_;                // best reduced cost of the routes generated after solving its subproblem
    float score_;                           // calculated based on the earliest possible pickup (vehicle portion)
    bool idle_;
    int vehicleIndex_;                      // used for considering a part of vehicle constraints in the Master problems
    boost::dynamic_bitset<> graphRequests_;// is not used now (help in selecting column disjoint columns to insert)
    int numPickup_;
    bool stateChanged_;
    bool removePickup_;
    bool removeDrop_;
    int preSolvePick_;

    // KPIs
    float idleTime_;                        // idle time of the vehicle
    float serviceTime_;                     // service time of the vehicle
    float driveFullTime_;                   // time the vehicle drives with passengers
    float driveEmptyTime_;                  // time the vehicle drives empty to reach passengers
    float returnEmptyTime_;                 // time the vehicle drives empty to return
//    boost::dynamic_bitset<> coveredRequests;


    // Constructor and Destructor
    Vehicle(int vehicleId, int capacity, float departTime, float endTime, const PNode &departNode, const PNode & sinkNode);

    virtual ~Vehicle();

    // Setters
    void setDepartTime(float departTime);
    void setEmptyRoute(const PInstance &pInst);
    void setSolutionRoute();
    void setCurrentRoute(const PRoute &currentRoute);

    // Display function
    std::string toString() const;

    // function to update vehicle depart time at each time and
    // update the situation of nodes and ride requests
    void updateStateTime(const PInstance & mainInst, float elapsedTime, boost::dynamic_bitset<> &removedRequests);
    void updateCurrentRoute(float elapsedTime, float wait_W1, float ride_W2);

    // this function is called at the end of the algorithm to set the final stos of the solution based on final epoch
    void finalizeSolutionRoutes(float elapsedTime);
    void updateDepartTime(float departTime, float wait_W1, float ride_W2);
    void handleIdleState(float epochEndTime);
    void setRequestStatus(const PNode &node, float reachTime) const;
    void adjustDuals();
 //   void checkCoveredRequests(std::vector<PRoute> &availableRoutes, int nbRequests);
};



#endif //VEHICLE_H

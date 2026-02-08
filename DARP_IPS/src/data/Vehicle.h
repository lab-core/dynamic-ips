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
    int vehicleID_;                         // Unique ID for each vehicle instance
    float startTime_;                       // vehicle start time
    float endTime_;                         // vehicle end time
    int capacity_;                          // vehicle capacity
    int numPassengers_;                     // number of passengers in the vehicle
    float departTime_;                      // time the vehicle arrives at its departing stop for the epoch
    PNode departNode_;                      // vehicle departing stop
    PNode sinkNode_;                        // vehicle sink node
    std::vector<std::string> onboards_;     // list of the nodeIDs for drop-off points for the onboard passengers

    PRoute currentRoute_;                   // current vehicle plan
    PRoute greedyRoute_;                    // route creating based on greedy insertion in initialization
    PRoute solutionRoute_;                  // actual vehicle plan (performed plan)
    PRoute emptyRoute_;                     // empty route which may contain a drop of points
    float dual_;                            // vehicle duals (sigma)
    float InitialDual_;                     // vehicle duals (sigma) used to save duals
    float bestReducedCost_;                 // best reduced cost of the routes generated after solving its subproblem
    int vehicleIndex_;                      // used for considering a part of vehicle constraints in the Master problems
    int numPickup_;                         // the selected limit for the number of pickups to solve SP
    bool stateChanged_;                     // has the state of the vehicle changed from last epoch (remove stops)
    bool removePickup_;                     // is a pickup stop node removed from the last epoch
    bool removeDrop_;                       // is a drop-off stop node, remove from the last epoch
    std::vector<std::string> removeNodes_;
    int preSolvePickLimit_;                 // the limit on the number of pickups in the prior solution of SP

    // KPIs
    float idleTime_;                        // idle time of the vehicle
    float serviceTime_;                     // service time of the vehicle
    float driveFullTime_;                   // time the vehicle drives with passengers
    float driveEmptyTime_;                  // time the vehicle drives empty to reach passengers
    float returnEmptyTime_;                 // time the vehicle drives empty to return
    int routeAvail_;
    int nbCommitted_;


    // Constructor and Destructor
    Vehicle(int vehicleId, int capacity, float departTime, float endTime, const PNode &departNode, const PNode & sinkNode);
    virtual ~Vehicle();

    // Display function
    std::string toString() const;

    // Setters
    void setDepartTime(float departTime);
    void setEmptyRoute(const PInstance &pInst);
    void setSolutionRoute();
    void setCurrentRoute(const PRoute &currentRoute);

    // function to update vehicle departure time at each time and
    // update the situation of nodes and ride requests
    void updateStateTime(const PInstance & mainInst, float elapsedTime, boost::dynamic_bitset<> &removedRequests);

    // function used to update the departure time of the route created for the vehicle in anytime mode when epoch<30 (s)
    void updateCurrentRoute(float elapsedTime, float wait_W1, float ride_W2);
    // helper function in updateCurrentRoute
    void updateDepartTime(float departTime, float wait_W1, float ride_W2);

    // this function is called at the end of the algorithm to set the final stos of the solution based on the final epoch
    void finalizeSolutionRoutes(float elapsedTime);

    // Handle idle state for vehicles with no stops
    void handleIdleState(float epochEndTime);

    // update the state of a request when its pick/drop node is visited by a vehicle
    void setRequestStatus(const PNode &node, float reachTime) const;

    // function to adjust duals and make sure their summation equals c_r
    void adjustDuals();
    void adjustZeroDuals();
    void adjustCPZeroDuals(float wait_W1, float ride_W2);

    std::string toStringOut(int epoch) const;
};



#endif //VEHICLE_H

//
// Created by Ella on 2021-10-26.
//

#ifndef _ROUTE_H
#define _ROUTE_H

#include "data/Graph.h"
#include <ilcplex/ilocplex.h>


//---------------------------------------------------------------------------------------------
//  Route class
//  Route for each vehicle
//---------------------------------------------------------------------------------------------

class Route {
private:
    const unsigned int routeID_;                // variable name or route ID


public:
    static unsigned int routeCount_;            // Counter the number of routes generated
    const char* name_;
    int vehicleID_;                             // the vehicle for which the route has created
    double totalDelay_;                         // sum of waiting times of the requests served by the route
    vector<PNode> routeNodes_;                  // ordered list of the nodes that are visited within the route
    std::vector<unsigned int> routeRequests;    // list of requests served by the route
    std::vector<float> plannedReachTime_;       // time that vehicle is planned to reach each node
    std::vector<int> plannedPassengers_;        // number of passengers in the vehicle at each node
    double reducedCost_;
    float incompatibilityDegree;
    unsigned int routeSize_;                    //number of stops in the route including start and stop

    // Constructor and Destructor
    Route(int vehicleId);
    virtual ~Route();

    // Getters and Setters
    void setIncompatibilityDegree(float incompatibilityDegree);
    const unsigned int getRouteId() const;
    void updateReducedCost(IloNumArray& requestDuals, IloNumArray& vehicleDual, std::unordered_map<int, int>& requestToOrder);

    // these functions are used to add nodes to the routes
    void addSource(PNode node, float departTime, int departPassengers);
    void addNode(PNode node);
    void addNode(PNode node, float departTime, int departPassengers);

    // this function is used to remove completed nodes from the routes
    void removeNode(int nodeIndex);

    // Display function
    std::string toString() const;

};

inline bool operator == (const PRoute &lhs, const PRoute &rhs) {
    return (
            ((lhs->totalDelay_ == rhs->totalDelay_) && (lhs->routeSize_ == rhs->routeSize_)&&
                    (lhs->plannedReachTime_.back() == rhs->plannedReachTime_.back()) &&
                    (lhs->vehicleID_ == rhs->vehicleID_ ))
            );
};

#endif //_ROUTE_H


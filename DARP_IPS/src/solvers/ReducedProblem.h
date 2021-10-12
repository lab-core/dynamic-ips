//
// Created by Ella on 2021-10-09.
//

#ifndef _REDUCEDPROBLEM_H
#define _REDUCEDPROBLEM_H

#include "utilities/MyTools.h"
#include "data/Graph.h"
struct Route;
typedef std::shared_ptr<Route> PRoute;

struct Route {
    int vehicleID_;                         // the vehicle for which the route has created
    const int routeID_;                     // variable name or route ID
    float totalDelay_;                      // sum of waiting times of the requests served by the route
    vector<PNode> routeNodes_;             // ordered list of the nodes that are visited within the route
    vector<int> routeRequests_;       // set of requests covered by the route
    std::vector<float> plannedReachTime_;    // time that vehicle is planned to reach each node
    std::vector<int> plannedPassengers_;     // number of passengers in the vehicle at each node

    // Constructor and Destructor
    Route(int vehicleId, const int routeId);
    virtual ~Route();

    // function that create column from the route
    std::vector<bool> createRouteColumn(int nbRequests, int nbVehicles);

};


class ReducedProblem {

};


#endif //_REDUCEDPROBLEM_H

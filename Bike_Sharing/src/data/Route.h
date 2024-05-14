//
// Created by Ella on 2021-10-26.
//

#ifndef ROUTE_H
#define ROUTE_H

#include "data/Graph.h"
#include "data/Stop.h"


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
    float totalBonus_;                           // sum of costs in performing the tasks
    double reducedCost_;
    double totalDuration_;
    unsigned int routeSize_;
    std::vector<PTask> assignedTasks_;          // list of tasks assigned to be performed
    std::vector<PStop> routeStops_;
    bool mpAdded_;                              // is the route has already been added to the RP/MP/CG model
    double score_;                              // equals to the reduced cost/number of pickups(or tasks)


    // Constructor and Destructor
    explicit Route(int vehicleId);
    virtual ~Route();

    // Getters and Setters
    unsigned int getRouteId() const;

    // these functions are used to add nodes to the routes
    void addSource(PNode &node, float departTime, int departPassengers);
    void addNode(PNode &node);
    void addTask(PTask &task);

};



class RoutePlan {

public:
    int vehicleID_;                             // the vehicle for which the route has created
    float totalDelay_;                          // sum of delays in performing the tasks after release time
    float totalCost_;                           // sum of costs in performing the tasks
    PStop departStop_;                          // departure point of the vehicle
    std::vector<PTask> assignedTasks_;          // list of tasks assigned to be performed

    bool mpAdded_;                              // is the route has already been added to the RP/MP/CG model
    double score_;                              // equals to the reduced cost/number of pickups(or tasks)

    // Constructor and Destructor
};




#endif //ROUTE_H


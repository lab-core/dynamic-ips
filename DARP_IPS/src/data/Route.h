//
// Created by Ella on 2021-10-26.
//

#ifndef ROUTE_H
#define ROUTE_H

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
    float totalDelay_;                         // sum of waiting times of the requests served by the route
    vector<PNode> routeNodes_;                  // ordered list of the nodes that are visited within the route
//    std::vector<unsigned int> routeRequests_;   // list of requests served by the route
    std::vector<PRequest> routeRequests_;   // list of requests served by the route
    std::vector<float> plannedReachTime_;       // time that vehicle is planned to reach each node
    std::vector<int> plannedPassengers_;        // number of passengers in the vehicle at each node
    double reducedCost_;
    int incompatibilityDegree_;
    unsigned int routeSize_;                    //number of stops in the route including start and stop
//    Eigen::MatrixXd fullPattern_;

    // Constructor and Destructor
    explicit Route(int vehicleId);
    virtual ~Route();

    // Getters and Setters
    unsigned int getRouteId() const;
    void updateReducedCost(IloNumArray& requestDuals, IloNumArray& vehicleDual);

    // these functions are used to add nodes to the routes
    void addSource(PNode &node, float departTime, int departPassengers);
    void addNode(PNode &node);
    void addNode(PNode &node, float reachTime);
    void addNode(PNode &node, float departTime, int departPassengers);

    // this function is used to remove completed nodes from the routes
    void removeNode(int nodeIndex);

    // Display function
    std::string toString() const;

    // this function is for testing the validation of the route
    void testRoute(PVehicle & vehicle, MainAlgorithm &mainAlgorithm);

    // This function is to reset the status of the nodes in the route
    void resetRoute();

    bool equal (Route const &routeObj) {
        if ((this->totalDelay_ == routeObj.totalDelay_)&& (this->routeSize_ == routeObj.routeSize_) &&
                (this->plannedReachTime_.back() == routeObj.plannedReachTime_.back()))
            return true;
        else
            return false;
    }

    // this function is for creating the column pattern from the route for CPLEX or IncDegree
    /*void createFullPattern(std::unordered_map<unsigned int, int>& incRequestToOrder,
                           std::unordered_map<int, int> &incVehicleToOrder);*/
};

inline bool operator == (const PRoute &lhs, const PRoute &rhs) {
    std::cout << "comparing";
    return (
            ((lhs->totalDelay_ == rhs->totalDelay_) && (lhs->routeSize_ == rhs->routeSize_)&&
                    (lhs->plannedReachTime_.back() == rhs->plannedReachTime_.back()) &&
                    (lhs->vehicleID_ == rhs->vehicleID_ ))
            );
}

#endif //ROUTE_H


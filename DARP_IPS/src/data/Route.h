//
// Created by Ella on 2021-10-26.
//

#ifndef ROUTE_H
#define ROUTE_H

#include "data/Graph.h"
#include "data/Request.h"


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
    float totalDelay_;                          // sum of waiting times of the requests served by the route
    vector<PNode> routeNodes_;                  // the ordered list of the nodes that are visited within the route
    std::vector<PRequest> routeRequests_;       // list of requests served by the route
    std::vector<float> plannedDelay_;
    std::vector<float> plannedReachTime_;       // time that vehicle is planned to reach each node
    std::vector<float> plannedDepartTime_;      // time that vehicle is planned to reach each node
    std::vector<int> plannedPassengers_;        // number of passengers in the vehicle at each node
    float reducedCost_;
    int incompatibilityDegree_;
    unsigned int routeSize_;                    //number of stops in the route including start and stop
    float createTime_;                         // time that route is created through solving subproblems
    float totalLength_;
    boost::dynamic_bitset<> column_;
    bool isCompatible_;                         // return false if the corresponding column is compatible
    bool mpAdded_;                              // True if the route has already been added to the RP/MP/CG model
    bool cpAdded_;                              // True if the route has already been added to the CP model
    float score_;                              // equals to the reduced cost/number of pickups(or tasks)
    float lambda_;
    float waitScore_;
    float IncScoreRatio_;
    float IncScore_;
    int nbCommitted_;

    // Constructor and Destructor
    explicit Route(int vehicleId);
    virtual ~Route();

    // Getters and Setters
    unsigned int getRouteId() const;

    // these functions are used to add nodes to the routes
    void addSource(const PNode &node, float departTime, int departPassengers);
    void addNode(const PNode &node);

    bool reConstructRoute(PVehicle & vehicle);
    bool reConstruct(PVehicle &vehicle);
    // function to add node to the solution route
    void addNode(const PNode &node, float reachTime, float departTime);
    void addSink(const PNode &node);

    // this function is used to remove completed nodes from the routes
    void removeNode(int nodeIndex);

    // Display function
    std::string toString() const;
    std::string routeMetricsToString(int epoch, int RMPCounter) const;

    // this function is for testing the validation of the route
    void testRoute(const PVehicle & vehicle);

    // This function is to reset the status of the nodes in the route
    void resetRoute() const;
    void calcMarginalCosts(PVehicle & vehicle);

    bool equal(const Route& routeObj) const {
        return this->column_ == routeObj.column_
            && this->routeSize_ == routeObj.routeSize_
            && this->totalDelay_ == routeObj.totalDelay_;
    }
    void createColumn(int nbRequests);
};

inline bool operator == (const PRoute &lhs, const PRoute &rhs) {
    std::cout << "comparing";
    return (
            ((lhs->totalDelay_ == rhs->totalDelay_) && (lhs->routeSize_ == rhs->routeSize_)&&
             (lhs->plannedReachTime_.back() == rhs->plannedReachTime_.back()) &&
             (lhs->vehicleID_ == rhs->vehicleID_ ))
    );
}

static std::string makeKey(const Route& r, int vehicleID) {
    std::vector<int> ids;
    for (auto & req : r.routeRequests_) ids.push_back(req->taskIndex_);

    std::string key = std::to_string(vehicleID) + "|";
    for (size_t i = 0; i < ids.size(); ++i) {
        if (i) key += ",";
        key += std::to_string(ids[i]);
    }
    return key;
}

#endif //ROUTE_H


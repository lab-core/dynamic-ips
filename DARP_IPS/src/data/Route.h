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
    const unsigned int routeID_;                // Unique ID for each route instance


public:
    static unsigned int routeCount_;            // Global counter for total routes created
    const char* name_;
    std::string key_;
    int vehicleID_;                             // the vehicle for which the route has been created
    float totalWait_;                           // sum of waiting times of the requests served by the route
    float totalTripDelay_;                      // sum of trip delays of the requests served by the route
    float objCoef_;                             // objective coefficient of the route in the MP based on the weights 
    vector<PNode> routeNodes_;                  // the ordered list of the nodes that are visited within the route
    std::vector<PRequest> routeRequests_;       // list of requests served by the route
    std::vector<float> plannedDelay_;           // delay that vehicle is planned to have at each pickup node   
    std::vector<float> plannedReachTime_;       // time that vehicle is planned to reach each node
    std::vector<float> plannedDepartTime_;      // time that vehicle is planned to depart each node
    std::vector<int> plannedPassengers_;        // number of passengers in the vehicle at each node
    unsigned int routeSize_;                    // number of stops in the route, including start and stop
    float createTime_;                          // time that route is created through solving subproblems
    float totalLength_;                         // total length of the route    
    int incompatibilityDegree_;                 // the incompatibility degree of the route used in the ISUD algorithm
    boost::dynamic_bitset<> column_;            // bitset representing the requests included in the route used in MP/CP models
    bool isCompatible_;                         // return true if the corresponding column is compatible w.r.t. the current basis
    bool mpAdded_;                              // True if the route has already been added to the RP/MP/CG model
    bool cpAdded_;                              // True if the route has already been added to the CP model
    float reducedCost_;                         // reduced cost of the route 
    float normal_RC_;                           // normalized reduced cost used as the sorting criterion
    float lambda_;                              // lambda score value used as the sorting criterion
    float waitScore_;                           // wait score value used as the sorting criteria    
    float IncScoreRatio_;                       // incompatible score ratio used as the sorting criteria
    float IncScore_;                            // incompatible score used as the sorting criterion
    int nbCommitted_;                           // number of committed requests in the route

    // Constructor and Destructor
    explicit Route(int vehicleId);
    virtual ~Route();
    void swapState(Route& other) noexcept;

    // Update the route key based on objCoef_ and routeRequests_
    void setKey();

    // Getters and Setters
    unsigned int getRouteId() const;

    // Display function
    std::string toString() const;
    std::string routeMetricsToString(int epoch, int RMPCounter) const;

    // these functions are used to add nodes to the routes
    void addSource(const PNode &node, float departTime, int departPassengers);
    void addNode(const PNode &node);

    // function to reconstruct the generated routes in the pool from the last epoch based on the current state
    bool reConstructRoute(const PVehicle & vehicle);
    bool reConstruct1(const PVehicle &vehicle, float wait_W1, float ride_W2);
    bool reConstruct(const PVehicle &vehicle, float wait_W1, float ride_W2);

    // function to add node to the solution route
    void addNode(const PNode &node, float reachTime, float departTime);
    void addSink(const PNode &node);

    // This function is used to remove completed nodes from the routes
    void removeNode(int nodeIndex);

    // this function is for testing the validation of the route
    void testRoute(const PVehicle & vehicle);

    // This function is to reset the status of the nodes in the route
    void resetRoute() const;

    // This function is to calculate the marginal costs of adding nodes to the route
    void calcMarginalCosts(float wait_W1, float ride_W2);

    // This function is to compare two routes
    bool equal(const Route& routeObj) const {
        return this->column_ == routeObj.column_
            && this->routeSize_ == routeObj.routeSize_
            && this->totalWait_ == routeObj.totalWait_
            && this->totalTripDelay_ == routeObj.totalTripDelay_;
    }

    // This function is to create the column bitset for the route (for MP/CP models)
    void createColumn(int nbRequests);

    // This function is to calculate total trip delay of the route
    void calculateTripDelay(float wait_W1, float ride_W2);
};

// Compares two routes to check if they represent identical vehicle plans.
inline bool operator == (const PRoute &lhs, const PRoute &rhs) {
    std::cout << "comparing";
    return (
            ((lhs->objCoef_ == rhs->objCoef_) && (lhs->routeSize_ == rhs->routeSize_)&&
             (lhs->plannedReachTime_.back() == rhs->plannedReachTime_.back()) &&
             (lhs->vehicleID_ == rhs->vehicleID_ ))
    );
}

// Creates a unique string key for caching or route lookup based on
// vehicle ID and the task indices of the requests in the route.
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


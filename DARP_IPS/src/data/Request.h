//
// Created by Ella on 2021-09-13.
//

#ifndef REQUEST_H
#define REQUEST_H

#include "utilities/MyTools.h"
#include "Parameters.h"


//-----------------------------------------------------------------------------
// Request class
// object use save trip information received from passengers
//-----------------------------------------------------------------------------

class Request {
private:
    const unsigned int requestID_;              // Unique ID for each request instance
public:
    /*  attributes to define a request when it is released */

    static unsigned int requestCount_;          // Counter the number of requests
    const char* name_;
    int PickUpID_;                              // pick up location ID
    int DropOffID_;                             // Drop off location ID
    int pickZoneID_;                            // zone ID of the pickup location
    int dropZoneID_;                            // zone ID of the drop off location
    int nbPassengers_;                          // number of passengers to pick up or drop off
    float requestTime_;                         // time that request is received by the system (release time)
    float earlyPick_;                           // earliest possible pickup time for the request
    float serviceTime_;                         // time to perform pick up or drop off
    float minTravelTime_;                       // minimum travel time between pickup and drop off location (direct route)
    float maxTravelTime_;                       // maximum allowed travel time between pickup and drop off location
    
    RequestStatus requestStatus_;               // status of the request: unassigned, assigned, committed, served, not served
    float penalty_;                             // penalty of not serving in the current period

    /* Other attributes used in the optimization process */

    float initialEarlyPick_;                    // initial earliest pickup time for the request (before commitment)
    float latestPickup_;                        // latest possible pickup time for the request
    float latestDrop_;                          // latest possible drop off time for the request
    float Req_W3_;                              // weight considering the number of passengers for each request in the objective function

    float InitialDual_;                         // when in parameters we use penalties as duals, we save previous duals in it
    float dual_;                                // dual value (pi) of the request in the Master problem
    float lastDual_;                            // dual value (pi) of the request in the previous epoch
    float minDual_;                             // minimum dual value among all epochs
    float maxDual_;                             // maximum dual value among all epochs
    int taskIndex_;                             // request index (row) in the Master model
    int taskIndexLabel_;                        // request index in sub-problems graph
    float marginalCost_;                        // marginal cost of inserting this request in a vehicle route in the greedy method

    boost::dynamic_bitset<> coveredVehicles_;   // used to track the vehicles that covered this request by generated routes in prior epoch
    boost::dynamic_bitset<> insertedVehicles_;  // used to track the vehicles that have the potential to serve this request
    

    /* attributes to track the request assignment and commitment process */
    float assignTime_;                          // the time that the request is first assigned to a plan, which may change
    float plannedDelay_;                        // the planned delay for the request based on the current assigned route
    float commitTime_;                          // the time that the request is committed to be served
    float committedPickTime_;                   // the committed pickup time that is announced to the customer
    float pickTime_;                            // actual pickup time of the request
    float dropTime_;                            // actual drop-off time of the request
    int nbSwitch_;                              // number of times that a request's assigned switches between vehicles before commitment
    int allocVehicleID_;                        // the vehicle that serves the request
    int solVehicleID_;                          // the vehicle that serves the request in the current solution
    int epochVehicleID_;                        // the vehicle currently assigned to the request (used in counting displacement)


    // Constructor and Destructor
    Request(int pickUpID, int dropOffID, float requestTime, float earlyPick, int nbPassengers, float deltaTime, int pickZoneID, int dropZoneID);

    virtual ~Request();

    // Display function
    std::string toString() const;

    // Getters and Setters
    unsigned int getRequestId() const;
    void setMinTravelTime(float minTravelTime);
    void setMaxTravelTime(const float &alphaParam, const float &betaParam);

    // This function updates penalties based on elapsed time for any time framework
    void setPenalty(float elapsedTime, const PParameters &parameters, float simulationStart);

    // This function updates min and max dual values along epochs
    void setMaxMinDual();
    
};

inline bool operator == (const PRequest &lhs, const PRequest &rhs) {return (lhs->getRequestId() == rhs->getRequestId()); }

#endif //REQUEST_H

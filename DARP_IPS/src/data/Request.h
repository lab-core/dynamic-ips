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
    const unsigned int requestID_;      // request ID
public:
    static unsigned int requestCount_;  // Counter the number of requests
    const char* name_;
    int PickUpID_;                      // pick up location ID
    int DropOffID_;                     // Drop off location ID
    float requestTime_;                   // earliest possible pickup time for the request
    float earlyPick_;                   // earliest possible pickup time for the request
    float initialEarlyPick_;
    float latestPickup_;                // latest possible pickup time for the request
    float latestDrop_;

    float pickTime_;                    // actual pickup time of the request
    float dropTime_;                    // actual pickup time of the request
    float serviceTime_;                 // time to perform pick up or drop off
    float minTravelTime_;               // minimum travel time between pickup and drop off location
    float maxTravelTime_;               // maximum allowed travel time between pickup and drop off location
    float commitTime_;                  // the time that request is commited to be served
    float committedPickTime_;             // the time that is announced to the customer
    float plannedDelay_;
    float assignTime_;                  // the time that request is first assigned to a plan which may change


    int nbPassengers_;                  // number of passengers to pick up or drop off
    float penalty_;                     // penalty of not serving at the current period
    RequestStatus requestStatus_;       // status of request 0:no action 1:on board 2:complete
    float dual_;
    float lastDual_;
    float marginalCost_;
    float InitialDual_;                // when in parameters we use penalties as duals, we save previous duals in it
    float minDual_;
    float maxDual_;
    int allocVehicleID_;                // the vehicle that serves the request
    int solVehicleID_;                  // this is compared with initialVehicleID_ to calculate displacement
    int epochVehicleID_;
    int taskIndex_;                     // request index (row) in the Master model
    int taskIndexLabel_;                // request index in sub problems graph
    int pickZoneID_;
    int dropZoneID_;
    int nbSwitch_;
    boost::dynamic_bitset<> coveredVehicles_;


    // Constructor and Destructor
    Request(int pickUpID, int dropOffID, float requestTime, float earlyPick, int nbPassengers, float deltaTime, int pickZoneID, int dropZoneID);

    virtual ~Request();

    // Getters and Setters
    unsigned int getRequestId() const;
    void setMinTravelTime(float minTravelTime);
    void setMaxTravelTime(const float &alphaParam, const float &betaParam);

    // This function updates penalties based on elapsed time for any time framework
    void setPenalty(float elapsedTime, const PParameters &parameters, float simulationStart);

    // Display function
    std::string toString() const;
    void setMaxMinDual();
};

inline bool operator == (const PRequest &lhs, const PRequest &rhs) {return (lhs->getRequestId() == rhs->getRequestId()); }

#endif //REQUEST_H

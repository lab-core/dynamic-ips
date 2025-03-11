//
// Created by Ella on 2021-09-13.
//

#ifndef REQUEST_H
#define REQUEST_H

#include "utilities/MyTools.h"
#include "Parameters.h"

/*enum RequestStatus {NO_ACTION = 0, ON_BOARD = 1, COMPLETED = 2};
static const std::vector<std::string> reqStatusName = {
        "NO_ACTION", "ON_BOARD ", "COMPLETED" };*/

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
    float requestTime_;                   // earliest possible pick up time for the request
    float earlyPick_;                   // earliest possible pick up time for the request
    float latestPickup_;                // latest possible pick up time for the request
    float pickTime_;                    // actual pick up time of the request
    float dropTime_;                    // actual pick up time of the request
    float serviceTime_;                 // time to perform pick up or drop off
    float minTravelTime_;               // minimum travel time between pickup and drop off location
    float maxTravelTime_;               // maximum allowed travel time between pickup and drop off location
    float commitTime_;                  // the time that request is commited to be served
    float plannedPickTime_;             // the time that is announced to the customer
    float assignTime_;                  // the time that request is first assigned to a plan which may change


    int nbPassengers_;                  // number of passengers to pick up or drop off
    float penalty_;                     // penalty of not serving at current period
    RequestStatus requestStatus_;       // status of the request 0:no action 1:on board 2:complete
    float dual_;
    float InitialDual_;                // when in parameters we use penalties as duals we save previous duals in it
    float minDual_;
    float avgDual_;
    int allocVehicleID_;                // the vehicle that serve the request
    int solVehicleID_;                  // this is compared with initialVehicleID_ to calculate displacement
    int epochVehicleID_;
    int taskIndex_;                     // request index (row) in master model
    int taskIndexLabel_;                // request index in sub problems graph
    int pickZoneID_;
    int dropZoneID_;
    int nbSwitch_;


    // Constructor and Destructor
    Request(int pickUpID, int dropOffID, float requestTime, float earlyPick, int nbPassengers, float deltaTime);
    Request(int pickUpID, int dropOffID, float requestTime, float earlyPick, int nbPassengers, float deltaTime, int pickZoneID, int dropZoneID);

    virtual ~Request();

    // Getters and Setters
    unsigned int getRequestId() const;
    void setMinTravelTime(float minTravelTime);
    void setMaxTravelTime(float &alphaParam, float &betaParam);

    // This function update penalties based on elapsed time for any time framework
    void setPenalty(float elapsedTime, PParameters &parameters, float simulationStart);

    // Display function
    std::string toString() const;
};

inline bool operator == (const PRequest &lhs, const PRequest &rhs) {return (lhs->getRequestId() == rhs->getRequestId()); }

#endif //REQUEST_H

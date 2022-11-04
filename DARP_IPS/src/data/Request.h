//
// Created by Ella on 2021-09-13.
//

#ifndef REQUEST_H
#define REQUEST_H

#include "utilities/MyTools.h"
#include "Parameters.h"

//---------------------------------------------------------------------------------------------
//  request class
//  contains the request info request time, number of passengers, pickup and drop off location
//---------------------------------------------------------------------------------------------

enum RequestStatus {NO_ACTION = 0, ON_BOARD = 1, COMPLETED = 2};
static const std::vector<std::string> reqStatusName = {
        "NO_ACTION", "ON_BOARD ", "COMPLETED" };

enum SubSelectionStatus {NOT_SELECTED = 0, SELECTED = 1};



class Request {
private:
    const unsigned int requestID_;      // request ID
public:
    static unsigned int requestCount_;  // Counter the number of requests
    const char* name_;
    int PickUpID_;                      // pick up location ID
    int DropOffID_;                     // Drop off location ID
    float earlyPick_;                   // earliest possible pick up time for the request
    float pickTime_;                    // actual pick up time of the request
    float dropTime_;                    // actual pick up time of the request
    int nbPassengers_;                  // number of passengers to pick up or drop off
    float deltaTime_;                   // time to perform pick up or drop off
    float minTravelTime_;               // minimum travel time between pickup and drop off location
    float maxTravelTime_;               // maximum allowed travel time between pickup and drop off location
    float penalty_;                     // penalty of not serving at current period
    RequestStatus requestStatus_;       // status of the request 0:no action 1:on board 2:complete
    double dual_;
    double CPDual_;
    int vehicleID_;                     // the vehicle that serve the request
    int taskIndex_;
    int taskIndexLabel_;
    int taskIncIndex_;

    // Constructor and Destructor
    Request(int pickUpID, int dropOffID, float earlyPick, int nbPassengers, float deltaTime);

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

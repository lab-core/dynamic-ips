//
// Created by Ella on 2021-09-13.
//

#ifndef _REQUEST_H
#define _REQUEST_H

#include "utilities/MyTools.h"

//---------------------------------------------------------------------------------------------
//  request class
//  contains the request info request time, number of passengers, pickup and drop off location
//---------------------------------------------------------------------------------------------

enum RequestStatus {NO_ACTION = 0, ON_BOARD = 1, COMPLETED = 2};
static const std::vector<std::string> reqStatusName = {
        "NO_ACTION", "ON_BOARD ", "COMPLETED" };

enum SubSelectionStatus {NOTSELECTED = 0, SELECTED = 1};



class Request {
private:
    const unsigned int requestID_;      // request ID
public:
    static unsigned int requestCount_;  // Counter the number of requests
    const char* name_;
    /*double PickUpLatitude_;              // pick up location latitude
    double PickUpLongitude_;             // pick up location longitude
    double DropOffLatitude_;             // Drop off location latitude
    double DropOffLongitude_;            // Drop off location longitude*/
    int PickUpID_;                      // pick up location ID
    int DropOffID_;                     // Drop off location ID
    float earlyPick_;                   // earliest possible pick up time for the request
    float pickTime_;                    // actual pick up time of the request
    float dropTime_;                    // actual pick up time of the request
    int nbPassengers_;                  // number of passengers to pick up or drop off
    float deltaTime_;                   // time to perform pick up or drop off
    float minReachTime_;                // minimum travel distance between pickup and drop off location
    float minTravelTime_;               // minimum travel time between pickup and drop off location
    float maxTravelTime_;               // maximum allowed travel time between pickup and drop off location
    float penalty_;                     // penalty of not serving at current period
    int readEpoch_;
    RequestStatus requestStatus_;       // status of the request 0:no action 1:on board 2:complete
    SubSelectionStatus selectStatus_;            // status of the request based on previous solution of sub problems
    double dual_;
    int vehicleID_;                     // the vehicle that serve the request

    // Constructor and Destructor
    Request(int pickUpID, int dropOffID, float earlyPick, int nbPassengers, float deltaTime);

    virtual ~Request();

    // Getters and Setters
    void setPenalty(int epoch, PParameters &parameters, float simulationStart);
    const unsigned int getRequestId() const;
    void setMinTravelTime(float minTravelTime);
    void setMinReachTime(float minReachTime);
    void setMaxTravelTime(float &alphaParam, float &betaParam);

    // Display function
    std::string toString() const;
};

inline bool operator == (const PRequest &lhs, const PRequest &rhs) {return (lhs->getRequestId() == rhs->getRequestId()); };

#endif //_REQUEST_H

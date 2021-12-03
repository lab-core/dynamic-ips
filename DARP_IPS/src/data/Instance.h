//
// Created by Ella on 2021-09-12.
//

#ifndef _INSTANCE_H
#define _INSTANCE_H

#include "utilities/MyTools.h"
#include "data/Vehicle.h"
#include "data/Request.h"
#include "data/Graph.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------




// I consider 10 seconds for each passenger to pickup or drop off
#define TimePerPassenger 0         			// service time (time to pickup or drop off) per passenger
/*static const float alphaParam = 1.5;
static const float betaParam = 240;
static const float deltaPram = 420;
static const int epochLength = 30;*/



class Instance {
public:
    const std::string name_;                    // Name of the instance

    int nbVehicles_;                            // Number of vehicles
    std::vector<PVehicle> vehicles_;            // List of vehicles

    int nbRequests_;                            // Number of requests
    int nbNewRequests_;                         // Number of requests added after each epoch
    std::vector<PRequest> requests_;            // List of requests
    std::map<std::string , PRequest> nameToRequest_;
    PGraph instGraph_;

    // Constructor and Destructor
    Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbRequests, PGraph &mainGraph);
//    Instance(std::string &name, const PInstance &mainInst, int epoch, int lastRecRequests);
    Instance(const Instance &mainInst);

    virtual ~Instance();

    // Getters and Setters

    // Display function
    std::string toString();

    // function to set the data of the partial instance based on the epoch
    void buildPartialData(const PInstance &mainInst, std::vector<PRequest> penaltyRequests, int epoch, int lastRecRequests);

    // function to add requests from previous epochs to the current partial instance
    void addRequest(PRequest request, int epoch);
};


#endif //_INSTANCE_H

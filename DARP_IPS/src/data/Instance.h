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
    std::vector<PRequest> requests_;            // List of requests
    std::map<std::string , PRequest> nameToRequest_;
    PGraph mainGraph_;

    // Constructor and Destructor
    Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbRequests, PGraph &mainGraph);

    virtual ~Instance();

    // Getters and Setters

    // Display function
    std::string toString();
};


#endif //_INSTANCE_H

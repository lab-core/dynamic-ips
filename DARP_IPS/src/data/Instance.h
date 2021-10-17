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

// useful types
class Instance;
typedef std::shared_ptr<Instance> PInstance;
class Graph;
typedef std::shared_ptr<Graph> PGraph;

#define TimePerPassenger 10         			// service time (time to pickup or drop off) per passenger


class Instance {
public:
    const std::string name_;                    // Name of the instance

    int nbVehicles_;                            // Number of vehicles
    std::vector<PVehicle> vehicles_;            // List of vehicles

    int nbRequests_;                            // Number of requests
    std::vector<PRequest> requests_;            // List of requests

    PGraph mainGraph_;

    // Constructor and Destructor
    Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbRequests, PGraph &mainGraph);

    virtual ~Instance();

    // Getters and Setters

    // Display function
    std::string toString();
};


#endif //_INSTANCE_H

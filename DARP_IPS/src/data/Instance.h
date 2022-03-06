//
// Created by Ella on 2021-09-12.
//

#ifndef _INSTANCE_H
#define _INSTANCE_H

#include "utilities/MyTools.h"
#include "data/Vehicle.h"
#include "data/Request.h"
#include "data/Graph.h"
#include "data/TravelTime.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

enum SortVehicle { DUAL = 0, DEPART_TIME = 1, ROURE_SIZE = 2, BEST_REDUCE_COST = 3};


// I consider 10 seconds for each passenger to pickup or drop off
#define TimePerPassenger 0         			// service time (time to pickup or drop off) per passenger

class Instance {
public:
    const std::string name_;                    // Name of the instance

    int nbVehicles_;                            // Number of vehicles
    std::vector<PVehicle> vehicles_;            // List of vehicles

    int nbRequests_;                            // Number of requests
    int nbNewRequests_;                         // Number of requests added after each epoch
    std::vector<PRequest> requests_;            // List of requests
    std::unordered_map<std::string , PRequest> nameToRequest_;
    PGraph instGraph_;
    PParameters parameters_;

    // Constructor and Destructor
    Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbRequests, PGraph &mainGraph);
    Instance(const Instance &mainInst);
    virtual ~Instance();


    // Display function
    std::string toString();
    std::string solutionToString();

    // function to set the data of the partial instance based on the epoch
    void buildPartialData(const PInstance &mainInst, std::vector<PRequest> penaltyRequests, int epoch, int lastRecRequests);

    // function to add requests from previous epochs to the current partial instance
    void addRequest(PRequest request, int epoch, PParameters &parameters);

    // setting min travel times of requests based on the distance matrix
    // initializing vehicles empty route and current route
    // initialize the departure time of the vehicle (2 * epochLength)
    void setInitialTimes();

    // function to sort vehicles based on ID
    void restVehicleOrder();
    void sortVehicles(SortVehicle sortBase);
    void resetRequestsSelectStatus();

    // print solutions in csv files
    void saveSolutionRoutes(string routeResultDir);
    void saveRequestsResults(string requestResultDir);
    void saveEpochRoutes(string finalSolutionDir , int epoch);
    void saveISUDRoutes(string isudSolutionDir, int epoch, int isudIter);
};


#endif //_INSTANCE_H

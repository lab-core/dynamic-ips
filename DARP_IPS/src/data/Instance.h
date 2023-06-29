//
// Created by Ella on 2021-09-12.
//

#ifndef INSTANCE_H
#define INSTANCE_H

#include "utilities/MyTools.h"
#include "utilities/Tools.h"
#include "data/Vehicle.h"
#include "data/Request.h"
#include "data/Graph.h"
#include "data/TravelTime.h"
#include "utilities/InputPaths.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------
extern vector2D<float> durationMatrix_;
enum SortVehicle { DUAL = 0, DEPART_TIME = 1, ROURE_SIZE = 2, BEST_REDUCE_COST = 3, SCORE = 4};

// I consider 10 seconds for each passenger to pickup or drop off
#define TimePerPassenger 0         			// service time (time to pickup or drop off) per passenger

class Instance {
public:
    const std::string name_;                    // Name of the instance
    float simulationStartTime_;

    int nbVehicles_;                            // Number of vehicles
    std::vector<PVehicle> vehicles_;            // List of vehicles

    int nbRequests_;                            // Number of requests
    int nbOnboards_;                            // Number of initial onboard requests
    int nbWaiting_;                             // Number of requests at the initial state
    int nbNewRequests_;                         // Number of requests added after each epoch
    int nbLocations_;                           // Number of stop locations
    std::vector<PRequest> requests_;            // List of requests
    std::map<std::string , PRequest> nameToRequest_;
    vector<unsigned int> orderToRequest_;
    PGraph instGraph_;
    PParameters parameters_;
    std::stringstream instRepStr_;   // save all the results as one record


    // Constructor and Destructor
    Instance(std::string &name, float simulationStart, int nbVehicles, int nbOnboards, int nbReceived,
             std::vector<PVehicle> &vehicles, int nbRequests, int nbLocations, PGraph &mainGraph);
    Instance(const Instance &mainInst);
    Instance(const Instance &mainInst, int zoneID);
    void resetInstance();
//    virtual ~Instance();


    // Display function
    std::string toString();
    std::string solutionToString();

    // functions to build the set of requests to be served including:
    // update the set of available requests, removed completed requests and update onboards
    void buildPartialData(const PInstance &mainInst, std::vector<PRequest> &penaltyRequests, float elapsedTime, int lastRecRequests);
    void buildStaticData(const PInstance &mainInst);
    void buildDataZone(const PInstance &mainInst, int zoneID);

    // function to add requests from previous epochs to the current partial instance
    void addRequest(PRequest &request);

    // setting min travel times of requests based on the distance matrix
    // initializing vehicles empty route and current route
    // initialize the departure time of the vehicle (2 * epochLength)
    void setInitialTimes();

    // function to sort vehicles based on ID
    void restVehicleOrder();
    void sortVehicles(SortVehicle sortBase);

    // function to update penalties in rolling horizon approach
//    void updatePenaltiesEpoch(int epoch);

    // function to update penalties in any time approach
    void updatePenalties(float elapsedTime);

    //determine an order for requests to use in CPLEX modeling
    void updateRequestOrder();
    //determine an order for requests in labeling procedure
    void updateTaskIndexLabeling();

    // print solution in csv files
    std::string saveSolutionRoutes();
    std::string saveRequestsResults();
    // save the solution route of the vehicles (current solution of ISUD)
    std::string saveEpochRoutes(int epoch);

    // save the current route of the vehicles (current solution of ISUD)
    std::string saveISUDRoutes(int epoch, int isudIter);

    // this function save the creation time of the current solution in a csv
    std::string saveRoutesTimes(int epoch);
    void saveStatus(InputPaths &inputPaths, float simulationStart);

    // this function set all the selected vehicles to none (partially solving the subproblems)
    void resetVehicleSelection();

    // this function determined the number of unselected vehicles
    int getNbUnselectedVehicles();

    // Function to calculate the L1 norm of duals
    double calculateL1NormReq();
    double calculateL1NormVeh();

    std::string saveReqDuals(int epoch, int isudIter, string model);
    std::string saveVehDuals(int epoch, int isudIter, string model);
};


#endif //INSTANCE_H

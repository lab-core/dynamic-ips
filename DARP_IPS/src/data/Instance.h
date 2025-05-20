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
#include "utilities/InputPaths.h"
#include <numeric>

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

class Instance {
public:
    const std::string name_;                    // Name of the instance
    float simulationStartTime_;

    int nbVehicles_;                                    // Number of vehicles
    std::vector<PVehicle> vehicles_;                    // List of vehicles
    int nbZones_;
    std::unordered_map<int, PZone> zones_;

    int nbRequests_;                                    // Number of requests
    int nbRejected_;
    int nbOnboards_;                                    // Number of initial onboard requests
    int nbWaiting_;                                     // Number of requests at the initial state
    int nbNewRequests_;                                 // Number of requests added after each epoch
    int nbLocations_;                                   // Number of stop locations
    std::vector<PRequest> requests_;                    // List of requests
    std::unordered_map<std::string , PRequest> nameToRequest_;
    int nbTasks_;                                       // number of tasks (rows/requests) in the Master problem
    PGraph instGraph_;
    PParameters parameters_;
    std::stringstream instRepStr_;                      // save all the results as one record
    std::vector<int> selectedVehicles_;                     // list of the vehicles selected to solve the subproblem
    int nbReturn_;
    int nbIdle_;
    int nbPotentialIdle_;
    int nbStateChanged_;
    std::vector<PRequest> lastCommittedRequests_;


    // Constructor and Destructor
    Instance(std::string name, float simulationStart, int nbVehicles, int nbOnboards, int nbReceived,
             std::vector<PVehicle> &vehicles, int nbRequests, int nbLocations, PGraph mainGraph);
    Instance(const Instance &mainInst);
    void resetInstance();
//    virtual ~Instance();


    // Display function
    std::string toString() const;
    std::string solutionToString();

    // update the set of available requests, removed completed requests and update the set of on-boards
    void buildPartialData(const PInstance &mainInst, const std::vector<PRequest> &penaltyRequests, float elapsedTime, int lastRecRequests);
    void buildStaticData(const PInstance &mainInst, int lastRecRequests);

    // function to add requests from previous epochs to the current partial instance
    void addRequest(const PRequest &request);

    // setting min travel times of requests based on the distance matrix
    // initializing vehicles empty route and current route
    // initialize the departure time of the vehicle (2 * epochLength)
    void setInitialTimes() const;

    // function to sort vehicles based on ID
    void resetVehicleOrder();
    void sortVehicles(SortVehicle sortBase);

    void sortZones();
    // function to update penalties in any time approach
    void updatePenalties(float elapsedTime);

    //determine an order for requests to use in CPLEX modeling
    void updateRequestOrder();
    void updateRequestInc();
    //determine an order for requests in labeling procedure
    void updateTaskIndexLabeling() const;

    // print solution in csv files
    std::string saveSolutionRoutes() const;
    std::string saveRequestsResults() const;
    std::string saveVehicleResults() const;
    // save the solution route of the vehicles (current solution of ISUD)
    std::string saveEpochRoutes(int epoch) const;

    // save the current route of the vehicles (current solution of ISUD)
    std::string saveISUDRoutes(int epoch, int isudIter) const;

    // this function saves the creation time of the current solution in a csv
    std::string saveRoutesTimes(int epoch) const;
    void saveStatus(const InputPaths &inputPaths, float simulationStart, float instDuration) const;

    std::string saveReqDuals(int epoch, int isudIter, const string& model) const;
    std::string saveVehDuals(int epoch, int isudIter, const string& model) const;
    void resetAssignedVehicles() const;
    void setAssignedEpochVehicles(float assignTime) const;
    void setNodeIndices() const;
    void resetDuals();
};

int getIndex(const PNode& node, int id, int nbPairs);
std::string getNode(const PInstance& pInst, int vehicleID, int index, int nbPairs);
#endif //INSTANCE_H

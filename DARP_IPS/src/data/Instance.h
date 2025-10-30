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
#include <numeric>
#include "utilities/InputPaths.h"
#include "data/SolutionMetrics.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

class Instance {
public:
    const std::string name_;                                    // Name of the instance
    float simulationStartTime_;

    int nbVehicles_;                                            // Number of vehicles
    std::vector<PVehicle> vehicles_;                            // List of vehicles
    int nbZones_;                                               // Number of zones  
    std::unordered_map<int, PZone> zones_;                      // map of zoneID to zone objects
    int nbRequests_;                                            // Number of requests
    std::vector<PRequest> requests_;                            // List of requests
    std::unordered_map<std::string , PRequest> nameToRequest_;  // map of request name to request objects
    int nbOnboards_;                                            // Number of onboard requests
    int nbWaiting_;                                             // Number of requests at the initial epoch
    int nbNewRequests_;                                         // Number of requests added after each epoch
    int nbLocations_;                                           // Number of stop locations 
    int nbRejected_;                                            // Number of rejected requests      
    int nbCommitted_;                                           // Number of committed requests
    int nbInitialOnboards_;                                     // Number of initial onboard requests
     
    int nbTasks_;                                               // number of tasks (rows/requests) in the Master problem
    PGraph instGraph_;                                          // Main graph of the instance  
    PParameters parameters_;                                    // Parameters of the instance
    std::stringstream instRepStr_;                              // save all the results as one record
    std::vector<int> selectedVehicles_;                         // list of the vehicles selected to solve the subproblem
    int nbReturn_;                                              // number of returning vehicles at each epoch  
    int nbIdle_;                                                // number of idle vehicles at each epoch    
    float passPerVehicle_;                                      // average number of passengers per vehicle 
    float requestPerVehicle_;                                   // average number of requests per vehicle
    float nodePerVehicle_;                                      // average number of nodes per vehicle
    int nbStateChanged_;                                        // number of vehicles with state changes at each epoch
    std::vector<PRequest> lastCommittedRequests_;               // list of committed requests during last 30 (s)


    // Constructor and Destructor
    Instance(std::string name, float simulationStart, int nbVehicles, int nbOnboards, int nbReceived,
             std::vector<PVehicle> &vehicles, int nbRequests, int nbLocations, PGraph mainGraph);
    Instance(const Instance &mainInst);
    void resetInstance();

    // Display function
    std::string toString() const;
    std::string solutionToString();

    // adjust instance parameters based on the config settings
    void adjustParameters(const PConfig& config) const;

    // update the set of available requests, removed completed requests and update the set of on-boards
    void buildPartialData(const PInstance &mainInst, const std::vector<PRequest> &penaltyRequests, float elapsedTime, int lastRecRequests);
    void buildPartialData(const PInstance &mainInst, float elapsedTime, int lastRecRequests);
    void buildStaticData(const PInstance &mainInst, int lastRecRequests);

    // function to add new requests to the current partial instance (used at the end of each epoch to set duals)
    void addNewRequests(const PInstance &mainInst, float elapsedTime, int lastRecRequests);

    // function to add requests from previous epochs to the current partial instance
    void addRequest(const PRequest &request);

    // setting min travel times of requests based on the distance matrix
    // initialize the departure time of the vehicle (2 * epochLength)
    void setInitialTimes() const;

    // function to sort vehicles based on ID
    void resetVehicleOrder();

    // function to sort vehicles based on different criteria
    void sortVehicles(SortVehicle sortBase);

    // function to sort successor zones of each zone based on their distance
    void sortZones();

    // function to update penalties in any time approach
    void updatePenalties(float elapsedTime) const;

    // determine an order for requests to use in MP modeling
    void updateRequestOrder();

    // determine an order for requests to use in CP modeling
    void updateRequestInc();
    
    //determine an order for requests in labeling procedure
    void updateTaskIndexLabeling() const;

    // function to reset dual values of requests (equal to penalties)
    void resetDuals() const;

    // function to calculate vehicle metrics such as average number of requests, passengers, nodes per vehicle
    void calcVehicleMetric();

    // function to count committed requests in the current instance
    void countCommittedRequests();

    // function to reset assigned vehicles to requests
    void resetAssignedVehicles() const;

    // function to set assigned vehicles to requests (count displacements)
    void setAssignedEpochVehicles(float assignTime) const;

    // function to set node indices used in solving SP with CPLEX
    void setNodeIndices() const;

    //-----------------------------------------------------------------------------
    // Functions to save results and outputs in csv and txt files
    //-----------------------------------------------------------------------------

    // print solution routes in csv files
    std::string saveSolutionRoutes() const;

    // save the request results in a csv file
    std::string saveRequestsResults() const;

    // save the vehicle results in a csv file
    std::string saveVehicleResults() const;

    // write final outputs at the end of the simulation
    void writeFinalOutputs(const InputPaths& inputPaths, const PConfig& config) const;

    // save the current route of the vehicles (current solution of MP)
    std::string saveMPRoutes(int epoch, int mpIter) const;

    // this function saves the generated routes in current epoch with their creation time
    std::string saveRoutesTimes(int epoch) const;

    // save the status of the instance (requests and vehicles) at the current time
    void saveStatus(const InputPaths &inputPaths, float simulationStart, float instDuration) const;

    // functions to save the dual values of requests and vehicles
    std::string saveReqDuals(int epoch, int isudIter, const string& model) const;
    std::string saveVehDuals(int epoch, int isudIter, const string& model) const;

    // process requests and vehicles to collect solution metrics
    void processRequestsAndCollectMetrics(std::stringstream& repStr, SolutionMetrics& metrics) const;
    void processVehiclesAndCollectMetrics(SolutionMetrics& metrics) const;
};

int getIndex(const PNode& node, int id, int nbPairs);
std::string getNode(const PInstance& pInst, int vehicleID, int index, int nbPairs);
#endif //INSTANCE_H



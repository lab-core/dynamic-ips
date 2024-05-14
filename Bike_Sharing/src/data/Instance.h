//
// Created by Ella on 2021-09-12.
//

#ifndef INSTANCE_H
#define INSTANCE_H

#include "utilities/MyTools.h"
#include "utilities/Tools.h"
#include "data/Vehicle.h"
#include "data/Task.h"
#include "data/Graph.h"
#include "utilities/InputPaths.h"

//-----------------------------------------------------------------------------
//  Instance class
//  contains the instance data including vehicle info and requests
//-----------------------------------------------------------------------------

class Instance {
public:
    const std::string name_;                            // Name of the instance
    int nbVehicles_;                                    // Number of vehicles
    int nbTasks_;                                       // number of tasks (rows/requests) in master problem
    int nbLocations_;                                   // Number of stop locations
    std::vector<PTask> tasks_;                          // List of requests
    std::vector<PVehicle> vehicles_;                    // List of vehicles
    PParameters parameters_;
    std::stringstream instRepStr_;                      // save all the results as one record
    std::vector<int> selectedVehicles_;                 // list of the vehicles selected to solve sub problem
    PGraph instGraph_;


    // Constructor and Destructor
    Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbTasks, int nbLocations);

    Instance(std::string &name, int nbLocations);
    Instance(const Instance &mainInst);
    void resetInstance();


    // Display function
    std::string toString();

};


#endif //INSTANCE_H

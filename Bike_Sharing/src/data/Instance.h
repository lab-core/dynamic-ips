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
    int nbVehicles_;                                    // Number of vehicles
    int nbTasks_;                                       // number of tasks (rows/requests) in master problem
    std::vector<PTask> tasks_;                          // List of requests
    std::vector<PVehicle> vehicles_;                    // List of vehicles
    PParameters parameters_;
    PSolverOption subProOptions_;
    std::stringstream instRepStr_;                      // save all the results as one record
    PGraph instGraph_;

private:
    vector2D<float> durationMatrix_;
    std::unordered_map<int, std::unordered_map<int, float>> durationMatrixDict_;

public:
    // Constructor and Destructor
    Instance(std::string &name, int nbVehicles, std::vector<PVehicle> &vehicles, int nbTasks);

    Instance();
    Instance(const Instance &mainInst);

    vector2D<float> &getDurationMatrix();

    const std::unordered_map<int, std::unordered_map<int, float>> &getDurationMatrixDict() const;


    void setDurationMatrix(vector2D<float> &durationMatrix);

    // Method to create a Solution as a JSON file (dictionary of vehicleIDs and currentRoutes)
    rapidjson::Value createSolutionJson() const;


    // Display function
    std::string toString();

};


#endif //INSTANCE_H

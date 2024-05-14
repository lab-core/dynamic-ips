//
// Created by Ella on 2021-09-13.
//

#ifndef TASK_H
#define TASK_H

#include "utilities/MyTools.h"
#include "Parameters.h"
//---------------------------------------------------------------------------------------------
//  task class
//  The ``Task`` class mostly serves as a structure for storing basic
//  information about the re-balancing tasks.
//---------------------------------------------------------------------------------------------

class Task {
private:
    unsigned int taskID_;      // task ID
public:
    int stationID_;                  // corresponding station ID
    std::string locationID_;                 // corresponding location ID
    int locationIndex_;                 // corresponding location ID
    int nbTransfer_;                 // number of bikes to transfer (load/unload)
    float reachTime_;              // request time to perform the task
    float performTime_;              // the end time of performing the task
    double relocateBonus_;            // the cost of performing the task
    int assignedVehicleID_;          // the vehicle that serve the request
    double dual_;

    // Constructor and Destructor
    Task(int stationId, const string &locationId, int locationIndex, int nbTransfer, double relocateBonus);

    // Getters and Setters
    unsigned int getTaskId() const;

    // Display function
    std::string toString() const;
};


inline bool operator == (const PTask &lhs, const PTask &rhs) {return (lhs->getTaskId() == rhs->getTaskId()); }

#endif //TASK_H

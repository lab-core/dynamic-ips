//
// Created by Elahe Amiri on 2025-10-29.
//

#ifndef CP_GUROBI_CPP_SOLUTIONMETRICS_H
#define CP_GUROBI_CPP_SOLUTIONMETRICS_H

#include <iomanip>

//-----------------------------------------------------------------------------
//  Class to hold solution metrics
//-----------------------------------------------------------------------------

class SolutionMetrics {
public:
    // Request metrics
    int numServed = 0;                          // Number of served requests in route
    int totalNumServed = 0;                     // Cumulative number of served requests over multiple solutions 
    int totalCustomers = 0;                     // Total number of customers considered
    int numRejected = 0;                        // Number of rejected requests in route    

    // Time metrics
    float totalRequestsWaiting = 0.0f;          // Cumulative waiting time of all requests
    float totalWeightedWaiting = 0.0f;          // Cumulative weighted waiting time of all requests (customer weights)
    float totalRouteWaiting = 0.0f;             // Cumulative waiting time across all routes
    float totalTripDelay = 0.0f;                // Cumulative trip delay of all requests
    float totalWeightedTripDelay = 0.0f;        // Cumulative weighted trip delay of all requests (customer weights)

    // Vehicle metrics
    float idleTime = 0.0f;                      // Cumulative idle time of all vehicles
    float serviceTime = 0.0f;                   // Cumulative service time of all vehicles  
    float driveFullTime = 0.0f;                 // Cumulative drive with passengers of all vehicles
    float driveEmptyTime = 0.0f;                // Cumulative drive without passengers of all vehicles         
    float returnEmptyTime = 0.0f;               // Cumulative return to depot time of all vehicles
    int nbIdle = 0;                             // Number of idle vehicles


    // Route metrics
    unsigned int totalStops = 0;                // Total number of stops across all routes
    int totalStopLoad = 0;                      // Total number of passengers across all stops


    // Cost metrics
    float penalty = 0.0f;                       // Cumulative penalty of unserved requests
    float totalObj = 0.0f;                      // Cumulative objective value of all routes

    // Constructor
    SolutionMetrics() = default;

    // Reset all metrics to zero
    void reset();

    // Printing methods for summary section
    void printTotalMetrics(std::stringstream& repStr, int sentenceSize, int nbVehicles,
                          int nbRequests, int nbInitialOnboards) const;

    void printTotalMetricsInMinutes(std::stringstream& repStr, int sentenceSize,
                                   float secondsPerMinute) const;

    void printAverageMetrics(std::stringstream& repStr, int sentenceSize, int nbVehicles) const;
};

#endif //CP_GUROBI_CPP_SOLUTIONMETRICS_H
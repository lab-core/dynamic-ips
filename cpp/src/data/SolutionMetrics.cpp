//
// Created by Elahe Amiri on 2025-10-29.
//

#include "SolutionMetrics.h"
#include <sstream>


//-----------------------------------------------------------------------------
// Reset all metrics to zero
//-----------------------------------------------------------------------------
void SolutionMetrics::reset() {
    numServed = 0;
    totalNumServed = 0;
    totalCustomers = 0;
    numRejected = 0;
    totalRequestsWaiting = 0.0f;
    totalWeightedWaiting = 0.0f;
    totalRouteWaiting = 0.0f;
    totalTripDelay = 0.0f;
    totalWeightedTripDelay = 0.0f;
    idleTime = 0.0f;
    serviceTime = 0.0f;
    driveFullTime = 0.0f;
    driveEmptyTime = 0.0f;
    returnEmptyTime = 0.0f;
    nbIdle = 0;
    totalStops = 0;
    totalStopLoad = 0;
    penalty = 0.0f;
    totalObj = 0.0f;
}

//-----------------------------------------------------------------------------
// Print total metrics in seconds with request statistics
//-----------------------------------------------------------------------------
void SolutionMetrics::printTotalMetrics(std::stringstream& repStr, int sentenceSize,
                                       int nbVehicles, int nbRequests,
                                       int nbInitialOnboards) const {
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# FINAL OBJECTIVE VALUE" << " = " << penalty + totalObj << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL ROUTE WAIT TIME" << " = " << totalRouteWaiting << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL REQUEST  WAIT TIME" << " = " << totalRequestsWaiting << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL WEIGHTED WAIT TIME" << " = " << totalWeightedWaiting << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL TRIP DELAY" << " = " << totalTripDelay << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL WEIGHTED TRIP DELAY" << " = " << totalWeightedTripDelay << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL IDLE TIME" << " = " << idleTime << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL SERVICE TIME" << " = " << serviceTime << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL DRIVE FULL TIME" << " = " << driveFullTime << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL DRIVE EMPTY TIME" << " = " << driveEmptyTime << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL RETURN EMPTY TIME" << " = " << returnEmptyTime << " (s)" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL UN_SERVED REQUESTS" << " = " << nbRequests - totalNumServed - nbInitialOnboards << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL REQUESTS SERVED" << " = " << numServed << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL REQUESTS REJECTED" << " = " << numRejected << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL NUMBER OF REQUESTS" << " = " << nbRequests - nbInitialOnboards << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL NUMBER OF REQUESTS/ONBOARDS" << " = " << nbRequests << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL NUMBER OF SERVED PASSENGERS" << " = " << totalCustomers << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL NUMBER OF EMPTY VEHICLES" << " = " << nbIdle << std::endl;
    repStr << "#" << std::endl;
}

//-----------------------------------------------------------------------------
// Print total metrics in minutes
//-----------------------------------------------------------------------------
void SolutionMetrics::printTotalMetricsInMinutes(std::stringstream& repStr, int sentenceSize,
                                                float secondsPerMinute) const {
    repStr << std::setw(sentenceSize) << "# TOTAL ROUTE WAIT TIME" << " = " << totalRouteWaiting / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL REQUEST WAIT TIME" << " = " << totalRequestsWaiting / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL WEIGHTED WAIT TIME" << " = " << totalWeightedWaiting / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL TRIP DELAY" << " = " << totalTripDelay / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL WEIGHTED TRIP DELAY" << " = " << totalWeightedTripDelay / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL IDLE TIME" << " = " << idleTime / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL SERVICE TIME" << " = " << serviceTime / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL DRIVE FULL TIME" << " = " << driveFullTime / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL DRIVE EMPTY TIME" << " = " << driveEmptyTime / secondsPerMinute << " (min)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TOTAL RETURN EMPTY TIME" << " = " << returnEmptyTime / secondsPerMinute << " (min)" << std::endl;
    repStr << "#" << std::endl;
}

//-----------------------------------------------------------------------------
// Print average metrics per request, passenger, and vehicle
//-----------------------------------------------------------------------------
void SolutionMetrics::printAverageMetrics(std::stringstream& repStr, int sentenceSize,
                                         int nbVehicles) const {
    repStr << "# ----------------------   AVERAGE VALUES   ----------------------" << std::endl;

    if (numServed != 0) {
        repStr << std::setw(sentenceSize) << "# WAIT TIME PER REQUEST" << " = " << totalRequestsWaiting / static_cast<float>(numServed) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# WAIT TIME PER PASSENGER" << " = " << totalWeightedWaiting / static_cast<float>(totalCustomers) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# TRIP DELAY PER REQUEST" << " = " << totalTripDelay / static_cast<float>(numServed) << " (s)" << std::endl;
        repStr << std::setw(sentenceSize) << "# TRIP DELAY PER PASSENGER" << " = " << totalWeightedTripDelay / static_cast<float>(totalCustomers) << " (s)" << std::endl;
    }
    repStr << std::setw(sentenceSize) << "# PASSENGERS IN VEHICLE" << " = " << static_cast<float>(totalStopLoad) / static_cast<float>(totalStops) << std::endl;
    repStr << std::setw(sentenceSize) << "# IDLE TIME PER VEHICLE" << " = " << idleTime / static_cast<float>(nbVehicles) << std::endl;
    repStr << std::setw(sentenceSize) << "# SERVICE TIME PER VEHICLE" << " = " << serviceTime / static_cast<float>(nbVehicles) << std::endl;
    repStr << std::setw(sentenceSize) << "# DRIVE FULL TIME PER VEHICLE" << " = " << driveFullTime / static_cast<float>(nbVehicles) << std::endl;
    repStr << std::setw(sentenceSize) << "# DRIVE EMPTY TIME PER VEHICLE" << " = " << driveEmptyTime / static_cast<float>(nbVehicles) << std::endl;
    repStr << std::setw(sentenceSize) << "# RETURN EMPTY TIME PER VEHICLE" << " = " << returnEmptyTime / static_cast<float>(nbVehicles) << std::endl;
}
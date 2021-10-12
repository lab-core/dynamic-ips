//
// Created by Ella on 2021-10-09.
//

#include "ReducedProblem.h"

Route::Route(int vehicleId, const int routeId) : vehicleID_(vehicleId), routeID_(routeId) {
    totalDelay_ = 0;
}

Route::~Route() {}


/* for dynamic version this part should be changed since the length of column is not fixed */
/* ==================================needs modification=================================== */
std::vector<bool> Route::createRouteColumn(int nbRequests, int nbVehicles) {
    std::vector<bool> column;
    column.resize(nbRequests+nbVehicles);
    for (int i = 0; i < routeRequests_.size(); ++i) {
        column[routeRequests_[i]] = 1;
    }
    column[nbRequests+vehicleID_] = 1;
    return column;
}
/* ==================================needs modification=================================== */
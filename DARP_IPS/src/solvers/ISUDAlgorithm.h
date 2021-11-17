//
// Created by Ella on 2021-10-09.
//

#ifndef _REDUCEDPROBLEM_H
#define _REDUCEDPROBLEM_H

#include "utilities/MyTools.h"
#include "solvers/CPLEXModeler.h"
#include "data/Route.h"
#include "data/Instance.h"
#include "ReducedProblem.h"
#include "Eigen/Dense"

//---------------------------------------------------------------------------------------------
//  ISUD algorithm to solve the master problem
//---------------------------------------------------------------------------------------------

class ISUDAlgorithm {
public:
    PReducedProblem ReducedPro_;
    std::map<std::string , PRoute> generatedRoutes_;        // list of all generated routes
//    std::vector<PRoute> availableRoutes_;                   // list of available routes based on set of requests
    std::map<int, std::vector<PRoute>> availableRoutes_;    // list of available routes for each vehicle


    // Solution containers
    std::vector<PRoute> routeSolution_;
    std::vector<PRequest> zSolution_;

    Eigen::MatrixXd incMatrix_;                             // incompatibility matrix
    std::map<int, int> incRequestToOrder_;                  // order of requests in incompatibility matrix


    // Constructor and Destructor
    ISUDAlgorithm(PInstance &pInst);

    // this function create initial routes serving only one request and fill zSolution_ with available requests
    void initialization(PInstance &pInst);

    // function to create M2 matrix for each column in the current solution
    Eigen::MatrixXd calcM2Matrix(PRoute solColumn);

    // function to calculate incompatibility matrix
    void calcIncMatrix();

    // function to calculate incompatibility degree of a route
    void calcIncompatibility(PRoute &route);

    // this function update the incompatibility degree of availableRoutes and
    // order them based on the incompatibility degree and reduced cost
    void updateRoutesIncDegree(int &vehicleID);

    // this function updates the reduced cost for the routes in the pool
    void updateReducedCosts(int &vehicleID);

    // improve the solution through solving Reduce problem
    void reduceProImprove(PInstance &pInst);


    void createEmptyRoute(PInstance &pInst);

};


// function that create column from the route
const std::vector<PRoute> createInitialRoutes(PInstance &pInst);

#endif //_REDUCEDPROBLEM_H
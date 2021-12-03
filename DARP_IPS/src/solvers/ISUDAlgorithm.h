//
// Created by Ella on 2021-10-09.
//

#ifndef _REDUCEDPROBLEM_H
#define _REDUCEDPROBLEM_H

#include "utilities/MyTools.h"
#include "solvers/CPLEXModeler.h"
#include "data/Route.h"
#include "data/Instance.h"
#include "solvers/ReducedProblem.h"
#include "solvers/ComplementPro.h"
#include "Eigen/Dense"

//---------------------------------------------------------------------------------------------
//  ISUD algorithm to solve the master problem
//---------------------------------------------------------------------------------------------

class ISUDAlgorithm {
public:
    PReducedProblem ReducedPro_;
    PComplementPro CompPro_;
    std::map<std::string , PRoute> generatedRoutes_;        // list of all generated routes
//    std::vector<PRoute> availableRoutes_;                 // list of available routes based on set of requests
    std::map<int, std::vector<PRoute>> availableRoutes_;    // list of available routes for each vehicle


    // Solution containers
    std::vector<PRoute> routeSolution_;
    std::vector<PRequest> zSolution_;

    Eigen::MatrixXd incMatrix_;                             // incompatibility matrix
    std::map<int, int> incRequestToOrder_;                  // order of requests in incompatibility matrix

    int improveIter_;
    double objValue_;
    Tools::Timer *isudTime_;

    int improveStatus_;


    // Constructor and Destructor
    ISUDAlgorithm();

    virtual ~ISUDAlgorithm();

    // this function create initial routes serving only one request and fill zSolution_ with available requests
    // Reduced problem is also solved to initialized dual costs
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

    // this function create empty routes that only goes form source to sink
    void createEmptyRoute(PInstance &pInst);

    // improve the solution through solving Reduce problem
    void reduceProImprove(PInstance &pInst);

    // improve the solution through solving Complementary problem
    void CPImprove(PInstance &pInst);

    void solveISUD(PInstance &pInst, int epoch);

    // Display function
    std::string toString() const;

    // function to evaluate available routes and find proper ones to be added to the models
    void updateRoutesToAdd(int compDegree, PInstance &pInst);

};


#endif //_REDUCEDPROBLEM_H
//
// Created by Ella on 2021-10-09.
//

#ifndef _REDUCEDPROBLEM_H
#define _REDUCEDPROBLEM_H

#include "data/Instance.h"
#include "solvers/ReducedProblem.h"
#include "solvers/ComplementPro.h"
#include "solvers/ZoomReducedProblem.h"
#include "solvers/GreedyModeler.h"

//---------------------------------------------------------------------------------------------
//  ISUD algorithm to solve the master problem
//---------------------------------------------------------------------------------------------

class ISUDAlgorithm {
public:
    PReducedProblem ReducedPro_;
    PComplementPro CompPro_;
    PZoomReducedProblem ZoomPro_;
    std::unordered_map<std::string , PRoute> generatedRoutes_;        // list of all generated routes
    std::unordered_map<int, std::vector<PRoute>> availableRoutes_;    // list of available routes for each vehicle


    // Solution containers
    std::vector<PRoute> routeSolution_;
    std::vector<PRequest> zSolution_;


    Eigen::MatrixXd incMatrix_;                                     // incompatibility matrix
    std::unordered_map<unsigned int, int> incRequestToOrder_;                // order of requests in incompatibility matrix
    std::unordered_map<int, int> incVehicleToOrder_;

    int improveIter_;
    int isudIter_;
    double objValue_;
    Tools::Timer *isudTime_;
    Tools::Timer *RPTime_;
    Tools::Timer *CPTime_;
    Tools::Timer *ZOOMTime_;



    // Constructor and Destructor
    ISUDAlgorithm();

    virtual ~ISUDAlgorithm();

    // Getters and Setters
    void setObjValue();

    // this function create initial routes serving only one request and fill zSolution_ with available requests
    // Reduced problem is also solved to initialized dual costs
    void initialization(PInstance &pInst, bool emptyStart);

    // function to create M2 matrix for each column in the current solution
    static Eigen::MatrixXd calcM2Matrix(PRoute &solColumn);
    static Eigen::MatrixXd calcM2Matrix(int nbRows);

    // function to calculate incompatibility matrix
    void calcIncMatrix();
    void calcIncMatrixFull();

    // function to calculate incompatibility degree of a route
    void calcIncompatibility(PRoute &route);
    void calcIncompatibilityFull(PRoute &route);

    // this function update the incompatibility degree of availableRoutes and
    // order them based on the incompatibility degree and reduced cost
    void updateRoutesIncDegree(int &vehicleID);

    // this function updates the reduced cost for the routes in the pool
    void updateReducedCosts(int &vehicleID);

    void solveISUD(PInstance &pInst, int epoch, const string& isudSolutionDir);
    void solveISUDMIP(PInstance &pInst, const string& isudSolutionDir);

    // Display function
    std::string toString() const;

    // function to evaluate available routes and find proper ones to be added to the models
    void updateRoutesToAdd(int compDegree, PInstance &pInst);
    void updateRoutesToAddZoom(PInstance &pInst);
    static bool isCompatible(PRoute &solutionRoute, PRoute &comingRoute, std::unordered_map<unsigned int, int> &requestToOrder);

    void restGeneratedRoutes(PInstance &pInst);

};


#endif //_REDUCEDPROBLEM_H
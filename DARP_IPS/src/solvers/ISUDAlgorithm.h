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
//    PReducedProblem ReducedPro_;
    PComplementPro CompPro_;
    PZoomReducedProblem MIPReducedPro_;
//    std::unordered_map<std::string , PRoute> generatedRoutes_;        // list of all generated routes
//    std::unordered_map<int, std::vector<PRoute>> availableRoutes_;    // list of available routes for each vehicle
    vector2D<PRoute> availableRoutes_;
    int nbRoutes_;

    // Solution containers
    std::vector<PRoute> routeSolution_;
    std::vector<PRequest> zSolution_;
    int maxIncDegree_;
    int cpIncDegree_;

    Eigen::MatrixXd incMatrix_;                                     // incompatibility matrix
//    std::unordered_map<unsigned int, int> incRequestToOrder_;                // order of requests in incompatibility matrix
    std::map<int, int> incVehicleToOrder_;
    int nbCoveredTasks_;
    double maxReducedCost_;      // max threshold for the reduced costs selection in CP
    double minReducedCost_;

    int isudIter_;
    double objValue_;
    double GreedyobjValue_;
    myTools::Timer *isudTime_;
    myTools::Timer *RPTime_;
    myTools::Timer *CPTime_;
    myTools::Timer *isudMIPTime_;
    myTools::Timer *updateDegreeTime_;
    myTools::Timer *timer1;

    Tools::LogOutput* pLogIsudResultsStream_;
    Tools::LogOutput* pLogIterSolutionStream_;

    // Constructor and Destructor
    explicit ISUDAlgorithm(InputPaths &inputPaths);

    virtual ~ISUDAlgorithm();

    // Getters and Setters
    void setObjValue();

    // this function create initial routes serving only one request and fill zSolution_ with available requests
    // Reduced problem is also solved to initialized dual costs
    void initialization(PInstance &pInst);

    // function to create M2 matrix for each column in the current solution
    static Eigen::MatrixXd calcM2Matrix(PRoute &solColumn);
    static Eigen::MatrixXd calcM2Matrix(int nbRows);

    // function to calculate incompatibility matrix
    void calcIncMatrix();
    void calcIncMatrixFull();

    // function to calculate incompatibility degree of a route
    void calcIncompatibility(PRoute &route);
    void calcIncompatibilityFull(PRoute &route);
    void calcIncompatibilityMatrix();

    // this function update the incompatibility degree of availableRoutes and
    // order them based on the incompatibility degree and reduced cost
    void updateIncDegrees(PInstance &pInst);
    void updateRoutesIncDegree(int &vehicleID);

    // this function updates the reduced cost for the routes in the pool
    void updateReducedCosts(int &vehicleID);
    void updateReducedCosts(PInstance &pInst, int &vehicleID);
    void solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths);

    void solveISUDMIP(PInstance &pInst, InputPaths &inputPaths);
    void solveRP_MIP(PInstance &pInst, int compDegree, InputPaths &inputPaths);

    // Display function
    std::string toString() const;
    std::string toStringTimersTotal() const;
    std::string toStringTimersAvg(int epoch) const;

    // function to evaluate available routes and find proper ones to be added to the models
    void updateRoutesToAdd(int compDegree, PInstance &pInst);
    void updateRoutesToAddZoom(PInstance &pInst);
    static bool isCompatible(PRoute &solutionRoute, PRoute &comingRoute, std::map<unsigned int, int> &requestToOrder);

    // function to save the reduced costs and incompatibility degree of the created routes
    void save_IncDegree_RDCost(InputPaths &inputPaths, int epoch, int isudIter);
    std::string save_ISUDResults(int epoch, const std::string& model, int nbColumns);

//    void updatePatterns(PInstance &pInst);
//    void updateFullPattern();
};


#endif //_REDUCEDPROBLEM_H
//
// Created by Ella on 2021-11-10.
//

#ifndef DARP_IPS_REDUCEDPROBLEM_H
#define DARP_IPS_REDUCEDPROBLEM_H

#include "solvers/MasterModeler.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------


class ReducedProblem : public MasterModeler{
public:
    // Variables
    IloNumVarArray routeVar_;
    IloNumVarArray zVar_;

    IloNumArray zLb_;
    IloNumArray rLb_;
    double objValue_;
    double auxObjValue_;

    std::vector<PRoute> compRoutes_;

    // Constructor and Destructor
    ReducedProblem();


    // this function initialized the model and define empty set of constraints
    void ResetRPModel();

    // this function adds routeVar to the model
    void addRouteVar(PRoute &newRoute, PInstance &pInst);
    void addRouteVars(std::vector<PRoute> &newRoutes);

    // this function adds zVar to the model used for the routes that served only one request
    void addZVars(std::vector<PRequest> &requests);
    void addZVar(PRequest &request);

    // this function add one route at each iteration of the algorithm during one epoch
    void updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution);

    // this function build the model at the start of each epoch
    void buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    // solve functions
    void solveModelLP(PInstance &pInst, InputPaths &inputPaths);

    void solveModelInt(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                       InputPaths &inputPaths, float availableTime, double preObj);
    void solveModelLPInt(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                         InputPaths &inputPaths, float availableTime, double preObj);

    // function to check whether the route is repeated before
    static bool isColumnRepeat(std::vector<PRoute> &routeSet, PRoute &newRoute, std::map<unsigned int, int>& requestToOrder);

    void solveModelIntAux(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                          InputPaths &inputPaths, float availableTime, double preObj);

    // Display function
    std::string toString() const override;
    void restartRP();

};


#endif //DARP_IPS_REDUCEDPROBLEM_H

//
// Created by Ella on 2021-11-10.
//

#ifndef DARP_IPS_REDUCEDPROBLEM_H
#define DARP_IPS_REDUCEDPROBLEM_H

#include "CplexModeler.h"
#include "DualAuxSolver.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------


class ReducedProblem : public CplexModeler{
public:
    // Variables
    IloNumVarArray routeVar_;               // route variables
    IloNumVarArray zVar_;                   // request(z) variables
    IloNumVarArray uVar_;
    IloNumVarArray vVar_;


    float objValue_;
    float auxObjValue_;                    // objective of auxiliary model use for getting duals from MIP
    std::vector<int> routeSolutionIndex_;

    std::vector<PRoute> compRoutes_;        // list of route variables in the model

    // Constructor and Destructor
    ReducedProblem();


    // this function initializes the model and defines an empty set of constraints
    void ResetRPModel();

    // this function adds routeVar to the model
    void addRouteVar(PRoute &newRoute, PInstance &pInst);
    void addRouteVarFloat(PRoute &newRoute, PInstance &pInst);

    // this function adds zVar to the model used for the routes that served only one request
    void addZVar(PRequest &request);

    // this function adds one route at each iteration of the algorithm during one epoch
    void updateRPModel(PInstance &pInst);

    // this function builds the model at the start of each epoch
    void buildModel(PInstance &pInst, std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    // solve functions
    void solveModelLP(const PInstance &pInst, const InputPaths &inputPaths);

    void solveModelInt(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                       const InputPaths &inputPaths, float availableTime, float preObj);
    void solveModelLPInt(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                         const InputPaths &inputPaths, float availableTime, float preObj);


    void solveModelIntAux_P(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                          const InputPaths &inputPaths, float availableTime, float preObj, float lpObj);

    void solveModelInt_box(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                          const InputPaths &inputPaths, float availableTime, float preObj, float lpObj);

    void solveModelIntAux_D(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                       InputPaths &inputPaths, float availableTime, float preObj, const PDualAuxSolver &DualAuxSolver_);

    void solveLPDual(const PInstance &pInst, const InputPaths &inputPaths);

    // Display function
    std::string toString() const override;
};


#endif //DARP_IPS_REDUCEDPROBLEM_H

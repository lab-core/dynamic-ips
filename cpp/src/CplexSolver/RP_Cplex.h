//
// Created by Ella on 2021-11-10.
//

#ifndef DARP_IPS_REDUCEDPROBLEM_H
#define DARP_IPS_REDUCEDPROBLEM_H

#include "CplexModeler.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------


class RP_Cplex : public CplexModeler{
public:
    float objValue_;
    float auxObjValue_;                    // objective of auxiliary model use for getting duals from MIP

    std::vector<PRoute> compRoutes_;        // list of route variables in the model

    // Constructor and Destructor
    RP_Cplex();


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
    void buildModelRP(PInstance &pInst, std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    // solve functions
    void solveModelLP(const PInstance &pInst, const InputPaths &inputPaths);

    void solveModelInt(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                       const InputPaths &inputPaths, float availableTime, float preObj);
    void solveModelRelaxInt(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                         const InputPaths &inputPaths, float availableTime, float preObj);


    void solveModelIntAux_P(const PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                          const InputPaths &inputPaths, float availableTime, float preObj, float lpObj);

    void solveLPDual(const PInstance &pInst, const InputPaths &inputPaths);

    // Mark columns that should be recycled into the next epoch
    void markColumnsToKeep();

    // Reset the model state for reuse in the next epoch
    void resetForNextIteration();

    // Display function
    std::string toString() const override;
};


#endif //DARP_IPS_REDUCEDPROBLEM_H

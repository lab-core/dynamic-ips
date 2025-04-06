//
// Created by Ella on 2021-11-17.
//

#ifndef COMPLEMENTPRO_H
#define COMPLEMENTPRO_H

#include "solvers/CplexModeler.h"

//---------------------------------------------------------------------------------------------
//  Complementary Problem class
//  Build and solve the Complementary problem of the ISUD
//---------------------------------------------------------------------------------------------

//enum SolutionStatus { NOT_SOLVED = 0, NEGATIVE_VALUE = 1, POSITIVE_VALUE = 2, FRACTIONAL = 3 , INFEASIBLE = 4};

class ComplementPro : public CplexModeler{
public:
    // Variables
    IloNumVarArray routeIncVar_;                // route Incompatible variables
    vector<PRoute> IncRoute_;                   // list of Incompatible routes
    IloNumVarArray zIncVar_;                    // request (z) Incompatible variables
    IloNumVarArray routeSolVar_;                // route compatible variables (current basis)
    IloNumVarArray zSolVar_;                    // request (z) compatible variables (current basis)
    IloNumVar auxVar_;                          // auxiliary variable for improved version

    std::vector<PRoute> fractionalRoutes_;      // list of routes in fractional solution
    std::vector<PRequest> fractionalZ_;         // list of requests in fractional solution

    // set of constraints
    IloRange normalConst_;
    SolutionStatus status_;

    // Constructor and Destructor
    ComplementPro();


    // this function initialized the model and define empty set of constraints
    void initializeCPModel(PInstance &pInst, int nbVehicles);

    // this function adds zVar to the model
    void addZVar(IloNumVarArray zVar, PRequest &request, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVar(IloNumVarArray routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst);
    void addAuxVar(PInstance &pInst, float cost, int nbVehicles);

    // this function build the model at each iteration
    void buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    void repairModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    // this function use auxiliary variable
    void buildModelCP_improved(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                               int nbVehicles, float preObj);

    // this function update the model and variables
    void updateModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

    // this function solve the model
    void solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                      InputPaths &inputPaths);

    void solveCP2Model(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                      InputPaths &inputPaths);

    // this function check the situation of the CP solution to be column disjoint
    bool isColumnDisjoint(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults, int nbVehicle);
    bool isColumnDisjointBit(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults);

    // Display function
    std::string toString() const override;

    // this function initialized the model and define empty set of constraints
    void ResetCPModel();
    void restartCp();
};


#endif //COMPLEMENTPRO_H

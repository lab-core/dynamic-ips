//
// Created by Ella on 2021-11-17.
//

#ifndef COMPLEMENTPRO_H
#define COMPLEMENTPRO_H

#include "solvers/MasterModeler.h"

//---------------------------------------------------------------------------------------------
//  Complementary Problem class
//  Build and solve the Complementary problem of the ISUD
//---------------------------------------------------------------------------------------------

enum SolutionStatus { NOT_SOLVED = 0, NEGATIVE_VALUE = 1, POSITIVE_VALUE = 2, FRACTIONAL = 3 , INFEASIBLE = 4};

class ComplementPro : public MasterModeler{
public:
    // Variables
    IloNumVarArray routeIncVar_;
    vector<PRoute> IncRoute_;
    IloNumVarArray zIncVar_;
    IloNumVarArray routeSolVar_;
    IloNumVarArray zSolVar_;
    IloNumVar auxVar_;

    std::vector<PRoute> fractionalRoutes_;
    std::vector<PRequest> fractionalZ_;

    // set of constraints
    IloRangeArray normalConst_;
    SolutionStatus status_;

    // Constructor and Destructor
    ComplementPro();


    // this function initialized the model and define empty set of constraints
    void initializeCPModel(PInstance &pInst, int nbVehicles);

    // this function adds zVar to the model
    void addZVar(IloNumVarArray zVar, PRequest &request, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVar(IloNumVarArray routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst);
    void addAuxVar(PInstance &pInst, double cost, int nbVehicles);

    // this function build the model at each iteration
    void buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    void repairModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    void buildModelCP2(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    int nbVehicles, double preObj);

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

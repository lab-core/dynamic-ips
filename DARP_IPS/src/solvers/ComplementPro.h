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
    std::vector<int> solIndexes_;
    IloNumVarArray zIncVar_;
    IloNumVarArray routeSolVar_;
    IloNumVarArray zSolVar_;

    std::vector<PRoute> fractionalRoutes_;
    std::vector<PRequest> fractionalZ_;

    // set of constraints
    IloRangeArray normalConst_;
    SolutionStatus status_;

    // Constructor and Destructor
    ComplementPro();


    // this function initialized the model and define empty set of constraints
    void initializeCPModel(PInstance &pInst);
    void initializeCPModelPartial(PInstance &pInst, int nbVehicles);

    // this function adds zVar to the model
    void addZVar(IloNumVarArray zVar, PRequest &request, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVar(IloNumVarArray routeVar, PRoute &newRoute, VarSign sign);
    void addRouteVarPartial(IloNumVarArray routeVar, PRoute &newRoute, VarSign sign, PInstance &pInst);

    // this function build the model at each iteration
    void buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);
    void buildModelPartial(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                           int nbVehicles);

    // this function update the model and variables
    void updateModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

    // this function solve the model
    /*void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    std::unordered_map<std::string , PRoute> &generatedRoutes);*/
    void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    InputPaths &inputPaths);

    /*void solveModelIndex(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    std::unordered_map<std::string , PRoute> &generatedRoutes);*/
    void solveModelIndex(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                         InputPaths &inputPaths);
    void solveModelPartial(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                           InputPaths &inputPaths);

    // this function check the situation of the CP solution to be column disjoint
    bool isColumnDisjoint(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults, int nbVehicle);
//    bool isColumnDisjointBit(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults, int nbVehicle, int size);

    // Display function
    std::string toString() const override;

    // this function initialized the model and define empty set of constraints
    void ResetCPModel();
    void restartCp();
};


#endif //COMPLEMENTPRO_H

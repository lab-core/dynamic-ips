//
// Created by Ella on 2021-11-17.
//

#ifndef _COMPLEMENTPRO_H
#define _COMPLEMENTPRO_H

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
    IloNumVarArray zIncVar_;
    IloNumVarArray routeSolVar_;
    IloNumVarArray zSolVar_;

    std::vector<PRoute> fractionalRoutes_;
    std::vector<PRequest> fractionalZ_;

    // set of constraints
    IloRangeArray normalConst_;
    SolutionStatus status_;

    Tools::Timer *CPRestTime_;
    Tools::Timer *CPInitialTime_;
    Tools::Timer *CPAddVarTime_;

    // Constructor and Destructor
    ComplementPro();


    // this function initialized the model and define empty set of constraints
    void initializeCPModel(PInstance &pInst);

    // this function adds zVar to the model
    void addZVar(IloNumVarArray zVar, PRequest &request, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVar(IloNumVarArray routeVar, PRoute &newRoute, VarSign sign);

    // this function build the model at each iteration
    void buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

    // this function update the model and variables
    void updateModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

    // this function solve the model
    void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    std::unordered_map<std::string , PRoute> &generatedRoutes);

    void solveModelIndex(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    std::unordered_map<std::string , PRoute> &generatedRoutes);

    // this function check the situation of the CP solution to be column disjoint
    static bool isColumnDisjoint(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults,
                          std::unordered_map<unsigned int, int>& requestToOrder, int nbVehicle);

    // Display function
    std::string toString() const override;

    // this function initialized the model and define empty set of constraints
    void ResetCPModel();
};


#endif //_COMPLEMENTPRO_H

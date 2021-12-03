//
// Created by Ella on 2021-11-17.
//

#ifndef _COMPLEMENTPRO_H
#define _COMPLEMENTPRO_H

#include "solvers/CPLEXModeler.h"
#include "data/Route.h"
#include "solvers/MasterModeler.h"
#include "Eigen/Dense"

//---------------------------------------------------------------------------------------------
//  Complementary Problem class
//  Build and solve the Complementary problem of the ISUD
//---------------------------------------------------------------------------------------------

enum SolutionStatus { NEGATIVE_VALUE = 0, POSITIVE_VALUE = 1, FRACTIONAL = 2 };

class ComplementPro : public MasterModeler{
public:
    // Variables
    IloNumVarArray routeIncVar_;
    IloNumVarArray zIncVar_;
    IloNumVarArray routeSolVar_;
    IloNumVarArray zSolVar_;

    // set of constraints
    IloRangeArray normalConst_;
    SolutionStatus status_;

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

    // this function solve the model
    void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    std::map<std::string , PRoute> &generatedRoutes);

    // this function check the situation of the CP solution to be column disjoint
    bool isColumnDisjoint(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults,
                          std::map<int, int>& requestToOrder);

    // Display function
    std::string toString() const;

    // this function initialized the model and define empty set of constraints
    void ResetCPModel();
};


#endif //_COMPLEMENTPRO_H

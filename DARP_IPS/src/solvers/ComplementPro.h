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
    IloNumVar auxVar_;                          // auxiliary variable for the improved version

    std::vector<PRoute> fractionalRoutes_;      // list of routes in the fractional solution
    std::vector<PRequest> fractionalZ_;         // list of requests in the fractional solution

    // set of constraints
    IloRange normalConst_;
    SolutionStatus status_;

    // Constructor and Destructor
    ComplementPro();


    // this function initializes the model and defines an empty set of constraints
    void initializeCPModel(const PInstance &pInst, int nbVehicles);

    // this function adds zVar to the model
    void addZVar(IloNumVarArray zVar, const PRequest &request, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVar(IloNumVarArray routeVar, const PRoute &newRoute, VarSign sign, const PInstance &pInst);
    void addAuxVar(const PInstance &pInst, float cost, int nbVehicles);

    // this function builds the model at each iteration
    void buildModel(const PInstance &pInst, const std::vector<PRequest> &zSolution, const std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    void repairModel(const PInstance &pInst, const vector<PRequest> &zSolution, const vector<PRoute> &routeSolution);

    // this function uses auxiliary variable
    void buildModelCP_improved(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                               int nbVehicles, float preObj);

    // this function updates the model and variables
    void updateModel(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution);

    // this function solves the model
    void solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                      InputPaths &inputPaths);

    void solveCP2Model(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                      const InputPaths &inputPaths);

    // this function checks the situation of the CP solution to be column disjoint
//    bool isColumnDisjoint(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults, int nbVehicle);

    static bool isColumnDisjointBit(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults);
    static bool isColumnDisjointFast(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults);

    // Display function
    std::string toString() const override;

    // this function initializes the model and defines an empty set of constraints
    void ResetCPModel();
    void restartCp();
};


#endif //COMPLEMENTPRO_H

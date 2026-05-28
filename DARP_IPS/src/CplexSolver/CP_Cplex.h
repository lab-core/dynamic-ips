//
// Created by Ella on 2021-11-17.
//

#ifndef COMPLEMENTPRO_H
#define COMPLEMENTPRO_H

#include "CplexModeler.h"

//---------------------------------------------------------------------------------------------
//  Complementary Problem class
//  Build and solve the Complementary problem of the ISUD
//---------------------------------------------------------------------------------------------

class CP_Cplex : public CplexModeler{
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

    // instance pointer cached during initializeCP for use in parameter-free helpers
    const PInstance* pInst_ = nullptr;

    // Constructor and Destructor
    CP_Cplex();

    // initialize model constraints (call once per epoch, before buildModel_batch)
    void initializeCP(const PInstance &pInst, bool reduced);

    // this function adds zVar to the model
    void addZVar(IloNumVarArray zVar, const PRequest &request, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVar(IloNumVarArray routeVar, const PRoute &newRoute, VarSign sign, const PInstance &pInst);

    // this function adds routeVar without vehicle constraint (reduced CP mode)
    void addRouteVarReduced(IloNumVarArray &routeVar, const PRoute &newRoute, const PRoute &currRoute);

    // full CP: add solution and incompatible z columns (no vehicle constraints needed for z)
    void buildModel_batch(const PInstance &pInst, const std::vector<PRoute> &routeSolution);

    // reduced CP: add incompatible route columns relative to current vehicle routes
    void buildModel_batch(const PInstance &pInst);

    // this function builds the model at each iteration (legacy, kept for buildModelCP_improved)
    void buildModel(const PInstance &pInst, const std::vector<PRequest> &zSolution, const std::vector<PRoute> &routeSolution,
                    int nbVehicles);

    void repairModel(const PInstance &pInst, const vector<PRequest> &zSolution, const vector<PRoute> &routeSolution);

    // add incompatible route columns from routesToAdd_
    void updateModel(const PInstance &pInst);

    // this function solves the model
    void solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                      InputPaths &inputPaths);

    // solve with optional in-place pivot update (update=true keeps model warm across iterations)
    void solveCPModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                      InputPaths &inputPaths, bool update);

    // Display function
    std::string toString() const override;

    // this function initializes the model and defines an empty set of constraints
    void ResetCPModel();
    void restartCp();

    void solveCP2Model(const PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                      const InputPaths &inputPaths);

    // this function checks the situation of the CP solution to be column disjoint
//    bool isColumnDisjoint(std::vector<PRequest> &zResults, std::vector<PRoute> &routeResults, int nbVehicle);

    static bool isColumnDisjointBit(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults, const PInstance &pInst);
    static bool isColumnDisjointFast(const std::vector<PRequest> &zResults, const std::vector<PRoute> &routeResults, const PInstance &pInst);
    bool isColumnDisjoint(const std::vector<PRequest>& zResults,
                                         const std::vector<PRoute>& routeResults,
                                         const PInstance& pInst);

    // legacy full-build helpers (used by buildModelCP_improved)
    void initializeCPModel(const PInstance &pInst, int nbVehicles);
    void buildModelCP_improved(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                               int nbVehicles, float preObj);

    void addAuxVar(const PInstance &pInst, float cost, int nbVehicles);

    void resetForNextIteration();
};


#endif //COMPLEMENTPRO_H

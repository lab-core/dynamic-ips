//
// Created by Elahe Amiri on 2025-02-01.
//

#ifndef DUALAUXSOLVER_H
#define DUALAUXSOLVER_H

#include "utilities/InputPaths.h"
#include "utilities/MyTools.h"

class DualAuxSolver {
public:
    IloEnv env_;
    IloModel Model_;
    IloCplex Cplex_;
    int nbRequestTask_;
    int nbVehicles_;

    // Variables
    IloNumVarArray requestDuals_;
    IloNumVarArray vehicleDuals_;
    IloNumVarArray epsilonVar_;
    IloNumVarArray deltaVar_;


    // set of constraints
    std::vector<IloExpr> routeExpr_;
    std::vector<IloExpr> zExpr_;

    IloRangeArray routeConst_;
    IloRangeArray zConst_;
    std::vector<IloExpr> objExpr_;
    IloRangeArray objConst_;


    float objValue_;
    myTools::Timer *solveTime_;                             // time to solve the model

    DualAuxSolver(int nbRequestTask, int nbVehicles);

    virtual ~DualAuxSolver();

    void initializeModel(const PInstance &pInst);
    void addRouteExpr(const PRoute &route);
    void buildModel(vector<PRoute> &RMPRoutes, vector<PRequest> &Requests);
    void solveModel(const PInstance &pInst, const InputPaths &inputPaths);
};



#endif //DUALAUXSOLVER_H

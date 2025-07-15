//
// Created by Elahe Amiri on 2025-06-25.
//

#ifndef RP_GUROBI_H
#define RP_GUROBI_H

#include "GurobiModeler.h"
#include "data/Instance.h"
#include "utilities/InputPaths.h"

class RP_Gurobi : public GurobiModeler {
public:

    float objValue_;
    float auxObjValue_;                    // objective of auxiliary model use for getting duals from MIP
    std::vector<int> routeSolutionIndex_;

    std::vector<PRoute> compRoutes_;        // list of route variables in the model



    // Constructor and Destructor
    RP_Gurobi(std::string outputLog);
    // Add variables
    void addRouteVar(PRoute& newRoute, PInstance& pInst);
    void addRouteVarFloat_RP(PRoute& newRoute, PInstance& pInst);
    void addZVar(PRequest& request);

    // Update model with new routes
    void updateRPModel(PInstance& pInst);
    void updateModel_batch(PInstance& pInst);

    // Build the complete model
    void buildModelRP(PInstance& pInst, std::vector<PRoute>& routeSolution, int nbVehicles);

    void extractSolution(const PInstance& pInst, std::vector<PRequest>& zSolution, std::vector<PRoute>& routeSolution);

    // Solve as LP
    void solveModelLP(const PInstance& pInst, const InputPaths& inputPaths);

    // Solve as MIP
    void solveModelInt(const PInstance& pInst, std::vector<PRequest>& zSolution,
                       std::vector<PRoute>& routeSolution, const InputPaths& inputPaths,
                       float availableTime, float preObj);

    void solveModelLPInt(const PInstance& pInst, std::vector<PRequest>& zSolution,
                       std::vector<PRoute>& routeSolution, const InputPaths& inputPaths,
                       float availableTime, float preObj);

    void solveModelRelaxInt(const PInstance& pInst, std::vector<PRequest>& zSolution,
                       std::vector<PRoute>& routeSolution, const InputPaths& inputPaths,
                       float availableTime, float preObj);

    void solveModelIntAux_P(const PInstance& pInst, std::vector<PRequest>& zSolution,
                       std::vector<PRoute>& routeSolution, const InputPaths& inputPaths,
                       float availableTime, float preObj, float lpObj);

    void tuneModel(const InputPaths &inputPaths, float tuneTimeLimit, float individualSolveTimeLimit,
                   int numTuneResults);

    void loadTunedParameters(const InputPaths &inputPaths);
    void recoverModelForDuals(PInstance &pInst, boost::dynamic_bitset<> &removedRequests);
};



#endif //RP_GUROBI_H

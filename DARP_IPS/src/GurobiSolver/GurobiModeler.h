//
// Created by Elahe Amiri on 2025-06-25.
//

#ifndef GUROBIMODELER_H
#define GUROBIMODELER_H

#include "gurobi_c++.h"
#include "data/Instance.h"

//-----------------------------------------------------------------------------
// General class for modeling and solving the MP components with Gurobi
// Reduced problem and Complementary problem
//-----------------------------------------------------------------------------

class GurobiModeler {
public:
    GRBEnv env_;
    GRBModel* model_;
    std::string outputLog_;

    // For dual values
    std::vector<double> requestDuals_;
    std::vector<double> vehicleDuals_;

    // Variables arrays
    std::vector<GRBVar> routeVar_;
    std::vector<GRBVar> zVar_;


    // Constraints
    std::vector<GRBConstr> requestConstr_;
    std::vector<GRBConstr> vehicleConstr_;

    // RHS values
    std::vector<double> requestRHS_;
    std::vector<double> vehicleRHS_;

    int nbRequestTask_;
    std::vector<PRoute> routesToAdd_;
    myTools::Timer* solveTime_;




    // Constructor and Destructor
    GurobiModeler(std::string outputLog);
    ~GurobiModeler();

    // Helper function for creating column
    GRBColumn createColumn(const PRoute& route, VarSign sign, const PInstance& pInst);

    // Display function
    std::string toString() const;

    // Initialize the model
    void initializeModel(const PInstance& pInst, int rhs, int nbVehicles);

    // Add z variables
    void addZVarInt(const PRequest& request, VarSign sign);
    void addZVarFloat(const PRequest& request, VarSign sign);

    // Add route variables
    void addRouteVarInt(const PRoute& newRoute, VarSign sign, const PInstance& pInst);
    void addRouteVarFloat(const PRoute& newRoute, VarSign sign, const PInstance& pInst);

    // Batch operations
    void beginBatchUpdate();
    void endBatchUpdate();

    // Set parameters
    void setParameters(const PInstance& pInst, float availableTime);

    // Solve the model
    int solve();

    // Get solution status and objective value
    int getStatus() const;
    double getObjValue() const;

    // Get variable values
    double getVarValue(const GRBVar& var) const;

    // Get dual values
    void getDuals(const PInstance& pInst);
    void getDualsFromRelaxed(GRBModel &relaxedModel, const PInstance& pInst);

    // Access to variables
    const std::vector<GRBVar>& getRouteVar() const { return routeVar_; }
    const std::vector<GRBVar>& getZVar() const { return zVar_; }

    // Timer access
    myTools::Timer* getSolveTimer() { return solveTime_; }

    // Convert variables to integer/continious
    void convertToInt();
    void convertToFloat();
};


#endif //GUROBIMODELER_H

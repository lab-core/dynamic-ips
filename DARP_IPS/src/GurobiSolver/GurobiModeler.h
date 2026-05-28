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
    GRBEnv& env_;
    GRBModel* model_;
    std::string outputLog_;

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
    GurobiModeler(const std::string &outputLog);
    ~GurobiModeler();

    // Helper function for creating column
    [[nodiscard]] GRBColumn createColumn(const PRoute& route, VarSign sign, const PInstance& pInst) const;

    // Display function
    [[nodiscard]] std::string toString() const;

    // Initialize the model
    void initializeModel(const PInstance& pInst, int rhs, int nbVehicles);

    // Add z variables
    void addZVarInt(const PRequest& request, VarSign sign);
    void addZVarFloat(const PRequest& request, VarSign sign);

    // Add route variables
    void addRouteVarInt(const PRoute& newRoute, VarSign sign, const PInstance& pInst);
    void addRouteVarFloat(const PRoute& newRoute, VarSign sign, const PInstance& pInst);


    // Set parameters
    void setParameters(const PInstance& pInst, float availableTime) const;

    // Solve the model
    int solve() const;

    // Get solution status and objective value
    int getStatus() const;
    double getObjValue() const;

    // Get variable values
    static double getVarValue(const GRBVar& var);

    // Get dual values
    void getDuals(const PInstance& pInst) const;

    void getBarrierDuals(const PInstance &pInst) const;

    void getDualsFromRelaxed(const GRBModel &relaxedModel, const PInstance& pInst) const;

    // Access to variables
    const std::vector<GRBVar>& getRouteVar() const { return routeVar_; }
    const std::vector<GRBVar>& getZVar() const { return zVar_; }

    // Timer access
    myTools::Timer* getSolveTimer() const { return solveTime_; }

    // Convert variables to integer/continious
    void convertToInt();
    void convertToFloat();
};


#endif //GUROBIMODELER_H

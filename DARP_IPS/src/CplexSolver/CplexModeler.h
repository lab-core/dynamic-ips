//
// Created by Ella on 2021-11-17.
//

#ifndef CPLEXMODELER_H
#define CPLEXMODELER_H


#include "data/Instance.h"
#include <ilcplex/ilocplex.h>

//-----------------------------------------------------------------------------
// General class for modelling and solving the MP components with Cplex
// Reduced problem and Complementary problem
//-----------------------------------------------------------------------------

class CplexModeler {
public:
    // CPLEX environment and model objects
    IloEnv env_;                             // owned value — each modeler has its own env
    IloModel Model_;
    IloCplex Cplex_;
    IloObjective objFunction_;

    // Variables
    IloNumVarArray routeVar_;               // route variables
    IloNumVarArray zVar_;                   // request(z) variables

    // dual variables
    IloNumArray requestDuals_;
    IloNumArray vehicleDuals_;

    // right-hand-side of constraints
    IloNumArray requestRHS_;
    IloNumArray vehicleRHS_;

    // set of constraints
    IloRangeArray requestConst_;
    IloRangeArray vehicleConst_;

    int nbRequestTask_;                     // number of request tasks in the model     
    std::vector<PRoute> routesToAdd_;       // routes to be added to the model at each iteration
    myTools::Timer *solveTime_;             // time to solve the model

    // Constructor and Destructor
    CplexModeler();

    // End the entire IloEnv and reinitialise all Concert base members from a
    // fresh env. Used in resetForNextIteration() as a debugging nuclear option
    // to confirm whether accumulated env-pool memory is the source of growth.
    void renewEnv();

    virtual ~CplexModeler();

    // this function clears all objects from the model at the start of each epoch
    void clearModel();

    virtual // Display function
    std::string toString() const;

    // function to create a pattern from routes
    static void createPattern (IloNumArray& pattern, const PRoute &route, VarSign sign);

    // this function initialized the model
    void initializeModel(const PInstance &pInst, int rhs, int nbVehicles);

    // this function adds zVar to the model
    void addZVarInt(IloNumVarArray &zVar, const PRequest &request, VarSign sign);
    void addZVarFloat(IloNumVarArray &zVar, const PRequest &request, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVarInt(IloNumVarArray &routeVar, const PRoute &newRoute, VarSign sign, const PInstance &pInst);
    void addRouteVarFloat(IloNumVarArray &routeVar, const PRoute &newRoute, VarSign sign, const PInstance &pInst);

    // this function sets parameters for the CPLEX solver
    void setParameters(const PInstance &pInst, float availableTime);

    // this function extracts dual variables from the solved model
    void getDuals(const PInstance &pInst);

    // this function save the mathematical model to a file
    void dump_cplex();
};

// function to create IloNumArray with identical elements
static void createIloNumArray (IloNumArray& numArray, unsigned int size, int elementValue) {
    numArray.setSize(size);
    for (int i = 0; i < size; ++i) {
        numArray[i] = elementValue;
    }
}


#endif //CPLEXMODELER_H

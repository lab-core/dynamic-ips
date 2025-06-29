//
// Created by Ella on 2021-11-17.
//

#ifndef CPLEXMODELER_H
#define CPLEXMODELER_H


#include "data/Instance.h"

//-----------------------------------------------------------------------------
// General class for modeling and solving the MP components with Cplex
// Reduced problem and Complementary problem
//-----------------------------------------------------------------------------

//enum VarSign { POSITIVE, NEGATIVE };

class CplexModeler {
public:
    IloEnv env_;
    IloModel Model_;
    IloCplex Cplex_;
    IloObjective objFunction_;


    // dual costs
    IloNumArray requestDuals_;
    IloNumArray vehicleDuals_;

    // right-hand-side of constraints
    IloNumArray requestRHS_;
    IloNumArray vehicleRHS_;

    // set of constraints
    IloRangeArray requestConst_;
    IloRangeArray vehicleConst_;

    int nbRequestTask_;

    std::vector<PRoute> routesToAdd_;
    myTools::Timer *solveTime_;                             // time to solve the model

    // Constructor and Destructor
    CplexModeler();

    virtual ~CplexModeler();

    // this function reset the model based the current set of routes and changed the set of constraints (size)
//    void updateRequestOrder(PInstance &pInst);

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
    void addUVarFloat(IloNumVarArray &uVar, IloNumVarArray &vVar, const PRequest &request);

    // this function adds routeVar to the model
    void addRouteVarInt(IloNumVarArray &routeVar, const PRoute &newRoute, VarSign sign, const PInstance &pInst);
    void addRouteVarFloat(IloNumVarArray &routeVar, const PRoute &newRoute, VarSign sign, const PInstance &pInst);
    void setParameters(const PInstance &pInst, float availableTime);

};

// function to create IloNumArray with identical elements
static void createIloNumArray (IloNumArray& numArray, unsigned int size, int elementValue) {
    numArray.setSize(size);
    for (int i = 0; i < size; ++i) {
        numArray[i] = elementValue;
    }
}


#endif //CPLEXMODELER_H

//
// Created by Ella on 2021-11-17.
//

#ifndef MASTERMODELER_H
#define MASTERMODELER_H


#include "solvers/CPLEXModeler.h"
#include "data/Instance.h"
#include "data/Route.h"

//-----------------------------------------------------------------------------
// General class for modeling and solving the ISUD components
// Reduced problem and Complementary problem
//-----------------------------------------------------------------------------

enum VarSign { POSITIVE, NEGATIVE };

class MasterModeler {
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
    myTools::Timer *AddVarTime_;


    vector<unsigned int> orderToRequest_;
//    std::unordered_map<unsigned int, int> requestToOrder_;

    std::vector<PRoute> routesToAdd_;

    // Constructor and Destructor
    MasterModeler();

    virtual ~MasterModeler();

    // this function reset the model based the current set of routes and changed the set of constraints (size)
//    void updateRequestOrder(PInstance &pInst);

    // this function clear all objects from the model at the start of each epoch
    void clearModel();

    virtual // Display function
    std::string toString() const;

    // function to create pattern from routes
    static void createPattern (IloNumArray& pattern, PRoute &route, VarSign sign);

    // this function initialized the model
    void initializeModel(PInstance &pInst, int rhs);

    // this function adds zVar to the model
    void addZVar(IloNumVarArray &zVar, PRequest &request, VarSign sign);
    void addZVars(IloNumVarArray &zVar, std::vector<PRequest> &requests, VarSign sign);

    // this function adds routeVar to the model
    void addRouteVar(IloNumVarArray &routeVar, PRoute &newRoute, VarSign sign);
    void addRouteVars(IloNumVarArray &routeVar, std::vector<PRoute> &newRoutes, VarSign sign);

};


#endif //MASTERMODELER_H

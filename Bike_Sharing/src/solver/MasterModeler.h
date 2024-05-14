//
// Created by Elahe Amiri on 2024-05-13.
//

#ifndef DARP_IPS_MASTERMODELER_H
#define DARP_IPS_MASTERMODELER_H

#include "data/Instance.h"
#include "data/Route.h"

class MasterModeler {
public:
    IloEnv env_;
    IloModel Model_;
    IloCplex Cplex_;

    IloObjective objFunction_;
    double objValue_;

    // Variables
    IloNumVarArray routeVar_;               // route variables
    std::vector<PRoute> modelRoutes_;        // list of route variables in the model


    // dual costs
    IloNumArray taskDuals_;
    IloNumArray vehicleDuals_;

    // right-hand-side of constraints
    IloNumArray taskRHS_;
    IloNumArray vehicleRHS_;

    // set of constraints
    IloRangeArray taskConst_;
    IloRangeArray vehicleConst_;

    std::vector<PRoute> routesToAdd_;
    myTools::Timer *solveTime_;

    int nbRequestTask_;

    // Constructor and Destructor
    MasterModeler();

    virtual ~MasterModeler();

    std::string toString() const;

    // function to create pattern from routes
    static void createPattern (IloNumArray& pattern, PRoute &route);

    // this function initialized the model
    void initializeModel(PInstance &pInst, int rhs, int nbVehicles);

    // this function adds routeVar to the model
    void addRouteVar(PRoute &newRoute, bool intVar);

    // this function add one route at each iteration of the algorithm during one epoch
    void updateModel();

    // this function build the model at the start of each epoch
    void buildModelMP(PInstance &pInst, std::vector<PRoute> &routeSolution, int nbVehicles);

    // solve functions
    void solveModelLP(PInstance &pInst, InputPaths &inputPaths);

    void solveModelInt(PInstance &pInst, std::vector<PRoute> &routeSolution,
                       InputPaths &inputPaths, float availableTime, double preObj);

    void solveModelLPInt(PInstance &pInst, std::vector<PRoute> &routeSolution,
                         InputPaths &inputPaths, float availableTime, double preObj);
};

// function to create IloNumArray with identical elements
static void createIloNumArray (IloNumArray& numArray, unsigned int size, int elementValue) {
    numArray.clear();
    for (int i = 0; i < size; ++i) {
        numArray.add(elementValue);
    }
}
#endif //DARP_IPS_MASTERMODELER_H

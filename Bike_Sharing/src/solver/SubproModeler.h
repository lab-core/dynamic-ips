//
// Created by Ella on 2/22/2022.
//


#ifndef SUBPROMODELER_H
#define SUBPROMODELER_H

#include "utilities/MyTools.h"
#include "data/Graph.h"
#include "utilities/types.h"

//-----------------------------------------------------------------------------
// SubproModeler class
// General class to define subproblem objects corresponding to each vehicle
//-----------------------------------------------------------------------------

class SubproModeler {
public:
    Vehicle* Vehicle_;                     // the vehicle for which we are solving the sub problem
    PGraph subGraph_;                       // the graph of the feasible solution for the vehicle
    std::vector<PTask> subTasks_;             // List of requests
    int nbNegativeColumns_;                 // number negative reduced cost routes found
    int nbTotalRequest_;                    // equals to the number of requests in the corresponding graph

    // Constructor and Destructor

    explicit SubproModeler(PVehicle &vehicle);
    virtual ~SubproModeler();

    void initSubGraph(PInstance &pInst);

};


#endif //SUBPROMODELER_H

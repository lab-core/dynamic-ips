//
// Created by Ella on 2/22/2022.
//


#ifndef SUBPROMODELER_H
#define SUBPROMODELER_H

#include "utilities/MyTools.h"
#include "data/Graph.h"

//-----------------------------------------------------------------------------
// SubproModeler class
// General class to define subproblem objects corresponding to each vehicle
//-----------------------------------------------------------------------------

class SubproModeler {
public:
    Vehicle* Vehicle_;                      // the vehicle for which we are solving the subproblem
    PGraph subGraph_;                       // the graph of the feasible solution for the vehicle
    std::vector<PRequest> subRequests_;     // List of requests
    int nbNegativeColumns_;                 // number negative reduced cost routes found
    int nbTotalRequest_;                    // equals to the number of requests in the corresponding graph
    bool reOptimize_;
    int possibleFirstInsert_;
    int possibleSecondInsert_;
    int labelSize_;

    // Constructor and Destructor
    explicit SubproModeler(const PVehicle &vehicle);
    virtual ~SubproModeler();

    // calculation of penalties and initialization of the subgraph
    void initSubGraph(const PInstance &pInst);
    void setNodeIndices() const;
};


#endif //SUBPROMODELER_H

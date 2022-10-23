//
// Created by Ella on 2/22/2022.
//


#ifndef _SUBPROMODELER_H
#define _SUBPROMODELER_H

#include "utilities/MyTools.h"
#include "data/Graph.h"


class SubproModeler {
public:
    PVehicle* Vehicle_;                     // the vehicle for which we are solving the sub problem
    PGraph subGraph_;                       // the graph of the feasible solution for the vehicle
    std::vector<PRequest> subRequests_;     // List of requests
//    std::set<PRequest> onboardRequests_;
    double bestReducedCost_;
    int nbNegativeColumns_;
    int nbTotalRequest_;

    // Constructor and Destructor
    explicit SubproModeler(PVehicle &vehicle);
    virtual ~SubproModeler();

    // calculation of penalties and initialization of the subgraph
    void initSubGraph(PInstance &pInst);

    void initSubGraph2(PInstance &pInst);

};


#endif //_SUBPROMODELER_H

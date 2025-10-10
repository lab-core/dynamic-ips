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
    int nbOnePickGenerated_;
    int nbTwoPickGenerated_;
    int nbOutCover_;
    int nbPriorCover_;
    int possibleInsert_;
    int labelSize_;

    // Constructor and Destructor
    explicit SubproModeler(const PVehicle &vehicle);
    virtual ~SubproModeler();

    // calculation of penalties and initialization of the subgraph
    void initSubGraph(const PInstance &pInst);
    void setNodeIndices() const;
    bool checkInsertionPossibility(PNode &pick);
};

inline float computeDetourDelay(PNode &before, PNode &pick, PNode &after)
{
    // delay = (before→pick + service(pick) + pick→after) - (before→after)
    float detour = durationMatrix_[before->locationID_][pick->locationID_]
           + pick->serviceTime_
           + durationMatrix_[pick->locationID_][after->locationID_]
           - durationMatrix_[before->locationID_][after->locationID_];

    return detour;
}
inline bool isPickupProfitable(PNode &before, PNode &pick, float beforeDepartTime)
{
    float reachTime = beforeDepartTime + durationMatrix_[before->locationID_][pick->locationID_];
    if (reachTime <= pick->related_Request_->latestPickup_ && (reachTime - pick->related_Request_->initialEarlyPick_ - pick->related_Request_->dual_ < 1))
        return true;
    return false;
}


#endif //SUBPROMODELER_H

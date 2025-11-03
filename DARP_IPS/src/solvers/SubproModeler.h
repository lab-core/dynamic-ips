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
    int nbTotalRequest_;                    // equals the number of requests in the corresponding graph
    bool reOptimize_;                       // whether we re-optimization the subproblem or not
    int nbOnePickGenerated_;                // number of one-pickup routes generated
    int nbTwoPickGenerated_;                // number of two-pickup routes generated
    int nbOutCover_;                        // number of requests covered in the current epoch more than prior epochs
    int nbPriorCover_;                      // number of requests covered in prior epochs
    int possibleInsert_;                    // number of possible requests insertions in the subproblem        
    int labelSize_;                         // size of labels in labeling algorithm (for bit sets)
    myTools::Timer *subproTime_;            // timer for labeling algorithm

    // Constructor and Destructor
    explicit SubproModeler(const PVehicle &vehicle);
    virtual ~SubproModeler();

    // calculation of penalties and initialization of the subgraph
    void initSubGraph(const PInstance &pInst);

    // set node indices in the subgraph
    void setNodeIndices() const;

    // function to check if inserting a pickup node is possible
    bool checkInsertionPossibility(PNode &pick);
};

// function to compute detour delay of inserting a pickup node between two nodes
inline float computeDetourDelay(PNode &before, PNode &pick, PNode &after)
{
    // delay = (before→pick + service(pick) + pick→after) - (before→after)
    float detour = durationMatrix_[before->locationID_][pick->locationID_]
           + pick->serviceTime_
           + durationMatrix_[pick->locationID_][after->locationID_]
           - durationMatrix_[before->locationID_][after->locationID_];

    return detour;
}

// function to check if inserting a pickup node is profitable
inline bool isPickupProfitable(PNode &before, PNode &pick, float beforeDepartTime)
{
    float reachTime = beforeDepartTime + durationMatrix_[before->locationID_][pick->locationID_];
    if (reachTime <= pick->related_Request_->latestPickup_ && (reachTime - pick->related_Request_->initialEarlyPick_ - pick->related_Request_->dual_ < 1))
        return true;
    return false;
}


#endif //SUBPROMODELER_H

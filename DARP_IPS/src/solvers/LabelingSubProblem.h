//
// Created by Ella on 1/28/2022.
//

#pragma once

#ifndef LABELINGSUBPROBLEM_H
#define LABELINGSUBPROBLEM_H

#include "solvers/SubproModeler.h"
#include "data/Label.h"


//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------

class LabelingSubProblem : public SubproModeler{
public:
    std::vector<PLabel> labelPool_;             // pool of generated labels to re-use
    std::vector<Node*> activeNodes_;            // list of nodes with active labels
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbPrunedPath_;                          // number of labels detected as non-promising by soft time window
    int nbPrunedArcs_;
    int nbEliminated_;                          // number of labels detected as Unreachable due to travel time
    int nbGenerated_;                           // number of generated labels
    int nbRecycledColumns_;
    int nbOutputs_;                             // total number of generated routes
    int maxPickup_;                             // number of pickups that are allowed in each path
    vector<PRoute> availableRoutes_;

    PSolverOption solverOptions_;
    myTools::Timer *subproTime_;                // timer for labeling algorithm
    std::string initialNodeID_;                 // it determines departing stop when we have committed nodes

    // Constructor and Destructor
    LabelingSubProblem(const PVehicle &vehicle, const PSolverOption &solverOptions);

    ~LabelingSubProblem() override;


    // this function sorts the list of nodes based on their dual values
    void sortSuccessors(const std::vector<PNode> &nodeList, bool prunedArcs) const;

    // reset that active lists of the nodes, create the first label at the source, add the onboard nodes
    void initialization();
    // main function of the dynamic programming
    bool labelExtend(const PLabel &parentLabel, Node *outNode, bool Terminate);
    bool labelExtendPick(const PLabel &parentLabel, Node *outNode);
    bool isLabelAdded(PLabel &newLabel, Node *outNode, bool Terminate);
    bool solveDynamic_pushing(float availableTime);
    // this function is the same as normal pushing strategy, but it does not do a pick after drops
    bool solveDynamic_pushingDrop(float availableTime);
    void solveDynamic_pushingWave();
    bool solveDynamic_pulling1(float availableTime);
    bool solveDynamic_pulling(float availableTime);
    bool solveDynamic_pullingWave(float availableTime);

    bool ResolveDynamic_pullingWave(float availableTime);

    bool solveDynamic_pullingWave1(float availableTime);
    bool solveDynamic(float availableTime);
    void removeDominated(Node *node, std::vector<PLabel> & labelPool) const;

    // function to convert the solution to routes and save them in the vehicle object
    void SolutionToRoutes(const PVehicle &vehicle, std::vector<PRoute> &availableRoutes, const PInstance & pInst,
        int nbRequests);
    void CollectLabels();
    void solutionSummery(std::vector<int> &subProResults) const;
    // Display function
    std::string toString() const;
    std::string toStringOut(int epoch) const;

    void truncateLabelList(Node *node, int MaxLabel, int MaxCommittedLabel, std::vector<PLabel> & labelPool) const;
    void extendToDropOnboards(const PLabel &selectedLabel);
    void constructLabels(const PLabel &initialLabel);

    void constructBaseLabels(const PLabel &initialLabel);
};


#endif //LABELINGSUBPROBLEM_H
//
// Created by Ella on 1/28/2022.
//

#pragma once

#ifndef LABELINGSUBPROBLEM_H
#define LABELINGSUBPROBLEM_H

#include "solvers/SubproModeler.h"
#include "data/Label.h"


//-----------------------------------------------------------------------------
//  Labelling Sub problem class
//  contains the labelling algorithm to solve the subproblem for each vehicle
//-----------------------------------------------------------------------------

class LabelingSubProblem : public SubproModeler{
public:
    std::vector<PLabel> labelPool_;             // pool of generated labels to re-use
    std::vector<Node*> activeNodes_;            // list of nodes with active labels
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbPrunedPath_;                          // number of labels detected as non-promising by soft time window
    int nbPrunedArcs_;                          // number of labels detected as non-promising by pruned arcs
    int nbEliminated_;                          // number of labels detected as Unreachable due to travel time
    int nbGenerated_;                           // number of generated labels
    int nbRecycledColumns_;                     // number of labels recycled from routes


    vector<PRoute> availableRoutes_;            // available routes in the pool to be recycled as labels

    PSolverOption solverOptions_;               // solver options for labeling algorithm

    std::string initialNodeID_;                 // it determines departing stop when we have committed nodes

    // Constructor and Destructor
    LabelingSubProblem(const PVehicle &vehicle, const PSolverOption &solverOptions);
    ~LabelingSubProblem() override;

    // Display function
    std::string toString() const;
    std::string toStringOut(int epoch) const;

    // main function of solving SP using dynamic programming
    bool solveSP(float availableTime, const PVehicle &vehicle, std::vector<PRoute> &availableRoutes, const PInstance & pInst,
        int nbRequests, std::unordered_set<std::string> &duplicatesRoutes);
    
    bool solveDynamic(float availableTime);

    // dynamic programming labelling algorithms with pushing strategies
    bool solveDynamic_pushing(float availableTime);
    bool solveDynamic_pushingWaveStep(float availableTime);

    // dynamic programming labelling algorithms with pulling strategies
    bool solveDynamic_pulling(float availableTime);
    bool solveDynamic_pullingWaveStep(float availableTime);

    /******************** helper functions for labelling algorithm ******************/

    // reset the active lists of the nodes, create the first label at the source, add the onboard nodes
    void initialization();

    // this function sorts the list of nodes based on their dual values
    void sortSuccessors(const std::vector<PNode> &nodeList, bool prunedArcs) const;
    void sortSuccessorsNew(const std::vector<PNode> &nodeList, bool prunedArcs) const;
    
    // function to extend a label to a new node
    bool labelExtend(const PLabel &parentLabel, Node *outNode, bool Terminate);
    void labelExtendOnly(const PLabel &parentLabel, Node *outNode);

    // function to check if a new label should be added to the active list of a node
    bool isLabelAdded(PLabel &newLabel, Node *outNode, bool Terminate);

    // function to remove dominated labels from the active list of a node
    void removeDominated(Node *node, std::vector<PLabel> & labelPool) const;

    // function to push a label to all drop-off onboard nodes
    bool pushToDrops(float availableTime, std::vector<PNode> &pickNodeList);

    // function to push a label to all outgoing pickup nodes
    void pushToPickups(float availableTime, std::vector<PNode> &pickNodeList);
    void pushToPickups(float availableTime);

    // function to pull a label to all outgoing pickup nodes
    void pullToPickups(float availableTime, std::vector<PNode> &pickNodeList, bool doTruncation);

    // function to extend a label to all drop-off onboard nodes
    void extendToDropOnboards(const PLabel &selectedLabel);

    // function to truncate the label list of a node based on the maximum allowed labels
    void truncateLabelList(Node *node, int MaxLabel, std::vector<PLabel> &labelPool) const;

    // function to truncate the label list of a node based on the maximum allowed labels and committed labels
    void truncateLabelList(Node *node, int MaxLabel, int MaxCommittedLabel, std::vector<PLabel> & labelPool) const;

    // construct labels from the initial label
    void constructLabels(const PLabel &initialLabel);

    // construct base labels from the initial label (used doing re-optimization RE_INSERT)
    void constructBaseLabels(const PLabel &initialLabel);

    // function to convert the solution to routes and save them in the vehicle object
    void SolutionToRoutes(const PVehicle &vehicle, std::vector<PRoute> &availableRoutes, const PInstance & pInst,
        int nbRequests, std::unordered_set<std::string> &duplicatesRoutes);

    // collect the generated labels for recycling when the subproblem is stopped early
    void CollectLabels();

    // function to save the solution summary of the labelling algorithm in subProResults
    void solutionSummery(std::vector<int> &subProResults) const;
};


#endif //LABELINGSUBPROBLEM_H
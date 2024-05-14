//
// Created by Ella on 1/28/2022.
//

#pragma once

#ifndef LABELINGSUBPROBLEM_H
#define LABELINGSUBPROBLEM_H

#include "solver/SubproModeler.h"
#include "data/Label.h"


//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------

class LabelingSubProblem : public SubproModeler{
public:
    std::vector<PLabel> labelPool_;             // pool of generated labels to re-use
    std::vector<Node*> activeNodes_;            // list of nodes with active labels
    int nbDominated_;                           // number of labels removed via Domination Rules
    int nbEliminated_;                          // number of labels removed via Elimination Rules
    int nbGenerated_;                           // number of generated labels
    int nbOutputs_;                             // total number of generated routes
    int maxPickup_;                             // number of pickups that are allowed in each path

    PSolverOption solverOptions_;
    std::string initialNodeID_;                 // it determines departing stop when we have committed nodes

    // Constructor and Destructor
    LabelingSubProblem(PVehicle &vehicle, PSolverOption solverOptions);

    // reset that active lists of the nodes, create the first label at the source, add onboards
    void initialization();
    // main function of the dynamic programming
    void labelExtend(PLabel &parentLabel, Node *outNode, bool Terminate);
    bool isLabelAdded(PLabel &newLabel, Node *outNode, bool Terminate);
    void solveDynamic_pushing();

    void solveDynamic_pulling();
    void solveDynamic();

    // function to convert solution to routes and save them in vehicle object
    void SolutionToRoutes(PVehicle &vehicle, std::vector<PRoute> &availableRoutes, PInstance & pInst);

    void solutionSummery(std::vector<int> &subProResults);
    // Display function
    std::string toString() const;
    std::string toStringOut(int epoch) const;

    void truncateLabelList(Node *node, int MaxLabel, std::vector<PLabel> & labelPool);
};


#endif //LABELINGSUBPROBLEM_H
//
// Created by Ella on 1/28/2022.
//

#pragma once

#ifndef _LABELINGSUBPROBLEM_H
#define _LABELINGSUBPROBLEM_H

#include "solvers/SubproModeler.h"
#include "data/Label.h"

//-----------------------------------------------------------------------------
//  Labeling Sub problem class
//-----------------------------------------------------------------------------

class LabelingSubProblem : public SubproModeler{
public:
    std::vector<PLabel> dominatedLabels_; // list of active labels, ordered based on reduced cost
    std::vector<PNode> activeNodes_;
    int nbDominated_;                                                       // number of labels removed via Domination Rules
    int nbEliminated_;                                                      // number of labels removed via Elimination Rules
    int nbGenerated_;
    int nbActivated_;
    PSolverOption solverOptions_;


    std::vector<PNode> nodesOrder_;

    // Constructor and Destructor
    LabelingSubProblem(PVehicle &vehicle, const PSolverOption &solverOptions);

    // this function sort the list of nodes based of their dual values
    void sortNodes();
    void sortSuccessors();

    // reset that active lists of the nodes, create the first label at the source, add onboards
    void initialization();
    // main function of the dynamic programming
    void labelExtend(PLabel &parentLabel, PNode &outNode, std::vector<PNode> &activeNodeList);
    bool isLabelAdded(PLabel &newLabel, PNode &outNode);
    void solveDynamic_pushing(int epoch);
    void solveDynamic_pulling(int epoch);
    void solveDynamic(int epoch);
    // function to convert solution to routes and save them in vehicle object

    void SolutionToRoutes(PVehicle &vehicle, std::vector<PRoute> &availableRoutes, std::unordered_map<std::string , PRoute> &generatedRoutes);
    // Display function
    std::string toString() const;
};
typedef std::shared_ptr<LabelingSubProblem> PLabelingSubPro;
void truncateLabelList(PNode &node, int MaxLabel);

#endif //_LABELINGSUBPROBLEM_H
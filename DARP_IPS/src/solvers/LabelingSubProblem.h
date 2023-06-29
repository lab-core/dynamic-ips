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
    std::vector<PLabel> labelPool_;           // list of active labels, ordered based on reduced cost
    std::vector<Node*> activeNodes_;
    int nbDominated_;                               // number of labels removed via Domination Rules
    int nbEliminated_;                              // number of labels removed via Elimination Rules
    int nbGenerated_;
    int nbOutputs_;
    int maxPickup_;

    PSolverOption solverOptions_;
//    myTools::Timer *subproTime_;
//    myTools::Timer *subproRouteTime_;
//    myTools::Timer *sortTime_;
    std::string initialNodeID_;

    // Constructor and Destructor
    LabelingSubProblem(PVehicle &vehicle, PSolverOption solverOptions);

    ~LabelingSubProblem() override;


    // this function sort the list of nodes based of their dual values
    void sortSuccessors(std::vector<PNode> &nodeList);

    // reset that active lists of the nodes, create the first label at the source, add onboards
    void initialization();
    // main function of the dynamic programming
    void labelExtend(PLabel &parentLabel, Node *outNode, bool Terminate);
    void labelExtend2(PLabel &parentLabel, Node *outNode);
    void labelDrop(PLabel &parentLabel);
    bool isLabelAdded(PLabel &newLabel, Node *outNode, bool Terminate);
    void solveDynamic_pushing();
    // this function is the same as normal pushing strategy, but it does not do a pick after drops
    void solveDynamic_pushingDrop();
    void solveDynamic_pushingWave();
    void solveDynamic_pulling();
    void solveDynamic_pullingWave();
    void solveDynamic();
    // this function is for reconstructing the routes generated in previous epoch
    void reconstructLabels(std::vector<PRoute> &availableRoutes);

    // function to convert solution to routes and save them in vehicle object
    void SolutionToRoutes(PVehicle &vehicle, std::vector<PRoute> &availableRoutes, PInstance & pInst);
    // Display function
    std::string toString() const;
    std::string toStringOut(int epoch) const;
    void restProblem();
};
typedef std::shared_ptr<LabelingSubProblem> PLabelingSubPro;
void truncateLabelList(Node *node, int MaxLabel, std::vector<PLabel> & labelPool);
void truncateLabelList(Node *node, int MaxLabel);
#endif //LABELINGSUBPROBLEM_H
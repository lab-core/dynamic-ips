//
// Created by Ella on 2/28/2022.
//

#ifndef _ZOOMREDUCEDPROBLEM_H
#define _ZOOMREDUCEDPROBLEM_H

#include "ReducedProblem.h"

class ZoomReducedProblem : public ReducedProblem {
public:

    void updateModel(PInstance &pInst, std::vector<PRequest> &fractionalZ);
    void updateModelPartial(PInstance &pInst, std::vector<PRequest> &fractionalZ);
    /*void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                   std::unordered_map<std::string , PRoute> &generatedRoutes);*/
    void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                    InputPaths &inputPaths, int availableTime, double preObj);
    void solveModelDual(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                        InputPaths &inputPaths, int availableTime, double preObj);

    void solveModelDualLP(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                        InputPaths &inputPaths, int availableTime, double preObj);

    /*void solveModelLP(PInstance &pInst, InputPaths &inputPaths);
    void solveModelIntD(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                       InputPaths &inputPaths, float availableTime, double preObj);*/
    void solveModelPartial(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution, InputPaths &inputPaths);

    // Display function
    std::string toString() const override;

};


#endif //_ZOOMREDUCEDPROBLEM_H

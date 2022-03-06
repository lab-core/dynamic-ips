//
// Created by Ella on 2/28/2022.
//

#ifndef _ZOOMREDUCEDPROBLEM_H
#define _ZOOMREDUCEDPROBLEM_H

#include "ReducedProblem.h"

class ZoomReducedProblem : public ReducedProblem {
public:
    void updateModel(PInstance &pInst, std::vector<PRequest> &fractionalZ);
    void solveModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution,
                   std::unordered_map<std::string , PRoute> &generatedRoutes);
};


#endif //_ZOOMREDUCEDPROBLEM_H

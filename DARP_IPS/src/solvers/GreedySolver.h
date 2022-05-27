//
// Created by Ella on 5/5/2022.
//

#ifndef _GREEDYSOLVER_H
#define _GREEDYSOLVER_H

#include "data/Instance.h"
#include "utilities/InputPaths.h"
#include "data/Label.h"

//-----------------------------------------------------------------------------
//  Contains Greedy function to solve the instance
//  Requests are assigned to the vehicles based on the minimum possible pickup
//-----------------------------------------------------------------------------


class GreedySolver {
public:
    std::vector<PLabel> routeLabels_;
    std::unordered_map<int, int> requestIDToInt_;

    //Constructor and destructor

    void initialization(PInstance &PInst);

    void SolveRideshare(PInstance& PInst);

};
// this function just assign requests to vehicles based on the minimum delay possible and do not consider ridesahring
// any pick up is followed by the drop off
void GreedySolver_noShare(PInstance& PInst);



#endif //_GREEDYSOLVER_H

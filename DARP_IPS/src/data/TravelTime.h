//
// Created by Ella on 2021-12-15.
//

#ifndef _TRAVELTIME_H
#define _TRAVELTIME_H

#include "utilities/MyTools.h"
#include "data/Graph.h"

//-----------------------------------------------------------------------------
//  TravelTime class
//  Define function for calculating travel times between stop locations
//-----------------------------------------------------------------------------

class TravelTime {
public:
    std::unordered_map<std::string, int> nodeIDToInt_;
    vector2D<float> durationValues_;


    // Setters
//    void setDurationMat(PGraph &graph);
    void setNodeIdToInt(const std::unordered_map<std::string, int> &nodeIdToInt);

    // function to return travel duration between two nodes
    float queryTravelTime(PNode &startNode, PNode &endNode);

};


#endif //_TRAVELTIME_H

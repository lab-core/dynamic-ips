//
// Created by Elahe Amiri on 2022-10-13.
//

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "utilities/MyTools.h"

//-----------------------------------------------------------------------------
//  Parameters Struct
//-----------------------------------------------------------------------------
struct Parameters {
public:
    // model Parameters
    int epochLength_{};             // epoch length for re-optimization
    int nbThreads_{};               // number of threads for parallel optimization
    MainAlgorithm mainAlgorithm_;   // the algorithm used for solving the MP
    bool oneIter_;                  // solve master problem one time at each iteration of the CG
    bool greedyReOptimize_;         // restart greedy (re-assigning) considering the current state of the system
    int nbColumn_;                  // number of columns to be added in column generation


    // label setting strategies
    bool isTruncated_{};            // acceleration strategy
    int MaxLabel_{};                // max number of labels to keep while using truncated labeling
    bool isDropPickPossible_{};     // acceleration strategy
    int nbStop_;
    SortPaths sortPath_;
    float MIPGap_{};


    // Constructor and Destructor
    Parameters(int epochLength, int nbThreads, MainAlgorithm mainAlgorithm, bool oneIter, bool greedyReOptimize,
               int nbColumn, bool isTruncated, int maxLabel, bool isDropPickPossible, int nbStop, SortPaths sortPath,
               float MIPGap);

    virtual ~Parameters();

    // Display function
    std::string toString() const;
    std::string toStr() const;
};


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------
struct solverOption {
    bool isTruncated_;
    bool isDropPickPossible_;
    int MaxLabel_;
    int nbStop_;
    SortPaths pathSort_;
    // Constructor and Destructor
    solverOption(bool isTruncated, int maxLabel, bool isDominanceReleased, int nbStop, SortPaths pathSort,
                 bool isDropPickPossible);

    explicit solverOption(PParameters &MainParams);

    virtual ~solverOption();
};

std::string boolToString(bool value);


#endif //PARAMETERS_H

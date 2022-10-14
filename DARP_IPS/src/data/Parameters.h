//
// Created by Elahe Amiri on 2022-10-13.
//

#ifndef _PARAMETERS_H
#define _PARAMETERS_H

#include "utilities/MyTools.h"

//-----------------------------------------------------------------------------
//  Parameters Struct
//-----------------------------------------------------------------------------
struct Parameters {
public:
    // model Parameters
    float alphaParam_{};
    float betaParam_{};
    float deltaPram_{};
    int epochLength_{};
    InitialDual initialDual_;
    MainAlgorithm mainAlgorithm_;
    bool addOneRequestColumn_{};

    // ISUD parameters
    warmStart initialStart_;
    int MIP_maxIncDegree_;
    int CP_IncDegree_;
    float minImp_;
    bool fixedEpoch_;

    // label setting strategies
    bool isTruncated_{};
    int MaxLabel_{};
    bool isDominanceReleased_{};
    bool isSuccessorsLimited_{};
    bool isDropPickPossible_{};
    SubProSolveStart SubproSolveStartState_;
    LabelingStrategy LabelingStrategy_;
    subproblemAlgorithm subAlgorithm_;

    //CPLEX Parameters
    int bigM_{};
    int solveTimeLimit_{};
    int populateTimeLimit_{};

    // Constructor and Destructor
    Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, InitialDual initialDual,
               MainAlgorithm mainAlgorithm, warmStart initialStart, int MIP_maxIncDegree, int CP_IncDegree,
               float minImp, bool fixedEpoch, bool isTruncated, int maxLabel, bool isSuccessorsLimited,
               bool isDominanceReleased, bool isDropPickPossible, SubProSolveStart subproSolveStartState,
               LabelingStrategy LabelingStrategy, subproblemAlgorithm subAlgorithm, int bigM, int solveTimeLimit,
               int populateTimeLimit, bool addOneRequestColumn);

    virtual ~Parameters();

    // Display function
    std::string toString() const;
};


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------
struct solverOption {
    float maxReachTime_;
    int maxPickup_;

    bool isTruncated_;
    bool isDominanceReleased_;
    bool isSuccessorsLimited_;
    bool isDropPickPossible_;
    LabelingStrategy LabelingStrategy_;
    int MaxLabel_;

    // Constructor and Destructor
    solverOption(float maxReachTime, int maxPickup, bool isTruncated, int maxLabel, bool isDominanceReleased,
                 bool isSuccessorsLimited, bool isDropPickPossible, LabelingStrategy labelingStrategy);

    solverOption(float maxReachTime, int maxPickup, PParameters &MainParams);

    virtual ~solverOption();
    void disableHeuristics();
    bool areHeuristicsDisabled() const;
};


#endif //_PARAMETERS_H

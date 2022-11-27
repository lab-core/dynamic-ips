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
    float alphaParam_{};
    float betaParam_{};
    float deltaPram_{};
    int epochLength_{};
    int penaltyL_{};
    float committedTime_{};
    int nbThreads_{};
    InitialDual initialDual_;
    MainAlgorithm mainAlgorithm_;
    bool addOneRequestColumn_{};
    SolutionMode solutionMode_;

    // ISUD parameters
    warmStart initialStart_;
    int MIP_maxIncDegree_;      // max incompatibility degree for Zoom
    int CP_IncDegree_;          // max incompatibility degree for CP
    bool useMultiStage_;       // min incompatibility degree that CP starts from in multi-stage
    float minImp_;
    bool useZoom_;


    // label setting strategies
    bool isTruncated_{};
    int MaxLabel_{};
    bool isDominanceReleased_{};
    bool isSuccessorsLimited_{};
    bool isDropPickPossible_{};
    SubProSolveMode SubproSolveMode_;
    LabelingStrategy LabelingStrategy_;
    subproblemAlgorithm subAlgorithm_;
    float vehicle_portion_;
    bool greedyPortion_{};

    //CPLEX Parameters
    int bigM_{};
    int solveTimeLimit_{};
    int populateTimeLimit_{};

    // Constructor and Destructor
    Parameters(float alphaParam, float betaParam, float deltaPram, int epochLength, int penaltyL,
               float committedTime, int nbThreads, InitialDual initialDual, MainAlgorithm mainAlgorithm,
               warmStart initialStart, int MIP_maxIncDegree, int CP_IncDegree, bool useMultiStage, float minImp,
               bool useZoom, bool isTruncated, int maxLabel, bool isSuccessorsLimited, bool isDominanceReleased,
               bool isDropPickPossible, SubProSolveMode subproSolveMode, LabelingStrategy LabelingStrategy,
               subproblemAlgorithm subAlgorithm, float vehicle_portion, bool greedyPortion, int bigM,
               int solveTimeLimit, int populateTimeLimit, bool addOneRequestColumn, SolutionMode solutionMode);

    virtual ~Parameters();

    // Display function
    std::string toString() const;
};


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------
struct solverOption {
    float maxReachTime_;
 //   int maxPickup_;

    bool isTruncated_;
    bool isDominanceReleased_;
    bool isSuccessorsLimited_;
    bool isDropPickPossible_;
    LabelingStrategy LabelingStrategy_;
    int MaxLabel_;

    // Constructor and Destructor
    solverOption(float maxReachTime, bool isTruncated, int maxLabel, bool isDominanceReleased,
                 bool isSuccessorsLimited, bool isDropPickPossible, LabelingStrategy labelingStrategy);

    explicit solverOption(PParameters &MainParams);
    void updateOptions(float maxReachTime);

    virtual ~solverOption();
    void disableHeuristics();
    bool areHeuristicsDisabled() const;
};


#endif //PARAMETERS_H

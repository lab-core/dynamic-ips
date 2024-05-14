//
// Created by Elahe Amiri on 2022-10-13.
//

#include "Parameters.h"

//************************************************************************
//                     FUNCTIONS FOR PARAMETERS CLASS
//************************************************************************

// Constructor and Destructor
Parameters::Parameters(int epochLength, int nbThreads, MainAlgorithm mainAlgorithm, bool oneIter, bool greedyReOptimize,
                       int nbColumn, bool isTruncated, int maxLabel, bool isDropPickPossible, int nbStop,
                       SortPaths sortPath, float MIPGap) :
        epochLength_(epochLength), nbThreads_(nbThreads), mainAlgorithm_(mainAlgorithm),
        oneIter_(oneIter), greedyReOptimize_(greedyReOptimize),
        nbColumn_(nbColumn), isTruncated_(isTruncated), MaxLabel_(maxLabel), isDropPickPossible_(isDropPickPossible),
        nbStop_(nbStop), sortPath_(sortPath),MIPGap_(MIPGap) {}

Parameters::~Parameters() = default;

// Display function
std::string Parameters::toString() const {
    std::stringstream repStr;
    repStr << "# MODEL PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    int setwLength = 30;
    repStr << std::left << std::fixed << std::setprecision(1) << std::boolalpha;
    repStr << std::setw(setwLength) << "# epoch Length " << " = " << epochLength_ << " (s)" << std::endl;
    repStr << std::setw(setwLength) << "# number of threads " << " = " << nbThreads_ << std::endl;
    repStr << std::setw(setwLength) << "# main algorithm " << " = " << mainAlgorithmName[mainAlgorithm_] << std::endl;
    repStr << std::setw(setwLength) << "# One iter per epoch " << " = " << oneIter_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Greedy Re-Optimized " << " = " << greedyReOptimize_ << std::endl;
    repStr << std::setw(setwLength) << "# Column added to MP " << " = " << nbColumn_ << std::endl;
    repStr << std::endl;

    repStr << "# LABEL SETTING STRATEGIES" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::setw(setwLength) << "# Use Truncated Labeling " << " = " << isTruncated_ << std::endl;
    repStr << std::setw(setwLength) << "# MaxLabel in Truncating " << " = " << MaxLabel_ << std::endl;
    repStr << std::setw(setwLength) << "# Is Pickup allowed after Drop " << " = " << isDropPickPossible_ << std::endl;
    repStr << std::setw(setwLength) << "# number of pickups allowed " << " = " << nbStop_ << std::endl;
    repStr << std::setw(setwLength) << "# Sorting mode of paths " << " = " << SortPathsName[sortPath_] << std::endl;
    repStr << std::endl;

    repStr << "# CPLEX PARAMETERS" << std::endl;
    repStr << "#" << std::endl;
    repStr << std::left << std::fixed << std::setprecision(0);
    repStr << std::setw(setwLength) << "# MIP Gap " << " = " << MIPGap_ << std::endl;

    return repStr.str();

}

std::string Parameters::toStr() const {
    std::stringstream repStr;
    repStr << epochLength_ << ",";
    repStr << nbThreads_ << ",";
    repStr << mainAlgorithmName[mainAlgorithm_] << ",";
    repStr << boolToString(oneIter_) << ",";
    repStr << boolToString(greedyReOptimize_) << ",";
    repStr << nbColumn_ << ",";
    repStr << boolToString(isTruncated_) << ",";
    repStr << MaxLabel_ << ",";
    repStr << boolToString(isDropPickPossible_) << ",";
    repStr << nbStop_ << ",";
    repStr << SortPathsName[sortPath_] << ",";
    repStr << MIPGap_ << "\n";
    return repStr.str();
}


//-----------------------------------------------------------------------------
//  Solver Option Struct
//-----------------------------------------------------------------------------

solverOption::solverOption(bool isTruncated, int maxLabel, bool isDominanceReleased, int nbStop, SortPaths pathSort,
                           bool isDropPickPossible) :
        isTruncated_(isTruncated), nbStop_(nbStop), MaxLabel_(maxLabel), pathSort_(pathSort),
        isDropPickPossible_(isDropPickPossible){}

solverOption::~solverOption() = default;


solverOption::solverOption(PParameters &MainParams) {
    isTruncated_ = MainParams->isTruncated_;
    MaxLabel_ = MainParams->MaxLabel_;
    isDropPickPossible_ = MainParams->isDropPickPossible_;
    nbStop_ = MainParams->nbStop_;
    pathSort_ = MainParams->sortPath_;
}


std::string boolToString(bool value) {
    return value ? "True" : "False";
}

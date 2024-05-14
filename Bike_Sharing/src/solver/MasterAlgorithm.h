//
// Created by Ella on 2021-10-09.
//

#ifndef _MASTERALGORITHM_H
#define _MASTERALGORITHM_H

#include "data/Instance.h"
#include "solver/MasterModeler.h"

//enum selectionMode { NR = 0, RP = 1, CP = 2};

//---------------------------------------------------------------------------------------------
//  ISUD algorithm to solve the master problem
//---------------------------------------------------------------------------------------------

class MasterAlgorithm {
public:

    PMasterModeler MasterPro_;
    vector2D<PRoute> availableRoutes_;
    int nbRoutes_;
    float availableTime_;           // time left from availableTime_ to solve master problem
    int timeLimit_;                 // total available time to solve master problem

    // Solution containers
    std::vector<PRoute> routeSolution_;

    double maxReducedCost_;      // max threshold for the reduced costs selection in CP
    double minReducedCost_;

    int RMPCounter_;            // iteration of solving Master Problem in each epoch
    int SPIter_;                    // number of sub problem iteration
    int LPIter_;                    // number of LRMP iteration
    int MIPIter_;                   // number of Reduce Problem iteration

    double objValue_;               // objective value during MP iterations
    double lpObjValue_;             // linear objective value
    double MPEpochSolveTime_;       // save the total time used to solve MP models (CG and MIP)

    myTools::Timer *masterTime_;
    myTools::Timer *MPBuildTime_;


    Tools::LogOutput* pLogIsudResultsStream_;


    // Constructor and Destructor
    explicit MasterAlgorithm(InputPaths &inputPaths);

    virtual ~MasterAlgorithm();

    // Getters and Setters
    void setObjValue();

    void setAvailableTime();

    // this function create initial routes serving only one request and fill zSolution_ with available requests
    // Reduced problem is also solved to initialized dual costs
    void initialization(PInstance &pInst, InputPaths &inputPaths);


    // this function updates the reduced cost for the routes in the pool
    void updateReducedCosts(PInstance &pInst);

    void solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime);

    // These functions are used to solve master problems (CG, MP and RP)
    void solveMP_LP(PInstance &pInst, InputPaths &inputPaths);

    // Display function
    std::string toString() const;
    std::string toStringTimersTotal() const;
    std::string toStringTimersAvg(int epoch) const;

    // function to evaluate available routes and find proper ones to be added to the models
    void updateRoutesToAdd(selectionMode selectMode, PInstance &pInst);

    std::string save_MPResults(int epoch, const std::string& model, int nbColumns, float reachTime, double subProTime,
                                 double auxObj) const;
};


#endif //_MASTERALGORITHM_H
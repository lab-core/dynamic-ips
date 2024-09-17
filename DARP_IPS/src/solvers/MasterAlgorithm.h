//
// Created by Ella on 2021-10-09.
//

#ifndef _MASTERALGORITHM_H
#define _MASTERALGORITHM_H

#include "data/Instance.h"
#include "solvers/ReducedProblem.h"
#include "solvers/ComplementPro.h"
#include "solvers/GreedyModeler.h"
#include "solvers/MIPMasterProblem.h"

//enum selectionMode { NR = 0, RP = 1, CP = 2};

//---------------------------------------------------------------------------------------------
//  ISUD algorithm to solve the master problem
//---------------------------------------------------------------------------------------------

class MasterAlgorithm {
public:
//    PReducedProblem ReducedPro_;
    PComplementPro CompPro_;
    PReducedProblem ReducedPro_;
    PMasterPro MasterPro_;
    vector2D<PRoute> availableRoutes_;
    //   vector2D<int> subProResults_;
    int nbRoutes_;
    int nbColumnsAdded_;
    int nbVehicles_;
    float availableTime_;           // time left from availableTime_ to solve master problem
    int timeLimit_;                 // total available time to solve master problem

    // Solution containers
    std::vector<PRoute> routeSolution_;
    std::vector<PRequest> zSolution_;

    std::vector<PRoute> InRouteSolution_;      // Initial solutions
    std::vector<PRequest> InZSolution_;

    int maxIncDegree_;
    int cpIncDegree_;

    int nbCoveredTasks_;
    double maxReducedCost_;      // max threshold for the reduced costs selection in CP
    double minReducedCost_;

    int RMPCounter_;            // iteration of solving Master Problem in each epoch
    int SPIter_;                    // number of sub problem iteration
    int SPIters_;                   // total number of sub problem iteration
    int RPIter_;                    // number of RP iteration
    int CPIter_;                    // number of CP iteration
    int LPIter_;                    // number of LRMP iteration
    int MIPIter_;                   // number of Reduce Problem iteration
    int ZoomIter_;                  // number of Zoom iteration
    int CPSuccess_;                 // number of time CP succeed in finding integer
    int CPFails_;                   // number of time CP fails in finding integer
    int CGSuccess_;                 // number of time that CG where able to converge
    double objValue_;               // objective value during MP iterations
    double lpObjValue_;             // linear objective value
    double totalWaitTime_;          // total waiting time without penalties
    double GreedyObjValue_;         // objective value of Greedy method
    double MPEpochSolveTime_;       // save the total time used to solve MP models (RP, CG and MIP)
    double CPEpochSolveTime_;       // save the total time used to solve CP models
    bool CPBuilt_;                  // check whether CP model is built or not (one time during each iteration CP is built)

    myTools::Timer *masterTime_;
    myTools::Timer *RPTime_;
    myTools::Timer *CPTime_;

    myTools::Timer *MPBuildTime_;
    myTools::Timer *CPBuildTime_;
    myTools::Timer *ZOOMTime_;

    Tools::LogOutput* pLogIsudResultsStream_;
 //   Tools::LogOutput* pLogIterReqDualStream_;
 //   Tools::LogOutput* pLogIterVehDualStream_;

    // Constructor and Destructor
    explicit MasterAlgorithm(InputPaths &inputPaths);

    virtual ~MasterAlgorithm();

    // Getters and Setters
    void setObjValue();

    void setAvailableTime();

    // this function create initial routes serving only one request and fill zSolution_ with available requests
    // Reduced problem is also solved to initialized dual costs
    void initialization(PInstance &pInst, InputPaths &inputPaths);
    void initializationCG(PInstance &pInst, InputPaths &inputPaths);

    // these function update the incompatibility degree of availableRoutes
    void calcIncompatibilityBit(PRoute &route, PInstance &pInst);
    void updateIncDegreesBit(PInstance &pInst);

    // this function updates the reduced cost for the routes in the pool
    void updateReducedCosts(PInstance &pInst);

    void solveISUD(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime);
    void solveISUD_improved(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime);
    void solveMP_CG(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime);
    void solveRLMP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime);
    void solveRMP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime);
    void solveMP_MIP(PInstance &pInst, int epoch, InputPaths &inputPaths, double subProTime);

    // These functions are used to solve master problems (CG, MP and RP)
    void solveRP_LPINT(PInstance &pInst, int compDegree, InputPaths &inputPaths);
    void solveMP_LP(PInstance &pInst, InputPaths &inputPaths);
    void solveMP_INT(PInstance &pInst, InputPaths &inputPaths);

    // Display function
    std::string toString() const;
    std::string toStringTimersTotal() const;
    std::string toStringTimersAvg(int epoch) const;

    // function to evaluate available routes and find proper ones to be added to the models
    void updateRoutesToAdd(selectionMode selectMode, PInstance &pInst);
    void updateRoutesToAddZoom();

    // function to save the reduced costs and incompatibility degree of the created routes
    void save_IncDegree_RDCost(InputPaths &inputPaths, int epoch, int isudIter);
    std::string save_ISUDResults(int epoch, const std::string& model, int nbColumns, float reachTime, double subProTime,
                                 double auxObj) const;
    void setAvailableTime(PInstance &pInst, double elapsedTime);
};


#endif //_MASTERALGORITHM_H
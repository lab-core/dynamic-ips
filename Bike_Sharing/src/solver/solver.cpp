//
// Created by Elahe Amiri on 2022-10-26.
//

#include "solver.h"

solver::solver(PInstance & mainInst, InputPaths &inputPaths) {
    elapsedTime_ = 0;
    SubproEpochTime_ = 0;
    epoch_ = 0;

    masterModel_ = std::make_shared<MasterAlgorithm>(inputPaths);
    subProOptions_ = std::make_shared<solverOption>(mainInst->parameters_);

    simulationTime_ = new myTools::Timer(); simulationTime_->init();
    subProblemTime_ = new myTools::Timer(); subProblemTime_->init();
    preprocessTime_ = new myTools::Timer(); preprocessTime_->init();

    masterEpochTime_ = 0;
    isudMIPEpochTime_ = 0;
    avgEpochRuntime_ = 0;
    epochRuntime_ = 0;
    nbGenerated_ = 0;
    nbDominated_ = 0;
    nbEliminated_ = 0;
    nbNegativeFound_ = 0;
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);


}

solver::~solver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;

}

void solver::solveCG_Epoch(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths) {
    // define required variables
    double previousObj;
    int nbNegativeFound;
    nbNegativeFound_ = 0;
    bool isSolved = false;
    int iter = 0;
    std::stringstream changeStr;
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);
//    masterModel_->availableTime_ = (int)(EpochInst->parameters_->epochLength_);
    masterModel_->availableTime_ = LARGE_CONSTANT;

    masterModel_->initialization(EpochInst, inputPaths);
    masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
    for (auto &vehicleObj: EpochInst->vehicles_) {
        vehicleObj->setEmptyRoute();
        masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
    }

    masterModel_->RMPCounter_ ++;

    SubproEpochTime_ = 0;
    masterModel_->lpObjValue_ = masterModel_->objValue_;

    masterModel_->nbRoutes_ = 0;
    EpochInst->selectedVehicles_.clear();
    EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
    while (true) {
        iter++;
        nbNegativeFound = 0;
        previousObj = masterModel_->objValue_;


        //***********************************************************************************//
        //                    L A B E L  S E T T I N G    M E T H O D
        //***********************************************************************************//

        // start the subproblems timer
        subProblemTime_->start();
        // defining subproblems

        nbGenerated_ = 0;
        nbDominated_ = 0;
        nbEliminated_ = 0;

        std::vector<PLabelingSubPro> subProSolve;


        // create subproblems

        for (auto &vehicleObj: mainInst->vehicles_) {
            subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
        }


        // initializing and solving subproblems
        /*std::stable_sort(subProSolve.begin(), subProSolve.end(),[](const PLabelingSubPro &lhs, const PLabelingSubPro &rhs){
            return lhs->subGraph_->nbNodes_ > rhs->subGraph_->nbNodes_;});*/
        masterModel_->SPIter_++;
        for (auto &subProblem: subProSolve){
            Tools::Job job([&]() {
                subProblem->initSubGraph(EpochInst);
                subProblem->labelPool_ = std::move(labelsPool_.pop_data());
                subProblem->solveDynamic();
                subProblem->SolutionToRoutes(EpochInst->vehicles_[subProblem->Vehicle_->vehicleID_],
                                             masterModel_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_],
                                             mainInst);
                labelsPool_.push_data(std::move(subProblem->labelPool_));
            });
            pPool->run(job);
        }
        //      pPool->wait();
        while(true){
            if (!pPool->wait())
                break;
        }

        for (auto &subProblem: subProSolve){
            masterModel_->nbRoutes_ += masterModel_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_].size();
            nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;
            nbNegativeFound_ = nbNegativeFound_ + subProblem->nbNegativeColumns_;
            nbGenerated_ += subProblem->nbGenerated_;
            nbEliminated_ += subProblem->nbEliminated_;
            nbDominated_ += subProblem->nbDominated_;
        }

        while(true){
            if (!pPool->wait())
                break;
        }

        preprocessTime_->start();
        subProSolve.clear();
        preprocessTime_->stop();
        subProblemTime_->stop();
        SubproEpochTime_ += subProblemTime_->dSinceStart().count();
        if (nbNegativeFound == 0) {
            break;
        }
        else {
            /*masterModel_->availableTime_ = (int)(EpochInst->parameters_->epochLength_ -
                                                 simulationTime_->dSinceStart().count());*/
            masterModel_->availableTime_ = LARGE_CONSTANT;
            if (iter == 1 && masterModel_->availableTime_ < 3)
                masterModel_->availableTime_ = 3;
            masterModel_->timeLimit_ = masterModel_->availableTime_;

             //solve the restricted Mater Problem
            masterModel_->solveMP_CG(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());

            // if the size of epoch is larger than 30s disable the following
            if (simulationTime_->dSinceStart().count()/iter > (EpochInst->parameters_->epochLength_ - simulationTime_->dSinceStart().count()))
                break;

        }
        if (previousObj == masterModel_->objValue_) {
            break;
        }

        subProSolve.clear();
        break;
    }  // end of CG while

    for (auto & routeObj : masterModel_->routeSolution_) {
        for (auto & taskObj : routeObj->assignedTasks_)
            taskObj->assignedVehicleID_ = routeObj->vehicleID_;
    }

    // revert solution
    for (auto & routeObj : masterModel_->routeSolution_) {
        EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }

    masterModel_->setObjValue();

}






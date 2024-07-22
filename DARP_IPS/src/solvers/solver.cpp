//
// Created by Elahe Amiri on 2022-10-26.
//

#include "solver.h"

solver::solver(PInstance & mainInst, InputPaths &inputPaths) {
    elapsedTime_ = 0;
    SubproEpochTime_ = 0;
    epoch_ = 0;

    masterModel_ = std::make_shared<MasterAlgorithm>(inputPaths);
    GreedyModel_ = std::make_shared<GreedyModeler>();
    subProOptions_ = std::make_shared<solverOption>(mainInst->parameters_);

    simulationTime_ = new myTools::Timer(); simulationTime_->init();
    subProblemTime_ = new myTools::Timer(); subProblemTime_->init();
    preprocessTime_ = new myTools::Timer(); preprocessTime_->init();

    masterEpochTime_ = 0;
    RPEpochTime_ = 0;
    CPEpochTime_ = 0;
    RPEpochBuildTime_ = 0;
    CPEpochBuildTime_ = 0;
    isudMIPEpochTime_ = 0;
    avgEpochRuntime_ = 0;
    epochRuntime_ = 0;
    GreedyTime_ = 0;
    AssignTime_ = 0;

    nbGenerated_ = 0;
    nbDominated_ = 0;
    nbEliminated_ = 0;
    nbNegativeFound_ = 0;
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);


    // this Stream define runtime outputs
    pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
    (*pLogRunTimesStream_) << "Epoch,nbRequests,nbNodes,EpochRuntime,ElapsedTime, MP_Runtime, "
                              "RP_Runtime,MP_BuildRuntime,MP_SolveRuntime,CP_Runtime,CP_BuildRuntime, "
                              "CP_SolveRuntime,ZoomISUD_Runtime,SubProbRuntime,destructTime,SubAssignTime, "
                              "GreedyTime,#SP Iter,totalColumn,#LGenerated,#LDominated,#LEliminated,nbNegative,GreedyObj,"
                              "Objective,LinearObjective,waitTime" << std::endl;


    pLogEpochSubRuntimeStream_ = new Tools::LogOutput(inputPaths.getOutputSubproSize());
    (*pLogEpochSubRuntimeStream_) << "Epoch,vehicleID,nbRequests,nbNodes,maxPick,nbGenerated,nbEliminated, "
                                 "nbDominated,nbRoutes,solveTime,ConvertRouteTime" << std::endl;

    /*pLogSolutionChange_ = new Tools::LogOutput(inputPaths.getOutputSolutionChange());
    (*pLogSolutionChange_) << "Epoch,#SubProblems,#Requests,#NewArrival,#PreScheduled, #ReScheduled, "
                              "#Inc(1 deg.),#Inc(2 deg.),#Inc(3 deg.),#Inc(4 deg.),#Inc(5 deg.),#Inc(total)" << std::endl;*/
}

solver::~solver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;
    pLogRunTimesStream_->close();
    pLogEpochSubRuntimeStream_->close();
//    pLogEpochSubRouteStream_->close();
//    pLogSolutionChange_->close();
    delete pLogRunTimesStream_;
    delete pLogEpochSubRuntimeStream_;
//    delete pLogEpochSubRouteStream_;
//    delete pLogSolutionChange_;
}
void solver::solveCG_Epoch(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths) {
    std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    // define required variables
    bool truncateState = mainInst->parameters_->isTruncated_;
    double previousObj;
    int nbNegativeFound;
    nbNegativeFound_ = 0;
    bool isSolved = false;
    int iter = 0;

    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);

    if (EpochInst->parameters_->initialStart_ == GREEDY_START)
        GreedyModel_->GreedySolver(EpochInst);

    // Set available time
    switch (EpochInst->parameters_->solutionMode_) {
        case ANYTIME:
            masterModel_->availableTime_ = static_cast<int>(EpochInst->parameters_->committedTime_);
            break;
        case DYNAMIC:
            masterModel_->availableTime_ = static_cast<int>(EpochInst->parameters_->epochLength_);
            break;
        case STATIC:
            masterModel_->availableTime_ = LARGE_CONSTANT;
            break;
    }
    masterModel_->initialization(EpochInst, inputPaths);
    for (auto &vehicleObj: EpochInst->vehicles_) {
        vehicleObj->vehicleIndex_ = -1;
        masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
    }
    for (auto & vehicleObj : EpochInst->vehicles_){
        for (auto & requestObj : vehicleObj->currentRoute_->routeRequests_)
            requestObj->initialVehicleID_ = vehicleObj->vehicleID_;
    }
    masterModel_->RMPCounter_ ++;

    SubproEpochTime_ = 0;
    masterModel_->lpObjValue_ = masterModel_->objValue_;

    masterModel_->nbRoutes_ = 0;
    EpochInst->updateTaskIndexLabeling();
    EpochInst->selectedVehicles_.clear();
    EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
    while (true) {
        iter++;
        nbNegativeFound = 0;
        previousObj = masterModel_->objValue_;
        masterModel_->SPIters_++;


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
        if ((EpochInst->parameters_->addOneRequestColumn_ && iter == 1)|| !(EpochInst->parameters_->greedyPortion_ || EpochInst->parameters_->zonePortion_)){
            if (EpochInst->parameters_->addOneRequestColumn_ && iter == 1)
                subProOptions_->isTruncated_ = false;
            // select all the vehicles
            EpochInst->selectedVehicles_.clear();
            EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
            for (auto & vehicleObj : EpochInst->vehicles_) {
                EpochInst->selectedVehicles_[vehicleObj->vehicleID_] = iter;
            }
        }

        else if (EpochInst->parameters_->greedyPortion_) {
            subProOptions_->isTruncated_ = truncateState;
            if (!isSolved){
                EpochInst->selectedVehicles_.clear();
                EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
            }
            bool state = EpochInst->parameters_->greedyReOptimize_;
            EpochInst->parameters_->greedyReOptimize_ = true;
            GreedyModel_->GreedyAssignment(EpochInst, iter);
            EpochInst->parameters_->greedyReOptimize_ = state;
            isSolved = true;
            //           EpochInst->resetZoneVehicles();
        }
        else if (EpochInst->parameters_->zonePortion_) {
            subProOptions_->isTruncated_ = truncateState;
            if (!isSolved) {
                EpochInst->resetZoneVehicles();
                isSolved = true;
            }
            EpochInst->selectVehiclesByZone(iter);
        }
        // add vehicles in previous solution
        if (EpochInst->parameters_->initialStart_ != GREEDY_START) {
            for (auto &vehicleObj: EpochInst->vehicles_) {
                if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                    EpochInst->selectedVehicles_[vehicleObj->vehicleID_] = iter;
                }
            }
        }
        // create subproblems
        masterModel_->nbVehicles_ = 0;
        for (auto &vehicleObj: EpochInst->vehicles_) {
            vehicleObj->vehicleIndex_ = -1;
            if (EpochInst->selectedVehicles_[vehicleObj->vehicleID_] >= 1) {
                subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
                if (iter > 1)
                    subProSolve.back()->maxPickup_ = subProOptions_->nbPick_;
                vehicleObj->vehicleIndex_ = masterModel_->nbVehicles_;
                masterModel_->nbVehicles_++;
            }
        }
        if (!EpochInst->parameters_->constPortion_){
            // the problem is solved in complete mode and not partially, reset the orders (for constraints)
            for (int v = 0; v < EpochInst->vehicles_.size(); v++) {
                EpochInst->vehicles_[v]->vehicleIndex_ = v;
            }
            masterModel_->nbVehicles_ = EpochInst->nbVehicles_;
        }

        // initializing and solving subproblems
        /*std::stable_sort(subProSolve.begin(), subProSolve.end(),[](const PLabelingSubPro &lhs, const PLabelingSubPro &rhs){
            return lhs->subGraph_->nbNodes_ > rhs->subGraph_->nbNodes_;});*/
        masterModel_->SPIter_++;
        for (auto &subProblem: subProSolve){
            if (EpochInst->parameters_->solutionMode_ == DYNAMIC && (masterModel_->availableTime_ - subProblemTime_->dSinceStart().count() <= 3)){
                if ((EpochInst->parameters_->addOneRequestColumn_ && iter > 2)||
                    (!EpochInst->parameters_->addOneRequestColumn_ && iter > 1))
                    break;
            }
            Tools::Job job([&]() {
                subProblem->initSubGraph(EpochInst);
                subProblem->labelPool_ = std::move(labelsPool_.pop_data());
                if (!subProblem->subRequests_.empty()) {
                    subProblem->solveDynamic();
                    subProblem->SolutionToRoutes(EpochInst->vehicles_[subProblem->Vehicle_->vehicleID_],
                                                 masterModel_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_],
                                                 mainInst);
                }
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
        preprocessTime_->start();
        subProSolve.clear();
        preprocessTime_->stop();
        subProblemTime_->stop();
        SubproEpochTime_ += subProblemTime_->dSinceStart().count();
        if (nbNegativeFound == 0) {
            masterModel_->CGSuccess_++;
            std::cout << "Terminate CG-> No negative column " << std::endl;
            break;
        }
        else {
            if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                masterModel_->availableTime_ = (int)(EpochInst->parameters_->committedTime_ -
                                                     simulationTime_->dSinceStart().count());
            else if (EpochInst->parameters_->solutionMode_ == DYNAMIC) {
                masterModel_->availableTime_ = (int)(EpochInst->parameters_->epochLength_ -
                                                     simulationTime_->dSinceStart().count());
                if ((EpochInst->parameters_->addOneRequestColumn_ && iter == 2)||
                (!EpochInst->parameters_->addOneRequestColumn_ && iter == 1)){
                    if (masterModel_->availableTime_ < 4)
                        masterModel_->availableTime_ = 4;
                }
                else if (masterModel_->availableTime_ <= 0){
                    std::cout << "available time: " << masterModel_->availableTime_ << std::endl;
                    break;
                }
            }
            else
                masterModel_->availableTime_ = LARGE_CONSTANT;

            if (iter <= 2 && masterModel_->availableTime_ < 4)
                masterModel_->availableTime_ = 4;


            masterModel_->timeLimit_ = masterModel_->availableTime_;

            //solve the restricted Mater Problem
            switch(EpochInst->parameters_->mainAlgorithm_) {
                case MP_CG:
                    masterModel_->solveMP_CG(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                case MP_MIP:
                    masterModel_->solveMP_MIP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                default: // MP_ISUD:
                    masterModel_->solveISUD(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
            }
            if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                break;
            if (mainInst->parameters_->oneIter_) {
                if ((EpochInst->parameters_->addOneRequestColumn_ && iter == 2) ||
                    (!EpochInst->parameters_->addOneRequestColumn_ && iter == 1))
                    break;
            }
        }
        if (previousObj == masterModel_->objValue_) {
            masterModel_->CGSuccess_++;
            std::cout << "No changes in Objective" << std::endl;
            break;
        }

        subProSolve.clear();
        std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    }  // end of CG while

    for (auto & routeObj : masterModel_->routeSolution_) {
        for (auto & requestObj : routeObj->routeRequests_)
            requestObj->allocVehicleID_ = routeObj->vehicleID_;
    }

    masterModel_->setObjValue();
    std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
}

void solver::solveCG_Epoch1(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths) {
    std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    // define required variables
    bool truncateState = mainInst->parameters_->isTruncated_;
    double previousObj;
    int nbNegativeFound;
    nbNegativeFound_ = 0;
    bool isSolved = false;
    int iter = 0;
 //   std::stringstream changeStr;

    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);
    if (EpochInst->parameters_->initialStart_ == GREEDY_START)
        GreedyModel_->GreedySolver(EpochInst);
    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
        masterModel_->availableTime_ = (int)(EpochInst->parameters_->committedTime_);
    else if (EpochInst->parameters_->solutionMode_ == DYNAMIC)
        masterModel_->availableTime_ = (int)(EpochInst->parameters_->epochLength_);
    else
        masterModel_->availableTime_ = LARGE_CONSTANT;

    masterModel_->initialization(EpochInst, inputPaths);
    for (auto &vehicleObj: EpochInst->vehicles_) {
        vehicleObj->vehicleIndex_ = -1;
        masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
    }
    for (auto & vehicleObj : EpochInst->vehicles_){
        for (auto & requestObj : vehicleObj->currentRoute_->routeRequests_)
            requestObj->initialVehicleID_ = vehicleObj->vehicleID_;
    }
    masterModel_->InRouteSolution_ = masterModel_->routeSolution_;
    masterModel_->InZSolution_ = masterModel_->zSolution_;

    masterModel_->RMPCounter_ ++;

    SubproEpochTime_ = 0;
    masterModel_->lpObjValue_ = masterModel_->objValue_;

    masterModel_->nbRoutes_ = 0;
    EpochInst->updateTaskIndexLabeling();
    EpochInst->selectedVehicles_.clear();
    EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
    while (true) {
        iter++;
        nbNegativeFound = 0;
        previousObj = masterModel_->objValue_;
        masterModel_->SPIters_++;


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

        masterModel_->nbVehicles_ = 0;
        if ((EpochInst->parameters_->addOneRequestColumn_ && iter == 1)|| !(EpochInst->parameters_->greedyPortion_ || EpochInst->parameters_->zonePortion_)){
            if (EpochInst->parameters_->addOneRequestColumn_ && iter == 1)
                subProOptions_->isTruncated_ = false;
            // select all the vehicles
            EpochInst->selectedVehicles_.clear();
            EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
            for (auto & vehicleObj : EpochInst->vehicles_) {
                EpochInst->selectedVehicles_[vehicleObj->vehicleID_] = iter;
            }
        }


        else if (EpochInst->parameters_->greedyPortion_) {
            subProOptions_->isTruncated_ = truncateState;
            if (!isSolved){
                EpochInst->selectedVehicles_.clear();
                EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
            }
            bool state = EpochInst->parameters_->greedyReOptimize_;
            EpochInst->parameters_->greedyReOptimize_ = true;
            GreedyModel_->GreedyAssignment(EpochInst, iter);
            EpochInst->parameters_->greedyReOptimize_ = state;
            isSolved = true;
 //           EpochInst->resetZoneVehicles();
        }
        else if (EpochInst->parameters_->zonePortion_) {
            subProOptions_->isTruncated_ = truncateState;
            if (!isSolved) {
                EpochInst->resetZoneVehicles();
                isSolved = true;
            }
            EpochInst->selectVehiclesByZone(iter);
        }

        // add vehicles in previous solution
        if (EpochInst->parameters_->initialStart_ != GREEDY_START) {
            for (auto &vehicleObj: EpochInst->vehicles_) {
                if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                    EpochInst->selectedVehicles_[vehicleObj->vehicleID_] = iter;
                }
            }
        }
        // create subproblems
        masterModel_->nbVehicles_ = 0;
        for (auto &vehicleObj: EpochInst->vehicles_) {
            vehicleObj->vehicleIndex_ = -1;
//           masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
            if (EpochInst->selectedVehicles_[vehicleObj->vehicleID_] >= 1) {
                subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
                if (iter > 1)
                    subProSolve.back()->maxPickup_ = subProOptions_->nbPick_;
                vehicleObj->vehicleIndex_ = masterModel_->nbVehicles_;
                masterModel_->nbVehicles_++;
            }
        }

        if (!EpochInst->parameters_->constPortion_){
            // the problem is solved in complete mode and not partially, reset the orders (for constraints)
            for (int v = 0; v < EpochInst->vehicles_.size(); v++) {
                EpochInst->vehicles_[v]->vehicleIndex_ = v;
            }
            masterModel_->nbVehicles_ = EpochInst->nbVehicles_;
        }

        /*if (masterModel_->RMPCounter_ == 1) {
            changeStr << epoch_ << "," << subProSolve.size() << "," << EpochInst->nbRequests_ << ",";
            changeStr << EpochInst->nbNewRequests_ << "," << EpochInst->nbRequests_ - EpochInst->nbNewRequests_ << ",";
        }*/


        /*std::cout << "nb Requests: " << EpochInst->nbRequests_ << std::endl;
        std::cout << "nb new Requests: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "nb of sub problems: " << subProSolve.size() << std::endl;*/

        // initializing and solving subproblems
        /*std::stable_sort(subProSolve.begin(), subProSolve.end(),[](const PLabelingSubPro &lhs, const PLabelingSubPro &rhs){
            return lhs->subGraph_->nbNodes_ > rhs->subGraph_->nbNodes_;});*/
        masterModel_->SPIter_++;
        for (auto &subProblem: subProSolve){
            if (EpochInst->parameters_->solutionMode_ == DYNAMIC && iter > 1 && (masterModel_->availableTime_ - subProblemTime_->dSinceStart().count() <= 3))
                break;
            Tools::Job job([&]() {
                subProblem->initSubGraph(EpochInst);
                subProblem->labelPool_ = std::move(labelsPool_.pop_data());
                if (!subProblem->subRequests_.empty()) {
                    subProblem->solveDynamic();
                    subProblem->SolutionToRoutes(EpochInst->vehicles_[subProblem->Vehicle_->vehicleID_],
                                                 masterModel_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_],
                                                 mainInst);
                }
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
        /*if (EpochInst->parameters_->initialStart_ != GREEDY_START)
            GreedyModel_->GreedySolver(EpochInst, masterModel_->availableRoutes_, EpochInst->nbRequests_);*/
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
            std::cout << "Terminate CG-> No negative column " << std::endl;
            break;
        }
        else {
            if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                masterModel_->availableTime_ = (int)(EpochInst->parameters_->committedTime_ -
                        simulationTime_->dSinceStart().count());
            else if (EpochInst->parameters_->solutionMode_ == DYNAMIC) {
                masterModel_->availableTime_ = (int)(EpochInst->parameters_->epochLength_ -
                        simulationTime_->dSinceStart().count());
                if (iter == 1 && masterModel_->availableTime_ < 3)
                    masterModel_->availableTime_ = 3;
            }
            else
                masterModel_->availableTime_ = LARGE_CONSTANT;
 //           std::cout << "Available time: " << masterModel_->availableTime_ << std::endl;
            if (masterModel_->availableTime_ <= 0 && EpochInst->parameters_->solutionMode_ == DYNAMIC){
                std::cout << "available time: " << masterModel_->availableTime_ << std::endl;
                break;
            }
            masterModel_->timeLimit_ = masterModel_->availableTime_;

             //solve the restricted Mater Problem
            switch(EpochInst->parameters_->mainAlgorithm_) {
                case MP_CG:
                    masterModel_->solveMP_CG(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                case MP_MIP:
                    masterModel_->solveMP_MIP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                /*case MP_CP:
                    masterModel_->solveMP_MIPCP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;*/
                default: // MP_ISUD:
                    masterModel_->solveISUD(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
            }
            if ((EpochInst->parameters_->solutionMode_ == ANYTIME)||(mainInst->parameters_->oneIter_))
                break;
            // if the size of epoch is larger than 30s disable the following
            /*else if (EpochInst->parameters_->solutionMode_ == DYNAMIC)
                if (simulationTime_->dSinceStart().count()/iter > (EpochInst->parameters_->epochLength_ - simulationTime_->dSinceStart().count())) {
                    std::cout << "iter: " << iter << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
                    std::cout << "Remaining time lest than half" << std::endl;
                    break;
                }*/
            /*else if (subProblemTime_->dSinceStart().count() > (EpochInst->parameters_->epochLength_ - simulationTime_->dSinceStart().count()))
                break;*/
        }
        if (previousObj == masterModel_->objValue_) {
            break;
        }

        subProSolve.clear();
        std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    }  // end of CG while

    for (auto & routeObj : masterModel_->routeSolution_) {
        for (auto & requestObj : routeObj->routeRequests_)
            requestObj->allocVehicleID_ = routeObj->vehicleID_;
    }
//    int numReplace = 0;
//    std::vector<int> numInc(6,0);
    /*for (auto & requestObj : EpochInst->requests_){
        if (requestObj->initialVehicleID_  < LARGE_CONSTANT && requestObj->allocVehicleID_ != requestObj->initialVehicleID_)
            numReplace++;
    }*/

    // rest solution
    for (auto & routeObj : masterModel_->InRouteSolution_) {
        EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }
    for (auto & requestObj : masterModel_->InZSolution_)
        requestObj->solVehicleID_  = LARGE_CONSTANT;
    /*for (auto & routeObj : masterModel_->routeSolution_) {
        masterModel_->calcIncompatibilityBit(routeObj, EpochInst);
        if (routeObj->incompatibilityDegree_>0)
            numInc[routeObj->incompatibilityDegree_]++;
    }*/
    /*changeStr << numReplace << "," << numInc[1] << "," << numInc[2] << "," << numInc[3] <<"," <<  numInc[4] <<"," <<  numInc[5] << ",";
    changeStr << std::accumulate(numInc.begin(), numInc.end(), 0) << "\n";
    (*pLogSolutionChange_) << changeStr.str();*/

    // revert solution
    for (auto & routeObj : masterModel_->routeSolution_) {
        EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }


    masterModel_->setObjValue();
    std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
}

void solver::anyTimeSolver(PInstance &mainInst, InputPaths &inputPaths, std::string& instNum, bool middleSave,
                           float saveTime) {
    // define required variables
    int nbReceivedRequest;
    epoch_ = 0;
    std::vector<float> EpochTime = {1,1,1};
//    int commitTime = mainInst->parameters_->epochLength_;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);

    while (nbReceivedRequest < mainInst->nbRequests_ || !masterModel_->zSolution_.empty()) {
        nextEpoch:
        simulationTime_->start();
 //       preprocessTime_->start();
        elapsedTime_ = simulationTime_->dSinceInit().count();
        std::cout << "---------------------"<< std::endl;
        std::cout << " ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << " PRE EPOCH TIME: " << epochRuntime_ << std::endl;
        std::cout << " EPOCH: " << epoch_ << std::endl;
        std::cout << "---------------------"<< std::endl;
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "---------------------------------------------------"<< std::endl;
        logFile << " EPOCH: " << epoch_ << std::endl;
        logFile.close();
        EpochTime[epoch_ % EpochTime.size()] = epochRuntime_;
        int avg = ceil(std::accumulate(EpochTime.begin(), EpochTime.end(),0) / EpochTime.size());
        if (mainInst->parameters_->committedTime_ > std::max(avg, mainInst->parameters_->epochLength_)) {
            mainInst->parameters_->committedTime_ = std::max(avg, mainInst->parameters_->epochLength_);
            std::cout << "dec commit time: " << mainInst->parameters_->committedTime_ << std::endl;
        }
        if (mainInst->parameters_->committedTime_ < epochRuntime_) {
            mainInst->parameters_->committedTime_ = ceil(epochRuntime_ + 2);
            std::cout << "inc commit time: " << mainInst->parameters_->committedTime_ << std::endl;
        }

//        mainInst->parameters_->committedTime_ = commitTime;

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
//            vehicleObj->updateCurrentRoute(elapsedTime_);
            vehicleObj->updateStateTime(elapsedTime_, mainInst->parameters_->committedTime_,
                                        mainInst->simulationStartTime_, mainInst->parameters_->vehicleReturn_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
        }
        masterModel_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, masterModel_->zSolution_ , elapsedTime_, nbReceivedRequest);
        EpochInst->updatePenalties(elapsedTime_);
        nbReceivedRequest += EpochInst->nbNewRequests_;
     //   std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;

        if (elapsedTime_ >= saveTime && middleSave) {
            if (EpochInst->parameters_->mainAlgorithm_ == GREEDY){
                for (auto & requestObj: EpochInst->requests_)
                    requestObj->dual_ = requestObj->penalty_;
            }
            /*inputPaths.makeInstanceOutput(instNum);
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + elapsedTime_,1.5 * mainInst->parameters_->epochLength_);*/
            inputPaths.makeInstanceOutput(instNum);
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + elapsedTime_,3600*5);
            break;
        }
        if (EpochInst->nbNewRequests_ == 0 && masterModel_->zSolution_.empty()) {
 //           std::cout << "next event" << std::endl;
            simulationTime_->stop();
            simulationTime_->addTime(mainInst->requests_[nbReceivedRequest]->earlyPick_ - mainInst->simulationStartTime_ - simulationTime_->dSinceInit().count());
 //           preprocessTime_->stop();
 //           (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
            epoch_++;
            goto nextEpoch;
        }
//        preprocessTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            solveCG_Epoch(EpochInst, mainInst, inputPaths);
        else if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);

        simulationTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes();
    }

}

void solver::anyTimeSolverEvent(PInstance &mainInst, InputPaths &inputPaths) {
    // define required variables
    int nbReceivedRequest;
    epoch_ = 0;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);

    while (nbReceivedRequest < mainInst->nbRequests_) {
        nextEpoch:
        simulationTime_->start();

        elapsedTime_ = mainInst->requests_[nbReceivedRequest]->earlyPick_ - mainInst->simulationStartTime_;
        std::cout << "---------------------"<< std::endl;
        std::cout << " ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << " PRE EPOCH TIME: " << epochRuntime_ << std::endl;
        std::cout << " EPOCH: " << epoch_ << std::endl;
        std::cout << "---------------------"<< std::endl;

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateStateTime(elapsedTime_, mainInst->parameters_->committedTime_,
                                        mainInst->simulationStartTime_, mainInst->parameters_->vehicleReturn_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
        }
        masterModel_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, masterModel_->zSolution_ , elapsedTime_, nbReceivedRequest);
        EpochInst->updatePenalties(elapsedTime_);
        nbReceivedRequest += EpochInst->nbNewRequests_;

        if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);

        // update routes
        /*for (auto &vehicleObj: mainInst->vehicles_)
            vehicleObj->updateCurrentRoute(elapsedTime_);*/

        simulationTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes();
    }

}

void solver::staticSolver(PInstance &mainInst, InputPaths &inputPaths, std::string& instNum, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;
    int nbReceivedRequest;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    // initialize vehicle routes at source
    for (auto & vehicleObj: mainInst->vehicles_) {
        vehicleObj->solutionRoute_ = std::make_shared<Route>(vehicleObj->vehicleID_);
        vehicleObj->solutionRoute_->addSource(vehicleObj->emptyRoute_->routeNodes_[0], vehicleObj->departTime_, vehicleObj->numPassengers_);
    }
    simulationTime_->start();
//    preprocessTime_->start();
    nbReceivedRequest = mainInst->nbOnboards_;
    PInstance StaticInst = std::make_shared<Instance>(*mainInst);
    StaticInst->buildStaticData(mainInst, nbReceivedRequest);

    simulationTime_->stop();
    masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
    for (auto &vehicleObj: mainInst->vehicles_){
        vehicleObj->currentRoute_->createColumn();
    }

    StaticInst->updatePenalties(0);

//    preprocessTime_->stop();
    switch(StaticInst->parameters_->mainAlgorithm_) {
        case MIP_CPLEX :
            simulationTime_->start();
            MIPSolver(StaticInst, inputPaths);
            /*std::cout << "# FINAL SOLUTION OF MIP solution : " << std::endl;
            for (int v = 0; v < StaticInst->nbVehicles_; ++v)
                std::cout << StaticInst->vehicles_[v]->currentRoute_->toString();*/
            simulationTime_->stop();
            break;
        case GREEDY:
            simulationTime_->start();
            GreedyModel_->GreedySolver(StaticInst);
            std::cout << "# FINAL SOLUTION OF Greedy solution : " << GreedyModel_->objValue_<< std::endl;
            for (auto & vehicleObj : StaticInst->vehicles_){
                vehicleObj->currentRoute_->testRoute(vehicleObj);
//                std::cout << vehicleObj->currentRoute_->toString();
            }

            if (middleSave) {
                inputPaths.makeInstanceOutput(instNum);
                float length = 0;
                for (auto & vehicleObj: StaticInst->vehicles_)
                    vehicleObj->updateStateTime(saveTime, length, mainInst->simulationStartTime_,
                                                mainInst->parameters_->vehicleReturn_);
                inputPaths.makeInstanceOutput(instNum);
                StaticInst->saveStatus(inputPaths, StaticInst->simulationStartTime_ + saveTime,mainInst->parameters_->epochLength_);
                inputPaths.makeInstanceOutput("2");
                StaticInst->saveStatus(inputPaths, StaticInst->simulationStartTime_ + saveTime,3600*5);
                break;
            }
            else {
                for (auto & vehicleObj : StaticInst->vehicles_)
                    vehicleObj->solutionRoute_ = vehicleObj->currentRoute_;
            }
            std::cout << "# TIME SPENT ON GREEDY " << "=" << GreedyModel_->greedyTime_->dSinceInit().count() << " (seconds)" << std::endl;
            simulationTime_->stop();
            break;
        default: // CG_CPLEX and CG_ISUD
            simulationTime_->start();
            solveCG_Epoch(StaticInst, mainInst, inputPaths);
            simulationTime_->stop();

    }

    if (!middleSave && StaticInst->parameters_->mainAlgorithm_ != GREEDY) {
        for (auto & vehicleObj : mainInst->vehicles_) {
            vehicleObj->finalizeSolutionRoutes();
        }
    }
}

void solver::dynamicSolver(PInstance &mainInst, InputPaths &inputPaths, std::string instNum, bool middleSave, float saveTime) {
    // define required variables
    bool MIP_Stop = false;
    int nbReceivedRequest;
    epoch_ = 0;
    int instance_count = 1;

    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);
    while ((!solveEpoch && (nbReceivedRequest < mainInst->nbRequests_ || !masterModel_->zSolution_.empty())) ||
           (solveEpoch && nbReceivedRequest < mainInst->nbRequests_)) {
        nextEpoch:
        // start simulation timer
        simulationTime_->start();
//        preprocessTime_->start();

        elapsedTime_ = simulationTime_->dSinceInit().count();
        std::cout << "---------------------"<< std::endl;
        std::cout << " ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << " EPOCH: " << epoch_ << std::endl;
        std::cout << "---------------------"<< std::endl;

        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "---------------------------------------------------"<< std::endl;
        logFile << " EPOCH: " << epoch_ << std::endl;
        logFile.close();
        // update vehicle status
        mainInst->nbOnboards_ = 0;
        masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateState(epoch_, mainInst->parameters_->epochLength_,
                                    mainInst->simulationStartTime_, mainInst->parameters_->vehicleReturn_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
        }
        masterModel_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in previous epoch

        EpochInst->buildPartialData(mainInst, masterModel_->zSolution_,
                                    static_cast<float>((epoch_) * mainInst->parameters_->epochLength_),
                                    nbReceivedRequest);
        for (auto &vehicleObj: mainInst->vehicles_){
            vehicleObj->currentRoute_->createColumn();
        }
        EpochInst->updatePenalties(mainInst->parameters_->epochLength_ * epoch_);
        if (epoch_ == 0 && mainInst->nbOnboards_ > 0)
            nbReceivedRequest += EpochInst->nbRequests_;
        else
            nbReceivedRequest += EpochInst->nbNewRequests_;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbRequests_ << std::endl;
        // saving the status in the middle of running
        if ((static_cast<float> (epoch_ * EpochInst->parameters_->epochLength_) >= saveTime) && middleSave ) {
            inputPaths.makeInstanceOutput(std::to_string(instance_count));
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ +
                                    static_cast<float>(epoch_ * EpochInst->parameters_->epochLength_),mainInst->parameters_->epochLength_);
            /*inputPaths.makeInstanceOutput("2");
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ +
                                    static_cast<float>(epoch_ * EpochInst->parameters_->epochLength_),3600*5);*/
            saveTime += 120;
            if (instance_count >= 30)
                break;
            instance_count ++;


  //          break;
        }

        if (EpochInst->nbRequests_ == 0) {
            simulationTime_->stop();
            epoch_++;
            goto nextEpoch;
        }
 //       preprocessTime_->stop();
        if (MIP_Stop) {
            if (epoch_ == 236) {
                EpochInst->parameters_->mainAlgorithm_ = MP_MIP;
                for (auto &requestObj: EpochInst->requests_)
                    requestObj->dual_ = requestObj->penalty_;
                for (auto &vehicleObj: EpochInst->vehicles_)
                    vehicleObj->dual_ = 0;
            }
            /*if (epoch_ == 4) {
                EpochInst->parameters_->mainAlgorithm_ = MP_ISUD;
                EpochInst->parameters_->oneIter_ = false;
 //               EpochInst->parameters_->greedyReOptimize_ = true;
//                EpochInst->parameters_->useZoom_ = true;
            }*/
            /*if (epoch_ == 5)
                break;*/
        }

        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            solveCG_Epoch(EpochInst, mainInst, inputPaths);
        else if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);
        simulationTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
 //       (*pLogEpochSolutionStream_) << EpochInst->saveEpochRoutes( epoch_);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes();
    }

}

// function to print epoch runTime to file
std::string solver::saveRuntimes(PInstance &EpochInst) {
    std::stringstream repStr;
    epochRuntime_ = simulationTime_->dSinceStart().count();
    avgEpochRuntime_ = simulationTime_->dSinceInit().count() / (epoch_ + 1);

    repStr << epoch_ << ",";
    repStr << EpochInst->nbRequests_ << ",";
    repStr << EpochInst->instGraph_->nbNodes_ - 2 * EpochInst->nbVehicles_ << ",";
    repStr << epochRuntime_ << ",";
    repStr << simulationTime_->dSinceInit().count() << ",";

    // master problem Times
    repStr << masterModel_->masterTime_->dSinceInit().count() - masterEpochTime_ << ",";
    repStr << masterModel_->RPTime_->dSinceInit().count() - RPEpochTime_ << ",";
    repStr << masterModel_->MPBuildTime_->dSinceInit().count() - RPEpochBuildTime_ << ",";
    repStr << masterModel_->MPEpochSolveTime_ << ",";
    repStr << masterModel_->CPTime_->dSinceInit().count() - CPEpochTime_ << ",";
    repStr << masterModel_->CPBuildTime_->dSinceInit().count() - CPEpochBuildTime_ << ",";
    repStr << masterModel_->CPEpochSolveTime_ << ",";
    repStr << masterModel_->ZOOMTime_->dSinceInit().count() - isudMIPEpochTime_ << ",";

    masterEpochTime_ = masterModel_->masterTime_->dSinceInit().count();
    RPEpochTime_ = masterModel_->RPTime_->dSinceInit().count();
    RPEpochBuildTime_ = masterModel_->MPBuildTime_->dSinceInit().count();
    CPEpochTime_ = masterModel_->CPTime_->dSinceInit().count();
    CPEpochBuildTime_ = masterModel_->CPBuildTime_->dSinceInit().count();
    isudMIPEpochTime_ = masterModel_->ZOOMTime_->dSinceInit().count();

    repStr << SubproEpochTime_ << ",";
    repStr << preprocessTime_ ->dSinceStart().count()<< ",";
    repStr << GreedyModel_->greedyAssignTime_->dSinceInit().count() - AssignTime_ << ",";
    repStr << GreedyModel_->greedyTime_->dSinceInit().count() - GreedyTime_ << ",";
    AssignTime_ = GreedyModel_->greedyAssignTime_->dSinceInit().count();
    GreedyTime_ = GreedyModel_->greedyTime_->dSinceInit().count();
    repStr << masterModel_->SPIter_ << ",";
    repStr << masterModel_->nbRoutes_ << ",";
    repStr << nbGenerated_ << ",";
    repStr << nbDominated_ << ",";
    repStr << nbEliminated_ << ",";
    repStr << nbNegativeFound_ << ",";
    repStr << masterModel_->GreedyObjValue_ << ",";
    repStr << masterModel_->objValue_ << ",";
    repStr << masterModel_->lpObjValue_ << ",";
    repStr << masterModel_->totalWaitTime_ << "\n";
    return repStr.str();
}

std::string solver::toString(PInstance & mainInst) const {
    std::stringstream repStr;
    repStr << "*************************************************************************************" << std::endl;
    repStr << "                        FINAL VEHICLE ROUTES AFTER " << std::setw(3) << epoch_ << " EPOCHS " << std::endl;
    repStr << "                                    " <<  solutionModeName[mainInst->parameters_->solutionMode_] << " MODE" << std::endl;
    repStr << "*************************************************************************************" << std::endl;
    repStr << std::endl << std::endl;
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << std::setw(30) << "# NUMBER OF VEHICLES" << " = " << mainInst->nbVehicles_ << std::endl;
    repStr << "# " << std::endl;
    repStr << "############################   PARAMETERS AND OPTIONS   ############################" << std::endl;
    repStr << mainInst->parameters_->toString();
    repStr << "# " << std::endl;
    repStr << std::endl << std::endl;
    repStr << "################################   FINAL ROUTES   ################################" << std::endl;
    repStr << "#" << std::left << std::endl;
    for (auto & vehicleObj : mainInst->vehicles_) {
        repStr << "#" << std::left << std::endl;
        repStr << "#\t" << std::setw(24) << "- IDLE_TIME" << " : " << vehicleObj->idleTime_ << std::endl;
        repStr << vehicleObj->solutionRoute_->toString();
    }
    repStr << std::endl << std::endl;
    repStr << "#################################   REQUESTS    #################################" << std::endl;
    repStr << "#" << std::endl;
    repStr << mainInst->solutionToString();
    repStr << std::left << std::fixed << std::setprecision(2);
    repStr << masterModel_->toStringTimersTotal();
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ASSIGNMENT" << " = " << GreedyModel_->greedyAssignTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << masterModel_->toStringTimersAvg(epoch_);
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count()/epoch_ << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count()/epoch_ << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ASSIGNMENT" << " = " << GreedyModel_->greedyAssignTime_->dSinceInit().count()/epoch_ << " (s)" << std::endl;
    mainInst->instRepStr_ << epoch_-1 << "," << masterModel_->LPIter_ << "," << masterModel_->MIPIter_ << ",";
    mainInst->instRepStr_ << masterModel_->RPIter_ << "," << masterModel_->CPIter_ << ",";
    mainInst->instRepStr_ << masterModel_->ZoomIter_ << ",";
    mainInst->instRepStr_ << masterModel_->SPIters_ << ",";
    mainInst->instRepStr_ << masterModel_->masterTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << masterModel_->RPTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_<< masterModel_->CPTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << masterModel_->ZOOMTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << subProblemTime_->dSinceInit().count() << "," << GreedyModel_->greedyTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << GreedyModel_->greedyAssignTime_->dSinceInit().count() << ",";
    float TotalTime = masterModel_->masterTime_->dSinceInit().count() + subProblemTime_->dSinceInit().count() +
                      GreedyModel_->greedyTime_->dSinceInit().count();
    mainInst->instRepStr_ << TotalTime << ",";
    if (masterModel_->masterTime_->dSinceInit().count() > 0 ){
        mainInst->instRepStr_ << masterModel_->RPTime_->dSinceInit().count() / masterModel_->masterTime_->dSinceInit().count() << ",";
        mainInst->instRepStr_ << masterModel_->CPTime_->dSinceInit().count() / masterModel_->masterTime_->dSinceInit().count() << ",";
    }
    else{
        mainInst->instRepStr_ << 0 << "," << 0 << ",";
    }
    mainInst->instRepStr_ << masterModel_->masterTime_->dSinceInit().count() / TotalTime << ",";
    mainInst->instRepStr_ << subProblemTime_->dSinceInit().count()/TotalTime << ",";
    mainInst->instRepStr_ << GreedyModel_->greedyTime_->dSinceInit().count()/TotalTime << ",";
    mainInst->instRepStr_ << masterModel_->CPSuccess_ << ",";
    mainInst->instRepStr_ << masterModel_->CPFails_ << ",";
    mainInst->instRepStr_ << masterModel_->CGSuccess_ << ",";
    return repStr.str();
}




//
// Created by Elahe Amiri on 2022-10-26.
//

#include "solver.h"

solver::solver(PInstance & mainInst, InputPaths &inputPaths) {
    elapsedTime_ = 0;
    SubproEpochTime_ = 0;
    epoch_ = 0;

    isudObj_ = std::make_shared<ISUDAlgorithm>(inputPaths);
    GreedyModel_ = std::make_shared<GreedyModeler>();
    subProOptions_ = std::make_shared<solverOption>(mainInst->parameters_);

    simulationTime_ = new myTools::Timer(); simulationTime_->init();
    subProblemTime_ = new myTools::Timer(); subProblemTime_->init();
    preprocessTime_ = new myTools::Timer(); preprocessTime_->init();

    isudEpochTime_ = 0;
    RPEpochTime_ = 0;
    CPEpochTime_ = 0;
    RPEpochBuildTime_ = 0;
    CPEpochBuildTime_ = 0;
    isudMIPEpochTime_ = 0;
    avgEpochRuntime_ = 0;
    epochRuntime_ = 0;
    GreedyTime_ = 0;
    AssignTime_ = 0;
    minSubSize_ = 0;
    maxSubSize_ = 0;
    avgSubSize_ = 0;
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);


    // this Stream define runtime outputs
    pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
    (*pLogRunTimesStream_) << "Epoch,nbRequests,nbNodes,EpochRuntime,AvgEpochRuntime,ElapsedTime, MP_Runtime, "
                              "RP_Runtime,RP_BuildRuntime,RP_SolveRuntime,CP_Runtime,CP_BuildRuntime, "
                              "CP_SolveRuntime,ZoomISUD_Runtime,SubProbRuntime,destructTime,SubAssignTime, "
                              "GreedyTime,minSubSize,maxSubSize,avgSubSize,GreedyObj,Objective,waitTime" << std::endl;

    pLogEpochSolutionStream_ = new Tools::LogOutput(inputPaths.getOutputEpochFinal());
    (*pLogEpochSolutionStream_) << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType,LocationID" << std::endl;

    pLogEpochSubRuntimeStream_ = new Tools::LogOutput(inputPaths.getOutputSubproSize());
    (*pLogEpochSubRuntimeStream_) << "Epoch,vehicleID,nbRequests,nbNodes,maxPick,nbGenerated,nbEliminated, "
                                 "nbDominated,nbRoutes,solveTime,ConvertRouteTime" << std::endl;

    pLogSolutionChange_ = new Tools::LogOutput(inputPaths.getOutputSolutionChange());
    (*pLogSolutionChange_) << "Epoch,#SubProblems,#Requests,#NewArrival,#PreScheduled, #ReScheduled, "
                              "#Inc(1 deg.),#Inc(2 deg.),#Inc(3 deg.),#Inc(4 deg.),#Inc(5 deg.),#Inc(total)" << std::endl;
}

solver::~solver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;
    pLogRunTimesStream_->close();
    pLogEpochSolutionStream_->close();
    pLogEpochSubRuntimeStream_->close();
//    pLogEpochSubRouteStream_->close();
    pLogSolutionChange_->close();
    delete pLogRunTimesStream_;
    delete pLogEpochSolutionStream_;
    delete pLogEpochSubRuntimeStream_;
//    delete pLogEpochSubRouteStream_;
    delete pLogSolutionChange_;
}

void solver::solveCG_Epoch(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths) {
    // define required variables
    double previousObj;
    int nbNegativeFound;
    bool isSolved = false;
    std::stringstream changeStr;
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);
    if (EpochInst->parameters_->initialStart_ == GREEDY_START)
        GreedyModel_->GreedySolver(EpochInst);
    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
        isudObj_->availableTime_ = (int)(EpochInst->parameters_->committedTime_);
    else
        isudObj_->availableTime_ = (int)(EpochInst->parameters_->epochLength_);

    isudObj_->initialization(EpochInst, inputPaths);
    for (auto & vehicleObj : EpochInst->vehicles_){
        for (auto & requestObj : vehicleObj->currentRoute_->routeRequests_)
            requestObj->initialVehicleID_ = vehicleObj->vehicleID_;
    }
    isudObj_->InrouteSolution_ = isudObj_->routeSolution_;
    isudObj_->InzSolution_ = isudObj_->zSolution_;
    // save initial solution
    /*if (epoch_ == 182)
        (*isudObj_->pLogIterSolutionStream_) << EpochInst->saveISUDRoutes(epoch_, isudObj_->isudIter_);*/
    isudObj_->isudIter_ ++;

    SubproEpochTime_ = 0;

    while (true) {
        nbNegativeFound = 0;
        previousObj = isudObj_->objValue_;


        //***********************************************************************************//
        //                    L A B E L  S E T T I N G    M E T H O D
        //***********************************************************************************//
        minSubSize_ = 999;
        maxSubSize_ = 0;
        avgSubSize_ = 0;
        // start the subproblems timer
        subProblemTime_->start();
        // defining subproblems
        isudObj_->nbRoutes_ = 0;
        EpochInst->updateTaskIndexLabeling();
        std::vector<PLabelingSubPro> subProSolve;

        if (!subProOptions_->usePick_ && EpochInst->nbRequests_ >= 200)
            subProOptions_->usePick_ = true;

        if (!subProOptions_->usePick_ && EpochInst->nbRequests_ >= 600)
            subProOptions_->MaxLabel_ = 10;

        isudObj_->nbVehicles_ = 0;
        if (EpochInst->parameters_->greedyPortion_){
            if (!isSolved) {
                bool state = EpochInst->parameters_->greedyReOptimize_;
                EpochInst->parameters_->greedyReOptimize_ = true;
                GreedyModel_->GreedyAssignment(EpochInst);
                EpochInst->parameters_->greedyReOptimize_ = state;
                isSolved = true;
                // add vehicles in previous solution
                if (EpochInst->parameters_->initialStart_ != GREEDY_START) {
                    for (auto &vehicleObj: EpochInst->vehicles_) {
                        if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                            GreedyModel_->selectedVehicles_[vehicleObj->vehicleID_]++;
                        }
                    }
                }
            }
            for (auto &vehicleObj: EpochInst->vehicles_) {
                vehicleObj->vehicleIndex_ = -1;
                isudObj_->availableRoutes_[vehicleObj->vehicleID_].clear();
                if (GreedyModel_->selectedVehicles_[vehicleObj->vehicleID_] > 0) {
                    subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
                    vehicleObj->vehicleIndex_ = isudObj_->nbVehicles_;
                    isudObj_->nbVehicles_++;
                }
            }
        }

        else {
            for (int v = 0; v < EpochInst->vehicles_.size(); v++) {
                isudObj_->availableRoutes_[EpochInst->vehicles_[v]->vehicleID_].clear();
                subProSolve.emplace_back(
                        std::make_shared<LabelingSubProblem>(EpochInst->vehicles_[v], subProOptions_));
                EpochInst->vehicles_[v]->vehicleIndex_ = isudObj_->nbVehicles_;
                isudObj_->nbVehicles_++;
            }
        }

        if (EpochInst->parameters_->vehicle_portion_ == 1){
            // the problem is solved in complete mode and not partially, rest the orders
            for (int v = 0; v < EpochInst->vehicles_.size(); v++) {
                EpochInst->vehicles_[v]->vehicleIndex_ = v;
            }
            isudObj_->nbVehicles_ = EpochInst->nbVehicles_;
        }


        changeStr << epoch_ << "," << subProSolve.size() << "," << EpochInst->nbRequests_ << ",";
        changeStr << EpochInst->nbNewRequests_ << "," << EpochInst->nbRequests_-EpochInst->nbNewRequests_ << ",";


        /*std::cout << "nb Requests: " << EpochInst->nbRequests_ << std::endl;
        std::cout << "nb new Requests: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "nb of sub problems: " << subProSolve.size() << std::endl;*/

        // initializing and solving subproblems
        /*std::stable_sort(subProSolve.begin(), subProSolve.end(),[](const PLabelingSubPro &lhs, const PLabelingSubPro &rhs){
            return lhs->subGraph_->nbNodes_ > rhs->subGraph_->nbNodes_;});*/
        isudObj_->SPIter_++;
        for (auto &subProblem: subProSolve){
            Tools::Job job([&]() {
                subProblem->initSubGraph2(EpochInst);
                subProblem->labelPool_ = std::move(labelsPool_.pop_data());
                if (!subProblem->subRequests_.empty()) {
                    subProblem->solveDynamic();
                    subProblem->SolutionToRoutes(EpochInst->vehicles_[subProblem->Vehicle_->vehicleID_],
                                                 isudObj_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_],
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

        /*for (auto &subProblem: subProSolve){
            subProblem->SolutionToRoutes(EpochInst->vehicles_[subProblem->Vehicle_->VehicleID_],
                                         isudObj_->availableRoutes_[(subProblem->Vehicle_)->VehicleID_], mainInst,
                                         EpochInst->nbRequests_);
        }*/
        for (auto &subProblem: subProSolve){
            isudObj_->nbRoutes_ += isudObj_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_].size();
            nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;

        }
        /*if (EpochInst->parameters_->initialStart_ != GREEDY_START)
            GreedyModel_->GreedySolver(EpochInst, isudObj_->availableRoutes_, EpochInst->nbRequests_);*/

        preprocessTime_->start();
        subProSolve.clear();
        preprocessTime_->stop();
        subProblemTime_->stop();
        SubproEpochTime_ += subProblemTime_->dSinceStart().count();
        if (nbNegativeFound == 0) {
            break;
        }
        else {
            if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                isudObj_->availableTime_ = (int)(EpochInst->parameters_->committedTime_ - simulationTime_->dSinceStart().count());
            else
                isudObj_->availableTime_ = 1000;
 //           std::cout << "Available time: " << isudObj_->availableTime_ << std::endl;
            if (isudObj_->availableTime_ <= 0 && EpochInst->parameters_->solutionMode_ == DYNAMIC){
                break;
            }

             //solve the restricted Mater Problem
            switch(EpochInst->parameters_->mainAlgorithm_) {
                case MP_CG:
                    isudObj_->solveMP_CG(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                case MP_MIP:
                    isudObj_->solveMP_MIP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                case MP_CP:
                    isudObj_->solveMP_MIPCP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                default: // MP_ISUD:
                    isudObj_->solveISUD(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
            }
            if ((EpochInst->parameters_->solutionMode_ == ANYTIME)||(mainInst->parameters_->oneIter_))
                break;
            else if (subProblemTime_->dSinceStart().count() > (EpochInst->parameters_->epochLength_ - simulationTime_->dSinceStart().count()))
                break;
        }
        if (previousObj == isudObj_->objValue_) {
            break;
        }
        while(true){
            if (!pPool->wait())
                break;
        }
        subProSolve.clear();
    }  // end of CG while

    for (auto & routeObj : isudObj_->routeSolution_) {
        for (auto & requestObj : routeObj->routeRequests_)
            requestObj->allocVehicleID_ = routeObj->vehicleID_;
    }
    int numReplace = 0;
    std::vector<int> numInc(6,0);
    for (auto & requestObj : EpochInst->requests_){
        if (requestObj->initialVehicleID_  < 999999 && requestObj->allocVehicleID_ != requestObj->initialVehicleID_)
            numReplace++;
    }

    // rest solution
    for (auto & routeObj : isudObj_->InrouteSolution_) {
        EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }
    for (auto & requestObj : isudObj_->InzSolution_)
        requestObj->solVehicleID_  = 999999;
    for (auto & routeObj : isudObj_->routeSolution_) {
        isudObj_->calcIncompatibilityBit(routeObj, EpochInst);
        if (routeObj->incompatibilityDegree_>0)
            numInc[routeObj->incompatibilityDegree_]++;
    }
    changeStr << numReplace << "," << numInc[1] << "," << numInc[2] << "," << numInc[3] <<"," <<  numInc[4] <<"," <<  numInc[5] << ",";
    changeStr << std::accumulate(numInc.begin(), numInc.end(), 0) << "\n";
    (*pLogSolutionChange_) << changeStr.str();

    // revert solution
    for (auto & routeObj : isudObj_->routeSolution_) {
        EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }


    isudObj_->setObjValue();
}

void solver::anyTimeSolver(PInstance &mainInst, InputPaths &inputPaths) {
    // define required variables
    int nbReceivedRequest;
    epoch_ = 0;
    std::vector<float> EpochTime = {1,1,1};
    int commitTime = mainInst->parameters_->committedTime_;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);

    while (nbReceivedRequest < mainInst->nbRequests_ || !isudObj_->zSolution_.empty()) {
        nextEpoch:
        simulationTime_->start();
 //       preprocessTime_->start();
        elapsedTime_ = simulationTime_->dSinceInit().count();
        /*std::cout << "---------------------"<< std::endl;
        std::cout << " ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << " PRE EPOCH TIME: " << epochRuntime_ << std::endl;
        std::cout << " EPOCH: " << epoch_ << std::endl;
        std::cout << "---------------------"<< std::endl;*/
        std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        logFile << "---------------------------------------------------"<< std::endl;
        logFile << " EPOCH: " << epoch_ << std::endl;
        logFile.close();
        EpochTime[epoch_ % EpochTime.size()] = epochRuntime_;
        int avg = ceil(std::accumulate(EpochTime.begin(), EpochTime.end(),0) / EpochTime.size());
        if (commitTime > std::max(avg, (int)mainInst->parameters_->committedTime_)) {
            commitTime = std::max(avg, (int)mainInst->parameters_->committedTime_);
            std::cout << "dec commit time: " << commitTime << std::endl;
        }
        if (commitTime < epochRuntime_) {
            commitTime = ceil(epochRuntime_ + 2);
            std::cout << "inc commit time: " << commitTime << std::endl;
        }

        mainInst->parameters_->committedTime_ = commitTime;

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        isudObj_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
//            vehicleObj->updateCurrentRoute(elapsedTime_);
            vehicleObj->updateStateTime(elapsedTime_, mainInst->parameters_->committedTime_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
        }
        isudObj_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, isudObj_->zSolution_ , elapsedTime_, nbReceivedRequest);
        EpochInst->updatePenalties(elapsedTime_);
        nbReceivedRequest += EpochInst->nbNewRequests_;
     //   std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;


        /*if ((epochRuntime_ > 150)||(EpochInst->nbRequests_ >= 400))
            break;*/
        if (EpochInst->nbNewRequests_ == 0 && isudObj_->zSolution_.empty()) {
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
        (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst);
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
        isudObj_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateStateTime(elapsedTime_, mainInst->parameters_->committedTime_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
        }
        isudObj_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, isudObj_->zSolution_ , elapsedTime_, nbReceivedRequest);
        EpochInst->updatePenalties(elapsedTime_);
        nbReceivedRequest += EpochInst->nbNewRequests_;

        if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);

        // update routes
        /*for (auto &vehicleObj: mainInst->vehicles_)
            vehicleObj->updateCurrentRoute(elapsedTime_);*/

        simulationTime_->stop();
        (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst);
    }

}

void solver::staticSolver(PInstance &mainInst, InputPaths &inputPaths, const std::string& instNum, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;
    int nbReceivedRequest;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
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
    isudObj_->availableRoutes_.resize(mainInst->nbVehicles_);
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
            std::cout << "# FINAL SOLUTION OF Greedy solution : " << std::endl;
            for (auto & vehicleObj : StaticInst->vehicles_){
                vehicleObj->currentRoute_->testRoute(vehicleObj, StaticInst->parameters_);
//                std::cout << vehicleObj->currentRoute_->toString();
            }

            if (middleSave) {
                inputPaths.makeInstanceOutput(instNum);
                float length = 0;
                for (auto & vehicleObj: StaticInst->vehicles_)
                    vehicleObj->updateStateTime(saveTime, length);
                StaticInst->saveStatus(inputPaths, StaticInst->simulationStartTime_ + saveTime);
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
            vehicleObj->finalizeSolutionRoutes(mainInst);
        }
    }
    (*pLogEpochSolutionStream_) << StaticInst->saveEpochRoutes( epoch_);
}

void solver::dynamicSolver(PInstance &mainInst, InputPaths &inputPaths, std::string instNum, bool middleSave, float saveTime) {
    // define required variables
    bool MIP_Stop = false;
    int nbReceivedRequest;
    epoch_ = 0;


    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);

    while (nbReceivedRequest < mainInst->nbRequests_ || !isudObj_->zSolution_.empty()) {
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
        isudObj_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateState(epoch_, mainInst->parameters_->epochLength_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
        }
        isudObj_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, isudObj_->zSolution_,
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
        //   std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;
        // saving the status in the middle of running
        if ((static_cast<float> (epoch_ * EpochInst->parameters_->epochLength_) >= saveTime) && middleSave ) {
            inputPaths.makeInstanceOutput(instNum);
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ +
                                    static_cast<float>(epoch_ * EpochInst->parameters_->epochLength_));
            break;
        }

        if (EpochInst->nbRequests_ == 0) {
            simulationTime_->stop();
            epoch_++;
            goto nextEpoch;
        }
 //       preprocessTime_->stop();
        if (MIP_Stop) {
            if (epoch_ == 115) {
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
        (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
 //       (*pLogEpochSolutionStream_) << EpochInst->saveEpochRoutes( epoch_);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst);
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
    repStr << avgEpochRuntime_ << ",";
    repStr << simulationTime_->dSinceInit().count() << ",";

    // isud Times
    repStr << isudObj_->isudTime_->dSinceInit().count() - isudEpochTime_ << ",";
    repStr << isudObj_->RPTime_->dSinceInit().count() - RPEpochTime_ << ",";
    repStr << isudObj_->RPBuildTime_->dSinceInit().count() - RPEpochBuildTime_ << ",";
    repStr << isudObj_->RPEpochSolveTime_ << ",";
    repStr << isudObj_->CPTime_->dSinceInit().count() - CPEpochTime_ << ",";
    repStr << isudObj_->CPBuildTime_->dSinceInit().count() - CPEpochBuildTime_ << ",";
    repStr << isudObj_->CPEpochSolveTime_ << ",";
    repStr << isudObj_->isudMIPTime_->dSinceInit().count() - isudMIPEpochTime_ << ",";

    isudEpochTime_ = isudObj_->isudTime_->dSinceInit().count();
    RPEpochTime_ = isudObj_->RPTime_->dSinceInit().count();
    RPEpochBuildTime_ = isudObj_->RPBuildTime_->dSinceInit().count();
    CPEpochTime_ = isudObj_->CPTime_->dSinceInit().count();
    CPEpochBuildTime_ = isudObj_->CPBuildTime_->dSinceInit().count();
    isudMIPEpochTime_ = isudObj_->isudMIPTime_->dSinceInit().count();

    repStr << SubproEpochTime_ << ",";
    repStr << preprocessTime_ ->dSinceStart().count()<< ",";
    repStr << GreedyModel_->greedyAssignTime_->dSinceInit().count() - AssignTime_ << ",";
    repStr << GreedyModel_->greedyTime_->dSinceInit().count() - GreedyTime_ << ",";
    AssignTime_ = GreedyModel_->greedyAssignTime_->dSinceInit().count();
    GreedyTime_ = GreedyModel_->greedyTime_->dSinceInit().count();
    repStr << minSubSize_ << ",";
    repStr << maxSubSize_ << ",";
    repStr << avgSubSize_<< ",";
    repStr << isudObj_->GreedyObjValue_ << ",";
    repStr << isudObj_->objValue_ << ",";
    repStr << isudObj_->totalWaitTime_ << "\n";
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
    repStr << isudObj_->toStringTimersTotal();
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ASSIGNMENT" << " = " << GreedyModel_->greedyAssignTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << isudObj_->toStringTimersAvg(epoch_);
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count()/epoch_ << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count()/epoch_ << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ASSIGNMENT" << " = " << GreedyModel_->greedyAssignTime_->dSinceInit().count()/epoch_ << " (s)" << std::endl;
    mainInst->instRepStr_ << epoch_-1 << "," << isudObj_->LPIter_ << "," << isudObj_->MPIter_ << ",";
    mainInst->instRepStr_ << isudObj_->RPIter_ << "," << isudObj_->CPIter_ << "," << isudObj_->isudTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << isudObj_->RPTime_->dSinceInit().count() << "," << isudObj_->CPTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << isudObj_->isudMIPTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << subProblemTime_->dSinceInit().count() << "," << GreedyModel_->greedyTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << GreedyModel_->greedyAssignTime_->dSinceInit().count() << ",";
    float TotalTime = isudObj_->isudTime_->dSinceInit().count() + subProblemTime_->dSinceInit().count() +
            GreedyModel_->greedyTime_->dSinceInit().count();
    mainInst->instRepStr_ << TotalTime << ",";
    if (isudObj_->isudTime_->dSinceInit().count() > 0 ){
        mainInst->instRepStr_ << isudObj_->RPTime_->dSinceInit().count()/isudObj_->isudTime_->dSinceInit().count()<< ",";
        mainInst->instRepStr_ << isudObj_->CPTime_->dSinceInit().count()/isudObj_->isudTime_->dSinceInit().count()<< ",";
    }
    else{
        mainInst->instRepStr_ << 0 << "," << 0 << ",";
    }
    mainInst->instRepStr_ << isudObj_->isudTime_->dSinceInit().count()/TotalTime << ",";
    mainInst->instRepStr_ << subProblemTime_->dSinceInit().count()/TotalTime << ",";
    mainInst->instRepStr_ << GreedyModel_->greedyTime_->dSinceInit().count()/TotalTime << ",";
    mainInst->instRepStr_ << isudObj_->CPSuccess_ << ",";
    mainInst->instRepStr_ << isudObj_->CPFails_ << ",";
    return repStr.str();
}





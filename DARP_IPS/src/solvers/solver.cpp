//
// Created by Elahe Amiri on 2022-10-26.
//

#include "solver.h"
#include "solvers/LabelingSubProblem.h"

solver::solver(const PInstance & mainInst, InputPaths &inputPaths) {
    elapsedTime_ = 0;
    SubproEpochTime_ = 0;
    epoch_ = 0;
    nbOnePick_ = 0;
    nbTwoPick_ = 0;
    nbThreePick_ = 0;
    nbRecycledColumns_ = 0;
    greedyRebalanceTime_ = 0;

    masterModel_ = std::make_shared<MasterAlgorithm>(inputPaths);
    GreedyModel_ = std::make_shared<GreedyModeler>();
    subProOptions_ = std::make_shared<solverOption>(mainInst->parameters_);

    simulationTime_ = new myTools::Timer(); simulationTime_->init();
    subProblemTime_ = new myTools::Timer(); subProblemTime_->init();
    preprocessTime_ = new myTools::Timer(); preprocessTime_->init();
    rebalancingTime_ = new myTools::Timer(); rebalancingTime_->init();

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
    nbPrunedArcs_ = 0;
    nbPrunedPath_ = 0;
    nbNegativeFound_ = 0;
    labelsPool_.defineSize(mainInst->parameters_->nbThreads_);


    // this Stream defines the runtime outputs
    pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
    (*pLogRunTimesStream_) << "Epoch,nbRequests,nbNewRequests,nbNodes,EpochRuntime,ElapsedTime,MP_Runtime,"
                              "RP_Runtime,MP_BuildRuntime,MP_SolveRuntime,CP_Runtime,CP_BuildRuntime,"
                              "CP_SolveRuntime,ZoomISUD_Runtime,SubProbRuntime,destructTime,SubAssignTime,"
                              "GreedyTime,#SP Iter,totalColumn,#LGenerated,#LDominated,#LEliminated,#nbPrunedArcs,"
                              " #nbPrunedPath,nbNegative,#ColumnsAdded,#RecycledColumns,GreedyObj,Objective,"
                              "LinearObjective,waitTime,#Return,#Idle,#PotentialIdle,#StateChanged,nbOnePick,nbTwoPick,nbThreePick" << std::endl;


    pLogEpochSubRuntimeStream_ = new Tools::LogOutput(inputPaths.getOutputSubproSize());
    (*pLogEpochSubRuntimeStream_) << "Epoch,vehicleID,nbRequests,nbNodes,maxPick,#LGenerated,#LDominated, "
                                     "#LEliminated, #LUnreachableDelay,nbRoutes,solveTime" << std::endl;

    /*pLogSolutionChange_ = new Tools::LogOutput(inputPaths.getOutputSolutionChange());
    (*pLogSolutionChange_) << "Epoch,#SubProblems,#Requests,#NewArrival,#PreScheduled, #ReScheduled, "
                              "#Inc(1 deg.),#Inc(2 deg.),#Inc(3 deg.),#Inc(4 deg.),#Inc(5 deg.),#Inc(total)" << std::endl;*/
}

solver::~solver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;
    delete rebalancingTime_;
    pLogRunTimesStream_->close();
    pLogEpochSubRuntimeStream_->close();
//    pLogEpochSubRouteStream_->close();
//    pLogSolutionChange_->close();
    delete pLogRunTimesStream_;
    delete pLogEpochSubRuntimeStream_;
//    delete pLogEpochSubRouteStream_;
//    delete pLogSolutionChange_;
}

void solver::selectVehiclesForSubproblem(const PInstance &EpochInst, int iter){
    if (!EpochInst->parameters_->vehiclePortion_ || iter == 1) {
        // The subproblems are solved for all the vehicles
        EpochInst->selectedVehicles_.assign(EpochInst->nbVehicles_, iter);
    }
    else if (iter == 2){
        // The subproblems are solved for the vehicles who have already some requests from the pre-iteration
        EpochInst->selectedVehicles_.assign(EpochInst->nbVehicles_, 0);
        for (const auto &vehicleObj : EpochInst->vehicles_) {
            if (!vehicleObj->currentRoute_->routeRequests_.empty()) {
                EpochInst->selectedVehicles_[vehicleObj->vehicleID_] = iter;
            }
        }
    }
}
void solver::solveCG_Epoch(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths) {
 //   std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;

    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);

    // Set available time
    masterModel_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), 1);
    EpochInst->updateTaskIndexLabeling();
    masterModel_->initialization(EpochInst, inputPaths, GreedyModel_);

    // define required variables

    int iter = 0;
    bool subProBreak = false;
    masterModel_->RMPCounter_ ++;
    masterModel_->nbRoutes_ = 0;
    SubproEpochTime_ = 0;
    masterModel_->lpObjValue_ = masterModel_->objValue_;
    EpochInst->selectedVehicles_.clear();
    EpochInst->selectedVehicles_.resize(EpochInst->nbVehicles_, 0);
    nbNegativeFound_ = 0;
    nbGenerated_ = 0;
    nbDominated_ = 0;
    nbPrunedArcs_ = 0;
    nbPrunedPath_ = 0;
    nbEliminated_ = 0;
    nbRecycledColumns_ = 0;


    while (true) {
        nbOnePick_ = 0;
        nbTwoPick_ = 0;
        nbThreePick_ = 0;

        // Set available time
        iter++;
        masterModel_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
        int nbNegativeFound = 0;
        float previousObj = masterModel_->objValue_;


        //***********************************************************************************//
        //                    LABEL SETTING METHOD
        //***********************************************************************************//
        // start the subproblems timer
        subProblemTime_->start();

        // defining subproblems
        std::vector<PLabelingSubPro> subProSolve;
        selectVehiclesForSubproblem(EpochInst, iter);

        // create subproblems
        int num = 0;
        for (auto &vehicleObj: EpochInst->vehicles_) {
            vehicleObj->bestReducedCost_ = INFINITY;

            if (EpochInst->selectedVehicles_[vehicleObj->vehicleID_] >= 1) {
                num++;
                subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
                if (EpochInst->parameters_->partialPricing_) {
                    if (vehicleObj->currentRoute_->routeRequests_.size() >= 2){
                        vehicleObj->numPickup_ = 2;
                        nbThreePick_ ++;
                    }
                    else if (!vehicleObj->currentRoute_->routeRequests_.empty()){
                        vehicleObj->numPickup_ = 2;
                        nbTwoPick_ ++;
                    }
                    else {
                        vehicleObj->numPickup_ = 1;
                        nbOnePick_ ++;
                    }
                    subProSolve.back()->maxPickup_= vehicleObj->numPickup_;
                    vehicleObj->preSolvePick_ = subProSolve.back()->maxPickup_;
                }
                else if (EpochInst->parameters_->dynamicPricing_) {
                    subProSolve.back()->maxPickup_ = std::min(iter, EpochInst->parameters_->nbPick_);
                    // if (iter >= 5) {
                    //     subProSolve.back()->solverOptions_->isTruncated_ = false;
                    //     subProSolve.back()->solverOptions_->isDropPickPossible_ = true;
                    // }
                }
                else
                    subProSolve.back()->maxPickup_ = EpochInst->parameters_->nbPick_;
 //                   subProSolve.back()->maxPickup_ = (epoch_ % 2 == 0) ? 1 : 2;

                if (EpochInst->parameters_->routeRecycle_)
                    subProSolve.back()->availableRoutes_ = masterModel_->availableRoutes_[vehicleObj->vehicleID_];
                masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
            }
        }
        // solving subproblems
        for (auto &subProblem: subProSolve) {
            if (subProBreak) break;
            if (EpochInst->parameters_->solutionMode_ == DYNAMIC &&
                masterModel_->availableTime_ - subProblemTime_->dSinceStart().count() <= 0 && iter > 1) {
                subProBreak = true;
                break;
            }
            Tools::Job job([&]() {
                subProblem->initSubGraph(EpochInst);
                subProblem->labelPool_ = std::move(labelsPool_.pop_data());
                if (!subProblem->subRequests_.empty()) {
                    if (subProblem->solveDynamic(
                            masterModel_->availableTime_ - subProblemTime_->dSinceStart().count())) {
                        subProblem->SolutionToRoutes(EpochInst->vehicles_[subProblem->Vehicle_->vehicleID_],
                                                     masterModel_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_],
                                                     mainInst, EpochInst->nbRequests_);
                    } else {
                        subProblem->CollectLabels();
                        subProBreak = true;
                    }
                }
                labelsPool_.push_data(std::move(subProblem->labelPool_));
            });
            pPool->run(job);
        }
        while(true){
            if (!pPool->wait())
                break;
        }

        for (auto &subProblem: subProSolve) {
            masterModel_->nbRoutes_ += static_cast<int>(masterModel_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_].size());
            nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;
            nbNegativeFound_ += subProblem->nbNegativeColumns_;
            nbGenerated_ += subProblem->nbGenerated_;
            nbDominated_ += subProblem->nbDominated_;
            nbEliminated_ += subProblem->nbEliminated_;
            nbPrunedPath_ += subProblem->nbPrunedPath_;
            nbPrunedArcs_ += subProblem->nbPrunedArcs_;
            nbRecycledColumns_ += subProblem->nbRecycledColumns_;
            //           (*pLogEpochSubRuntimeStream_) << subProblem->toStringOut(epoch_);
        }
        preprocessTime_->start();
        subProSolve.clear();
        preprocessTime_->stop();


        subProblemTime_->stop();
        SubproEpochTime_ += subProblemTime_->dSinceStart().count();
        if (subProBreak) {
            std::cout << "Terminate CG-> Not enough time to run the subproblems! " << std::endl;
            break;
        }
        masterModel_->SPIters_++;
        masterModel_->SPIter_++;

        if (nbNegativeFound == 0) {
            masterModel_->CGSuccess_++;
            std::cout << "Terminate CG-> No negative column " << std::endl;
            break;
        }
        else {
            // Update available time
            masterModel_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
            if (masterModel_->availableTime_ <= 0){
                break;
            }
            masterModel_->timeLimit_ = masterModel_->availableTime_;
            //solve the restricted Mater Problem
            masterModel_->epochTime_ += subProblemTime_->dSinceStart().count();
            if (iter == 1)
                EpochInst->resetDuals();
            switch(EpochInst->parameters_->mainAlgorithm_) {
                case RT_CG:
                    masterModel_->solveRLMP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                case A_CG:
                    masterModel_->solveMP_CG(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                case MP_MIP:
                    masterModel_->solveMP_MIP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                case MP_CP:
                    masterModel_->solveMP_CP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
                default: // MP_ISUD:
                    masterModel_->solveISUD(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
                    break;
            }
            // Update available time
            masterModel_->setAvailableTime(EpochInst, simulationTime_->dSinceStart().count(), iter);
            if (mainInst->parameters_->numIter_ == iter){
                break;
            }
        }
        if (masterModel_->availableTime_ <= 0)
            break;

        if (EpochInst->parameters_->mainAlgorithm_ == MP_ISUD && previousObj == masterModel_->objValue_) {
            masterModel_->CGSuccess_++;
            std::cout << "No changes in Objective" << std::endl;
            break;
        }

        std::cout << " simulation time: " << simulationTime_->dSinceStart().count() << std::endl;
    }  // end of CG while

    masterModel_->setObjValue();

    if (EpochInst->parameters_->mainAlgorithm_ == RT_CG) {
        masterModel_->availableTime_ = EpochInst->parameters_->epochLength_ - simulationTime_->dSinceStart().count();
        masterModel_->timeLimit_ = std::max(masterModel_->availableTime_, 10.0f);

        masterModel_->solveRMP(EpochInst, epoch_, inputPaths, subProblemTime_->dSinceStart().count());
    }

    // reset MP models
    if (EpochInst->parameters_->mainAlgorithm_ == MP_ISUD){
        masterModel_->CompPro_.reset();
        masterModel_->ReducedPro_.reset();
    }
    else {
        masterModel_->MasterPro_.reset();
        if (EpochInst->parameters_->initialDual_ == AUX_D)
            masterModel_->DualAuxSolver_.reset();
    }

    /*int lastEpoch = 0;
    if (mainInst->parameters_->solutionMode_ == ANYTIME)
        lastEpoch = EpochInst->simulationStartTime_ + elapsedTime_ - mainInst->parameters_->committedTime_;
    else
        lastEpoch = EpochInst->simulationStartTime_ + static_cast<float> (epoch_ * EpochInst->parameters_->epochLength_) - mainInst->parameters_->epochLength_;

    // Return Idle Vehicles
    if (EpochInst->parameters_->vehicleReturn_) {
        for (auto &vehicleObj: EpochInst->vehicles_){
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                if (vehicleObj->currentRoute_->routeNodes_.back()->locationID_ != vehicleObj->sinkNode_->locationID_){
                    vehicleObj->currentRoute_->addSink(vehicleObj->sinkNode_);
                    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                        vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
                }
            }
        }
    }*/
    if (EpochInst->parameters_->vehicleReturn_) {
        if (rebalancingTime_->dSinceStart().count() >= EpochInst->parameters_->epochLength_ || EpochInst->parameters_->solutionMode_ == DYNAMIC) {
            if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
                returnVehicles(EpochInst);
            else if (EpochInst->parameters_->returnPolicy_ == ZONE)
                returnVehiclesZone(EpochInst);
            else
                returnVehiclesAlonso(mainInst);
        }
    }


    if (EpochInst->parameters_->solutionMode_ == ANYTIME){
        for (auto &vehicleObj: EpochInst->vehicles_){
            if (vehicleObj->preSolvePick_ >= 2 && vehicleObj->idle_){
                vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
            }
        }
    }
    /*if (labelsPool_.getSize() > 70000) {
        labelsPool_.clear();
        labelsPool_.defineSize(mainInst->parameters_->nbThreads_);
    }*/

    std::cout << " end time: " << simulationTime_->dSinceStart().count() << std::endl;
}


void solver::anyTimeSolver(PInstance &mainInst, InputPaths &inputPaths, const std::string& instNum, bool middleSave,
                           float saveTime) {
    // define required variables
    epoch_ = 0;
    greedyRebalanceTime_ = 0;
    rebalancingTime_->start();
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

    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);
    float preObjective = masterModel_->objValue_;
    bool skip = false;
    masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
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
        float avg = ceil(std::accumulate(EpochTime.begin(), EpochTime.end(),0.0f) / static_cast<float>(EpochTime.size()));
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
        boost::dynamic_bitset<> removedRequests(EpochInst->nbRequests_);

        for (auto &vehicleObj: mainInst->vehicles_) {
//            vehicleObj->updateCurrentRoute(elapsedTime_);
            vehicleObj->updateStateTime(mainInst, mainInst->simulationStartTime_ + elapsedTime_, removedRequests);
            mainInst->nbOnboards_ += static_cast<int>(vehicleObj->onboards_.size());
        }
        if (mainInst->parameters_->routeRecycle_) {
            for (auto &vehicleObj: mainInst->vehicles_) {
                if (vehicleObj->stateChanged_)
                    masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
            }
            if (removedRequests.count()) {
                updateAvailableRoutes(removedRequests, masterModel_->availableRoutes_);
            }
        }
        else {
            masterModel_->availableRoutes_.clear();
            masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
        }

        masterModel_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in the previous epoch
        EpochInst->buildPartialData(mainInst, masterModel_->zSolution_ , elapsedTime_, nbReceivedRequest);
        for (auto &vehicleObj: mainInst->vehicles_){
            vehicleObj->currentRoute_->createColumn(EpochInst->nbRequests_);
            vehicleObj->emptyRoute_->createColumn(EpochInst->nbRequests_);
        }
        if (EpochInst->parameters_->timeWindow_ == 0)
            EpochInst->updatePenalties(elapsedTime_);
        if (epoch_ == 0 && mainInst->nbOnboards_ > 0)
            nbReceivedRequest += EpochInst->nbRequests_;
        else
            nbReceivedRequest += EpochInst->nbNewRequests_;
        std::cout << "# TOTAL NUMBER OF REQUESTS: " << EpochInst->nbRequests_ << std::endl;
        /*std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "# NUMBER OF NODES: " << EpochInst->instGraph_->nbNodes_ - 2 * EpochInst->nbNewRequests_ << std::endl;*/

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
        std::cout << "# TOTAL NUMBER OF NEW REQUESTS: " << EpochInst->nbNewRequests_ << std::endl;
        if (EpochInst->nbNewRequests_ == 0 && skip) {
            //           std::cout << "next event" << std::endl;
            simulationTime_->stop();
            simulationTime_->addTime(mainInst->requests_[nbReceivedRequest]->requestTime_ - mainInst->simulationStartTime_ - simulationTime_->dSinceInit().count());
            //           preprocessTime_->stop();
            //           (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
 //           epoch_++;
            goto nextEpoch;
        }
//        preprocessTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            solveCG_Epoch(EpochInst, mainInst, inputPaths);
        else if (EpochInst->parameters_->mainAlgorithm_ == GREEDY) {
            GreedyModel_->GreedySolver(EpochInst);
            if (EpochInst->parameters_->vehicleReturn_) {
                if (elapsedTime_ - greedyRebalanceTime_ >= EpochInst->parameters_->epochLength_ || EpochInst->parameters_->solutionMode_ == DYNAMIC) {
                    if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
                        returnVehicles(EpochInst);
                    else if (EpochInst->parameters_->returnPolicy_ == ZONE)
                        returnVehiclesZone(EpochInst);
                    else {
                        greedyRebalanceTime_ = elapsedTime_;
                        returnVehiclesAlonso(mainInst);
                    }
                }
            }
            if (EpochInst->parameters_->solutionMode_ == ANYTIME){
                for (auto &vehicleObj: EpochInst->vehicles_){
                    if (vehicleObj->currentRoute_->routeSize_ > 1 && vehicleObj->idle_){
                        vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
                    }
                }
            }
        }
        if (preObjective != masterModel_->objValue_)
            skip = false;
        else
            skip = true;

        preObjective = masterModel_->objValue_;
        EpochInst->setAssignedEpochVehicles(EpochInst->simulationStartTime_ + elapsedTime_ + simulationTime_->dSinceInit().count());
        simulationTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            *pLogRunTimesStream_ << saveRuntimes(EpochInst);
        epoch_++;
    }
    elapsedTime_ = simulationTime_->dSinceInit().count();
    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_ + elapsedTime_);
    }
    rebalancingTime_->stop();
}

void solver::staticSolver(PInstance &mainInst, InputPaths &inputPaths, std::string& instNum, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    simulationTime_->start();
//    preprocessTime_->start();
    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance StaticInst = std::make_shared<Instance>(*mainInst);
    StaticInst->buildStaticData(mainInst, nbReceivedRequest);


    masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
    for (auto &vehicleObj: mainInst->vehicles_){
        vehicleObj->currentRoute_->createColumn(StaticInst->nbRequests_);
    }
    if (StaticInst->parameters_->timeWindow_ == 0)
        StaticInst->updatePenalties(0);

//    preprocessTime_->stop();
    switch(StaticInst->parameters_->mainAlgorithm_) {
        case MIP_CPLEX :
            MIPModel_ = std::make_shared<MIPSolver>();
            MIPModel_->SolveMIP(StaticInst, inputPaths);
            MIPModel_.reset();
            break;
        case GREEDY:
            GreedyModel_->GreedySolver(StaticInst);
            std::cout << "# FINAL SOLUTION OF Greedy solution : " << GreedyModel_->objValue_<< std::endl;
            for (auto & vehicleObj : StaticInst->vehicles_)
                vehicleObj->solutionRoute_ = vehicleObj->currentRoute_;
            std::cout << "# TIME SPENT ON GREEDY " << "=" << GreedyModel_->greedyTime_->dSinceInit().count() << " (seconds)" << std::endl;
            break;
        default: // CG_CPLEX and CG_ISUD
            solveCG_Epoch(StaticInst, mainInst, inputPaths);
            break;
    }
    StaticInst->setAssignedEpochVehicles(simulationTime_->dSinceInit().count());
    simulationTime_->stop();
    if (!middleSave && StaticInst->parameters_->mainAlgorithm_ != GREEDY) {
        for (auto & vehicleObj : mainInst->vehicles_) {
            vehicleObj->departNode_->departTime_ = vehicleObj->currentRoute_->plannedDepartTime_[0];
            vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_ + saveTime);
        }
    }
}

void solver::dynamicSolver(PInstance &mainInst, InputPaths &inputPaths, bool middleSave, float saveTime) {
    // define required variables
    epoch_ = 0;
    rebalancingTime_->start();
    int instance_count = 1;

    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setSolutionRoute();
        if (mainInst->vehicles_[v]->currentRoute_ == nullptr)
            mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    int nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);
    while (nbReceivedRequest < mainInst->nbRequests_){
        /*while ((!solveEpoch && (nbReceivedRequest < mainInst->nbRequests_ || !masterModel_->zSolution_.empty())) ||
           (solveEpoch && nbReceivedRequest < mainInst->nbRequests_)) {*/
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
        boost::dynamic_bitset<> removedRequests;
        removedRequests.resize(EpochInst->nbRequests_);
        if (mainInst->parameters_->routeRecycle_) {
            if (removedRequests.count()) {
                for (auto &vehicleObj: mainInst->vehicles_) {
                    if (vehicleObj->stateChanged_)
                        masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
                }
                updateAvailableRoutes(removedRequests, masterModel_->availableRoutes_);
            }
        }
        else {
            masterModel_->availableRoutes_.clear();
            masterModel_->availableRoutes_.resize(mainInst->nbVehicles_);
        }
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateStateTime(mainInst, static_cast<float>(mainInst->simulationStartTime_) +
                                        static_cast<float>(epoch_) * mainInst->parameters_->epochLength_, removedRequests);
            mainInst->nbOnboards_ += static_cast<int>(vehicleObj->onboards_.size());
        }
        masterModel_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in the previous epoch

        EpochInst->buildPartialData(mainInst, masterModel_->zSolution_,
                                    static_cast<float>(epoch_) * mainInst->parameters_->epochLength_,
                                    nbReceivedRequest);
        for (auto &vehicleObj: mainInst->vehicles_){
            vehicleObj->currentRoute_->createColumn(EpochInst->nbRequests_);
            vehicleObj->emptyRoute_->createColumn(EpochInst->nbRequests_);
        }
        if (EpochInst->parameters_->timeWindow_ == 0)
            EpochInst->updatePenalties(mainInst->parameters_->epochLength_ * static_cast<float>(epoch_));
        if (epoch_ == 0 && mainInst->nbOnboards_ > 0)
            nbReceivedRequest += EpochInst->nbRequests_;
        else
            nbReceivedRequest += EpochInst->nbNewRequests_;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << EpochInst->nbRequests_ << std::endl;
        // saving the status in the middle of running
        if ((static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_ >= saveTime) && middleSave ) {
            inputPaths.makeInstanceOutput(std::to_string(instance_count));
            mainInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ +
                                             static_cast<float>(epoch_) * EpochInst->parameters_->epochLength_,mainInst->parameters_->epochLength_);

            saveTime += 60;
            /*if (instance_count >= 30)
                break;*/
            instance_count ++;


            //          break;
        }

        if (EpochInst->nbRequests_ == 0) {
            simulationTime_->stop();
            epoch_++;
            goto nextEpoch;
        }
        //       preprocessTime_->stop();

        switch (EpochInst->parameters_->mainAlgorithm_) {
            case GREEDY:
                GreedyModel_->GreedySolver(EpochInst);
                if (EpochInst->parameters_->vehicleReturn_) {
                    if (EpochInst->parameters_->returnPolicy_ == TO_SOURCE)
                        returnVehicles(EpochInst);
                    else if (EpochInst->parameters_->returnPolicy_ == ZONE)
                        returnVehiclesZone(EpochInst);
                    else {
                        returnVehiclesAlonso(mainInst);
                    }
                }
                break;
            case MIP_CPLEX:
                MIPModel_ = std::make_shared<MIPSolver>();
                MIPModel_->SolveMIP(EpochInst, inputPaths);
                masterModel_->zSolution_ = MIPModel_->zSolution_;
                masterModel_->routeSolution_ = MIPModel_->routeSolution_;
                MIPModel_.reset();
                break;
            default:
                solveCG_Epoch(EpochInst, mainInst, inputPaths);
                break;
        }

        if (EpochInst->parameters_->mainAlgorithm_ != GREEDY)
            *pLogRunTimesStream_ << saveRuntimes(EpochInst);
        //       (*pLogEpochSolutionStream_) << EpochInst->saveEpochRoutes( epoch_);
        EpochInst->setAssignedEpochVehicles(mainInst->simulationStartTime_
                                        + static_cast<float>(epoch_ + 1) * mainInst->parameters_->epochLength_);
        epoch_++;
        simulationTime_->stop();
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst->simulationStartTime_
                                           + static_cast<float>(epoch_+1) * mainInst->parameters_->epochLength_);
    }
    rebalancingTime_->stop();

}

// function to print epoch runTime to file
std::string solver::saveRuntimes(const PInstance &EpochInst) {
    std::stringstream repStr;
    epochRuntime_ = simulationTime_->dSinceStart().count();
    avgEpochRuntime_ = simulationTime_->dSinceInit().count() / static_cast<float>(epoch_ + 1);

    repStr << epoch_ << ",";
    repStr << EpochInst->nbRequests_ << ",";
    repStr << EpochInst->nbNewRequests_ << ",";
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
    repStr << nbPrunedArcs_ << ",";
    repStr << nbPrunedPath_ << ",";
    repStr << nbNegativeFound_ << ",";
    repStr << masterModel_->nbColumnsAdded_ << ",";
    repStr << nbRecycledColumns_ << ",";
    repStr << masterModel_->GreedyObjValue_ << ",";
    repStr << masterModel_->objValue_ << ",";
    repStr << masterModel_->lpObjValue_ << ",";
    repStr << masterModel_->totalWaitTime_ << ",";
    repStr << EpochInst->nbReturn_ << ",";
    repStr << EpochInst->nbIdle_ << ",";
    repStr << EpochInst->nbPotentialIdle_ << ",";
    repStr << EpochInst->nbStateChanged_ << ",";
    repStr << nbOnePick_ << ",";
    repStr << nbTwoPick_ << ",";
    repStr << nbThreePick_ << "\n";
    return repStr.str();
}

std::string solver::toString(const PInstance & mainInst) const {
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
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedyTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON ASSIGNMENT" << " = " << GreedyModel_->greedyAssignTime_->dSinceInit().count()/static_cast<float>(epoch_) << " (s)" << std::endl;
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

void solver::CreateOneStopRoutes(const PVehicle &vehicle, vector<PRoute> &availableRoutes, const PInstance &pInst,
                                 const PInstance &EpochInst, int &nbNegative) {
    for (auto & requestObj : EpochInst->requests_) {
        PRoute newRoute = std::make_shared<Route>(vehicle->vehicleID_);
        newRoute->addSource(vehicle->departNode_, vehicle->departTime_, vehicle->numPassengers_);

        if (!vehicle->onboards_.empty()) {
            for (auto & nodeId : vehicle->onboards_){
                newRoute->addNode(pInst->instGraph_->nodes_[nodeId]);
            }
        }
        newRoute->addNode(pInst->instGraph_->pickNodes_[requestObj->getRequestId()]);
        newRoute->addNode(pInst->instGraph_->dropNodes_[requestObj->getRequestId()]);
        newRoute->reducedCost_ = newRoute->totalDelay_ - requestObj->dual_ - vehicle->dual_;
        newRoute->score_ = newRoute->reducedCost_ / static_cast<float>(newRoute->routeRequests_.size());
        newRoute->lambda_ = newRoute->totalDelay_/(newRoute->totalDelay_ - newRoute->reducedCost_ + 0.001f);
        newRoute->totalLength_ = newRoute->plannedDepartTime_.back() - vehicle->departTime_;
        if (newRoute->reducedCost_ < 0)
            nbNegative++;
        availableRoutes.emplace_back(std::move(newRoute));
        availableRoutes.back()->createColumn(EpochInst->nbRequests_);
    }
}


void solver::updateAvailableRoutes(boost::dynamic_bitset<> &removedRequests, vector2D<PRoute> &availableRoutes) {
    for (auto &vehicleRoutes : availableRoutes) {
        // Remove routes with overlapping requests or empty route requests
        vehicleRoutes.erase(
                std::remove_if(vehicleRoutes.begin(), vehicleRoutes.end(),
                               [&removedRequests](const PRoute &routeObj) {
                                   return (routeObj->column_ & removedRequests).any() || routeObj->routeRequests_.empty();
                               }),
                vehicleRoutes.end()
        );
    }
}

void solver::returnVehicles(const PInstance & EpochInst) const {
    float lastEpoch = 0;
    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
        lastEpoch = EpochInst->simulationStartTime_ + elapsedTime_ - EpochInst->parameters_->committedTime_;
    else
        lastEpoch = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_ - EpochInst->parameters_->epochLength_;

    // Return Idle Vehicles
    if (EpochInst->parameters_->vehicleReturn_) {
        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                if (vehicleObj->currentRoute_->routeNodes_.back()->locationID_ != vehicleObj->sinkNode_->locationID_){
                    PNode sinkNode = std::make_shared<Node>(vehicleObj->sinkNode_);
                    vehicleObj->currentRoute_->addSink(sinkNode);
                    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                        vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_ + elapsedTime_+ simulationTime_->dSinceStart().count());
                }
            }
        }
    }
}


void solver::returnVehiclesZone(const PInstance & EpochInst) const {
    if (EpochInst->parameters_->vehicleReturn_) {
        float lastEpoch = 0;
        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            lastEpoch = EpochInst->simulationStartTime_ + elapsedTime_ - EpochInst->parameters_->WaitForReturn_;
        else
            lastEpoch = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_ - EpochInst->parameters_->WaitForReturn_;

        for (auto & zoneObj : EpochInst->zones_) {
            zoneObj.second->nbVehicles_ = 0;
        }

        for (auto &vehicleObj: EpochInst->vehicles_) {
            EpochInst->zones_[vehicleObj->currentRoute_->routeNodes_.back()->zoneID_]->nbVehicles_++;
        }
        for (auto &zoneObj : EpochInst->zones_) {
            if (zoneObj.second->nbVehicles_ < zoneObj.second->nbVehiclesRef_ * 0.95 && zoneObj.second->highDemandZone_) {
                zoneObj.second->underCapacity_ = true;
            }
            else
                zoneObj.second->underCapacity_ = false;
        }

        // Return Idle Vehicles
        std::vector<PVehicle> idleVehicles;

        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                if (!EpochInst->zones_[vehicleObj->departNode_->zoneID_]->underCapacity_ ) {
                    idleVehicles.push_back(vehicleObj);
                }
            }
        }

        if (!idleVehicles.empty()) {
            for (auto &vehicleObj : idleVehicles) {
                // Retrieve the current zone from the vehicle's last route node.
                int currentZoneID = vehicleObj->currentRoute_->routeNodes_.back()->zoneID_;
                PZone currentZone = EpochInst->zones_[currentZoneID];

                // Iterate over the current zone's successors (assumed to be ordered by proximity).
                for (Zone* successor : currentZone->successors_) {
                    // Check if the successor zone is under capacity (i.e., less than 90% of its reference).
                    if (successor->nbVehicles_ < successor->nbVehiclesRef_ * 0.95 && successor->highDemandZone_ &&
                        vehicleObj->sinkNode_->zoneID_ != successor->zoneID_) {
                        PNode sinkNode = std::make_shared<Node>(vehicleObj->sinkNode_);
                        sinkNode->zoneID_ = static_cast<int>(successor->zoneID_);
                        sinkNode->locationID_ = successor->centerLocationID_;
                        successor->nbVehicles_++; // Update vehicle count.
                        vehicleObj->currentRoute_->addSink(sinkNode);
                        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                            vehicleObj->updateCurrentRoute(EpochInst->simulationStartTime_
                                + elapsedTime_ + simulationTime_->dSinceStart().count());
                        break; // Assign to the nearest eligible successor.
                        }
                }
                // If no eligible successor is found, the vehicle remains unassigned.
            }
        }
    }
}


void solver::returnVehiclesAssign(const PInstance & EpochInst) const {
    if (!EpochInst->parameters_->vehicleReturn_) return;
    /* ---------- 1. reference time for “idle” test ---------- */

    float epochStartTime = 0;
    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
        epochStartTime = EpochInst->simulationStartTime_ + elapsedTime_;
    else
        epochStartTime = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_;
    float lastEpoch = epochStartTime - EpochInst->parameters_->WaitForReturn_;

    /* ---------- 2. collect idle vehicles ---------- */
    std::vector<PVehicle> idleVehicles;
    for (auto &vehicleObj: EpochInst->vehicles_) {
        if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
            vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch &&
            !EpochInst->zones_[vehicleObj->departNode_->zoneID_]->highDemandZone_) {
            idleVehicles.push_back(vehicleObj);
        }
    }
    if (idleVehicles.empty()) return;
    int totalNeed = 0;
    /* ---------- 3. compute zone weights ---------- */
    for (auto & zoneObj : EpochInst->zones_)
        zoneObj.second->nbUnserved_ = 0;

    for (auto &requestObj: EpochInst->requests_) {
        if (requestObj->committedPickTime_ == LARGE_CONSTANT)
            EpochInst->zones_[requestObj->pickZoneID_]->nbUnserved_++;
    }
    /* keep zones with positive demand */
    std::vector<int> zoneIDs;
    for (auto &zp : EpochInst->zones_) {
        if (zp.second->nbUnserved_ > 0) {
            zoneIDs.push_back(zp.first);
            totalNeed += zp.second->nbUnserved_;
            std::cout << "Zone: " << zp.first << " - " << zp.second->nbUnserved_ << std::endl;
        }
    }
    if (zoneIDs.empty()) return;

/* ---------- 4. build and solve assignment MIP ---------- */
    IloEnv   env;
    try {
        const std::size_t nV = idleVehicles.size();
        const std::size_t nZ = zoneIDs.size();

        IloModel model(env);
        IloArray<IloBoolVarArray> y(env, nV);   // y[v][z]

        for (std::size_t v = 0; v < nV; ++v) {
            y[v] = IloBoolVarArray(env, nZ);
            for (std::size_t z = 0; z < nZ; ++z)
                y[v][z] = IloBoolVar(env);
        }

        /* objective */
        IloExpr obj(env);
        for (std::size_t v = 0; v < nV; ++v) {
            int vehLoc = idleVehicles[v]->departNode_->locationID_;
            for (std::size_t z = 0; z < nZ; ++z) {
                float distance = durationMatrix_[vehLoc][EpochInst->zones_[zoneIDs[z]]->centerLocationID_];
                float cost = distance / EpochInst->zones_[zoneIDs[z]]->nbUnserved_;
                obj += cost * y[v][z];
            }
        }
        model.add(IloMinimize(env, obj)); obj.end();
        IloExpr total(env);
        /* constraints: (1) exactly one zone per vehicle */
        for (std::size_t v = 0; v < nV; ++v) {
            IloExpr sum(env);
            for (std::size_t z = 0; z < nZ; ++z) {
                sum += y[v][z];
                total += y[v][z];
            }
            model.add(sum <= 1); sum.end();
        }

        /* (2) capacity: do not oversupply a zone */
        for (std::size_t z = 0; z < nZ; ++z) {
            IloExpr cap(env);
            for (std::size_t v = 0; v < nV; ++v) cap += y[v][z];

            int capZ = EpochInst->zones_[zoneIDs[z]]->nbUnserved_;   // w_z
            model.add(cap <= capZ);
            cap.end();
        }

        /* (3) assign exactly min(|V|, total demand) vehicles */
        model.add(total == std::min(totalNeed, static_cast<int>(nV)));
        total.end();

        IloCplex cplex(model);
        cplex.setOut(env.getNullStream());
        cplex.setParam(IloCplex::Param::RootAlgorithm, 2);
        cplex.setParam(IloCplex::Param::Threads, EpochInst->parameters_->nbThreads_);
        cplex.solve();

        /* ---------- 5. write back assignments ---------- */
        for (std::size_t v = 0; v < nV; ++v)
            for (std::size_t z = 0; z < nZ; ++z)
                if (cplex.getValue(y[v][z]) > 0.5) {
  //                  std::cout << "Vehicle " << idleVehicles[v]->departNode_->zoneID_ << " to " << "Zone " << zoneIDs[z] << " with unserved: " << EpochInst->zones_[zoneIDs[z]]->nbUnserved_<< std::endl;

                    PNode sinkNode = std::make_shared<Node>(idleVehicles[v]->sinkNode_);
                    sinkNode->zoneID_ = zoneIDs[z];
                    sinkNode->locationID_ = EpochInst->zones_[zoneIDs[z]]->centerLocationID_;
                    idleVehicles[v]->currentRoute_->addSink(sinkNode);
                    if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                        idleVehicles[v]->updateCurrentRoute(EpochInst->simulationStartTime_
                            + elapsedTime_ + simulationTime_->dSinceStart().count());
                    break;
                }
    }
    catch (const IloException &e) {
        std::cerr << "CPLEX error in returnVehiclesAssign: "
                  << e.getMessage() << std::endl;
    }
    env.end();
}

void solver::returnVehiclesAlonso(const PInstance & EpochInst) const {
    rebalancingTime_->stop();
    if (!EpochInst->lastCommittedRequests_.empty()) {
        /* ---------- 1. reference time for “idle” test ---------- */

        float epochStartTime = 0;
        if (EpochInst->parameters_->solutionMode_ == ANYTIME)
            epochStartTime = EpochInst->simulationStartTime_ + elapsedTime_;
        else
            epochStartTime = EpochInst->simulationStartTime_ + static_cast<float> (epoch_) * EpochInst->parameters_->epochLength_;
        float lastEpoch = epochStartTime - EpochInst->parameters_->WaitForReturn_;

        /* ---------- 2. collect idle vehicles ---------- */
        std::vector<PVehicle> idleVehicles;
        for (auto &vehicleObj: EpochInst->vehicles_) {
            if (vehicleObj->currentRoute_->routeSize_ == 1 && vehicleObj->currentRoute_->plannedReachTime_[0]+
                vehicleObj->currentRoute_->routeNodes_.back()->serviceTime_ < lastEpoch) {
                idleVehicles.push_back(vehicleObj);
            }
        }
        if (!idleVehicles.empty()) {

            /* ---------- 4. build and solve assignment MIP ---------- */
            IloEnv   env;
            try {
                const std::size_t nV = idleVehicles.size();
                const std::size_t nR = EpochInst->lastCommittedRequests_.size();
                const int need = static_cast<int>(std::min(nV, nR));

                IloModel model(env);
                IloArray<IloBoolVarArray> y(env, nV);          // y[v][r]
                for (std::size_t v = 0; v < nV; ++v) {
                    y[v] = IloBoolVarArray(env, nR);
                    for (std::size_t r = 0; r < nR; ++r) y[v][r] = IloBoolVar(env);
                }

                /* objective */
                IloExpr obj(env);
                for (std::size_t v = 0; v < nV; ++v) {
                    int vehLoc = idleVehicles[v]->departNode_->locationID_;
                    for (std::size_t r = 0; r < nR; ++r) {
                        int reqLoc = EpochInst->lastCommittedRequests_[r]->PickUpID_;
                        float cost = durationMatrix_[vehLoc][reqLoc];
                        obj += cost * y[v][r];
                    }
                }
                model.add(IloMinimize(env, obj)); obj.end();
                IloExpr total(env);
                /* (1) at most one request per vehicle */
                for (std::size_t v = 0; v < nV; ++v) {
                    IloExpr sum(env);
                    for (std::size_t r = 0; r < nR; ++r) sum += y[v][r];
                    model.add(sum <= 1); sum.end();
                }

                /* (2) at most one vehicle per request */
                for (std::size_t r = 0; r < nR; ++r) {
                    IloExpr sum(env);
                    for (std::size_t v = 0; v < nV; ++v) sum += y[v][r];
                    model.add(sum <= 1); sum.end();
                }

                /* (3) use exactly min(|V_idle|, |R_late|) vehicles */
                IloExpr tot(env);
                for (std::size_t v = 0; v < nV; ++v)
                    for (std::size_t r = 0; r < nR; ++r) tot += y[v][r];
                model.add(tot == need); tot.end();

                IloCplex cplex(model);
                cplex.setOut(env.getNullStream());
                cplex.setParam(IloCplex::Param::RootAlgorithm, 2);
                cplex.setParam(IloCplex::Param::Threads, EpochInst->parameters_->nbThreads_);
                cplex.solve();

                /* ---------- 5. write back assignments ---------- */
                for (std::size_t v = 0; v < nV; ++v)
                    for (std::size_t r = 0; r < nR; ++r)
                        if (cplex.getValue(y[v][r]) > 0.5) {
                            std::cout << "Vehicle " << idleVehicles[v]->departNode_->zoneID_ << " to " << "Requests " << EpochInst->lastCommittedRequests_[r]->pickZoneID_ << std::endl;

                            PNode sinkNode = std::make_shared<Node>(idleVehicles[v]->sinkNode_);
                            sinkNode->zoneID_ = EpochInst->lastCommittedRequests_[r]->pickZoneID_;
                            sinkNode->locationID_ = EpochInst->lastCommittedRequests_[r]->PickUpID_;
                            idleVehicles[v]->currentRoute_->addSink(sinkNode);
                            if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                                idleVehicles[v]->updateCurrentRoute(EpochInst->simulationStartTime_
                                    + elapsedTime_ + simulationTime_->dSinceStart().count());
                            break;
                        }
            }
            catch (const IloException &e) {
                std::cerr << "CPLEX error in returnVehiclesAssign: "
                          << e.getMessage() << std::endl;
            }
            env.end();
        }
    }
    EpochInst->lastCommittedRequests_.clear();
    rebalancingTime_->start();
}
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
    preprocessBuildTime_ = new myTools::Timer(); preprocessBuildTime_->init();

    isudEpochTime_ = 0;
    RPEpochTime_ = 0;
    CPEpochTime_ = 0;
    isudMIPEpochTime_ = 0;
    avgEpochRuntime_ = 0;
    epochRuntime_ = 0;

    pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
    (*pLogRunTimesStream_) << "Epoch, nbRequests, nbNodes, EpochRuntime, AvgEpochRuntime, ElapsedTime, ISUD_Runtime, RP_Runtime, "
                              "CP_Runtime, MIPISUD_Runtime, SubProblemRuntime, PreProcessTime, PreProcessBuildTime, minSubSize, maxSubSize, avgSubSize" << std::endl;

    pLogEpochSolutionStream_ = new Tools::LogOutput(inputPaths.getOutputEpochFinal());
    (*pLogEpochSolutionStream_) << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID" << std::endl;

    pLogEpochSubproStream_ = new Tools::LogOutput(inputPaths.getOutputSubproSize());
    (*pLogEpochSubproStream_) << "Epoch, vehicleID, nbRequests, nbNodes, maxPick, runtime" << std::endl;
}

solver::~solver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;
    delete preprocessBuildTime_;
    pLogRunTimesStream_->close();
    pLogEpochSolutionStream_->close();
    pLogEpochSubproStream_->close();
    delete pLogRunTimesStream_;
    delete pLogEpochSolutionStream_;
    delete pLogEpochSubproStream_;
}

void solver::solveCG_ISUD(PInstance &EpochInst, InputPaths &inputPaths) {
    // define required variables
    double previousObj;
    int nbNegativeFound;
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);

    isudObj_->initialization(EpochInst);
    // save initial solution
//    EpochInst->saveISUDRoutes(inputPaths.getOutputEpochIsud(), epoch_, isudObj_->isudIter_);
    (*isudObj_->pLogIterSolutionStream_) << EpochInst->saveISUDRoutes(epoch_, isudObj_->isudIter_);
    isudObj_->isudIter_ ++;

    SubproEpochTime_ = 0;
    /*isudEpochTime_ = 0;
    RPEpochTime_ = 0;
    CPEpochTime_ = 0;
    isudMIPEpochTime_ = 0;*/

    while (true) {
        nbNegativeFound = 0;

        int maxPick = EpochInst->nbRequests_;
        float maxReachTime = MAXReachTime;
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
        std::vector<PLabelingSubPro> subProConst;
  //      if ((EpochInst->parameters_->greedyPortion_)&&(EpochInst->nbNewRequests_ > 0)){
        if (EpochInst->parameters_->greedyPortion_){
            GreedyModel_->GreedySolverFast(EpochInst);
       //     GreedyModel_->GreedySolver(EpochInst);
            for (auto &vehicleObj: EpochInst->vehicles_) {
 //               if (vehicleObj->currentRoute_->routeSize_ - 1 - vehicleObj->onboards_.size() > 0) {
                if (GreedyModel_->selectedVehicles_[vehicleObj->vehicleID_] > 0) {
                    subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
                    vehicleObj->selected_ = true;
                    if (GreedyModel_->selectedVehicles_[vehicleObj->vehicleID_] < 2)
                        subProSolve.back()->maxPickup_ = 2;
                    else
                        subProSolve.back()->maxPickup_ = GreedyModel_->selectedVehicles_[vehicleObj->vehicleID_];
                }
                else
                    subProConst.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
      //          vehicleObj->currentRoute_->resetRoute();
            }
            /*for (auto & routeObj : isudObj_->routeSolution_) {
                EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
            }*/
        }
        else {
            EpochInst->sortVehicles(SCORE);
            int portion = std::max((int)(EpochInst->parameters_->vehicle_portion_*EpochInst->nbVehicles_),EpochInst->nbNewRequests_);
     //       int portion = (int)(EpochInst->parameters_->vehicle_portion_*EpochInst->nbVehicles_);
            if (EpochInst->getNbUnselectedVehicles() < (0.75 * portion))
                EpochInst->resetVehicleSelection();
            for (int v = 0; v < EpochInst->vehicles_.size(); v++) {
                if ((subProSolve.size() < portion) && (!EpochInst->vehicles_[v]->selected_))
                    subProSolve.emplace_back(
                            std::make_shared<LabelingSubProblem>(EpochInst->vehicles_[v], subProOptions_));
                else
                    subProConst.emplace_back(std::make_shared<LabelingSubProblem>(EpochInst->vehicles_[v], subProOptions_));
            }
            EpochInst->restVehicleOrder();
        }
        //int vehicleMaxPick = std::max(maxPick, EpochInst->vehicles_[v]->capacity_);
        //   subProOptions->maxPickup_ = vehicleMaxPick;
        //subProOptions_->maxPickup_ = maxPick;
        /*if (EpochInst->parameters_->SubproSolveMode_ == NUM_PICK_RESTRICTED){
            if (subProSolve.empty())
                maxPick = 2;
            else {
   //             maxPick = (int)floor(EpochInst->nbRequests_ / subProSolve.size())+1;
                maxPick = 2;
            }
        }*/
        std::cout << "nb Requests: " << EpochInst->nbRequests_ << std::endl;
        std::cout << "nb new Requests: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "max pick: " << maxPick << std::endl;
        std::cout << "nb of sub problems: " << subProSolve.size() << std::endl;
        /*if (EpochInst->parameters_->SubproSolveMode_ == TIME_RESTRICTED)
            maxReachTime = static_cast<float>(4 * EpochInst->parameters_->epochLength_);

        subProOptions_->updateOptions(maxReachTime);*/
        // initializing and solving subproblems
        sort(subProSolve.begin(), subProSolve.end(),[](const PLabelingSubPro &lhs, const PLabelingSubPro &rhs){
            return lhs->maxPickup_ > rhs->maxPickup_;});
        for (auto &subProblem: subProSolve){
            Tools::Job job([&]() {
                subProblem->initSubGraph2(EpochInst);
                subProblem->solveDynamic();

            });
            pPool->run(job);
        }
        for (auto &subProblem: subProConst){
            Tools::Job job([&]() {
                subProblem->initSubGraph2(EpochInst);
                subProblem->reconstructLabels(isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_]);

            });
            pPool->run(job);
        }
        std::stringstream repStr;
        for (auto & subProblem : subProSolve) {
            if (maxSubSize_ < subProblem->subGraph_->nbNodes_)
                maxSubSize_ = subProblem->subGraph_->nbNodes_;
            if (minSubSize_ > subProblem->subGraph_->nbNodes_)
                minSubSize_ = subProblem->subGraph_->nbNodes_;
            avgSubSize_ += subProblem->subGraph_->nbNodes_;
            repStr << epoch_ << ",";
            repStr << (*subProblem->Vehicle_)->vehicleID_ << ",";
            repStr << subProblem->subRequests_.size() << ",";
            repStr << subProblem->subGraph_->nbNodes_-2 << ",";
            repStr << subProblem->maxPickup_ << ",";
            repStr << subProblem->subproTime_->dSinceInit().count() << "\n";
        }
        (*pLogEpochSubproStream_) << repStr.str();
        pPool->wait();
        if (!subProSolve.empty())
            avgSubSize_ = (int) avgSubSize_/subProSolve.size();
        for (auto &subProblem: subProSolve){
            isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].clear();
            subProblem->SolutionToRoutes((*subProblem->Vehicle_),
                                         isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_], EpochInst);
            isudObj_->nbRoutes_ += isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].size();
            nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;

        }
        for (auto &subProblem: subProConst){
            isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].clear();
            subProblem->SolutionToRoutes((*subProblem->Vehicle_),
                                         isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_], EpochInst);
            isudObj_->nbRoutes_ += isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].size();
            nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;

        }
        /*std::cout << "# ==============================================================" << std::endl;
        std::cout << "# TIME SPENT ON SOLVING SUBPROBLEMS =";
        std::cout << subProblemTime_->dSinceStart().count() << " (s)" << std::endl;
        std::cout << "# ==============================================================" << std::endl;
        std::cout << "#" << std::endl;*/
        subProblemTime_->stop();
        SubproEpochTime_ += subProblemTime_->dSinceStart().count();
        if (nbNegativeFound == 0) {
  //          std::cout << " *****************************  The Column Generation Terminated!  *****************************" << std::endl;
            break;
        }
        else {
            if (EpochInst->parameters_->mainAlgorithm_ == CG_CPLEX) {
                isudObj_->solveISUDMIP(EpochInst, inputPaths);

                /*isudEpochTime_ += isudObj_->isudTime_->dSinceStart().count();
                RPEpochTime_ += isudObj_->RPTime_->dSinceStart().count();
                CPEpochTime_ += isudObj_->CPTime_->dSinceStart().count();
                isudMIPEpochTime_ += isudObj_->isudMIPTime_->dSinceStart().count();*/

                break;
            }
            else if (EpochInst->parameters_->mainAlgorithm_ == CG_ISUD){
                isudObj_->solveISUD3(EpochInst, epoch_, inputPaths);
                break;
                /*isudEpochTime_ += isudObj_->isudTime_->dSinceStart().count();
                RPEpochTime_ += isudObj_->RPTime_->dSinceStart().count();
                CPEpochTime_ += isudObj_->CPTime_->dSinceStart().count();
                isudMIPEpochTime_ += isudObj_->isudMIPTime_->dSinceStart().count();*/
                if (EpochInst->parameters_->solutionMode_ == ANYTIME)
                    break;
            }
        }
        if (previousObj == isudObj_->objValue_) {
            /*std::cout
                    << " *****************************  The Column Generation Terminated!  *****************************"
                    << std::endl;*/
            break;
        }
    }  // end of CG while
    for (auto & routeObj : isudObj_->routeSolution_) {
        EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
//        std::cout << EpochInst->vehicles_[routeObj->vehicleID_]->currentRoute_->toString() << std::endl;
    }
    isudObj_->setObjValue();
//    std::cout << "# FINAL SOLUTION OF ISUD AFTER EPOCH " << epoch_ << " : " << std::endl;
//    std::cout << isudObj_->toString();

//    EpochInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch_);

    (*pLogEpochSolutionStream_) << EpochInst->saveEpochRoutes( epoch_);
}

void solver::dynamicSolver(PInstance &mainInst, InputPaths &inputPaths) {
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
        preprocessTime_->start();

        elapsedTime_ = simulationTime_->dSinceInit().count();
        std::cout << "*************************************************************************************"<< std::endl;
        std::cout << "                        ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << "                               EPOCH: " << epoch_ << std::endl;
        std::cout << "*************************************************************************************"<< std::endl;
        // update vehicle status
        mainInst->nbOnboards_ = 0;
        isudObj_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateState(epoch_, mainInst->parameters_->epochLength_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
 //           isudObj_->availableRoutes_[vehicleObj->vehicleID_].clear();
        }
        isudObj_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        preprocessBuildTime_->start();
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, isudObj_->zSolution_,
                                    static_cast<float>((epoch_) * mainInst->parameters_->epochLength_),
                                    nbReceivedRequest);
        preprocessBuildTime_->stop();
        EpochInst->updatePenalties(mainInst->parameters_->epochLength_ * epoch_);
        nbReceivedRequest += EpochInst->nbNewRequests_;
     //   std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;

        if (epoch_ == 0 && nbReceivedRequest == 0) {

            preprocessTime_->stop();
            simulationTime_->stop();
 //           saveRuntimes(EpochInst, inputPaths.getOutputEpochRunTime());
            (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
            epoch_++;
            goto nextEpoch;
        }
        preprocessTime_->stop();

        if (EpochInst->parameters_->mainAlgorithm_ == CG_ISUD || EpochInst->parameters_->mainAlgorithm_ == CG_CPLEX)
            solveCG_ISUD(EpochInst,inputPaths);
        else if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);
        simulationTime_->stop();
//        saveRuntimes(EpochInst, inputPaths.getOutputEpochRunTime());
        (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst);
    }

}
void solver::anyTimeSolver(PInstance &mainInst, InputPaths &inputPaths) {
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
        preprocessTime_->start();
        elapsedTime_ = simulationTime_->dSinceInit().count();
        std::cout << "*************************************************************************************"<< std::endl;
        std::cout << "                        ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << "                               EPOCH: " << epoch_ << std::endl;
        std::cout << "*************************************************************************************"<< std::endl;

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        isudObj_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateStateTime(elapsedTime_, mainInst->parameters_->committedTime_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
            isudObj_->availableRoutes_[vehicleObj->vehicleID_].clear();
        }
        isudObj_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();

        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, isudObj_->zSolution_ , elapsedTime_, nbReceivedRequest);
        EpochInst->updatePenalties(elapsedTime_);
        nbReceivedRequest += EpochInst->nbNewRequests_;
     //   std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;

        if (epoch_ == 0 && nbReceivedRequest == 0) {
            simulationTime_->stop();
            preprocessTime_->stop();
 //           saveRuntimes(EpochInst, inputPaths.getOutputEpochRunTime());
            (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
            epoch_++;
            goto nextEpoch;
        }
        preprocessTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ == CG_ISUD || EpochInst->parameters_->mainAlgorithm_ == CG_CPLEX)
            solveCG_ISUD(EpochInst,inputPaths);
        else if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);
        simulationTime_->stop();
 //       saveRuntimes(EpochInst, inputPaths.getOutputEpochRunTime());
        (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
        epoch_++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst);
    }

}

void solver::staticSolver(PInstance &mainInst, InputPaths &inputPaths, std::string instNum, bool middleSave, float saveTime) {
// define required variables
    epoch_ = 0;

    // start simulation timer
    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    // initialize vehicle routes at source
    for (auto & vehicleObj: mainInst->vehicles_) {
        vehicleObj->solutionRoute_ = std::make_shared<Route>(vehicleObj->vehicleID_);
        vehicleObj->solutionRoute_->addSource(vehicleObj->emptyRoute_->routeNodes_[0], vehicleObj->departTime_, vehicleObj->numPassengers_);
    }
    simulationTime_->start();
    preprocessTime_->start();
    PInstance StaticInst = std::make_shared<Instance>(*mainInst);
    StaticInst->buildStaticData(mainInst);
    StaticInst->updatePenalties(0);
    simulationTime_->stop();
    preprocessTime_->stop();
    switch(StaticInst->parameters_->mainAlgorithm_) {
        case MIP_CPLEX :
            simulationTime_->start();
            MIPSolver(StaticInst, inputPaths);
            std::cout << "# FINAL SOLUTION OF MIP solution : " << std::endl;
            for (int v = 0; v < StaticInst->nbVehicles_; ++v)
                std::cout << StaticInst->vehicles_[v]->currentRoute_->toString();
            simulationTime_->stop();
            break;
        case GREEDY:
            simulationTime_->start();
            GreedyModel_->GreedySolver(StaticInst);
            std::cout << "# FINAL SOLUTION OF Greedy solution : " << std::endl;
            for (auto & vehicleObj : StaticInst->vehicles_)
                std::cout << vehicleObj->currentRoute_->toString();
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
            std::cout << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY " << "=" << GreedyModel_->greedyTime_->dSinceInit().count() << " (seconds)" << std::endl;
            simulationTime_->stop();
            break;
        default: // CG_CPLEX and CG_ISUD
            if (StaticInst->parameters_->subAlgorithm_ == CPLEX) {
                simulationTime_->start();
                // define required variables
                double previousObj;
                int nbNegativeFound;

                isudObj_->initialization(StaticInst);
                // save initial solution
            //    StaticInst->saveISUDRoutes(inputPaths.getOutputEpochIsud(), epoch_, isudObj_->isudIter_);
                (*isudObj_->pLogIterSolutionStream_) << StaticInst->saveISUDRoutes(epoch_, isudObj_->isudIter_);
                isudObj_->isudIter_++;

                SubproEpochTime_ = 0;
                /*isudEpochTime_ = 0;
                RPEpochTime_ = 0;
                CPEpochTime_ = 0;
                isudMIPEpochTime_ = 0;*/

                while (true) {
                    nbNegativeFound = 0;

                    int maxPick = StaticInst->nbRequests_;
                    float maxReachTime = MAXReachTime;
                    previousObj = isudObj_->objValue_;

                    if (StaticInst->parameters_->SubproSolveMode_ == NUM_PICK_RESTRICTED)
                        maxPick = (int) floor(StaticInst->nbRequests_ / StaticInst->nbVehicles_) + 2;
                    if (StaticInst->parameters_->SubproSolveMode_ == TIME_RESTRICTED)
                        maxReachTime = static_cast<float>(4 * StaticInst->parameters_->epochLength_);

                    //***********************************************************************************//
                    //                    C P L E X   M E T H O D
                    //***********************************************************************************//
                    // start the subproblems timer
                    subProblemTime_->start();
                    // defining subproblems
                    isudObj_->nbRoutes_ = 0;
                    for (auto &vehicleObj: StaticInst->vehicles_) {
                        PCplexSubPro subProblem = std::make_shared<CPLEXSubProblem>(vehicleObj);
                        int vehicleMaxPick = std::max(maxPick, vehicleObj->capacity_);
                        subProblem->initSubGraph2(StaticInst);
                        subProblem->BuildModelCPLEX(vehicleMaxPick);
                        subProblem->SolveCPLEX();
                        std::cout << subProblem->toString();
                        isudObj_->availableRoutes_[vehicleObj->vehicleID_].clear();
                        subProblem->SolutionToRoutes(isudObj_->availableRoutes_[vehicleObj->vehicleID_]);
                        nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;
                    }

                    std::cout << "# ==============================================================" << std::endl;
                    std::cout << "# TIME SPENT ON SOLVING SUBPROBLEMS =";
                    std::cout << subProblemTime_->dSinceStart().count() << " (s)" << std::endl;
                    std::cout << "# ==============================================================" << std::endl;
                    std::cout << "#" << std::endl;
                    subProblemTime_->stop();
                    SubproEpochTime_ += subProblemTime_->dSinceStart().count();
                    if (nbNegativeFound == 0) {
                        /*std::cout
                                << " *****************************  The Column Generation Terminated!  *****************************"
                                << std::endl;*/
                        break;
                    } else {
                        if (StaticInst->parameters_->mainAlgorithm_ == CG_CPLEX) {
                            isudObj_->solveISUDMIP(StaticInst, inputPaths);

                            /*isudEpochTime_ += isudObj_->isudTime_->dSinceStart().count();
                            RPEpochTime_ += isudObj_->RPTime_->dSinceStart().count();
                            CPEpochTime_ += isudObj_->CPTime_->dSinceStart().count();
                            isudMIPEpochTime_ += isudObj_->isudMIPTime_->dSinceStart().count();*/

                            break;
                        } else if (StaticInst->parameters_->mainAlgorithm_ == CG_ISUD) {
                            isudObj_->solveISUD3(StaticInst, epoch_, inputPaths);

                            /*isudEpochTime_ += isudObj_->isudTime_->dSinceStart().count();
                            RPEpochTime_ += isudObj_->RPTime_->dSinceStart().count();
                            CPEpochTime_ += isudObj_->CPTime_->dSinceStart().count();
                            isudMIPEpochTime_ += isudObj_->isudMIPTime_->dSinceStart().count();*/
                        }
                    }
                    if (previousObj == isudObj_->objValue_) {
                        /*std::cout
                                << " *****************************  The Column Generation Terminated!  *****************************"
                                << std::endl;*/
                        break;
                    }
                }  // end of CG while
                for (auto &routeObj: isudObj_->routeSolution_) {
                    StaticInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                }
                isudObj_->setObjValue();
                std::cout << "# FINAL SOLUTION OF ISUD AFTER EPOCH " << epoch_ << " : " << std::endl;
                std::cout << isudObj_->toString();

 //               StaticInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch_);
                (*pLogEpochSolutionStream_) << StaticInst->saveEpochRoutes( epoch_);
                simulationTime_->stop();
            }
            else {
                simulationTime_->start();
                solveCG_ISUD(StaticInst,inputPaths);
                simulationTime_->stop();
            }

    }

    if (!middleSave && StaticInst->parameters_->mainAlgorithm_ != GREEDY) {
        for (auto & vehicleObj : mainInst->vehicles_) {
            vehicleObj->finalizeSolutionRoutes(mainInst);
        }
    }
//    StaticInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch_);
    (*pLogEpochSolutionStream_) << StaticInst->saveEpochRoutes( epoch_);
}

// function to print epoch runTime to file
void solver::saveRuntimes(PInstance & EpochInst, const string &EpochRunTimeDir) {
    epochRuntime_ = simulationTime_->dSinceStart().count();
    avgEpochRuntime_ = simulationTime_->dSinceInit().count() / (epoch_ + 1);

    std::ofstream myFile;
    myFile.open (EpochRunTimeDir, std::ofstream::app);
    myFile << epoch_ << ",";
    myFile << EpochInst->nbRequests_ << ",";
    myFile << EpochInst->instGraph_->nbNodes_ - 2 * EpochInst->nbVehicles_ << ",";
    myFile << epochRuntime_ << ",";
    myFile << avgEpochRuntime_ << ",";
    myFile << simulationTime_->dSinceInit().count() << ",";

    // isud Times
    myFile << isudObj_->isudTime_->dSinceInit().count() - isudEpochTime_ << ",";
    myFile << isudObj_->RPTime_->dSinceInit().count() - RPEpochTime_ << ",";
    myFile << isudObj_->CPTime_->dSinceInit().count() - CPEpochTime_ << ",";
    myFile << isudObj_->isudMIPTime_->dSinceInit().count() - isudMIPEpochTime_ << ",";

    isudEpochTime_ = isudObj_->isudTime_->dSinceInit().count();
    RPEpochTime_ = isudObj_->RPTime_->dSinceInit().count();
    CPEpochTime_ = isudObj_->CPTime_->dSinceInit().count();
    isudMIPEpochTime_ = isudObj_->isudMIPTime_->dSinceInit().count();

    myFile << SubproEpochTime_ << ",";
    myFile << preprocessTime_ ->dSinceStart().count()<< "\n";
    myFile.close();
}

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
    repStr << isudObj_->CPTime_->dSinceInit().count() - CPEpochTime_ << ",";
    repStr << isudObj_->isudMIPTime_->dSinceInit().count() - isudMIPEpochTime_ << ",";

    isudEpochTime_ = isudObj_->isudTime_->dSinceInit().count();
    RPEpochTime_ = isudObj_->RPTime_->dSinceInit().count();
    CPEpochTime_ = isudObj_->CPTime_->dSinceInit().count();
    isudMIPEpochTime_ = isudObj_->isudMIPTime_->dSinceInit().count();

    repStr << SubproEpochTime_ << ",";
    repStr << preprocessTime_ ->dSinceStart().count()<< ",";
    repStr << preprocessBuildTime_ ->dSinceStart().count()<< ",";
    repStr << minSubSize_ << ",";
    repStr << maxSubSize_ << ",";
    repStr << avgSubSize_<< "\n";
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
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedySolveTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel_->greedySolTime_->dSinceInit().count() << " (s)" << std::endl;
    repStr << isudObj_->toStringTimersAvg(epoch_);
    repStr << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProblemTime_->dSinceInit().count()/epoch_ << " (s)" << std::endl;
    return repStr.str();
}





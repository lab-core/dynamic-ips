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
    isudMIPEpochTime_ = 0;
    avgEpochRuntime_ = 0;
    epochRuntime_ = 0;
    GreedyTime_ = 0;
    AssignTime_ = 0;
    minSubSize_ = 0;
    maxSubSize_ = 0;
    avgSubSize_ = 0;


    // this Stream define runtime outputs
    pLogRunTimesStream_ = new Tools::LogOutput(inputPaths.getOutputEpochRunTime());
    (*pLogRunTimesStream_) << "Epoch, nbRequests, nbNodes, EpochRuntime, AvgEpochRuntime, ElapsedTime, ISUD_Runtime, RP_Runtime, "
                              "CP_Runtime, MIPISUD_Runtime, SubProblemRuntime, PreProcessTime, SubAssignTime, GreedyTime, minSubSize, maxSubSize, avgSubSize, GreedyObj, ISUD_Obj" << std::endl;

    pLogEpochSolutionStream_ = new Tools::LogOutput(inputPaths.getOutputEpochFinal());
    (*pLogEpochSolutionStream_) << "Epoch,VehicleID,NodeID,RequestTime,ReachTime,NodeType, LocationID" << std::endl;

    pLogEpochSubRuntimeStream_ = new Tools::LogOutput(inputPaths.getOutputSubproSize());
    (*pLogEpochSubRuntimeStream_) << "Epoch, vehicleID, nbRequests, nbNodes, maxPick, nbGenerated, nbEliminated, "
                                 "nbDominated, nbRoutes, solveTime, ConvertRouteTime" << std::endl;

    pLogEpochSubRouteStream_ = new Tools::LogOutput(inputPaths.getOutputSubproRouteTime());
    (*pLogEpochSubRouteStream_) << "Epoch, vehicleID, routeID, totalDelay, nbRequests, createTime, totalLength" << std::endl;
}

solver::~solver() {
    delete simulationTime_;
    delete subProblemTime_;
    delete preprocessTime_;
    pLogRunTimesStream_->close();
    pLogEpochSolutionStream_->close();
    pLogEpochSubRuntimeStream_->close();
    pLogEpochSubRouteStream_->close();
    delete pLogRunTimesStream_;
    delete pLogEpochSolutionStream_;
    delete pLogEpochSubRuntimeStream_;
    delete pLogEpochSubRouteStream_;
}

void solver::solveCG_ISUD(PInstance &EpochInst, PInstance & mainInst, InputPaths &inputPaths) {
    // define required variables
    double previousObj;
    int nbNegativeFound;
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(EpochInst->parameters_->nbThreads_);
    if (EpochInst->parameters_->initialStart_ == GREEDY_START)
        GreedyModel_->GreedySolver(EpochInst);
    isudObj_->initialization(EpochInst);
    // save initial solution
//    (*isudObj_->pLogIterSolutionStream_) << EpochInst->saveISUDRoutes(epoch_, isudObj_->isudIter_);
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
        std::vector<PLabelingSubPro> subProConst;

        /*if (std::floor(EpochInst->nbRequests_/50) == 1)
            subProOptions_->MaxLabel_ = 50;
        else if (std::floor(EpochInst->nbRequests_/50) == 2)
            subProOptions_->MaxLabel_ = 25;
        else if (std::floor(EpochInst->nbRequests_/50) == 3)
            subProOptions_->MaxLabel_ = 15;
        else if (std::floor(EpochInst->nbRequests_/50) > 3)
            subProOptions_->MaxLabel_ = 10;*/

        if (!subProOptions_->usePick_ && EpochInst->nbRequests_ >= 200)
            subProOptions_->usePick_ = true;

        if ((EpochInst->parameters_->greedyPortion_)&&(EpochInst->nbRequests_ >= 15)){
            GreedyModel_->GreedySolverFast(EpochInst);
            for (auto &vehicleObj: EpochInst->vehicles_) {
                if (GreedyModel_->selectedVehicles_[vehicleObj->vehicleID_] > 0) {
                    subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
                }
                else
                    isudObj_->availableRoutes_[vehicleObj->vehicleID_].clear();
//                    subProConst.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions_));
            }
        }
        else {
            if (EpochInst->parameters_->vehicle_portion_ < 1) {
                std::vector<PVehicle> vehicleList = EpochInst->vehicles_;
                std::stable_sort(vehicleList.begin(), vehicleList.end(),[](const PVehicle &lhs, const PVehicle &rhs){
                    return lhs->score_ < rhs->score_;});
   //             EpochInst->sortVehicles(SCORE);
                int portion = (int)(EpochInst->parameters_->vehicle_portion_*EpochInst->nbVehicles_);
                if (EpochInst->getNbUnselectedVehicles() < (0.75 * portion))
                    EpochInst->resetVehicleSelection();
                for (int v = 0; v < vehicleList.size(); v++) {
                    if ((subProSolve.size() < portion) && (!EpochInst->vehicles_[vehicleList[v]->vehicleID_]->selected_)
                    &&(EpochInst->nbRequests_ > 0))
                        subProSolve.emplace_back(
                                std::make_shared<LabelingSubProblem>(EpochInst->vehicles_[vehicleList[v]->vehicleID_], subProOptions_));
                    else if (!isudObj_->availableRoutes_[EpochInst->vehicles_[vehicleList[v]->vehicleID_]->vehicleID_].empty())
                        subProConst.emplace_back(std::make_shared<LabelingSubProblem>(EpochInst->vehicles_[vehicleList[v]->vehicleID_], subProOptions_));
                }
                vehicleList.clear();
            }
            else {
                for (int v = 0; v < EpochInst->vehicles_.size(); v++) {
                    subProSolve.emplace_back(
                            std::make_shared<LabelingSubProblem>(EpochInst->vehicles_[v], subProOptions_));
                }

            }
        }

        std::cout << "nb Requests: " << EpochInst->nbRequests_ << std::endl;
        std::cout << "nb new Requests: " << EpochInst->nbNewRequests_ << std::endl;
        std::cout << "nb of sub problems: " << subProSolve.size() << std::endl;

        // initializing and solving subproblems
        std::stable_sort(subProSolve.begin(), subProSolve.end(),[](const PLabelingSubPro &lhs, const PLabelingSubPro &rhs){
            return lhs->subGraph_->nbNodes_ > rhs->subGraph_->nbNodes_;});
        for (auto &subProblem: subProSolve){
            Tools::Job job([&]() {
                subProblem->initSubGraph2(EpochInst);
                if (!subProblem->subRequests_.empty())
                    subProblem->solveDynamic();

            });
            pPool->run(job);
        }
  //      pPool->wait();
        while(true){
            if (!pPool->wait())
                break;
        }
        for (auto &subProblem: subProConst){
            Tools::Job job([&]() {
                subProblem->initSubGraph2(EpochInst);
                subProblem->reconstructLabels(isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_]);

            });
            pPool->run(job);
        }
        while(true){
            if (!pPool->wait())
                break;
        }

        for (auto &subProblem: subProSolve){
            isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].clear();
            subProblem->SolutionToRoutes((*subProblem->Vehicle_),
                                         isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_], mainInst,
                                         EpochInst->nbRequests_);
            isudObj_->nbRoutes_ += isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].size();
            nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;

        }
        for (auto &subProblem: subProConst){
            isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].clear();
            subProblem->SolutionToRoutes((*subProblem->Vehicle_),
                                         isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_], mainInst,
                                         EpochInst->nbRequests_);
            isudObj_->nbRoutes_ += isudObj_->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].size();
            nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;

        }
        /*if (EpochInst->parameters_->initialStart_ != GREEDY_START)
            GreedyModel_->GreedySolver(EpochInst, isudObj_->availableRoutes_);*/
        std::stringstream repStr;
        for (auto & subProblem : subProSolve) {
            if (maxSubSize_ < subProblem->subGraph_->nbNodes_)
                maxSubSize_ = subProblem->subGraph_->nbNodes_;
            if (minSubSize_ > subProblem->subGraph_->nbNodes_)
                minSubSize_ = subProblem->subGraph_->nbNodes_;
            avgSubSize_ += subProblem->subGraph_->nbNodes_;
            repStr << subProblem->toStringOut(epoch_);
        }
        for (auto & subProblem : subProConst) {
            repStr << subProblem->toStringOut(epoch_);
        }
//        (*pLogEpochSubRuntimeStream_) << repStr.str();
        if (!subProSolve.empty())
            avgSubSize_ = (int) avgSubSize_/subProSolve.size();
        subProSolve.clear();
        subProblemTime_->stop();
        SubproEpochTime_ += subProblemTime_->dSinceStart().count();
        if (nbNegativeFound == 0) {
            break;
        }
        else {
            if (EpochInst->parameters_->mainAlgorithm_ == CG_CPLEX) {
                isudObj_->solveISUDMIP(EpochInst, inputPaths);
                break;
            }
            else if (EpochInst->parameters_->mainAlgorithm_ == CG_ISUD){
                isudObj_->solveISUD(EpochInst, epoch_, inputPaths);
                if ((EpochInst->parameters_->solutionMode_ == ANYTIME)||(mainInst->parameters_->oneIter_))
                    break;
            }
        }
        if (previousObj == isudObj_->objValue_) {
            break;
        }
    }  // end of CG while
    for (auto & routeObj : isudObj_->routeSolution_) {
        EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }
 //   (*pLogEpochSubRouteStream_) << EpochInst->saveRoutesTimes( epoch_);
    isudObj_->setObjValue();
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
        std::cout << "                      PRE EPOCH TIME: " << epochRuntime_ << std::endl;
        std::cout << "                               EPOCH: " << epoch_ << std::endl;
        std::cout << "*************************************************************************************"<< std::endl;

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
     //   std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;


        /*if ((epochRuntime_ > 150)||(EpochInst->nbRequests_ >= 400))
            break;*/
        if (EpochInst->nbRequests_ == 0) {
            simulationTime_->stop();
            preprocessTime_->stop();
 //           (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
            epoch_++;
            goto nextEpoch;
        }
        preprocessTime_->stop();
        if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);
        else if (EpochInst->parameters_->mainAlgorithm_ == CG_ISUD || EpochInst->parameters_->mainAlgorithm_ == CG_CPLEX)
            solveCG_ISUD(EpochInst, mainInst, inputPaths);

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
            if (StaticInst->parameters_->subAlgorithm_ == CPLEX) {
                simulationTime_->start();
                // define required variables
                double previousObj;
                int nbNegativeFound;

                isudObj_->initialization(StaticInst);
                // save initial solution
   //             (*isudObj_->pLogIterSolutionStream_) << StaticInst->saveISUDRoutes(epoch_, isudObj_->isudIter_);
                isudObj_->isudIter_++;
                SubproEpochTime_ = 0;

                while (true) {
                    nbNegativeFound = 0;

                    int maxPick = StaticInst->nbRequests_;
                    previousObj = isudObj_->objValue_;

                    if (StaticInst->parameters_->SubproSolveMode_ == NUM_PICK_RESTRICTED)
                        maxPick = (int) floor(StaticInst->nbRequests_ / StaticInst->nbVehicles_) + 2;
                    /*if (StaticInst->parameters_->SubproSolveMode_ == TIME_RESTRICTED)
                        maxReachTime = static_cast<float>(4 * StaticInst->parameters_->epochLength_);*/

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

                            break;
                        } else if (StaticInst->parameters_->mainAlgorithm_ == CG_ISUD) {
                            isudObj_->solveISUD(StaticInst, epoch_, inputPaths);
                        }
                    }
                    if (previousObj == isudObj_->objValue_) {
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
                solveCG_ISUD(StaticInst, mainInst, inputPaths);
                simulationTime_->stop();
            }

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
    int nbReceivedRequest;
    epoch_ = 0;


    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);

    while (nbReceivedRequest < mainInst->nbRequests_) {
        nextEpoch:
        // start simulation timer
        simulationTime_->start();
        preprocessTime_->start();

        elapsedTime_ = simulationTime_->dSinceInit().count();
        std::cout << "*************************************************************************************"<< std::endl;
        std::cout << "                        ELAPSED TIME: " << elapsedTime_ << std::endl;
        std::cout << "                               EPOCH: " << epoch_ << std::endl;
        std::cout << "*************************************************************************************"<< std::endl;
        // update vehicle status
        mainInst->nbOnboards_ = 0;
        if (epoch_ > 20)
            break;
        isudObj_->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto &vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateState(epoch_, mainInst->parameters_->epochLength_);
            mainInst->nbOnboards_ += (int) vehicleObj->onboards_.size();
            //           isudObj_->availableRoutes_[vehicleObj->vehicleID_].clear();
        }
        isudObj_->nbRoutes_ = 0;

        // resetting a subInstance
        EpochInst->resetInstance();
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, isudObj_->zSolution_,
                                    static_cast<float>((epoch_) * mainInst->parameters_->epochLength_),
                                    nbReceivedRequest);
        EpochInst->updatePenalties(mainInst->parameters_->epochLength_ * epoch_);
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
            preprocessTime_->stop();
            simulationTime_->stop();
            epoch_++;
            goto nextEpoch;
        }
        preprocessTime_->stop();

        if (EpochInst->parameters_->mainAlgorithm_ == CG_ISUD || EpochInst->parameters_->mainAlgorithm_ == CG_CPLEX)
            solveCG_ISUD(EpochInst, mainInst, inputPaths);
        else if (EpochInst->parameters_->mainAlgorithm_ == GREEDY)
            GreedyModel_->GreedySolver(EpochInst);
        simulationTime_->stop();
        (*pLogRunTimesStream_) << saveRuntimes(EpochInst);
        (*pLogEpochSolutionStream_) << EpochInst->saveEpochRoutes( epoch_);
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
    repStr << isudObj_->CPTime_->dSinceInit().count() - CPEpochTime_ << ",";
    repStr << isudObj_->isudMIPTime_->dSinceInit().count() - isudMIPEpochTime_ << ",";

    isudEpochTime_ = isudObj_->isudTime_->dSinceInit().count();
    RPEpochTime_ = isudObj_->RPTime_->dSinceInit().count();
    CPEpochTime_ = isudObj_->CPTime_->dSinceInit().count();
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
    repStr << isudObj_->objValue_ << "\n";
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
    mainInst->instRepStr_ << epoch_ << "," << isudObj_->isudTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << isudObj_->RPTime_->dSinceInit().count() << "," << isudObj_->CPTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << isudObj_->isudMIPTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << subProblemTime_->dSinceInit().count() << "," << GreedyModel_->greedyTime_->dSinceInit().count() << ",";
    mainInst->instRepStr_ << GreedyModel_->greedyAssignTime_->dSinceInit().count() << ",";
    return repStr.str();
}





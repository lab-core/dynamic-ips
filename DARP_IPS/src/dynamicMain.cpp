//
// Created by Ella on 2021-11-28.
//

#include "utilities/ReadWrite.h"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/CPLEXSubProblem.h"
#include "solvers/LabelingSubProblem.h"
#include "data/Instance.h"
#include "solvers/MIPSolver.h"


using namespace std::chrono;
vector2D<float> durationMatrix_;

int main() {

    // definition of the solution Parameters
    double previousObj = 0;
    SubProSolveStart subStartStatus;
    int nbReceivedRequest = 0;
    int epoch = 0;
    float saveTime = 550;
    bool middleSave = true;
    bool showLog = true;

    Tools::Timer *subProTime = new Tools::Timer(); subProTime->init();

    std::string dataDir = "datasets/";
    std::string instanceName = "20150713_07-62m";

    // build the path of input files
    // create output files for epoch results
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance and parameters in output path
    std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
    PInstance mainInst = ReadWrite::createMainInstance(inputPaths);
    std::cout << std::endl;
    std::cout << mainInst->toString();
//    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, 2 * mainInst->nbRequests_ + 1);
    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, 1605);
    if (!showLog)
        freopen (inputPaths.getOutputSolutionLog().c_str(),"w",stdout);

    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>();
    while (nbReceivedRequest < mainInst->nbRequests_) {
        subStartStatus = mainInst->parameters_->SubproSolveStartState_;
        std::cout << " *****************************  epoch " << std::setw(3) << epoch << "  *****************************" << std::endl;

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        for (auto & vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateState(epoch, mainInst->parameters_->epochLength_);
            mainInst->nbOnboards_ += vehicleObj->onboards_.size();
            isudObj->availableRoutes_[vehicleObj->vehicleID_].clear();
        }

        // creating a subInstance
        PInstance EpochInst = std::make_shared<Instance>(*mainInst);
        // reading the data received in previous epoch
        EpochInst->buildPartialData(mainInst, isudObj->zSolution_ , epoch, nbReceivedRequest);
        nbReceivedRequest += EpochInst->nbNewRequests_;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;

        // saving the status in the middle of running
        if ((epoch*EpochInst->parameters_->epochLength_ >= saveTime) && middleSave ) {
            EpochInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + epoch * EpochInst->parameters_->epochLength_);
            break;
        }

        switch(EpochInst->parameters_->mainAlgorithm_) {
            case MIP_CPLEX :
                MIPSolver(EpochInst, inputPaths);
                std::cout << "# FINAL SOLUTION OF MIP solution AFTER EPOCH " << epoch << " : " << std::endl;
                for (int v = 0; v < EpochInst->nbVehicles_; ++v)
                    std::cout << EpochInst->vehicles_[v]->currentRoute_->toString();
                break;
            case GREEDY:
                break;
            default: // CG_CPLEX and CG_ISUD (Column generation approaches)
                isudObj->initialization(EpochInst, EpochInst->parameters_->emptyStart_);
                // save initial solution
                EpochInst->saveISUDRoutes(inputPaths.getOutputEpochIsud(), epoch, isudObj->isudIter_);
                isudObj->isudIter_ ++;

                std::cout << "# VEHICLE ROUTES AFTER INITIALIZATION: " << std::endl;
                for (auto & vehicleObj : EpochInst->vehicles_) {
                    std::cout << vehicleObj->currentRoute_->toString();
                }
                while (true) {
                    int nbNegativeNotFound = 0;
                    int maxPick = EpochInst->nbRequests_;
                    float maxReachTime = MAXReachTime;
                    previousObj = isudObj->objValue_;
                    // sort the list of vehicles
                    sort(EpochInst->vehicles_.begin(), EpochInst->vehicles_.end(),
                         [](const PVehicle &lhs, const PVehicle &rhs) {
                             return std::tie(lhs->currentRoute_->routeSize_, lhs->departTime_) <
                                    std::tie(rhs->currentRoute_->routeSize_, rhs->departTime_);
                         });
                    // start the time
                    subProTime->start();
                    EpochInst->resetRequestsSelectStatus();
                    if (subStartStatus != NOT_RESTRICTED) {
                        if (subStartStatus == NUM_PICK_RESTRICTED)
                            maxPick = floor(EpochInst->nbRequests_ / EpochInst->nbVehicles_) + 2;
                        if (subStartStatus == TIME_RESTRICTED)
                            maxReachTime = 4 * EpochInst->parameters_->epochLength_;
                    }

                    switch (EpochInst->parameters_->subAlgorithm_) {
                        //*************************************************************//
                        //                    C P L E X   M E T H O D
                        //*************************************************************//
                        case CPLEX:
                            for (auto &vehicleObj: EpochInst->vehicles_) {
                                EpochInst->resetRequestsSelectStatus();
                                PCPLEXsubPro subProblem = std::make_shared<CPLEXSubProblem>(vehicleObj);
                                subProblem->initSubGraph(EpochInst);
                                subProblem->BuildModelCPLEX(isudObj->ReducedPro_->requestToOrder_, maxPick);
                                subProblem->SolveCPLEX();
                                std::cout << subProblem->toString();
                                isudObj->availableRoutes_[vehicleObj->vehicleID_].clear();
                                if (subProblem->bestReducedCost_ >= -0.0001) {
                                    nbNegativeNotFound ++;
                                }
                                else
                                    subProblem->SolutionToRoutes(isudObj->availableRoutes_[vehicleObj->vehicleID_], isudObj->generatedRoutes_);
                            }

                            //*************************************************************//
                            //         L A B E L S E T T IN G    M E T H O D
                            //*************************************************************//
                        case LABEL_SETTING:
                            for (auto &vehicleObj: EpochInst->vehicles_) {
                                EpochInst->resetRequestsSelectStatus();
                                PSolverOption subProOptions = std::make_shared<solverOption>(maxReachTime, maxPick,
                                                                                             EpochInst->parameters_);
                                PLabelingSubPro subProblem = std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions);
                                subProblem->initSubGraph(EpochInst);
                                subProblem->solveDynamic(epoch);
                                isudObj->availableRoutes_[vehicleObj->vehicleID_].clear();
                                if (subProblem->bestReducedCost_ > 0)
                                    nbNegativeNotFound++;
                                else
                                    subProblem->SolutionToRoutes(vehicleObj, isudObj->availableRoutes_[vehicleObj->vehicleID_],
                                                                 isudObj->generatedRoutes_);
                            }
//                    subStartStatus = NOT_RESTRICTED;
                    }

                    std::cout << "# ============================================================" << std::endl;
                    std::cout << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUBPROBLEMS " << "=";
                    std::cout << subProTime->dSinceStart().count() << " (seconds)" << std::endl;
                    std::cout << "# ============================================================" << std::endl;
                    std::cout << "#" << std::endl;
                    subProTime->stop();
                    EpochInst->restVehicleOrder();

                    if (nbNegativeNotFound == EpochInst->nbVehicles_) {
                        std::cout << " *****************************  The Column Generation Terminated!  *****************************" << std::endl;
                        break;
                    }
                    else {
                        if (mainInst->parameters_->mainAlgorithm_ == CG_CPLEX) {
                            isudObj->solveISUDMIP(EpochInst, epoch, inputPaths.getOutputEpochIsud());
                            break;
                        }
                        else if (mainInst->parameters_->mainAlgorithm_ == CG_ISUD){
                            isudObj->solveISUD(EpochInst, epoch, inputPaths.getOutputEpochIsud());
                            std::cout << "# SOLUTION ROUTES AFTER SOLVING ISUD FOR EPOCH " << epoch << ":" << std::endl;
                            for (auto &routeObj: isudObj->routeSolution_)
                                std::cout << routeObj->toString();
                        }
                    }
                    if (previousObj == isudObj->objValue_) {
                        std::cout << " *****************************  The Column Generation Terminated!  *****************************" << std::endl;
                        break;
                    }
                }  // end of CG while
                for (auto & routeObj : isudObj->routeSolution_) {
                    EpochInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
                }
                isudObj->setObjValue();
                std::cout << "# FINAL SOLUTION OF ISUD AFTER EPOCH " << epoch << " : " << std::endl;
                std::cout << isudObj->toString();
                break;
        }

        EpochInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch);
        epoch++;
    }

    for (auto & vehicleObj : mainInst->vehicles_) {
        if (vehicleObj->solutionRoute_->routeNodes_.back()->type_ == SOURCE) {
            for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                vehicleObj->currentRoute_->routeNodes_[i]->reachTime_ = vehicleObj->currentRoute_->plannedReachTime_[i];
                vehicleObj->solutionRoute_->addNode(vehicleObj->currentRoute_->routeNodes_[i]);

                if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->pickTime_ =
                            vehicleObj->currentRoute_->plannedReachTime_[i];
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->vehicleID_ = vehicleObj->vehicleID_;
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
                }
                else if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == DROPOFF) {
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->dropTime_ =
                            vehicleObj->currentRoute_->plannedReachTime_[i];
                    vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
                }
            }
        }
        else
            vehicleObj->solutionRoute_->addNode(mainInst->instGraph_->nodes_[vehicleObj->sinkID_]);

    }
    std::cout << std::endl << std::endl;
    if (!showLog)
        fclose (stdout);
    freopen (inputPaths.getOutputFinalLog().c_str(),"w",stdout);


    std::cout << "*************************************************************************************" << std::endl;
    std::cout << "*********************** FINAL VEHICLE ROUTES AFTER " << std::setw(3) << epoch << " EPOCHS *********************** " << std::endl;
    std::cout << "*************************************************************************************" << std::endl;
    std::cout << std::endl << std::endl;

    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        std::cout << mainInst->vehicles_[v]->solutionRoute_->toString();
    }
    std::cout << "#" << std::endl;
    std::cout << mainInst->solutionToString();
    std::cout << std::left << std::fixed << std::setprecision(2);
    std::cout << std::setw(sentenceSize) << "# TORAL TIME SPENT ON ISUD IMPROVEMENT" << " = " << isudObj->isudTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# TOTAL TIME SPENT ON SOLVING SUBPROBLEMS" << " = " << subProTime->dSinceInit().count() << " (s)" << std::endl;


    // save vehicle solutions in csv file
    mainInst->saveSolutionRoutes(inputPaths.getOutputFinalRoutes());
    mainInst->saveRequestsResults(inputPaths.getOutputFinalRequests());
    // save the final route solution
    mainInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch);

    fclose (stdout);
}

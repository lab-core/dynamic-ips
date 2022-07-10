//
// Created by Ella on 2022-05-05.
//


#include "utilities/ReadWrite.h"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/CPLEXSubProblem.h"
#include "solvers/LabelingSubProblem.h"
#include "data/Instance.h"
#include "solvers/MIPSolver.h"
#include "solvers/GreedyModeler.h"


using namespace std::chrono;
vector2D<float> durationMatrix_;

int main() {

    // definition of the solution Parameters
    double previousObj;
    SubProSolveStart subStartStatus;
    int epoch = 0;
    float saveTime = 3600;
    bool middleSave = false;
    bool showLog = true;

    auto *subProTime = new Tools::Timer(); subProTime->init();

    std::string dataDir = "datasets/";
    std::string instanceName = "20160622_11-240m-4";

    // build the path of input files
    // create output files for epoch results
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance and parameters in output path
    std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
    PInstance mainInst = ReadWrite::createMainInstance(inputPaths);
    std::cout << std::endl;
    std::cout << mainInst->toString();
    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, 2 * mainInst->nbLocations_ + 1);
    if (!showLog)
        freopen (inputPaths.getOutputSolutionLog().c_str(),"w",stdout);

    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }
    subStartStatus = mainInst->parameters_->SubproSolveStartState_;

    // initialize vehicle routes at source
    for (auto & vehicleObj: mainInst->vehicles_) {
        vehicleObj->solutionRoute_ = std::make_shared<Route>(vehicleObj->vehicleID_);
        vehicleObj->solutionRoute_->addSource(vehicleObj->emptyRoute_->routeNodes_[0], vehicleObj->departTime_, vehicleObj->numPassengers_);
 //       vehicleObj->solutionRoute_->routeNodes_.back()->reachTime_ = vehicleObj->departTime_;
    }

    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>();
    PGreedyModeler GreedyModel = std::make_shared<GreedyModeler>();
    // creating a solvable Instance
    PInstance StaticInst = std::make_shared<Instance>(*mainInst);
    StaticInst->buildStaticData(mainInst);

    switch(StaticInst->parameters_->mainAlgorithm_) {
        case MIP_CPLEX :
            MIPSolver(StaticInst, inputPaths);
            std::cout << "# FINAL SOLUTION OF MIP solution : " << std::endl;
            for (int v = 0; v < StaticInst->nbVehicles_; ++v)
                std::cout << StaticInst->vehicles_[v]->currentRoute_->toString();
            break;
        case GREEDY:
            GreedyModel->initialization(StaticInst);
//            GreedyModel->solve(StaticInst);
            GreedyModel->solveInsertion(StaticInst);
            GreedyModel->solutionToRoute(StaticInst);
            std::cout << "# FINAL SOLUTION OF Greedy solution : " << std::endl;
            for (auto & vehicleObj : StaticInst->vehicles_)
                std::cout << vehicleObj->currentRoute_->toString();
            if (middleSave) {
                for (auto & vehicleObj: StaticInst->vehicles_)
                    vehicleObj->updateStateTime(saveTime);
                StaticInst->saveStatus(inputPaths, StaticInst->simulationStartTime_ + saveTime);
            }
            else {
                for (auto & vehicleObj : StaticInst->vehicles_)
                    vehicleObj->solutionRoute_ = vehicleObj->currentRoute_;
            }
            break;
        default: // CG_CPLEX and CG_ISUD (Column generation approaches)

            isudObj->initialization(StaticInst, StaticInst->parameters_->emptyStart_);
            // save initial solution
            StaticInst->saveISUDRoutes(inputPaths.getOutputEpochIsud(), epoch, isudObj->isudIter_);
            isudObj->isudIter_ ++;

            std::cout << "# VEHICLE ROUTES AFTER INITIALIZATION: " << std::endl;
            for (auto & vehicleObj : StaticInst->vehicles_) {
                std::cout << vehicleObj->currentRoute_->toString();
            }
            while (true) {
                int nbNegativeNotFound = 0;
                int maxPick = StaticInst->nbRequests_;
                float maxReachTime = MAXReachTime;
                previousObj = isudObj->objValue_;
                // sort the list of vehicles
                sort(StaticInst->vehicles_.begin(), StaticInst->vehicles_.end(),
                     [](const PVehicle &lhs, const PVehicle &rhs) {
                         return std::tie(lhs->currentRoute_->routeSize_, lhs->departTime_) <
                                std::tie(rhs->currentRoute_->routeSize_, rhs->departTime_);
                     });
                // start the time
                subProTime->start();
                StaticInst->resetRequestsSelectStatus();
                if (subStartStatus != NOT_RESTRICTED) {
                    if (subStartStatus == NUM_PICK_RESTRICTED)
                        maxPick = floor(StaticInst->nbRequests_ / StaticInst->nbVehicles_ );
                    if (subStartStatus == TIME_RESTRICTED)
                        maxReachTime = static_cast<float> (4 * StaticInst->parameters_->epochLength_);
                }

                switch (StaticInst->parameters_->subAlgorithm_) {
                    //*************************************************************//
                    //                    C P L E X   M E T H O D
                    //*************************************************************//
                    case CPLEX:
                        for (auto &vehicleObj: StaticInst->vehicles_) {
                            StaticInst->resetRequestsSelectStatus();
                            PCplexSubPro subProblem = std::make_shared<CPLEXSubProblem>(vehicleObj);
                            subProblem->initSubGraph(StaticInst);
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
                        for (auto &vehicleObj: StaticInst->vehicles_) {
                            StaticInst->resetRequestsSelectStatus();
                            PSolverOption subProOptions = std::make_shared<solverOption>(maxReachTime, maxPick,
                                                                                         StaticInst->parameters_);
                            if (isudObj->isudIter_> 1)
                                subProOptions->disableHeuristics();
                            PLabelingSubPro subProblem = std::make_shared<LabelingSubProblem>(vehicleObj, subProOptions);
                            subProblem->initSubGraph(StaticInst);
                            subProblem->solveDynamic();
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
                StaticInst->restVehicleOrder();

                if (nbNegativeNotFound == StaticInst->nbVehicles_) {
                    std::cout << " *****************************  The Column Generation Terminated!  *****************************" << std::endl;
                    break;
                }
                else {
                    if (StaticInst->parameters_->mainAlgorithm_ == CG_CPLEX) {
                        isudObj->solveISUDMIP(StaticInst, inputPaths.getOutputEpochIsud());
 //                       break;
                    }
                    else if (StaticInst->parameters_->mainAlgorithm_ == CG_ISUD){
                        isudObj->solveISUD(StaticInst, epoch, inputPaths.getOutputEpochIsud());
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
                StaticInst->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
            }
            isudObj->setObjValue();
            std::cout << "# FINAL SOLUTION OF ISUD AFTER EPOCH " << epoch << " : " << std::endl;
            std::cout << isudObj->toString();
            break;
    }

    StaticInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch);


    if (!middleSave && StaticInst->parameters_->mainAlgorithm_ != GREEDY) {
        for (auto &vehicleObj: StaticInst->vehicles_) {
            if (vehicleObj->solutionRoute_->routeNodes_.back()->type_ == SOURCE) {
                for (int i = 1; i < vehicleObj->currentRoute_->routeSize_; ++i) {
                    vehicleObj->currentRoute_->routeNodes_[i]->nodeStatus_ = DONE;
                    vehicleObj->currentRoute_->routeNodes_[i]->reachTime_ = vehicleObj->currentRoute_->plannedReachTime_[i];
                    vehicleObj->currentRoute_->routeNodes_[i]->departTime_ = vehicleObj->currentRoute_->plannedReachTime_[i];
                    vehicleObj->solutionRoute_->addNode(vehicleObj->currentRoute_->routeNodes_[i],
                                                        vehicleObj->currentRoute_->plannedReachTime_[i]);

                    if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == PICKUP) {
                        vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->pickTime_ =
                                vehicleObj->currentRoute_->plannedReachTime_[i];
                        vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->vehicleID_ = vehicleObj->vehicleID_;
                        vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
                    } else if (vehicleObj->currentRoute_->routeNodes_[i]->type_ == DROPOFF) {
                        vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->dropTime_ =
                                vehicleObj->currentRoute_->plannedReachTime_[i];
                        vehicleObj->currentRoute_->routeNodes_[i]->related_Request_->requestStatus_ = COMPLETED;
                    }
                }
            } else
                vehicleObj->solutionRoute_->addNode(StaticInst->instGraph_->nodes_[vehicleObj->sinkID_]);

        }
    }
    // testing the solution route
    for(auto  &vehicleObj : mainInst->vehicles_)
        vehicleObj->solutionRoute_->testRoute(vehicleObj, StaticInst->parameters_->mainAlgorithm_ );

    std::cout << std::endl << std::endl;

    if (!showLog)
        fclose (stdout);
    freopen (inputPaths.getOutputFinalLog().c_str(),"w",stdout);


    std::cout << "*************************************************************************************" << std::endl;
    std::cout << "*********************** FINAL VEHICLE ROUTES AFTER " << std::setw(3) << epoch << " EPOCHS *********************** " << std::endl;
    std::cout << "*************************************************************************************" << std::endl;
    std::cout << std::endl << std::endl;

    for (auto & vehicleObj : mainInst->vehicles_) {
        std::cout << "#" << std::left << std::endl;
        std::cout << "#\t" << std::setw(25) << "- IDLE_TIME" << " : " << vehicleObj->idleTime_ << std::endl;
        std::cout << vehicleObj->solutionRoute_->toString();
    }
    std::cout << "#" << std::endl;
    try {
        std::cout << mainInst->solutionToString();
    }
    catch (std::exception e) {
        std::cout << e.what() << std::endl;
    }

    std::cout << std::left << std::fixed << std::setprecision(2);
    std::cout << "#" << std::endl;
    std::cout << std::setw(sentenceSize) << "# TOTAL TIME SPENT ON ISUD IMPROVEMENT" << " = " << isudObj->isudTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# TOTAL TIME SPENT ON RP IMPROVEMENT" << " = " << isudObj->RPTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# TOTAL TIME SPENT ON CP IMPROVEMENT" << " = " << isudObj->CPTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# TOTAL TIME SPENT ON ZOOM IMPROVEMENT" << " = " << isudObj->ZOOMTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# TOTAL TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProTime->dSinceInit().count() << " (s)" << std::endl;

    // save vehicle solutions in csv file
    mainInst->saveSolutionRoutes(inputPaths.getOutputFinalRoutes());
    mainInst->saveRequestsResults(inputPaths.getOutputFinalRequests());
    // save the final route solution
    mainInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch);

    fclose (stdout);
}

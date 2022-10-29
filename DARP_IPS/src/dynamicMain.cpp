//
// Created by Ella on 2021-11-28.
//

#include "utilities/ReadWrite.h"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/CPLEXSubProblem.h"
#include "solvers/LabelingSubProblem.h"
#include "data/Instance.h"
#include "solvers/MIPSolver.h"
#include "solvers/GreedyModeler.h"
#include "utilities/Tools.h"



using namespace std::chrono;
vector2D<float> durationMatrix_;

int main() {

    // definition of the solution Parameters
    double previousObj;
    SubProSolveMode subStartStatus;
    int nbReceivedRequest;
    int epoch = 0;
    float saveTime = 3600;
    bool middleSave = false;
    bool showLog = true;
 //   bool disabledHeuristics;

    float elapsedTime = 0;
    float avgLengthOut;
    float lastLengthOut;

    auto *simulationTime = new myTools::Timer(); simulationTime->init();
    auto *subProTime = new myTools::Timer(); subProTime->init();


    std::string dataDir = "datasets/";
    std::string instanceName = "20160222_17-120m_3";


    // build the path of input files
    // create output files for epoch results
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance and parameters in output path
    std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
    PInstance mainInst = ReadWrite::createMainInstance(inputPaths);
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(mainInst->parameters_->nbThreads_);
    std::cout << mainInst->toString();

    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, mainInst->nbLocations_);
    if (!showLog)
        freopen (inputPaths.getOutputSolutionLog().c_str(),"w",stdout);



    mainInst->setInitialTimes();
    for (int v = 0; v < mainInst->nbVehicles_; ++v) {
        mainInst->vehicles_[v]->setEmptyRoute(mainInst);
        mainInst->vehicles_[v]->setCurrentRoute(mainInst->vehicles_[v]->emptyRoute_);
    }

    nbReceivedRequest = mainInst->nbOnboards_;
    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>();
    PGreedyModeler GreedyModel = std::make_shared<GreedyModeler>();
    PInstance EpochInst = std::make_shared<Instance>(*mainInst);
    PSolverOption subProOptions = std::make_shared<solverOption>(mainInst->parameters_);

    simulationTime->start();
    while (nbReceivedRequest < mainInst->nbRequests_) {
        label:
        subStartStatus = mainInst->parameters_->SubproSolveMode_;

        elapsedTime = simulationTime->dSinceInit().count();
        std::cout << "*************************************************************************************" << std::endl;
        std::cout << "                        ELAPSED TIME: " << elapsedTime << std::endl;
        std::cout << "                               EPOCH: " << epoch << std::endl;
        std::cout << "           LAST RE-OPTIMIZATION TIME: " << lastLengthOut << std::endl;
        std::cout << "        AVERAGE RE-OPTIMIZATION TIME: " << avgLengthOut << std::endl;
        std::cout << "*************************************************************************************" << std::endl;

 //       isudObj->restGeneratedRoutes(mainInst);

        // update vehicle status
        mainInst->nbOnboards_ = 0;
        isudObj->availableRoutes_.resize(mainInst->nbVehicles_);
        for (auto & vehicleObj: mainInst->vehicles_) {
            vehicleObj->updateState(epoch, mainInst->parameters_->epochLength_);
            mainInst->nbOnboards_ += (int)vehicleObj->onboards_.size();
            isudObj->availableRoutes_[vehicleObj->vehicleID_].clear();
        }
        isudObj->nbRoutes_ = 0;

        // creating a subInstance
//        PInstance EpochInst = std::make_shared<Instance>(*mainInst);


        // reading the data received in previous epoch
        EpochInst->resetInstance();
        EpochInst->buildPartialData(mainInst, isudObj->zSolution_,
                                    static_cast<float>((epoch) * mainInst->parameters_->epochLength_),
                                    nbReceivedRequest);
        EpochInst->updatePenalties(mainInst->parameters_->epochLength_ * epoch);
        nbReceivedRequest += EpochInst->nbNewRequests_;
        std::cout << "# TOTAL NUMBER OF RECEIVED REQUESTS: " << nbReceivedRequest << std::endl;

        // saving the status in the middle of running
        if ((static_cast<float> (epoch*EpochInst->parameters_->epochLength_) >= saveTime) && middleSave ) {
            EpochInst->saveStatus(inputPaths, EpochInst->simulationStartTime_ + static_cast<float>(epoch * EpochInst->parameters_->epochLength_));
            break;
        }
        if (epoch == 0 && nbReceivedRequest == 0) {
            epoch++;
            goto label;
        }

        switch(EpochInst->parameters_->mainAlgorithm_) {
            case MIP_CPLEX :
                MIPSolver(EpochInst, inputPaths);
                break;
            case GREEDY:
                GreedyModel->GreedySolver(EpochInst);
                break;
            default: // CG_CPLEX and CG_ISUD (Column generation approaches)
                isudObj->initialization(EpochInst);
                // save initial solution
                EpochInst->saveISUDRoutes(inputPaths.getOutputEpochIsud(), epoch, isudObj->isudIter_);
                isudObj->isudIter_ ++;
/*                if (!EpochInst->parameters_->isSuccessorsLimited_ && !EpochInst->parameters_->isTruncated_)
                    disabledHeuristics = true;
                else
                    disabledHeuristics = false;*/
                while (true) {
                    int nbNegativeFound = 0;

                    int maxPick = EpochInst->nbRequests_;
                    float maxReachTime = MAXReachTime;
                    previousObj = isudObj->objValue_;
                    // start the time
                    subProTime->start();
                    if (subStartStatus != NOT_RESTRICTED) {
                        if (subStartStatus == NUM_PICK_RESTRICTED)
                            maxPick = (int)floor(EpochInst->nbRequests_ / EpochInst->nbVehicles_)+2;
                        if (subStartStatus == TIME_RESTRICTED)
                            maxReachTime = static_cast<float>(4 * EpochInst->parameters_->epochLength_);
                    }

                    subProOptions->updateOptions(maxReachTime, maxPick);
                    /*if (disabledHeuristics)
                        subProOptions->disableHeuristics();*/

                    switch (EpochInst->parameters_->subAlgorithm_) {
                        //*************************************************************//
                        //                    C P L E X   M E T H O D
                        //*************************************************************//
                        case CPLEX:
                            for (auto &vehicleObj: EpochInst->vehicles_) {
                                PCplexSubPro subProblem = std::make_shared<CPLEXSubProblem>(vehicleObj);
                                int vehicleMaxPick = std::max(maxPick, vehicleObj->capacity_);
                                subProblem->initSubGraph2(EpochInst);
                                subProblem->BuildModelCPLEX(vehicleMaxPick);
                                subProblem->SolveCPLEX();
                                std::cout << subProblem->toString();
                                isudObj->availableRoutes_[vehicleObj->vehicleID_].clear();
                                subProblem->SolutionToRoutes(isudObj->availableRoutes_[vehicleObj->vehicleID_]);
                                nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;
                            }
                            break;
                            //*************************************************************//
                            //         L A B E L S E T T IN G    M E T H O D
                            //*************************************************************//
                        case LABEL_SETTING:
                            // defining subproblems
                            isudObj->nbRoutes_ = 0;
                            EpochInst->updateTaskIndexLabeling();
                            std::vector<PLabelingSubPro> subProblems;
                            for (auto &vehicleObj: EpochInst->vehicles_) {
                                int vehicleMaxPick = std::max(maxPick, vehicleObj->capacity_);
                                //                       subProOptions->maxPickup_ = vehicleMaxPick;
                                subProOptions->maxPickup_ = maxPick;
                                subProblems.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj,
                                                                                              subProOptions));
                                isudObj->availableRoutes_[(*subProblems.back()->Vehicle_)->vehicleID_].clear();
                            }
                            // initializing and solving subproblems
                            for (auto &subProblem: subProblems){
                                Tools::Job job([&]() {
                                    subProblem->initSubGraph2(EpochInst);
                                    subProblem->solveDynamic();

                                });
                                pPool->run(job);
                            }
                            pPool->wait();
                            for (auto &subProblem: subProblems){
                                subProblem->SolutionToRoutes((*subProblem->Vehicle_),
                                                             isudObj->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_], EpochInst);
                                isudObj->nbRoutes_ += isudObj->availableRoutes_[(*subProblem->Vehicle_)->vehicleID_].size();
                                nbNegativeFound = nbNegativeFound + subProblem->nbNegativeColumns_;

                            }
                            break;
                    }
                    std::cout << "# ============================================================" << std::endl;
                    std::cout << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUBPROBLEMS " << "=";
                    std::cout << subProTime->dSinceStart().count() << " (seconds)" << std::endl;
                    std::cout << "# ============================================================" << std::endl;
                    std::cout << "#" << std::endl;
                    subProTime->stop();
                    if (nbNegativeFound == 0) {
                        std::cout << " *****************************  The Column Generation Terminated!  *****************************" << std::endl;
                        break;
                    }
                    else {
                        if (mainInst->parameters_->mainAlgorithm_ == CG_CPLEX) {
                            isudObj->solveISUDMIP(EpochInst, inputPaths.getOutputEpochIsud());
                            break;
                        }
                        else if (mainInst->parameters_->mainAlgorithm_ == CG_ISUD){
                            isudObj->solveISUD3(EpochInst, epoch, inputPaths.getOutputEpochIsud(),
                                               inputPaths.getOutputIncDegreeRdCost());
                            std::cout << "# SOLUTION ROUTES AFTER SOLVING ISUD FOR EPOCH " << epoch << ":" << std::endl;
 //                           break;
                        }
                    }
                    if (previousObj == isudObj->objValue_) {
                        std::cout
                                << " *****************************  The Column Generation Terminated!  *****************************"
                                << std::endl;
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

        // print epoch runTime to file
        lastLengthOut = simulationTime->dSinceInit().count() - elapsedTime;
        avgLengthOut = simulationTime->dSinceInit().count() / epoch;
        std::ofstream myFile;
        myFile.open (inputPaths.getOutputEpochRunTime(), std::ofstream::app);
        myFile << epoch << ",";
        myFile << lastLengthOut << ",";
        myFile << avgLengthOut << ",";
        myFile << EpochInst->nbRequests_ << ",";
        myFile << EpochInst->instGraph_->nbNodes_ - 2*EpochInst->nbVehicles_ << "\n";
        myFile.close();

    }

    simulationTime->stop();
    for (auto & vehicleObj : mainInst->vehicles_) {
        vehicleObj->finalizeSolutionRoutes(mainInst);
    }

    // testing the solution route
    for(auto  &vehicleObj : mainInst->vehicles_)
        vehicleObj->solutionRoute_->testRoute(vehicleObj, mainInst->parameters_->mainAlgorithm_ );

    std::cout << std::endl << std::endl;
    if (!showLog)
        fclose (stdout);
    freopen (inputPaths.getOutputFinalLog().c_str(),"w",stdout);


    std::cout << "*************************************************************************************" << std::endl;
    std::cout << "                        FINAL VEHICLE ROUTES AFTER " << std::setw(3) << epoch << " EPOCHS " << std::endl;
    std::cout << "                              " <<  solutionModeName[mainInst->parameters_->solutionMode_] << std::endl;
    std::cout << "*************************************************************************************" << std::endl;
    std::cout << std::endl << std::endl;
    std::cout << std::left << std::fixed << std::setprecision(2);
    std::cout << std::setw(30) << "# NUMBER OF VEHICLES" << " = " << mainInst->nbVehicles_ << std::endl;
    std::cout << "# " << std::endl;
    std::cout << "############################   PARAMETERS AND OPTIONS   ############################" << std::endl;
    std::cout << mainInst->parameters_->toString();
    std::cout << "# " << std::endl;
    std::cout << std::endl << std::endl;
    std::cout << "################################   FINAL ROUTES   ################################" << std::endl;
    std::cout << "#" << std::left << std::endl;
    for (auto & vehicleObj : mainInst->vehicles_) {
        std::cout << "#" << std::left << std::endl;
        std::cout << "#\t" << std::setw(24) << "- IDLE_TIME" << " : " << vehicleObj->idleTime_ << std::endl;
        std::cout << vehicleObj->solutionRoute_->toString();
    }
    std::cout << std::endl << std::endl;
    std::cout << "#################################   REQUESTS    #################################" << std::endl;
    std::cout << "#" << std::endl;
    std::cout << mainInst->solutionToString();
    std::cout << std::left << std::fixed << std::setprecision(2);
    std::cout << "#" << std::endl;
    std::cout << isudObj->toStringTimersTotal();
    std::cout << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProTime->dSinceInit().count() << " (s)" << std::endl;
    std::cout << std::setw(sentenceSize) << "# TIME SPENT ON GREEDY" << " = " << GreedyModel->greedyTime_->dSinceInit().count() << " (s)" << std::endl;
    std::cout << isudObj->toStringTimersAvg(epoch);
    std::cout << std::setw(sentenceSize) << "# TIME SPENT ON SOLVING SUB PROBLEMS" << " = " << subProTime->dSinceInit().count()/epoch << " (s)" << std::endl;



    // save vehicle solutions in csv file
    mainInst->saveSolutionRoutes(inputPaths.getOutputFinalRoutes());
    mainInst->saveRequestsResults(inputPaths.getOutputFinalRequests());
    // save the final route solution
    mainInst->saveEpochRoutes(inputPaths.getOutputEpochFinal(), epoch);

    fclose (stdout);
    isudObj.reset();
    std::cout << "END";
}

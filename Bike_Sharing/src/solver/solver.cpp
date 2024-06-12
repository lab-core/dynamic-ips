//
// Created by Elahe Amiri on 2022-10-26.
//

#include "solver.h"

solver::solver() {
    mainInstance_ = std::make_shared<Instance>();
    simulationTime_ = new myTools::Timer(); simulationTime_->init();
}

solver::~solver() {
    delete simulationTime_;
}

void solver::createInstance(const std::string& jsonStrDuration, const std::string& jsonStrParam,
                                    const std::string& jsonStrTasks, const std::string& jsonStrVehicles) {

    ReadWrite::readDurations_py(jsonStrDuration, mainInstance_->getDurationMatrix());
    ReadWrite::readParameters_py(jsonStrParam, mainInstance_);
    ReadWrite::readTasks_py(jsonStrTasks, mainInstance_);
    ReadWrite::readVehicles_py(jsonStrVehicles, mainInstance_);

    std::cout << "Instance created!!!";


}

void solver::createInstanceFile(const std::string& strDurFile, const std::string& strParamFile,
                                        const std::string& strTaskFile, const std::string& strVehicleFile) {

    ReadWrite::readDurations(strDurFile, mainInstance_->getDurationMatrix());
    ReadWrite::readParameters(strParamFile, mainInstance_);
    ReadWrite::readTasks(strTaskFile, mainInstance_);
    ReadWrite::readVehicles(strVehicleFile, mainInstance_);
}

std::string solver::solveCG() {
    // define required variables
    simulationTime_->start();
    myTools::SharedVector<PLabel> labelsPool_;
    labelsPool_.defineSize(mainInstance_->parameters_->nbThreads_);
    std::shared_ptr<MasterAlgorithm> masterModel_ = std::make_shared<MasterAlgorithm>();
    double previousObj;
    int nbNegativeFound;
    int iter = 0;
    std::stringstream changeStr;
    Tools::PThreadsPool pPool = Tools::ThreadsPool::newThreadsPool(mainInstance_->parameters_->nbThreads_);
//    masterModel_->availableTime_ = (int)(EpochInst->parameters_->epochLength_);
    masterModel_->availableTime_ = LARGE_CONSTANT;

    masterModel_->initialization(mainInstance_);
    masterModel_->availableRoutes_.resize(mainInstance_->nbVehicles_);
    for (auto &vehicleObj: mainInstance_->vehicles_) {
        vehicleObj->setEmptyRoute();
        masterModel_->availableRoutes_[vehicleObj->vehicleID_].clear();
    }

    masterModel_->RMPCounter_ ++;

    masterModel_->lpObjValue_ = masterModel_->objValue_;

    masterModel_->nbRoutes_ = 0;
    while (true) {
        iter++;
        nbNegativeFound = 0;
        previousObj = masterModel_->objValue_;


        //***********************************************************************************//
        //                    L A B E L  S E T T I N G    M E T H O D
        //***********************************************************************************//

        // start the subproblems timer
        // defining subproblems

        std::vector<PLabelingSubPro> subProSolve;


        // create subproblems

        for (auto &vehicleObj: mainInstance_->vehicles_) {
            subProSolve.emplace_back(std::make_shared<LabelingSubProblem>(vehicleObj, mainInstance_->subProOptions_,
                                                                          mainInstance_->getDurationMatrix()));
        }


        // initializing and solving subproblems
        /*std::stable_sort(subProSolve.begin(), subProSolve.end(),[](const PLabelingSubPro &lhs, const PLabelingSubPro &rhs){
            return lhs->subGraph_->nbNodes_ > rhs->subGraph_->nbNodes_;});*/
        masterModel_->SPIter_++;
        for (auto &subProblem: subProSolve){
            Tools::Job job([&]() {
                subProblem->initSubGraph(mainInstance_);
                subProblem->labelPool_ = std::move(labelsPool_.pop_data());
                subProblem->solveDynamic();
                subProblem->SolutionToRoutes(mainInstance_->vehicles_[subProblem->Vehicle_->vehicleID_],
                                             masterModel_->availableRoutes_[(subProblem->Vehicle_)->vehicleID_],
                                             mainInstance_);
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
        }

        while(true){
            if (!pPool->wait())
                break;
        }

        subProSolve.clear();
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
            masterModel_->solveMP_CG(mainInstance_);

            // if the size of epoch is larger than 30s disable the following
            if (simulationTime_->dSinceStart().count()/iter > (mainInstance_->parameters_->epochLength_ - simulationTime_->dSinceStart().count()))
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
        mainInstance_->vehicles_[routeObj->vehicleID_]->setCurrentRoute(routeObj);
    }

    masterModel_->setObjValue();
    simulationTime_->stop();
    std::cout << "Objective: " << masterModel_->objValue_ << std::endl;
    std::cout << "simulation time: " << simulationTime_->dSinceInit().count();

    // Convert the solution to a JSON string
    rapidjson::Value solutionJson = mainInstance_->createSolutionJson();

    // Print the JSON string to the console
    rapidjson::StringBuffer buffer;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(buffer);
    solutionJson.Accept(writer);

    return buffer.GetString();
}






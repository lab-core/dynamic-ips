//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"
#include "solvers/solver.h"



using namespace std::chrono;
float saveTime = 3600;
bool middleSave = false;
bool savePartial = false ;
std::string instNum = "1";
int numEpochTests = 30;
int numVehicles;
int saveScratch = 0;
bool solveEpoch = true;

int main(int argc, char** argv) {
    std::ios_base::sync_with_stdio(false);
    std::string dataDir = "datasets/";
    std::string vehicleFile = "";
    std::string vehicleFolder = "";
    std::string paramFile = "";
    int nbLocations = 1718;

    std::vector<std::string> instNames;         // vector of instance file names
    std::string instFolder;                     // folder of instances
    std::cout << "Number of arguments = " << argc << std::endl;
    int mainAlgo = -1;
    int solMode = -1;
    if (argc == 8){
        std::string instanceNames = "datasets/InstanceNames.txt";
        ReadWrite::readInstNames(instanceNames, instNames , 24, "_07-120m");
        std::cout << "24 Instance read!! " << std::endl;
        vehicleFolder = argv[1];
        instFolder = argv[2];
        std::string word = argv[3];
        vehicleFile = "vehicles_" + word + "_4";
        numVehicles = std::stoi(argv[3]);
        mainAlgo = std::stoi(argv[4]);
        solMode = std::stoi(argv[5]);
        paramFile = argv[6];
        saveScratch = std::stoi(argv[7]);
    }
    else if (argc == 9){
        vehicleFolder = argv[1];
        instFolder = argv[2];
        instNames.emplace_back(argv[3]);
        std::cout << "Instance : " << argv[2] << "/" << argv[3]  << std::endl;
        std::string word = argv[4];
        vehicleFile = "vehicles_" + word + "_4";
        numVehicles = std::stoi(argv[4]);
        mainAlgo = std::stoi(argv[5]);
        solMode = std::stoi(argv[6]);
        paramFile = argv[7];
        saveScratch = std::stoi(argv[8]);
    }
    else
        myTools::myException::throwError("There should be at least 6 arguments!");

    // build the path of input files
    // create output files for epoch results
    InputPaths inputPaths(dataDir, vehicleFile, vehicleFolder, paramFile);
    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, nbLocations);
    std::string instName = instNames[0];
    int num_i = 1;
    int num_j = 1;

    if (paramFile == "truncate") {
        num_i = 7;
        num_j = 3;
    }

    else if (paramFile == "pruning") {
        num_i = 3;
    }
    else if (paramFile == "dropPick") {
        num_i = 1;
    }
    else if (paramFile == "Dual") {
        num_i = 3;
    }
    else if (paramFile == "Partial") {
        num_i = 2;
    }

    for (auto & instanceName : instNames){
        for (int i = 0; i < num_i; ++i) {
            for (int j = 0; j < num_j; ++j){
                std::this_thread::sleep_for(std::chrono::seconds(2));
                // create output files for epoch results
                inputPaths.initializeInputs(instFolder, instanceName);

                // Read data files and initialize instance and parameters in output path
                std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
                Request::requestCount_ = 0;
                PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
                if (!solveEpoch)
                    mainInst->nbVehicles_ = numVehicles;
                ReadWrite::readParameters(inputPaths.getInputParamFile(), mainInst);
                mainInst->parameters_->saveScratch_ = saveScratch;

                if (paramFile == "truncate") {
                    mainInst->parameters_->MaxLabel_ = (i+1)*5;
                    mainInst->parameters_->sortPath_ = static_cast<SortPaths>(j);
                }
                else if (paramFile == "pruning") {
                    if (i == 0) {
                        mainInst->parameters_->pruneNodes_ = true;
                    }
                    else if (i == 1) {
                        mainInst->parameters_->pruneNodes_ = true;
                        mainInst->parameters_->pruneArcs_ = true;
                    }
                    else if (i == 2){
                        mainInst->parameters_->pruneNodes_ = true;
                        mainInst->parameters_->pruneArcs_ = true;
                        mainInst->parameters_->discardSuboptimalPath_ = true;
                    }
                }
                else if (paramFile == "dropPick") {
                    if (i == 1)
                        mainInst->parameters_->isDropPickPossible_ = true;
                }
                else if (paramFile == "Dual") {
                    if (i == 0)
                        mainInst->parameters_->initialDual_ = LMP;
                    else if (i == 1)
                        mainInst->parameters_->initialDual_ = AUX_P;
                    else if (i == 2)
                        mainInst->parameters_->initialDual_ = LP_CP;
                }
                else if (paramFile == "Partial") {
                    if (i == 0)
                        mainInst->parameters_->dynamicPricing_ = true;
                    else
                        mainInst->parameters_->partialPricing_ = true;
                }

                ReadWrite::readZones(inputPaths.getInputZones(), mainInst);
                mainInst->parameters_->savePartial_ = savePartial;
                mainInst->parameters_->mainAlgorithm_ = static_cast<MainAlgorithm>(mainAlgo);
                mainInst->parameters_->solutionMode_ = static_cast<SolutionMode>(solMode);
                ReadWrite::readDatafiles(inputPaths, mainInst, mainInst->parameters_->saveScratch_, paramFile);
                std::cout << mainInst->toString();

                // create solver
                std::shared_ptr<solver> instanceSolver = std::make_shared<solver>(mainInst, inputPaths);
                if (mainInst->parameters_->solutionMode_ == DYNAMIC) {
                    try {
                        instanceSolver->dynamicSolver(mainInst, inputPaths, middleSave, saveTime);
                    } catch (const std::exception &e) {
                        std::cout << "DYNAMIC solving caught an exception=: "
                                  << e.what() << std::endl;
                    }
                } else if (mainInst->parameters_->solutionMode_ == ANYTIME) {
                    try {
                        /*if (mainInst->parameters_->mainAlgorithm_ == GREEDY)
                            instanceSolver->anyTimeSolverEvent(mainInst, inputPaths);
                        else*/
                        instanceSolver->anyTimeSolver(mainInst, inputPaths, instNum, middleSave, saveTime);
                    } catch (const std::exception &e) {
                        std::cout << "ANY_TIME solving caught an exception=: "
                                  << e.what() << std::endl;
                    }
                } else {
                    try {
                        instanceSolver->staticSolver(mainInst, inputPaths, instNum, middleSave, saveTime);
                    } catch (const std::exception &e) {
                        std::cout << "STATIC solving caught an exception=: "
                                  << e.what() << std::endl;
                    }
                }
                // testing the solution route
                for (auto &vehicleObj: mainInst->vehicles_)
                    vehicleObj->solutionRoute_->testRoute(vehicleObj);
                if (!middleSave) {
                    if (mainInst->parameters_->timeWindow_ > 0){
                        for (auto &reqestObj: mainInst->requests_){
                            if (reqestObj->requestStatus_ != COMPLETED) {
                                reqestObj->requestStatus_ = REJECTED;
                            }
                        }
                    }

                    std::cout << std::endl << std::endl;

                    // print final solution to txt file
                    Tools::LogOutput finalStream(inputPaths.getOutputFinalLog());
                    finalStream << instanceSolver->toString(mainInst);
                    finalStream.close();

                    // print final routes to csv
                    Tools::LogOutput solutionRoutesStream(inputPaths.getOutputFinalRoutes());
                    solutionRoutesStream << mainInst->saveSolutionRoutes();
                    solutionRoutesStream.close();

                    // print requests results to csv
                    Tools::LogOutput requestResultsStream(inputPaths.getOutputFinalRequests());
                    requestResultsStream << mainInst->saveRequestsResults();
                    requestResultsStream.close();

                    // print vehicles results to csv
                    Tools::LogOutput vehiclesResultsStream(inputPaths.getOutputFinalVehicles());
                    vehiclesResultsStream << mainInst->saveVehicleResults();
                    vehiclesResultsStream.close();

                    Tools::LogOutput finalInstanceStream(inputPaths.getOutputSummary(), true);
                    finalInstanceStream
                            << "VehicleFile,paramFile,Name,Instance,Algorithm,Mode,#vehicles,#requests,#initialOnboards,#customers,customer Group,"
                               "#served Req,#Rejected Req,wait/req,wait/cust,tripDelay/req,#(Lim)served Req,#(Lim)Rejected Req,"
                               "#(Lim)served Cust,(Lim)wait/req,(Lim)wait/cust,(Lim)tripDelay/req,"
                               "idle time/vehicle,#Idle Vehicles,#pass in vehicle,#epoch,#LMP Iter,#IMP Iter,"
                               "#RP Iter,#CP Iter,#Zoom Iter,#SP Iter ,MASTER time,RP time,CP time,Zoom time,SP time,Greedy time,Assign time,"
                               "Total time,RP/ISUD,CP/ISUD,MASTER/Total,SP/Total,Greedy/Total,CPSuccess,CPFails,CGSuccess";
                    finalInstanceStream << "\n" << vehicleFolder << "," << paramFile << ",";
                    finalInstanceStream << mainInst->instRepStr_.str();
                    finalInstanceStream.close();
                }
            }
        }
    }
}

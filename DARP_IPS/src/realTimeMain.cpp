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
bool savePartial = true;
std::string instNum = "S";
int numVehicles;

int main(int argc, char** argv) {
    std::ios_base::sync_with_stdio(false);
    std::string dataDir = "datasets/";
    std::string vehicleFile = "";
    std::string vehicleFolder = "";
    int nbLocations = 1718;

    std::vector<std::string> instNames;         // vector of instance file names
    std::string instFolder;                     // folder of instances
    std::cout << "Number of arguments = " << argc << std::endl;
    int mainAlgo = -1;
    int solMode = -1;
    if (argc == 6){
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
    }
    else if (argc == 7){
        vehicleFolder = argv[1];
        instFolder = argv[2];
        instNames.emplace_back(argv[3]);
        std::cout << "Instance : " << argv[2] << "/" << argv[3]  << std::endl;
        std::string word = argv[4];
        vehicleFile = "vehicles_" + word + "_4";
        numVehicles = std::stoi(argv[4]);
        mainAlgo = std::stoi(argv[5]);
        solMode = std::stoi(argv[6]);
    }
    else
        myTools::myException::throwError("There should be at least 6 arguments!");

    // build the path of input files
    // create output files for epoch results
    InputPaths inputPaths(dataDir, vehicleFile, vehicleFolder);
    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_, nbLocations);


    for (auto & instanceName : instNames){
        // create output files for epoch results
        inputPaths.initializeInputs(instFolder, instanceName);

        // Read data files and initialize instance and parameters in output path
        std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
        Request::requestCount_ = 0;
        PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
        mainInst->nbVehicles_ = numVehicles;
        ReadWrite::readParameters(inputPaths.getInputParamFile(), mainInst);
        ReadWrite::readZones(inputPaths.getInputZones(), mainInst);
        mainInst->parameters_->savePartial_ = savePartial;
        mainInst->parameters_->mainAlgorithm_ = static_cast<MainAlgorithm>(mainAlgo);
        mainInst->parameters_->solutionMode_ = static_cast<SolutionMode>(solMode);
        ReadWrite::readDatafiles(inputPaths, mainInst, mainInst->parameters_->saveScratch_);
        std::cout << mainInst->toString();

        // create solver
        std::shared_ptr<solver> instanceSolver = std::make_shared<solver>(mainInst, inputPaths);
        if (mainInst->parameters_->solutionMode_ == DYNAMIC){
            try {
                instanceSolver->dynamicSolver(mainInst, inputPaths, instNum, middleSave, saveTime);
            } catch (const std::exception &e) {
                std::cout << "DYNAMIC solving caught an exception=: "
                          << e.what() << std::endl;
            }
        }
        else if (mainInst->parameters_->solutionMode_ == ANYTIME){
            try {
                /*if (mainInst->parameters_->mainAlgorithm_ == GREEDY)
                    instanceSolver->anyTimeSolverEvent(mainInst, inputPaths);
                else*/
                    instanceSolver->anyTimeSolver(mainInst, inputPaths, instNum, middleSave, saveTime);
            } catch (const std::exception &e) {
                std::cout << "ANY_TIME solving caught an exception=: "
                          << e.what() << std::endl;
            }
        }

        else {
            try {
                instanceSolver->staticSolver(mainInst, inputPaths, instNum, middleSave, saveTime);
            } catch (const std::exception &e) {
                std::cout << "STATIC solving caught an exception=: "
                          << e.what() << std::endl;
            }
        }
        // testing the solution route
        for(auto  &vehicleObj : mainInst->vehicles_)
            vehicleObj->solutionRoute_->testRoute(vehicleObj);
        if (!middleSave) {

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
            Tools::LogOutput finalInstanceStream(inputPaths.getOutputSummary(), true);
            finalInstanceStream
                    << "VehicleFile,Name,Instance,Algorithm,Mode,#vehicles,#requests,#customers,customer Group,"
                       "#served Req,wait/req,wait/cust,tripDelay/req,#(Lim)served Req,"
                       "#(Lim)served Cust,(Lim)wait/req,(Lim)wait/cust,(Lim)tripDelay/req,"
                       "idle time/vehicle,#Idle Vehicles,#pass in vehicle,#epoch,#LMP Iter,#IMP Iter,"
                       "#RP Iter,#CP Iter,#Zoom Iter,#SP Iter ,MASTER time,RP time,CP time,Zoom time,SP time,Greedy time,Assign time,"
                       "Total time,RP/ISUD,CP/ISUD,MASTER/Total,SP/Total,Greedy/Total, CPSuccess, CPFails";
            finalInstanceStream << "\n" << vehicleFolder << ",";
            finalInstanceStream << mainInst->instRepStr_.str();
            finalInstanceStream.close();
        }
    }
}

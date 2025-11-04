//
// Created by Elahe Amiri on 2022-10-29.
//

#include "data/Instance.h"
#include "solvers/AnytimeSolver.h"
#include "solvers/BatchSolver.h"
#include "solvers/OfflineSolver.h"
#include "utilities/Tools.h"
#include "utilities/MyTools.h"
#include "utilities/ReadWrite.h"
#include "utilities/ConfigParser.h"

using namespace std::chrono;

float saveTime = 3600;
bool middleSave = false;
int numEpochTests = 30;

int main(int argc, char** argv) {
    std::ios_base::sync_with_stdio(false);

    // Parse command line arguments
    PConfig config = std::make_unique<ProgramConfig>();
    if (!ConfigParser::parseArguments(argc, argv, config)) {
        return 1; // Exit with error code
    }

    // Print configuration for verification
    std::cout << "Starting program with the following configuration:\n";
    config->printConfig();
    std::cout << std::endl;

    // Set up paths and constants
    std::string dataDir = "my_datasets/";
    int numVehicles = config->numVehicles_;

    // Prepare instance names
    std::vector<std::string> instNames;

    if (!config->instanceName_.empty()) {
        // Use specific instance
        instNames.push_back(config->instanceName_);
        std::cout << "Processing specific instance: " << config->instFolder_
                  << "/" << config->instanceName_ << std::endl;
    } else {
        // Read instances from file
        std::string instanceNames = "datasets/InstanceNames.txt";
        ReadWrite::readInstNames(instanceNames, instNames, 24, "_07-120m");
        std::cout << "24 instances read from file!" << std::endl;
    }

    // Build the path of input files
    InputPaths inputPaths(dataDir, config);
    ReadWrite::readDurations(inputPaths.getInputDurationData(), durationMatrix_);

    int max_i = 1, max_j = 1;

    if (config->scenario_ == "truncate") {
        max_i = 7;
        max_j = 3;
    }
    else if (config->scenario_ == "pruning")
        max_i = 3;
    else if (config->scenario_ == "dropPick")
        max_i = 2;
    else if (config->scenario_ == "nbPickup")
        max_i = 2;

    for (auto & instanceName : instNames){
        for (int i = 0; i < max_i; ++i) {
            for (int j = 0; j < max_j; ++j) {
                std::this_thread::sleep_for(std::chrono::seconds(2));

                // Create output files for epoch results
                inputPaths.initializeInputs(config->instFolder_, instanceName);

                // Read data files and initialize instance and parameters
                std::cout << "# INITIALIZE OF THE MAIN INSTANCE" << std::endl;
                Request::requestCount_ = 0;
                PInstance mainInst = ReadWrite::readInstance(inputPaths.getInputInstanceData());
                if (config->initialState_ < 2)
                    mainInst->nbVehicles_ = numVehicles;


                ReadWrite::readParametersJson(inputPaths.getInputParamFile(), mainInst, config->scenario_);
                mainInst->adjustParameters(config);

                // Configure parameters based on parameter file
                if (config->scenario_ == "truncate") {
                    mainInst->parameters_->MaxLabel_ = (i + 1) * 5;
                    mainInst->parameters_->sortPath_ = static_cast<SortPaths>(j);
                }
                else if (config->scenario_ == "pruning") {
                    mainInst->parameters_->pruneNodes_ = true;
                    if (i >= 1) mainInst->parameters_->pruneArcs_ = true;
                    if (i >= 2) mainInst->parameters_->discardSuboptimalPath_ = true;
                }
                else if (config->scenario_ == "dropPick") {
                    mainInst->parameters_->isDropPickPossible_ = (i == 1);
                }
                else if (config->scenario_ == "nbPickup") {
                    if (i == 0) {
                        mainInst->parameters_->dynamicPricing_ =  true;
                        mainInst->parameters_->partialPricing_ =  false;
                    }
                    else {
                        mainInst->parameters_->partialPricing_ =  true;
                        mainInst->parameters_->dynamicPricing_ =  false;
                    }
                }


                ReadWrite::readDatafiles(inputPaths, mainInst, mainInst->parameters_->saveScratch_,
                    config->scenario_, config->initialState_);

                // Create solver and run appropriate algorithm
                std::unique_ptr<BaseSolver> instanceSolver;
                switch (mainInst->parameters_->solutionMode_) {
                    case DYNAMIC: {
                        instanceSolver = std::make_unique<BatchSolver>(mainInst, inputPaths);
                        instanceSolver->doSimulation(mainInst, inputPaths, middleSave, saveTime);
                        break;
                    }

                    case ANYTIME: {
                        instanceSolver = std::make_unique<AnytimeSolver>(mainInst, inputPaths);
                        instanceSolver->doSimulation(mainInst, inputPaths, middleSave, saveTime);
                        break;
                    }

                    case STATIC: {
                        instanceSolver = std::make_unique<OfflineSolver>(mainInst, inputPaths);
                        instanceSolver->doSimulation(mainInst, inputPaths, middleSave, saveTime);
                        break;
                    }
                }
                // print final outputs
                Tools::LogOutput finalStream(inputPaths.getOutputFinalLog());
                finalStream << instanceSolver->toString(mainInst);
                finalStream.close();
                mainInst->writeFinalOutputs(inputPaths, config);
            }
        }
    }
    std::cout << "Program completed successfully." << std::endl;
    return 0;
}
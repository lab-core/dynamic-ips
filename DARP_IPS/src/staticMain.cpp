
#include "utilities/ReadWrite.h"
#include <iostream>
#include "Eigen/Dense"
#include "solvers/ISUDAlgorithm.h"
#include "solvers/CPLEXSubProblem.h"
#include "solvers/MIPSolver.h"





using std::string;
using namespace std::chrono;

int main() {

    string dataDir = "datasets/";
    string instanceName = "20150713_16-06m";

    // build the path of input files
    InputPaths inputPaths(dataDir, instanceName);

    // Read data files and initialize instance
    std::cout << "# INITIALIZE THE INSTANCE" << std::endl;
    PInstance pInst = ReadWrite::readInstance(inputPaths.getInstanceData());
    ReadWrite::readTripRequests(inputPaths.getTripData(), pInst);
    std::cout << std::endl;
    std::cout << pInst->toString();

//    MIPSolver(pInst);

    std::shared_ptr<ISUDAlgorithm> isudObj = std::make_shared<ISUDAlgorithm>(pInst);
    isudObj->initialization(pInst);


    int flagImprove = 0;
    while (flagImprove == 0) {
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            PSubProblem subProblem = std::make_shared<SubProblem>(pInst->vehicles_[v]);
            subProblem->subGraph_ = pInst->mainGraph_;
            subProblem->subRequests_ = pInst->requests_;
            subProblem->BuildModelCPLEX(isudObj->ReducedPro_->requestDuals_, isudObj->ReducedPro_->vehicleDuals_[v],
                                        isudObj->ReducedPro_->requestToOrder_);
            subProblem->SolveCPLEX();
            std::cout << subProblem->toString() << std::endl;
            if (subProblem->SubProbCplex_.getObjValue() >= -0.00001) {
                flagImprove ++;
            }
            else
                subProblem->SolutionToRoutes(isudObj->availableRoutes_[pInst->vehicles_[v]->vehicleID_], isudObj->generatedRoutes_);
        }
        if (flagImprove == pInst->nbVehicles_) {
            std::cout << " ******************  The Column Generation Terminated!  ******************" << std::endl;
            break;
        }
        else {
            isudObj->solveISUD(pInst);
            std::cout << "# Solution Result after ISUD:" << std::endl;
            for (int r = 0; r < isudObj->routeSolution_.size(); ++r) {
                std::cout << isudObj->routeSolution_[r]->toString();
            }
            flagImprove = 0;
        }
    }
    std::cout << "# Final Solution:" << std::endl;
    std::cout << isudObj->toString();

}




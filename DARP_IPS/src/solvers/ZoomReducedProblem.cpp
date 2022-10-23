//
// Created by Ella on 2/28/2022.
//

#include "ZoomReducedProblem.h"



void ZoomReducedProblem::updateModel(PInstance &pInst, vector<PRequest> &fractionalZ) {
//    env_.out() << Model_;
    if (routesToAdd_.empty()) {
        std::cout << "There is no route to be added" << std::endl;
        throw Tools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (auto routeObj : routesToAdd_) {
        addRouteVar(routeObj);
    }

    for (auto & zRequest : fractionalZ)
        addZVar(zRequest);
}

void ZoomReducedProblem::solveModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution) {
    try {
        Model_.add(IloConversion(env_, zVar_, ILOINT));
        Model_.add(IloConversion(env_, routeVar_, ILOINT));

        std::cout << routeVar_[0].getType() << std::endl;
        Cplex_ = IloCplex(Model_);
//        env_.out() << Model_;
        Cplex_.solve();

        // printing solution status
        std::cout << toString();

        // saving the result and remove out of base variables
        zSolution.clear();
        routeSolution.clear();

        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);


        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);


        for (int r = (int) routeVal.getSize() - 1; r >= 0; --r) {
            if (routeVal[r] > 0.9) {
                routeSolution.push_back(compRoutes_[r]);
                pInst->vehicles_[compRoutes_[r]->vehicleID_]->setCurrentRoute(compRoutes_[r]);
                /*routeSolution.push_back(generatedRoutes[routeVar_[r].getName()]);
                pInst->vehicles_[generatedRoutes[routeVar_[r].getName()]->vehicleID_]->setCurrentRoute(
                        generatedRoutes[routeVar_[r].getName()]);*/
            }
        }

        for (int i = (int) zVal.getSize() - 1; i >= 0; --i) {
            if (zVal[i] > 0.9) {
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            }
        }
        std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
                  << " are selected to served." << std::endl;
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

std::string ZoomReducedProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ====================  REDUCED PROBLEM SOLVED INTEGER MODE ==================== " << std::endl;
    repStr << MasterModeler::toString();
    return repStr.str();
}



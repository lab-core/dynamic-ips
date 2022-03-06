//
// Created by Ella on 2/28/2022.
//

#include "ZoomReducedProblem.h"



void ZoomReducedProblem::updateModel(PInstance &pInst, vector<PRequest> &fractionalZ) {
    if (routesToAdd_.size() == 0) {
        std::cout << "There is no route to be added" << std::endl;
        throw Tools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }

    // add the new compatible column to the model
    for (int i = 0; i < routesToAdd_.size(); ++i) {
        addRouteVar(routesToAdd_[i]);
    }

    for (auto & zRequest : fractionalZ)
        addZVar(zRequest);
}

void ZoomReducedProblem::solveModel(PInstance &pInst, vector<PRequest> &zSolution, vector<PRoute> &routeSolution,
                                    std::unordered_map<std::string, PRoute> &generatedRoutes) {
    try {
        Model_.add(IloConversion(env_, zVar_, ILOINT));
        Model_.add(IloConversion(env_, routeVar_, ILOINT));

        env_.out() << routeVar_[0].getType();
        Cplex_ = IloCplex(Model_);
        //       env_.out() << Model_;
        Cplex_.solve();

        // printing solution status
        std::cout << MasterModeler::toString();

        // saving the result and remove out of base variables
        zSolution.clear();
        routeSolution.clear();

        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);


        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);

 //       env_.out() << routeVal << std::endl;
 //       env_.out() << zVal << std::endl;

        for (int r = routeVal.getSize()-1; r >= 0; --r) {
            if (routeVal[r] > 0.9) {
                routeSolution.push_back(generatedRoutes[routeVar_[r].getName()]);
                pInst->vehicles_[generatedRoutes[routeVar_[r].getName()]->vehicleID_]->setCurrentRoute(generatedRoutes[routeVar_[r].getName()]);
            }
        }

        for (int i = zVal.getSize()-1; i >= 0; --i) {
            if (zVal[i] > 0.9)
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
        }
        int nbRequests = 0;
        for (auto & requestObj: pInst->requests_) {
            if (requestObj->requestStatus_ == NO_ACTION)
                nbRequests++;
        }
        std::cout << "# from " << nbRequests << " request, " << nbRequests - zSolution.size()
                  << " are selected to served." << std::endl;
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}


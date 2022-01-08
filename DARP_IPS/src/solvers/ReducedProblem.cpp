//
// Created by Ella on 2021-11-10.
//

#include "ReducedProblem.h"

//---------------------------------------------------------------------------------------------
//  Reduced Problem class
//  Build and solve the Reduced problem of the ISUD
//---------------------------------------------------------------------------------------------

// Constructor and Destructor
ReducedProblem::ReducedProblem() : MasterModeler() {

    // defining variable
    zVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
    routeVar_ = IloNumVarArray(env_, 0.0, 0.0, ILOFLOAT);
}


// this function initialized the model and define empty set of constraints
void ReducedProblem::ResetRPModel() {

    try {
        int modelExist = 0;
        for (int r = routeVar_.getSize()-1; r >= 0; --r) {
            routeVar_[r].end();
            routeVar_.remove(r,1);
            modelExist = 1;
        }
        for (int i = zVar_.getSize()-1; i >= 0; --i) {
            zVar_[i].end();
            zVar_.remove(i,1);
            modelExist = 1;
        }
        if (modelExist == 1) {
            Model_.remove(requestConst_);
            Model_.remove(vehicleConst_);
        }

    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

// this function adds routeVar to the model
void ReducedProblem::addRouteVar(PRoute &newRoute) {
        MasterModeler::addRouteVar(routeVar_, newRoute, POSITIVE);
}

// this function adds zVar to the model used for the routes that served only one request
void ReducedProblem:: addZVar(PRequest &request) {
    MasterModeler::addZVar(zVar_, request, POSITIVE);
}

// this function add one route at each iteration of the algorithm during one epoch
void ReducedProblem::updateModel(PInstance &pInst, std::vector<PRoute> &routeSolution) {
    if (routesToAdd_.size() == 0) {
        std::cout << "There is no route to be added" << std::endl;
        throw Tools::myException("The input route is empty, No new column is passed to be added", __LINE__);
    }
    /*std::cout << "# number of route columns before update: " << routeVar_.getSize() << std::endl;
    std::cout << "# number of z columns before update: " << zVar_.getSize() << std::endl;*/

    // add the new compatible column to the model
    for (int i = 0; i < routesToAdd_.size(); ++i) {
        addRouteVar(routesToAdd_[i]);
    }

    // add compatible z variables
    // just z variables related to requests that are served in routes with one request are compatible
    for (int r = 0; r < routeSolution.size(); ++r) {
        if (routeSolution[r]->routeRequests.size() == 1) {
            addZVar(*routeSolution[r]->routeNodes_[1]->related_Request_);
        }
    }
    /*std::cout << "# number of route columns after update: " << routeVar_.getSize() << std::endl;
    std::cout << "# number of z columns after update: " << zVar_.getSize() << std::endl;*/
}

// this function build the model at the start of each epoch
void ReducedProblem::buildModel(PInstance &pInst, std::vector<PRequest> &zSolution, std::vector<PRoute> &routeSolution) {

    // model initialization (defining empty set of constraints and adding objective)
    ResetRPModel();
    int rhs = 1;
    MasterModeler::initializeModel(pInst, rhs);


 //   MasterModeler::initializeModel(pInst, rhs);

        // adding request columns (z variables)
    for (int i = 0; i < zSolution.size(); ++i) {
        addZVar(zSolution[i]);
//        env_.out() << Model_;
    }

    // adding solution columns
    for (int r = 0; r < routeSolution.size(); ++r) {
        addRouteVar(routeSolution[r]);
//        env_.out() << Model_;

        // add related compatible z variables
        if (routeSolution[r]->routeRequests.size() == 1) {
            addZVar(*routeSolution[r]->routeNodes_[1]->related_Request_);
//            env_.out() << Model_;
        }
    }

    //adding new route variables
    for (int r = 0; r < routesToAdd_.size(); ++r) {
        addRouteVar(routesToAdd_[r]);
//        env_.out() << Model_;
    }
//    env_.out() << Model_;

}

// this function solve the model and remove all columns except than the current base
void ReducedProblem::solveModel(PInstance &pInst, std::vector<PRequest> &zSolution,
                                std::vector<PRoute> &routeSolution, std::map<std::string , PRoute> &generatedRoutes) {
    try {

        Cplex_ = IloCplex(Model_);
 //       env_.out() << Model_;
        Cplex_.solve();

        // getting dual values
        requestDuals_.clear();
        vehicleDuals_.clear();

        // define dual container size
        requestDuals_ = IloNumArray(env_, pInst->nbRequests_);
        vehicleDuals_ = IloNumArray(env_, pInst->nbVehicles_);

//        std::cout << "REDUCED DUALS:" << std::endl;
        for (int r = 0; r < pInst->nbRequests_; ++r) {
            requestDuals_[r] = Cplex_.getDual(requestConst_[r]);
//            std::cout << "requestDuals[" << r <<"]: " << requestDuals_[r] << std::endl;
        }
        for (int v = 0; v < pInst->nbVehicles_; ++v) {
            vehicleDuals_[v] = Cplex_.getDual(vehicleConst_[v]);
            pInst->vehicles_[v]->dual_ = vehicleDuals_[v];
//            std::cout << "vehicleDuals[" << v <<"]: " << vehicleDuals_[v] << std::endl;
        }

        // printing solution status
        std::cout << MasterModeler::toString();

        // saving the result and remove out of base variables
        zSolution.clear();
        routeSolution.clear();

        IloNumArray zVal(env_);
        IloNumArray routeVal(env_);


        Cplex_.getValues(zVal, zVar_);
        Cplex_.getValues(routeVal, routeVar_);
        /*env_.out() << routeVal << std::endl;
        env_.out() << zVal << std::endl;*/

        for (int r = routeVal.getSize()-1; r >= 0; --r) {
            if (routeVal[r] > 0.9) {
                routeSolution.push_back(generatedRoutes[routeVar_[r].getName()]);
                pInst->vehicles_[generatedRoutes[routeVar_[r].getName()]->vehicleID_]->setCurrentRoute(generatedRoutes[routeVar_[r].getName()]);
            }
            else {
                routeVar_[r].end();
                routeVar_.remove(r,1);
            }
        }

        for (int i = zVal.getSize()-1; i >= 0; --i) {
            if (zVal[i] > 0.9)
                zSolution.push_back(pInst->nameToRequest_[zVar_[i].getName()]);
            else {
                zVar_[i].end();
                zVar_.remove(i,1);
            }
        }
        std::cout << "# from " << pInst->nbRequests_ << " request, " << pInst->nbRequests_ - zSolution.size()
        << " are selected to served." << std::endl;
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }

}

//************************************************************************
// Display function
//************************************************************************
std::string ReducedProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# =======================  REDUCED PROBLEM SOLVED  ======================= " << std::endl;
    repStr << MasterModeler::toString();
    return repStr.str();
}





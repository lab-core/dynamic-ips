//
// Created by Ella on 2021-10-13.
//

#include "SubProblem.h"

//-----------------------------------------------------------------------------
//  SubProblem class
//  Algorithms for solving the sub problems and getting results
//-----------------------------------------------------------------------------

// Constructor and Destructor
SubProblem::SubProblem(PVehicle &vehicle) : Vehicle_(&vehicle){
    subGraph_ = std::make_shared<Graph>();
    /*numRoutes_ = 0;
    bestReducedCost_ = 0;*/
    SubProModel_ = IloModel(env_);
}
SubProblem::~SubProblem() {
    env_.end();
}

// calculation of penalties and initialization of the subgraph
void SubProblem::initSubGraph(PInstance &pInst) {

    // create graph with source and sink
    subGraph_ = std::make_shared<Graph>(pInst->instGraph_->nodes_[(*Vehicle_)->departID_],
                                        pInst->instGraph_->nodes_[(*Vehicle_)->sinkID_]);

    // adding onboard nodes to the graph
    for (int i = 0; i < (*Vehicle_)->onboards_.size(); ++i) {
        subGraph_->addNewNode(pInst->instGraph_->nodes_[(*Vehicle_)->onboards_[i]]);
    }

    // adding available nodes based on the penalty
    for (int r = 0; r < pInst->nbRequests_; ++r) {
        if ((pInst->requests_[r]->requestStatus_ == NO_ACTION)&&(pInst->requests_[r]->subStatus_ == NOTSELECTED)) {
            /*double minWait = (*Vehicle_)->departTime_ +
                            queryTravelTime(pInst->instGraph_->nodes_[(*Vehicle_)->departID_],
                                           pInst->instGraph_->nodes_[Tools::createNodeID(pInst->requests_[r]->getRequestId(), PICKUP)])
                            - pInst->requests_[r]->earlyPick_;*/
            float minWait = (*Vehicle_)->departTime_ +
                             travelMat->queryTravelTime(pInst->instGraph_->nodes_[(*Vehicle_)->departID_],
                                             pInst->instGraph_->nodes_[Tools::createNodeID(pInst->requests_[r]->getRequestId(), PICKUP)])
                             - pInst->requests_[r]->earlyPick_;
            if (minWait <= pInst->requests_[r]->penalty_) {
                subGraph_->addNewNode(pInst->instGraph_->nodes_[Tools::createNodeID(pInst->requests_[r]->getRequestId(), PICKUP)]);
                subGraph_->addNewNode(pInst->instGraph_->nodes_[Tools::createNodeID(pInst->requests_[r]->getRequestId(), DROPOFF)]);

                // adding available requests
                subRequests_.push_back(pInst->requests_[r]);
            }
        }
    }
}

//************************************************************************
// Build the SUb Problem model with CPLEX
//************************************************************************

void SubProblem::BuildModelCPLEX(IloNumArray& requestDuals, IloNum& vehicleDual, std::map<int, int>& requestToOrder)
{
    // Definition of variables
    X = IloNumVar2D(env_, subGraph_->nbNodes_);
    U = IloNumVarArray(env_, subGraph_->nbNodes_, 0, IloInfinity, ILOFLOAT);
    W = IloNumVarArray(env_, subGraph_->nbNodes_, 0, IloInfinity, ILOFLOAT);

    for (int i = 0; i < subGraph_->nbNodes_; ++i) {
        X[i] = IloNumVarArray(env_, subGraph_->nbNodes_, 0.0, 1.0, ILOBOOL);
    }

    // define objective
    IloExpr objExpr(env_);
    for (int i = 0; i < subRequests_.size(); ++i) {
        int nodeIndex = subGraph_->nodeIDToInt_[Tools::createNodeID(subRequests_[i]->getRequestId(), PICKUP)];
        objExpr += (U[nodeIndex] - subRequests_[i]->earlyPick_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            objExpr -= (X[nodeIndex][j] * requestDuals[requestToOrder[subRequests_[i]->getRequestId()]]);
        }
    }

    objExpr -= vehicleDual;
    SubProModel_.add(IloMinimize(env_, objExpr));

    // -----------------------------------------
    // defining constraints
    // -----------------------------------------

    int sourceIndex = subGraph_->nodeIDToInt_[(*Vehicle_)->departID_];
    int sinkIndex = subGraph_->nodeIDToInt_[(*Vehicle_)->sinkID_];

    // add this constraint to be sure that each request is served at most once
    for (int i = 0; i < subRequests_.size(); ++i) {
        std::string nodeID = Tools::createNodeID(subRequests_[i]->getRequestId(), PICKUP);
        int nodeIndex = subGraph_->nodeIDToInt_[nodeID];

        IloExpr exprV(env_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            if (subGraph_->nodes_[subGraph_->intToNodeID_[j]]->type_ != SOURCE)
                exprV += X[nodeIndex][j];
        }
        SubProModel_.add(exprV <= 1);
    }

    // add these constraints for just solving the extraction problem of variables
    IloExpr exprP(env_);
    for (int j = 0; j < subGraph_->nbNodes_; ++j) {
        exprP += X[j][sourceIndex] + X[sinkIndex][j] + X[j][j];
    }
    SubProModel_.add(exprP == 0);

    // constraints 3c -------------------
    IloExpr expr3(env_);
    for (int j = 0; j < subGraph_->nbNodes_; ++j) {
        if (subGraph_->nodes_[subGraph_->intToNodeID_[j]]->type_ != SOURCE)
            expr3 += X[sourceIndex][j];
    }
    SubProModel_.add(expr3 == 1);


    // constraints 4c -------------------
    IloExpr expr4(env_);
    for (int j = 0; j < subGraph_->nbNodes_; ++j) {
        if (subGraph_->nodes_[subGraph_->intToNodeID_[j]]->type_ != SINK)
            expr4 += X[j][sinkIndex];
    }
    SubProModel_.add(expr4 == 1);

    // add this constraint for re-optimization when the vehicle is not idle
    // at the start and have some passengers onboard
    SubProModel_.add(W[sourceIndex] >= (*Vehicle_)->numPassengers_);

    // constraints 8c -------------------
    SubProModel_.add(U[sourceIndex] >= (*Vehicle_)->departTime_);

    // constraints 9c -------------------
    SubProModel_.add(U[sinkIndex] <= (*Vehicle_)->endTime_);

    for (int i = 0; i < subRequests_.size(); ++i) {
        std::string pickID = Tools::createNodeID(subRequests_[i]->getRequestId(), PICKUP);
        int pickIndex = subGraph_->nodeIDToInt_[pickID];

        std::string dropID = Tools::createNodeID(subRequests_[i]->getRequestId(), DROPOFF);
        int dropIndex = subGraph_->nodeIDToInt_[dropID];

        // constraints 5c -------------------
        IloExpr expr5(env_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            if (subGraph_->nodes_[subGraph_->intToNodeID_[j]]->type_ != SOURCE)
                expr5 += (X[pickIndex][j] - X[dropIndex][j]);
        }
        SubProModel_.add(expr5 == 0);

        // without following constraints some drop off points are before their pickups
 //       SubProModel_.add(U[pickIndex] - U[dropIndex] <= 0);

        // constraints 10c -------------------
        SubProModel_.add(U[pickIndex] >= subRequests_[i]->earlyPick_);

        // constraints 11c -------------------
        IloExpr expr11(env_);
        expr11 = U[dropIndex] - U[pickIndex] - subRequests_[i]->deltaTime_;
    //    float t = calcTravelTime(subGraph_->nodes_[pickID], subGraph_->nodes_[dropID]);
        float t = subRequests_[i]->minTravelTime_;

        SubProModel_.add(expr11 <= std::max(alphaParam * t, betaParam + t));
        SubProModel_.add(expr11 >=t);
    }

    // add this constraint to control the length of generated routes in subproblems
    IloExpr exprT(env_);
    for (int i = 0; i < subGraph_->nbNodes_; ++i) {
        if (subGraph_->nodes_[subGraph_->intToNodeID_[i]]->type_ == PICKUP) {
            for (int j = 0; j < subGraph_->nbNodes_; ++j) {
                exprT += X[i][j];
            }
        }
    }
    SubProModel_.add(exprT <= 3);

    for (int i = 0; i < subGraph_->nbNodes_; ++i) {
        // constraints 2c -------------------
        IloExpr expr2(env_);
        if ((subGraph_->nodes_[subGraph_->intToNodeID_[i]]->type_ == PICKUP) ||
            (subGraph_->nodes_[subGraph_->intToNodeID_[i]]->type_ == DROPOFF)) {
            for (int j = 0; j < subGraph_->nbNodes_; ++j) {
                if (subGraph_->nodes_[subGraph_->intToNodeID_[j]]->type_ != SOURCE)
                    expr2 += X[i][j];
            }
            for (int j = 0; j < subGraph_->nbNodes_; ++j) {
                if (subGraph_->nodes_[subGraph_->intToNodeID_[j]]->type_ != SINK)
                    expr2 -= X[j][i];
            }
            SubProModel_.add(expr2 == 0);
        }

        for (int j = 0; j < subGraph_->nbNodes_; ++j) {

            // constraints 7c -------------------
            IloExpr expr7(env_);
            /*expr7 = U[i] + subGraph_->nodes_[subGraph_->intToNodeID_[i]]->deltaTime_
                    + queryTravelTime(subGraph_->nodes_[subGraph_->intToNodeID_[i]],
                                                 subGraph_->nodes_[subGraph_->intToNodeID_[j]])
                    - U[j] - 7200 * (1 - X[i][j]);*/
            expr7 = U[i] + subGraph_->nodes_[subGraph_->intToNodeID_[i]]->deltaTime_
                    + travelMat->queryTravelTime(subGraph_->nodes_[subGraph_->intToNodeID_[i]],
                                     subGraph_->nodes_[subGraph_->intToNodeID_[j]])
                    - U[j] - 7200 * (1 - X[i][j]);
            SubProModel_.add(expr7 <= 0);

            // constraints 13c -------------------
            IloExpr expr13(env_);
            expr13 = W[i] + subGraph_->nodes_[subGraph_->intToNodeID_[j]]->nbPassengers_
                     - W[j] - 100 * (1 - X[i][j]);
            SubProModel_.add(expr13 <= 0);
        }
        // constraints 14c -------------------
        SubProModel_.add(W[i] <= (*Vehicle_)->capacity_);

    }
    for (int i = 0; i < (*Vehicle_)->onboards_.size(); ++i) {
        std::string onboardID = subGraph_->nodes_[(*Vehicle_)->onboards_[i]]->nodeID_;
 //       std::string pickNodeID = subGraph_->nodes_[onboardID]->pairNodeID_;

        // constraints 6c -------------------
        IloExpr expr6(env_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            expr6 += X[j][subGraph_->nodeIDToInt_[onboardID]];
        }
        SubProModel_.add(expr6 == 1);

        // constraints 12c -------------------
        IloExpr expr12(env_);
        expr12 = U[subGraph_->nodeIDToInt_[onboardID]] - (*subGraph_->nodes_[onboardID]->related_Request_)->pickTime_
                 - (*subGraph_->nodes_[onboardID]->related_Request_)->deltaTime_;

        float t = (*subGraph_->nodes_[onboardID]->related_Request_)->minTravelTime_;
        SubProModel_.add(expr12 <= std::max(alphaParam * t, betaParam + t));
        SubProModel_.add(expr12 >= t);
    }
}

//************************************************************************
// Solve the SUb Problem model with CPLEX
//************************************************************************
void SubProblem::SolveCPLEX() {
    try {
        SubProbCplex_ = IloCplex(SubProModel_);
        SubProbCplex_.setOut(env_.getNullStream());

        // set the parameters
        SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.5);
//    SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::Replace, 2);
//    SubProbCplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.05);
    SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::Capacity, 20);
//    SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::Intensity, 3);
        SubProbCplex_.setParam(IloCplex::TiLim, 100);
        SubProbCplex_.solve();
        if (SubProbCplex_.getObjValue() <= -0.00001) {
            SubProbCplex_.setParam(IloCplex::TiLim, 50);
            SubProbCplex_.populate();
        }

        /*if (SubProbCplex_.getObjValue() < 0) {
            std::cout << "# Incumbent objective value = " << SubProbCplex_.getObjValue() << std::endl;
            std::cout << "# Second phase (population is started: " << std::endl;
            SubProbCplex_.populate();
        }*/

//        SubProbCplex_.solve();
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

// function to convert solution to routes and save them in vehicle object
void SubProblem::SolutionToRoutes(std::vector<PRoute> &availableRoutes, std::map<std::string , PRoute> &generatedRoutes) {
    try {
        int repeatFlag = 0;
        availableRoutes.clear();
        for (int s = 0; s < SubProbCplex_.getSolnPoolNsolns(); ++s) {

            // extracting the value of variables in each solution
            if (SubProbCplex_.getObjValue(s) < 0) {
                repeatFlag = 0;
                IloNumArray uVal(env_);
                IloNumArray wVal(env_);
                IloNum2D xVal(env_, subGraph_->nbNodes_);

                SubProbCplex_.getValues(uVal, U, s);
                SubProbCplex_.getValues(wVal, W, s);

                for (int i = 0; i < subGraph_->nbNodes_; ++i) {
                    xVal[i] = IloNumArray(env_);
                    SubProbCplex_.getValues(xVal[i], X[i], s);
                }

                // creating the route
                int sourceIndex = subGraph_->nodeIDToInt_[(*Vehicle_)->departID_];
                int sinkIndex = subGraph_->nodeIDToInt_[(*Vehicle_)->sinkID_];
//                (*Vehicle_)->generatedRoutes_.emplace_back(std::make_shared<Route>((*Vehicle_)->vehicleID_));
                PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);

                // adding the source node of the route
                /*(*Vehicle_)->generatedRoutes_.back()->reducedCost_ = SubProbCplex_.getObjValue(s);
                (*Vehicle_)->generatedRoutes_.back()->addNode(subGraph_->nodes_[(*Vehicle_)->departID_],
                                                              (*Vehicle_)->departTime_, (*Vehicle_)->numPassengers_);*/
                newRoute->reducedCost_ = SubProbCplex_.getObjValue(s);
                newRoute->addNode(subGraph_->nodes_[(*Vehicle_)->departID_],
                                                              (*Vehicle_)->departTime_, (*Vehicle_)->numPassengers_);

                int currentNodeIndex = sourceIndex;
                while (currentNodeIndex != sinkIndex) {
                    for (int i = 0; i < subGraph_->nbNodes_; ++i) {
                        if (xVal[currentNodeIndex][i] > 0.9) {
                            newRoute->addNode(subGraph_->nodes_[subGraph_->intToNodeID_[i]],
                                                                          uVal[i], wVal[i]);
                            /*(*Vehicle_)->generatedRoutes_.back()->addNode(subGraph_->nodes_[subGraph_->intToNodeID_[i]],
                                                                          uVal[i], wVal[i]);*/
                            if ((s == 0)&&(newRoute->routeNodes_.back()->nodeID_ != (*Vehicle_)->sinkID_))
                                (*newRoute->routeNodes_.back()->related_Request_)->subStatus_ = SELECTED;
                            currentNodeIndex = i;
                            break;
                        }
                    }
                }
                for (int r = 0; r < availableRoutes.size(); ++r) {
                    if (newRoute == availableRoutes[r]) {
                        repeatFlag = 1;
                        break;
                    }
                }
                if (repeatFlag == 0) {
                    availableRoutes.push_back(newRoute);
                    generatedRoutes.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
                }

            }
        }
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
}

//************************************************************************
// Display function
//************************************************************************
std::string SubProblem::toString() const {
    std::stringstream repStr;
    repStr << std::endl;
    repStr << "# ------------------------------------------------------------" << std::endl;
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << (*Vehicle_)->vehicleID_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << SubProbCplex_.getStatus() << std::endl;
    repStr << "# Incumbent objective value = " << SubProbCplex_.getObjValue() << std::endl;
    repStr << "# The solution pool contains = " << SubProbCplex_.getSolnPoolNsolns() << " solutions." << std::endl;
    repStr << "# " << SubProbCplex_.getSolnPoolNreplaced() << " solutions were replaced in the solution pool " << std::endl;

    repStr << "# In total, " << SubProbCplex_.getSolnPoolNsolns() + SubProbCplex_.getSolnPoolNreplaced() <<
    " solutions were generated." << std::endl;

    int nbValidSolution = 0;
    for (int i = 0; i < SubProbCplex_.getSolnPoolNsolns(); ++i) {
        if (SubProbCplex_.getObjValue(i) < 0)
            nbValidSolution ++;
    }
    repStr << "# The solution pool contains = " << nbValidSolution << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}





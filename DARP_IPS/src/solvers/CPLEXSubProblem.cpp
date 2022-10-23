//
// Created by Ella on 2021-10-13.
//

#include "CPLEXSubProblem.h"

//-----------------------------------------------------------------------------
//  CPLEXSubProblem class
//  Algorithms for solving the sub problems and getting results
//-----------------------------------------------------------------------------

// Constructor and Destructor
CPLEXSubProblem::CPLEXSubProblem(PVehicle &vehicle) : SubproModeler(vehicle){
    SubProModel_ = IloModel(env_);
}
CPLEXSubProblem::~CPLEXSubProblem() {
    env_.end();
}

//************************************************************************
// Build the SUb Problem model with CPLEX
//************************************************************************

void CPLEXSubProblem::BuildModelCPLEX(int maxPickUp)
{
    // Definition of variables
    X = IloNumVar2D(env_, subGraph_->nbNodes_);
    U = IloNumVarArray(env_, subGraph_->nbNodes_, 0, IloInfinity, ILOFLOAT);
    W = IloNumVarArray(env_, subGraph_->nbNodes_, 0, IloInfinity, ILOFLOAT);

    /*int sourceIndex = subGraph_->nodeIDToInt_[(*Vehicle_)->departID_];
    int sinkIndex = subGraph_->nodeIDToInt_[(*Vehicle_)->sinkID_];*/
    int sourceIndex = subGraph_->nodes_[(*Vehicle_)->departID_]->nodeIndex_;
    int sinkIndex = subGraph_->nodes_[(*Vehicle_)->sinkID_]->nodeIndex_;

    for (int i = 0; i < subGraph_->nbNodes_; ++i) {
        X[i] = IloNumVarArray(env_, subGraph_->nbNodes_, 0.0, 1.0, ILOBOOL);
    }

    // define objective
    IloExpr objExpr(env_);
    for (auto & requestObj : subRequests_) {
//        int nodeIndex = subGraph_->nodeIDToInt_[myTools::createNodeID(requestObj->getRequestId(), PICKUP)];
        int nodeIndex = subGraph_->nodes_[myTools::createNodeID(requestObj->getRequestId(), PICKUP)]->nodeIndex_;
        objExpr += (U[nodeIndex] - requestObj->earlyPick_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            objExpr -= (X[nodeIndex][j] * requestObj->dual_);
        }
    }

//    objExpr += U[sinkIndex];
    objExpr -= (*Vehicle_)->dual_;
    SubProModel_.add(IloMinimize(env_, objExpr));

    // -----------------------------------------
    // defining constraints
    // -----------------------------------------

    // add this constraint to be sure that each request is served at most once
    for (auto & requestObj : subRequests_) {
        std::string nodeID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
 //       int nodeIndex = subGraph_->nodeIDToInt_[nodeID];
        int nodeIndex = subGraph_->nodes_[nodeID]->nodeIndex_;

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

    for (auto & requestObj : subRequests_) {
        std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
  //      int pickIndex = subGraph_->nodeIDToInt_[pickID];
        int pickIndex = subGraph_->nodes_[pickID]->nodeIndex_;

        std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
//        int dropIndex = subGraph_->nodeIDToInt_[dropID];
        int dropIndex = subGraph_->nodes_[dropID]->nodeIndex_;

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
        SubProModel_.add(U[pickIndex] >= requestObj->earlyPick_);

        // constraints 11c -------------------
        IloExpr expr11(env_);
        expr11 = U[dropIndex] - U[pickIndex] - requestObj->deltaTime_;

        SubProModel_.add(expr11 <= requestObj->maxTravelTime_);
        SubProModel_.add(expr11 >= requestObj->minTravelTime_);
    }

    // add this constraint to control the length of generated routes in subproblems
    // this constraint is added only at the start of iteration
    IloExpr exprT(env_);
    for (int i = 0; i < subGraph_->nbNodes_; ++i) {
        if (subGraph_->nodes_[subGraph_->intToNodeID_[i]]->type_ == PICKUP) {
            for (int j = 0; j < subGraph_->nbNodes_; ++j) {
                exprT += X[i][j];
            }
        }
    }
    SubProModel_.add(exprT <= maxPickUp);


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

            expr7 = U[i] + subGraph_->nodes_[subGraph_->intToNodeID_[i]]->deltaTime_
                    + durationMatrix_[subGraph_->nodes_[subGraph_->intToNodeID_[i]]->locationID_]
                    [subGraph_->nodes_[subGraph_->intToNodeID_[j]]->locationID_]- U[j] - 27000 * (1 - X[i][j]);

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
    for (auto &onboardID : (*Vehicle_)->onboards_) {
//        std::string onboardID = subGraph_->nodes_[(*Vehicle_)->onboards_[i]]->nodeID_;

        // constraints 6c -------------------
        IloExpr expr6(env_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
  //          expr6 += X[j][subGraph_->nodeIDToInt_[onboardID]];
            expr6 += X[j][subGraph_->nodes_[onboardID]->nodeIndex_];
        }
        SubProModel_.add(expr6 == 1);

        // constraints 12c -------------------
        IloExpr expr12(env_);
        /*expr12 = U[subGraph_->nodeIDToInt_[onboardID]] - subGraph_->nodes_[onboardID]->related_Request_->pickTime_
                 - subGraph_->nodes_[onboardID]->related_Request_->deltaTime_;*/
        expr12 = U[subGraph_->nodes_[onboardID]->nodeIndex_] - subGraph_->nodes_[onboardID]->related_Request_->pickTime_
                 - subGraph_->nodes_[onboardID]->related_Request_->deltaTime_;

//        float t = (*subGraph_->nodes_[onboardID]->related_Request_)->minTravelTime_;
        SubProModel_.add(expr12 <= subGraph_->nodes_[onboardID]->related_Request_->maxTravelTime_);
        SubProModel_.add(expr12 >= subGraph_->nodes_[onboardID]->related_Request_->minTravelTime_);

    }
}

//************************************************************************
// Solve the SUb Problem model with CPLEX
//************************************************************************
void CPLEXSubProblem::SolveCPLEX() {
    try {
        SubProbCplex_ = IloCplex(SubProModel_);
        SubProbCplex_.setOut(env_.getNullStream());

        // set the Parameters
        SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.5);
        SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::Capacity, 20);
//    SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::Intensity, 3);
//    SubProbCplex_.setParam(IloCplex::Param::MIP::Pool::Replace, 2);
//    SubProbCplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.05);
        SubProbCplex_.setParam(IloCplex::TiLim, 300);
        if ( !SubProbCplex_.solve()) {
            std::cout << "Failed to optimize the subproblem" << std::endl;
        }
//        SubProbCplex_.solve();
        else {
            if (SubProbCplex_.getObjValue() <= -0.0001) {
                bestReducedCost_ = SubProbCplex_.getObjValue();
                SubProbCplex_.setParam(IloCplex::TiLim, 200);
                SubProbCplex_.populate();
            }
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
void CPLEXSubProblem::SolutionToRoutes(std::vector<PRoute> &availableRoutes) {
    try {

//        availableRoutes.clear();
        for (int s = 0; s < SubProbCplex_.getSolnPoolNsolns(); ++s) {
//            bool isRepeated = false;

            // extracting the value of variables in each solution
            if (SubProbCplex_.getObjValue(s) < 0) {
//                isRepeated = 0;
                IloNumArray uVal(env_);
                IloNumArray wVal(env_);
                IloNum2D xVal(env_, subGraph_->nbNodes_);

                SubProbCplex_.getValues(uVal, U, s);
                SubProbCplex_.getValues(wVal, W, s);

                for (int i = 0; i < subGraph_->nbNodes_; ++i) {
                    xVal[i] = IloNumArray(env_);
                    SubProbCplex_.getValues(xVal[i], X[i], s);
                }
                /*std::cout << "--------------------------" << std::endl;
                std::cout << "Solution: " << s << std::endl;
                std::cout << "--------------------------" << std::endl;

                for (int i = 0; i < subGraph_->nbNodes_; ++i) {
                    for (int j = 0; j < subGraph_->nbNodes_; ++j) {
                        if (xVal[i][j] > 0)
                            std::cout << "X[" << subGraph_->intToNodeID_[i] <<"][" << subGraph_->intToNodeID_[j] <<"]: " << xVal[i][j] << std::endl;
                    }
                }
                for (int i = 0; i < subGraph_->nbNodes_; ++i) {
                    for (int j = 0; j < subGraph_->nbNodes_; ++j) {
                        if (xVal[i][j] > 0) {
                            std::cout << "W[" << subGraph_->intToNodeID_[i] <<"]: " << wVal[i] << std::endl;
                            std::cout << "U[" << subGraph_->intToNodeID_[i] <<"]: " << uVal[i] << std::endl;
                        }
                    }
                }

                double newObj = 0;

                for (int i = 0; i < subRequests_.size(); ++i) {
                    int nodeIndex = subGraph_->nodeIDToInt_[myTools::createNodeID(subRequests_[i]->getRequestId(), PICKUP)];
                    newObj += (uVal[nodeIndex] - subRequests_[i]->earlyPick_);
                    std::cout << "Request U: " << subRequests_[i]->getRequestId() << " :: ";
                    std::cout << uVal[nodeIndex] << "--" << subRequests_[i]->earlyPick_ << std::endl;
                }
                std::cout << "new object delay: " << newObj << std::endl;
                for (int i = 0; i < subRequests_.size(); ++i) {
                    int nodeIndex = subGraph_->nodeIDToInt_[myTools::createNodeID(subRequests_[i]->getRequestId(), PICKUP)];
                    for (int j = 0; j < subGraph_->nbNodes_; ++j) {
                        newObj -= (xVal[nodeIndex][j] * subRequests_[i]->dual_);
                        if (xVal[nodeIndex][j] > 0) {
                            std::cout << "Request U: " << subRequests_[i]->getRequestId() << " :: ";
                            std::cout << subRequests_[i]->dual_ << std::endl;
                        }
                    }
                }
                std::cout << "new object after request dual: " << newObj << std::endl;
                newObj -= (*Vehicle_)->dual_;
                std::cout << "new object final: " << newObj << std::endl;*/


                // creating the route
                int sourceIndex = subGraph_->nodes_[(*Vehicle_)->departID_]->nodeIndex_;
                int sinkIndex = subGraph_->nodes_[(*Vehicle_)->sinkID_]->nodeIndex_;
//                (*Vehicle_)->generatedRoutes_.emplace_back(std::make_shared<Route>((*Vehicle_)->vehicleID_));
                PRoute newRoute = std::make_shared<Route>((*Vehicle_)->vehicleID_);

                // adding the source node of the route
                /*(*Vehicle_)->generatedRoutes_.back()->reducedCost_ = SubProbCplex_.getObjValue(s);
                (*Vehicle_)->generatedRoutes_.back()->addNode(subGraph_->nodes_[(*Vehicle_)->departID_],
                                                              (*Vehicle_)->departTime_, (*Vehicle_)->nbPassengers_);*/
                newRoute->reducedCost_ = SubProbCplex_.getObjValue(s);
                newRoute->addSource(subGraph_->nodes_[(*Vehicle_)->departID_],
                                                              (*Vehicle_)->departTime_, (*Vehicle_)->numPassengers_);

                int currentNodeIndex = sourceIndex;
                while (currentNodeIndex != sinkIndex) {
                    for (int i = 0; i < subGraph_->nbNodes_; ++i) {
                        if (xVal[currentNodeIndex][i] > 0.9) {
                            if (i != sinkIndex) {
     //                           newRoute->addNode(subGraph_->nodes_[subGraph_->intToNodeID_[i]], uVal[i], wVal[i]);
                                newRoute->addNode(subGraph_->nodes_[subGraph_->intToNodeID_[i]]);
                                /*if (s == 0)
                                    newRoute->routeNodes_.back()->related_Request_->selectStatus_ = SELECTED;*/
                            }
                            /*if ((s == 0)&&(newRoute->routeNodes_.back()->nodeID_ != (*Vehicle_)->sinkID_))
                                (*newRoute->routeNodes_.back()->related_Request_)->selectStatus_ = SELECTED;*/
                            currentNodeIndex = i;
                            break;
                        }
                    }
                }
                availableRoutes.push_back(newRoute);
 //               generatedRoutes.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
                /*for (int r = 0; r < availableRoutes.size(); ++r) {
                    if (newRoute == availableRoutes[r]) {
                        isRepeated = true;
                        break;
                    }
                }
                if (!isRepeated) {
                    availableRoutes.push_back(newRoute);
                    generatedRoutes.insert(std::pair <std::string , PRoute> (newRoute->name_ , newRoute));
                }*/
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
std::string CPLEXSubProblem::toString() const {
    std::stringstream repStr;
    repStr << "#" << std::endl;
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





//
// Created by Ella on 2021-10-15.
//

#include "MIPSolver.h"
//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Cplex
//  This is just for testing the results and comparison
//-----------------------------------------------------------------------------

void MIPSolver(PInstance& PInst, InputPaths &filePaths)
{
    IloEnv env;
    IloModel MIPModel(env);

    // Definition of variables
    IloNumVarArray Z(env, PInst->nbRequests_, 0.0, 1.0, ILOBOOL);
    IloNumVar3D X(env, PInst->nbVehicles_);
    IloNumVar2D U(env, PInst->nbVehicles_);
    IloNumVar2D W(env, PInst->nbVehicles_);

    for (int v = 0; v < PInst->nbVehicles_; ++v) {
        U[v] = IloNumVarArray (env, PInst->instGraph_->nbNodes_, 0, IloInfinity, ILOFLOAT);
        W[v] = IloNumVarArray (env, PInst->instGraph_->nbNodes_, 0, IloInfinity, ILOFLOAT);
        X[v] = IloNumVar2D(env, PInst->instGraph_->nbNodes_);
        for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i) {
            X[v][i] = IloNumVarArray(env, PInst->instGraph_->nbNodes_, 0.0, 1.0, ILOBOOL);
        }
    }

    // define objective function
    IloExpr objExpr(env);
    for (int i = 0; i < PInst->nbRequests_; ++i) {
        objExpr += Z[i] * PInst->requests_[i]->penalty_;
        for (int v = 0; v < PInst->nbVehicles_; ++v) {
            int nodeIndex = PInst->instGraph_->nodes_[myTools::createNodeID(PInst->requests_[i]->getRequestId(), PICKUP)]->nodeIndex_;
            objExpr += (U[v][nodeIndex] - PInst->requests_[i]->earlyPick_);
        }
    }
    MIPModel.add(IloMinimize(env, objExpr));

    // -----------------------------------------
    // defining constraints
    // -----------------------------------------


    // constraints 2a -------------------
    for (int i = 0; i < PInst->nbRequests_; ++i) {
        std::string nodeID = myTools::createNodeID(PInst->requests_[i]->getRequestId(), PICKUP);
        int nodeIndex = PInst->instGraph_->nodes_[nodeID]->nodeIndex_;

        IloExpr expr2(env);
        expr2 += Z[i];
        for (int v = 0; v < PInst->nbVehicles_; ++v) {
            for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SOURCE)
                    expr2 += X[v][nodeIndex][j];
            }
        }
        MIPModel.add(expr2 == 1);
    }

    for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
        if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ == SOURCE) {
            IloExpr exprS(env);
            for (int v = 0; v < PInst->nbVehicles_; ++v) {
                for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i)
                    exprS += X[v][j][i];
            }
            MIPModel.add(exprS <= 1);
        }
    }


    for (int v = 0; v < PInst->nbVehicles_; ++v) {
        int sourceIndex = PInst->vehicles_[v]->departNode_->nodeIndex_;
        int sinkIndex = PInst->vehicles_[v]->sinkNode_->nodeIndex_;
        // add these constraints for just solving the extraction problem of variables
        /*IloExpr exprP(env);
        for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
            exprP += X[v][j][j];
            if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ == SOURCE) {
                for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i){
                    exprP += X[v][i][j];
                }
            }
            else if  (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ == SINK) {
                for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i)
                    exprP += X[v][j][i];
            }
        }
        MIPModel.add(exprP == 0);*/


        // constraints 4a -------------------
        IloExpr expr4(env);
        for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
            if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SOURCE)
                expr4 += X[v][sourceIndex][j];
        }
        MIPModel.add(expr4 == 1);

        // constraints 5a -------------------
        IloExpr expr5(env);
        for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
            if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SINK)
                expr5 += X[v][j][sinkIndex];
        }
        MIPModel.add(expr5 == 1);

        // add this constraint for re-optimization when the vehicle is not idle
        // at the start and have some passengers onboard
        MIPModel.add(W[v][sourceIndex] >= PInst->vehicles_[v]->numPassengers_);

        // constraints 9a -------------------
        MIPModel.add(U[v][sourceIndex] >= PInst->vehicles_[v]->departTime_);

        // constraints 10a -------------------
        MIPModel.add(U[v][sinkIndex] <= PInst->vehicles_[v]->endTime_);

        for (auto & requestObj: PInst->requests_) {
            std::string pickID = myTools::createNodeID(requestObj->getRequestId(), PICKUP);
            int pickIndex = PInst->instGraph_->nodes_[pickID]->nodeIndex_;

            std::string dropID = myTools::createNodeID(requestObj->getRequestId(), DROPOFF);
            int dropIndex = PInst->instGraph_->nodes_[dropID]->nodeIndex_;

            // constraints 6a -------------------
            IloExpr expr6(env);
            for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SOURCE)
                    expr6 += (X[v][pickIndex][j] - X[v][dropIndex][j]);
            }
            MIPModel.add(expr6 == 0);

            // constraints 11a -------------------
            MIPModel.add(U[v][pickIndex] >= requestObj->earlyPick_);

            // constraints 12a -------------------
            IloExpr expr12(env);
            expr12 = U[v][dropIndex] - U[v][pickIndex] - requestObj->serviceTime_;
            MIPModel.add(expr12 <= requestObj->maxTravelTime_);
            MIPModel.add(expr12 >= requestObj->minTravelTime_);

        }

        for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i) {
            // constraints 3a -------------------
            IloExpr expr3(env);
            if ((PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]->type_ == PICKUP) ||
                (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]->type_ == DROPOFF)) {
                for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                    if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SOURCE)
                        expr3 += X[v][i][j];
                }
                for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                    if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SINK)
                        expr3 -= X[v][j][i];
                }
                MIPModel.add(expr3 == 0);
            }

            for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                // constraints 8a -------------------
                IloExpr expr8(env);
                expr8 = U[v][i] + PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]->serviceTime_
                        + durationMatrix_[PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]->locationID_]
                        [PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->locationID_]*X[v][i][j]
                        - U[v][j] - 27000 * (1 - X[v][i][j]);
                MIPModel.add(expr8 <= 0);

                // constraints 14a -------------------
                IloExpr expr14(env);
                expr14 = W[v][i] + PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->nbPassengers_*X[v][i][j]
                         - W[v][j] - 100 * (1 - X[v][i][j]);
                MIPModel.add(expr14 <= 0);

            }

            // constraints 15a -------------------
            MIPModel.add(W[v][i] <= PInst->vehicles_[v]->capacity_);
        }

        for (auto & onboardID : PInst->vehicles_[v]->onboards_) {

            // constraints 7a -------------------
            IloExpr expr7(env);
            for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                expr7 += X[v][j][PInst->instGraph_->nodes_[onboardID]->nodeIndex_];
            }
            MIPModel.add(expr7 == 1);

            // constraints 13a -------------------
            IloExpr expr13(env);
            expr13 = U[v][PInst->instGraph_->nodes_[onboardID]->nodeIndex_] - PInst->instGraph_->nodes_[onboardID]->related_Request_->pickTime_
                     - PInst->instGraph_->nodes_[onboardID]->serviceTime_;
            MIPModel.add(expr13 <= PInst->instGraph_->nodes_[onboardID]->related_Request_->maxTravelTime_);
            MIPModel.add(expr13 >= PInst->instGraph_->nodes_[onboardID]->related_Request_->minTravelTime_);
        }
    }



    IloCplex MIPCplex(MIPModel);

    MIPCplex.setParam(IloCplex::Param::MIP::Limits::RepairTries, 10);
    MIPCplex.setParam(IloCplex::Param::MIP::PolishAfter::Time, 100);
    MIPCplex.setParam(IloCplex::Param::TimeLimit, 300);
    MIPCplex.setParam(IloCplex::Param::Emphasis::MIP, 2);
//    MIPCplex.setParam(IloCplex::Param::Threads, 6);
//    MIPCplex.addMIPStart(startVar, startVal);
//    MIPCplex.setParam(IloCplex::SimDisplay, 2);
//    MIPCplex.setParam(IloCplex::Param::MIP::Strategy::Branch, 1);
//    MIPCplex.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, 6000);
//    MIPCplex.setParam(IloCplex::Param::MIP::Strategy::VariableSelect, 4);
//    MIPCplex.setParam(IloCplex::Param::MIP::Strategy::NodeSelect, 3);

    try {
        MIPCplex.solve();
        std::cout << "The objective value: " << MIPCplex.getObjValue() << std::endl;
        std::cout << " ================  THE RESULT OF SOLVING WITH MIP SOLVER  ================ " << std::endl;

        for (int v = 0; v < PInst->nbVehicles_; ++v) {
            IloNumArray uVal(env);
            IloNumArray wVal(env);

            IloNum2D xVal(env, PInst->instGraph_->nbNodes_);

            MIPCplex.getValues(uVal, U[v]);
            MIPCplex.getValues(wVal, W[v]);

            for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i) {
                xVal[i] = IloNumArray(env);
                MIPCplex.getValues(xVal[i], X[v][i]);
            }

            // creating the route
            int sourceIndex = PInst->vehicles_[v]->departNode_->nodeIndex_;
            int sinkIndex = PInst->vehicles_[v]->sinkNode_->nodeIndex_;
            PRoute newRoute = std::make_shared<Route>(PInst->vehicles_[v]->vehicleID_);

            newRoute->addSource(PInst->vehicles_[v]->departNode_,PInst->vehicles_[v]->departTime_,
                                PInst->vehicles_[v]->numPassengers_);

            int currentNodeIndex = sourceIndex;
            while (currentNodeIndex != sinkIndex) {
                for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i) {
                    if (xVal[currentNodeIndex][i] > 0.9) {
                        newRoute->addNode(PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]);
                        currentNodeIndex = i;
                        break;
                    }
                }
            }
            PInst->vehicles_[v]->currentRoute_ = newRoute;
//            PInst->vehicles_[v]->solutionRoute_ = newRoute;
        }
        PInst->saveSolutionRoutes();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
        std::cout << e << std::endl;
    }
    env.end();
}

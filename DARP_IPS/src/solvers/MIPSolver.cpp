//
// Created by Ella on 2021-10-15.
//

#include "MIPSolver.h"
//-----------------------------------------------------------------------------
//  Contains MIP formulation to solve with Cplex
//  This is just for testing the results and comparison
//-----------------------------------------------------------------------------

void MIPSolver(PInstance& PInst)
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
            int nodeIndex = PInst->instGraph_->nodeIDToInt_[Tools::createNodeID(PInst->requests_[i]->getRequestId(), PICKUP)];
            objExpr += (U[v][nodeIndex] - PInst->requests_[i]->earlyPick_);
        }
    }
    MIPModel.add(IloMinimize(env, objExpr));

    // -----------------------------------------
    // defining constraints
    // -----------------------------------------

    // constraints 2a -------------------
    for (int i = 0; i < PInst->nbRequests_; ++i) {
        std::string nodeID = Tools::createNodeID(PInst->requests_[i]->getRequestId(), PICKUP);
        int nodeIndex = PInst->instGraph_->nodeIDToInt_[nodeID];

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


    for (int v = 0; v < PInst->nbVehicles_; ++v) {

        // add these constraints for just solving the extraction problem of variables
        IloExpr exprP(env);
        for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
            exprP += X[v][j][0] + X[v][1][j] + X[v][j][j];
        }
        MIPModel.add(exprP == 0);


        // constraints 4a -------------------
        IloExpr expr4(env);
        for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
            if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SOURCE)
                expr4 += X[v][0][j];
        }
        MIPModel.add(expr4 == 1);

        // constraints 5a -------------------
        IloExpr expr5(env);
        for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
            if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SINK)
                expr5 += X[v][j][1];
        }
        MIPModel.add(expr5 == 1);

        // constraints 9a -------------------
        MIPModel.add(U[v][0] >= PInst->vehicles_[v]->startTime_);

        // constraints 10a -------------------
        MIPModel.add(U[v][1] <= PInst->vehicles_[v]->endTime_);


        for (int i = 0; i < PInst->nbRequests_; ++i) {
            std::string pickID = Tools::createNodeID(PInst->requests_[i]->getRequestId(), PICKUP);
            int pickIndex = PInst->instGraph_->nodeIDToInt_[pickID];

            std::string dropID = Tools::createNodeID(PInst->requests_[i]->getRequestId(), DROPOFF);
            int dropIndex = PInst->instGraph_->nodeIDToInt_[dropID];

            // constraints 6a -------------------
            IloExpr expr6(env);
            for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->type_ != SOURCE)
                    expr6 += (X[v][pickIndex][j] - X[v][dropIndex][j]);
            }
            MIPModel.add(expr6 == 0);

            // without following constraints some drop off points are before their pickups
            MIPModel.add(U[v][pickIndex]- U[v][dropIndex] <= 0 );


            // constraints 11a -------------------
            MIPModel.add(U[v][pickIndex] >= PInst->requests_[i]->earlyPick_);

            // constraints 12a -------------------
            IloExpr expr12(env);
            expr12 = U[v][dropIndex] - U[v][pickIndex] - PInst->requests_[i]->deltaTime_;
            /*float t = queryTravelTime(PInst->instGraph_->nodes_[pickID],
                                     PInst->instGraph_->nodes_[dropID]);*/
            float t = travelMat->queryTravelTime(PInst->instGraph_->nodes_[pickID],
                                      PInst->instGraph_->nodes_[dropID]);
            MIPModel.add(t <= expr12 <= std::max(alphaParam * t, betaParam + t));

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
                /*expr8 = U[v][i] + PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]->deltaTime_
                        + queryTravelTime(PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]],
                                         PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]])
                        - U[v][j] - 7200 * (1 - X[v][i][j]);*/
                expr8 = U[v][i] + PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]->deltaTime_
                        + travelMat->queryTravelTime(PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]],
                                          PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]])
                        - U[v][j] - 7200 * (1 - X[v][i][j]);
                MIPModel.add(expr8 <= 0);

                // constraints 14a -------------------
                IloExpr expr14(env);
                expr14 = W[v][i] + PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[j]]->nbPassengers_
                         - W[v][j] - 100 * (1 - X[v][i][j]);
                MIPModel.add(expr14 <= 0);

            }

            // constraints 15a -------------------
            MIPModel.add(W[v][i] <= PInst->vehicles_[v]->capacity_);


            /*if (PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]]->nodeStatus_ == PLANNED) {
                std::string onboardID = PInst->instGraph_->intToNodeID_[i];
                std::string pickNodeID = PInst->instGraph_->nodes_[onboardID]->pairNodeID_;

                // constraints 7a -------------------
                IloExpr expr7(env);
                for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                    expr7 += X[v][j][PInst->instGraph_->nodeIDToInt_[onboardID]];
                }
                MIPModel.add(expr7 == 1);

                // constraints 13a -------------------
                IloExpr expr13(env);
                expr13 = U[v][PInst->instGraph_->nodeIDToInt_[onboardID]] - PInst->instGraph_->nodes_[pickNodeID]->reachTime_
                         - PInst->instGraph_->nodes_[pickNodeID]->deltaTime_;

                float t = calcTravelTime(PInst->instGraph_->nodes_[onboardID],
                                         PInst->instGraph_->nodes_[pickNodeID]);
                MIPModel.add(t <= expr13 <= std::max(alphaParam * t, betaParam + t));
            }*/
        }
    }

    IloCplex MIPCplex(MIPModel);
    MIPCplex.setParam(IloCplex::TiLim, 1000);

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
            int sourceIndex = PInst->instGraph_->nodeIDToInt_[PInst->vehicles_[v]->departID_];
            int sinkIndex = PInst->instGraph_->nodeIDToInt_[PInst->vehicles_[v]->sinkID_];
            PRoute newRoute = std::make_shared<Route>(PInst->vehicles_[v]->vehicleID_);

            newRoute->addNode(PInst->instGraph_->nodes_[PInst->vehicles_[v]->departID_],
                              PInst->vehicles_[v]->departTime_, PInst->vehicles_[v]->numPassengers_);

            int currentNodeIndex = sourceIndex;
            while (currentNodeIndex != sinkIndex) {
                for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i) {
                    if (xVal[currentNodeIndex][i] > 0.9) {
                        newRoute->addNode(PInst->instGraph_->nodes_[PInst->instGraph_->intToNodeID_[i]],
                                          uVal[i], wVal[i]);
                        currentNodeIndex = i;
                        break;
                    }
                }
            }

            std::cout << newRoute->toString() << std::endl;
        }




        /*for (int i = 0; i < PInst->nbRequests_; ++i) {
            float Zval = MIPCplex.getValue(Z[i]);
            if (Zval > 0)
                std::cout << "Z[" << i <<"]: " << Zval << std::endl;
        }
        for (int i = 0; i < PInst->instGraph_->nbNodes_; ++i) {
            for (int v = 0; v < PInst->nbVehicles_; ++v) {
                float Uval = MIPCplex.getValue(U[v][i]);
                float Wval = MIPCplex.getValue(W[v][i]);
                std::cout << "W[" << v <<"][" << i <<"]: " << Wval << std::endl;
                std::cout << "U[" << v <<"][" << i <<"]: " << Uval << std::endl;
                for (int j = 0; j < PInst->instGraph_->nbNodes_; ++j) {
                    float Xval = MIPCplex.getValue(X[v][i][j]);
                    if (Xval > 0)
                        std::cout << "X[" << v <<"][" << i <<"][" << j <<"]: " << Xval << std::endl;
                }
            }
        }*/
    }
    catch (IloException& e) {
        std::cout << e << std::endl;
    }
    env.end();
}

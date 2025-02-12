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
void CPLEXSubProblem::initializeModel(PInstance &pInst) {
    pInst->setNodeIndices();
    // update order of requests
    nbNodes_ = subGraph_->nbNodes_;
    nbRequests_ = subGraph_->pickNodes_.size();

    // define variables
    X_ = IloNumVar2D(env_, nbNodes_);
    U_ = IloNumVarArray (env_, nbNodes_, 0, IloInfinity, ILOFLOAT);
    W_ = IloNumVarArray (env_, nbNodes_, 0, IloInfinity, ILOFLOAT);

    for (int i = 0; i < nbNodes_; ++i) {

        // Generate a name like "U_i"
        std::ostringstream uName;
        uName << "U_i" << i;
        U_[i].setName(uName.str().c_str());

        // Generate a name like "W_i"
        std::ostringstream wName;
        wName << "W_i" << i;
        W_[i].setName(wName.str().c_str());

        X_[i] = IloNumVarArray(env_, nbNodes_, 0.0, 1.0, ILOINT);
        for (int j = 0; j < nbNodes_; ++j) {
            // Generate a name like "X_v_i_j"
            std::ostringstream xName;
            xName << "X_i" << i << "_j" << j;
            X_[i][j].setName(xName.str().c_str());
        }
    }
}

void CPLEXSubProblem::buildModel(PInstance &pInst){
    int i,j,i_n;
    std::vector<PNode> nodes = concatenateVectors(pInst->instGraph_->pickNodes_, pInst->instGraph_->dropNodes_);

    // define objective function
    IloExpr objExpr(env_);
    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        objExpr += (U_[pickNode->nodeIndex_] - pickNode->readyTime_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            objExpr -= (X_[pickNode->nodeIndex_][j] * pickNode->related_Request_->dual_);
        }
    }

    objExpr -= (Vehicle_)->dual_;
    SubProModel_.add(IloMinimize(env_, objExpr));

    // -----------------------------------------
    // defining constraints
    // -----------------------------------------

    IloExpr expr_extract(env_);

    int sourceIndex = Vehicle_->departNode_->nodeIndex_;
    int sinkIndex = Vehicle_->sinkNode_->nodeIndex_;

    std::vector<PNode> nodesWithOnboards = nodes;
    for (auto & nodeID : Vehicle_->onboards_)
        nodesWithOnboards.push_back(pInst->instGraph_->nodes_[nodeID]);
    std::vector<PNode> allNodes = nodesWithOnboards;
    allNodes.push_back(Vehicle_->sinkNode_);
    allNodes.push_back(Vehicle_->departNode_);

    // constraints 1d 1e -------------------
    IloExpr expr_1d(env_);
    IloExpr expr_1e(env_);
    for (auto & otherNode : nodesWithOnboards) {
        expr_1d += X_[sourceIndex][otherNode->nodeIndex_];
        expr_1e += X_[otherNode->nodeIndex_][sinkIndex];

        expr_extract += X_[otherNode->nodeIndex_][sourceIndex];
        expr_extract += X_[sinkIndex][otherNode->nodeIndex_];
    }
    expr_1d += X_[sourceIndex][sinkIndex];
    expr_1e += X_[sourceIndex][sinkIndex];
    expr_extract += X_[sinkIndex][sourceIndex] + X_[sinkIndex][sinkIndex] +
        X_[sourceIndex][sourceIndex];

    IloRange c_1d = (expr_1d == 1);
    IloRange c_1e = (expr_1e == 1);

    SubProModel_.add(c_1d);
    SubProModel_.add(c_1e);

    // constraints 1i , 1j -------------------
    IloRange c_1i = (U_[sourceIndex] >= Vehicle_->departTime_);
    IloRange c_1j = (U_[sinkIndex] <= Vehicle_->endTime_);
    SubProModel_.add(c_1i);
    SubProModel_.add(c_1j);

    for (auto & pickNode : pInst->instGraph_->pickNodes_) {
        i = pickNode->nodeIndex_;
        i_n = pickNode->pairNode_->nodeIndex_;
        // constraints 1f -------------------
        IloExpr expr_1f(env_);
        for (auto & otherNode : nodesWithOnboards) {
            j = otherNode->nodeIndex_;
            expr_1f += (X_[i][j]  - X_[i_n][j]);
        }
        expr_1f +=  (X_[i][sinkIndex] - X_[i_n][sinkIndex]);
        IloRange c_1f = (expr_1f == 0);
        SubProModel_.add(c_1f);

        // constraints 1k -------------------
        IloRange c_1k = (U_[i] >= pickNode->readyTime_);
        SubProModel_.add(c_1k);

        IloRange c_1k_1 = (U_[i] <= pickNode->related_Request_->latestPickup_);
        SubProModel_.add(c_1k_1);

        IloRange c_1k_2 = (U_[i_n] >= (pickNode->readyTime_ +
            pickNode->related_Request_->minTravelTime_ + pickNode->serviceTime_));
        SubProModel_.add(c_1k_2);

        IloRange c_1k_3 = (U_[i_n] <= (pickNode->related_Request_->latestPickup_ +
            pickNode->related_Request_->maxTravelTime_ + pickNode->serviceTime_));
        SubProModel_.add(c_1k_3);


        // constraints 1l -------------------
        IloExpr expr_1l(env_);
        expr_1l = U_[i_n] - U_[i] - pickNode->serviceTime_;
        IloRange c_1l_1 = (expr_1l <= pickNode->related_Request_->maxTravelTime_);
        SubProModel_.add(c_1l_1);

        IloRange c_1l_2 = (expr_1l >= pickNode->related_Request_->minTravelTime_);
        SubProModel_.add(c_1l_2);

        expr_extract += X_[i_n][i] + X_[i][i] + X_[i_n][i_n];
    }

    for (auto & onboardID : Vehicle_->onboards_) {
        // constraints 1g -------------------
        IloExpr expr_1g(env_);
        PNode onNode = pInst->instGraph_->nodes_[onboardID];
        j = onNode->nodeIndex_;
        for (auto & otherNode : nodesWithOnboards) {
            if (otherNode->nodeIndex_ != j)
                expr_1g += X_[otherNode->nodeIndex_][j];
        }
        expr_1g += X_[sourceIndex][j];
        IloRange c_1g = (expr_1g == 1);
        SubProModel_.add(c_1g);

        // constraints 1m -------------------
        IloExpr expr_1m(env_);
        expr_1m = U_[j] -
                onNode->related_Request_->pickTime_ -
                onNode->related_Request_->serviceTime_;
        IloRange c_1m_1 = (expr_1m <= onNode->related_Request_->maxTravelTime_);
        SubProModel_.add(c_1m_1);

        IloRange c_1m_2 = (expr_1m >= onNode->related_Request_->minTravelTime_);
        SubProModel_.add(c_1m_2);

        expr_extract += X_[j][j];

        IloRange c_1k_4 = (U_[j] >= (onNode->related_Request_->pickTime_ +
            onNode->related_Request_->minTravelTime_ + onNode->related_Request_->serviceTime_));
        SubProModel_.add(c_1k_4);

        IloRange c_1k_5 = (U_[j] <= (onNode->related_Request_->latestPickup_ +
            onNode->related_Request_->maxTravelTime_ + onNode->serviceTime_));
        SubProModel_.add(c_1k_5);

    }

    // constraints 1c -------------------
    for (auto & node : nodesWithOnboards) {
        i = node->nodeIndex_;
        IloExpr expr_1c(env_);
        for (auto & otherNode : nodesWithOnboards) {
            j = otherNode->nodeIndex_;
            if (i != j) {
                expr_1c += (X_[i][j] - X_[j][i]);
            }
        }
        expr_1c += (X_[i][sinkIndex] - X_[sourceIndex][i]);
        IloRange c_1c = (expr_1c == 0);
        SubProModel_.add(c_1c);
    }


    for (auto & node : allNodes) {
        i = node->nodeIndex_;

        // constraints 1o -------------------
        IloRange c_1o_1 = (W_[i] <= Vehicle_->capacity_);
        SubProModel_.add(c_1o_1);

        IloRange c_1o_2 = (W_[i] >= 0);
        SubProModel_.add(c_1o_2);

        for (auto & otherNode : allNodes) {
            j = otherNode->nodeIndex_;
            if (i != j) {
                // constraints 1h -------------------
                IloExpr expr_1h(env_);
                expr_1h = U_[i] + node->serviceTime_ + durationMatrix_[node->locationID_][otherNode->locationID_]
                          - U_[j] - 61200 * (1 - X_[i][j]);
                IloRange c_1h = (expr_1h <= 0);
                SubProModel_.add(c_1h);

                // constraints 1n -------------------
                IloExpr expr_1n(env_);
                expr_1n = W_[i] + otherNode->nbPassengers_ - W_[j] - Vehicle_->capacity_ * (1 - X_[i][j]);
                IloRange c_1n = (expr_1n <= 0);
                SubProModel_.add(c_1n);
                objExpr += 0 * X_[i][j] * durationMatrix_[node->locationID_][otherNode->locationID_];
            }
        }
    }

    // add this constraint for re-optimization when the vehicle is not idle
    // at the start and have some passengers onboard
    IloRange c = (W_[sinkIndex] >= Vehicle_->numPassengers_);
    SubProModel_.add(c);
    SubProModel_.add(IloMinimize(env_, objExpr));

    IloRange extarct = (expr_extract == 0);
    SubProModel_.add(extarct);
}

void CPLEXSubProblem::solveModel(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    try {

        Cplex_ = IloCplex(SubProModel_);

        // std::ofstream logFile(inputPaths.getOutputCplexLog(), std::ofstream::app);
        // logFile << "----------------------- MIP SOlver ------------------------" << std::endl;
        // std::streambuf *coutBuffer = std::cout.rdbuf();
        // std::cout.rdbuf(logFile.rdbuf());

     //   Cplex_.setParam(IloCplex::Param::TimeLimit, 3600);
     //   Cplex_.setParam(IloCplex::Param::Threads, pInst->parameters_->nbThreads_);
    //    Cplex_.setParam(IloCplex::Param::Preprocessing::Presolve, 0);
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);
        Cplex_.setParam(IloCplex::Param::NodeAlgorithm, 2);
        Cplex_.setParam(IloCplex::Param::MIP::Limits::RepairTries, 10);
        Cplex_.setParam(IloCplex::Param::MIP::PolishAfter::Time, 50);


    //    Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, pInst->parameters_->MIPGap_);

        // Export model for manual inspection
    //    Cplex_.exportModel("model.lp");

        // Try solving the model
        if (!Cplex_.solve()) {
            std::cout << "Failed to optimize the MIP." << std::endl;

        }
        else {
            Vehicle_->bestReducedCost_ = Cplex_.getObjValue();
            std::cout << "The objective value: " << Cplex_.getObjValue() << std::endl;

            IloNumArray uVal(env_);
            IloNumArray wVal(env_);

            int nbNodes = nbRequests_ * 2 + 2 + Vehicle_->onboards_.size();
            IloNum2D xVal(env_, nbNodes);

            Cplex_.getValues(uVal, U_);
            Cplex_.getValues(wVal, W_);

            for (int i = 0; i < nbNodes; ++i) {
                xVal[i] = IloNumArray(env_);
                Cplex_.getValues(xVal[i], X_[i]);
            }

            // creating the route
            PRoute newRoute = std::make_shared<Route>(Vehicle_->vehicleID_);

            newRoute->addSource(Vehicle_->departNode_,Vehicle_->departTime_,
                                Vehicle_->numPassengers_);

            int currentNodeIndex = Vehicle_->departNode_->nodeIndex_;

            while (currentNodeIndex != Vehicle_->sinkNode_->nodeIndex_) {
                for (int i = 0; i < nbNodes_; ++i) {
                    if (xVal[currentNodeIndex][i] > 0.5) {
                        PNode nodeSelected = pInst->instGraph_->nodes_[getNode(pInst, Vehicle_->vehicleID_,i, nbRequests_)];
                        if (nodeSelected->nodeIndex_ != Vehicle_->sinkNode_->nodeIndex_)
                            newRoute->addNode(nodeSelected);
                        currentNodeIndex = i;
                        break;
                    }
                }
            }
            availableRoutes.push_back(newRoute);
        }
        /*std::cout.rdbuf(coutBuffer);
        logFile.close();*/
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error occurred at line: " << __LINE__ << std::endl;
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
    repStr << "# SUB PROBLEM SOLUTION RESULT FOR VEHICLE: " << (Vehicle_)->vehicleID_ << std::endl;
    repStr << "#" << std::endl;
    repStr << "# Solution status = " << Cplex_.getStatus() << std::endl;
    repStr << "# Incumbent objective value = " << Cplex_.getObjValue() << std::endl;
    repStr << "# The solution pool contains = " << Cplex_.getSolnPoolNsolns() << " solutions." << std::endl;
    repStr << "# " << Cplex_.getSolnPoolNreplaced() << " solutions were replaced in the solution pool " << std::endl;

    repStr << "# In total, " << Cplex_.getSolnPoolNsolns() + Cplex_.getSolnPoolNreplaced() <<
    " solutions were generated." << std::endl;

    int nbValidSolution = 0;
    for (int i = 0; i < Cplex_.getSolnPoolNsolns(); ++i) {
        if (Cplex_.getObjValue(i) < 0)
            nbValidSolution ++;
    }
    repStr << "# The solution pool contains = " << nbValidSolution << " solutions with negative reduced cost." << std::endl;
    return repStr.str();
}





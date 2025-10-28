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
    nbGenerated_ = 0;
}
CPLEXSubProblem::~CPLEXSubProblem() {
    env_.end();
}

//************************************************************************
// Build the SUb Problem model with CPLEX
//************************************************************************
void CPLEXSubProblem::initializeModel(PInstance &pInst) {
    setNodeIndices();
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
    std::vector<PNode> nodes = concatenateVectors(subGraph_->pickNodes_, subGraph_->dropNodes_);

    // define objective function
    IloExpr objExpr(env_);
    for (auto & pickNode : subGraph_->pickNodes_) {
        objExpr += (U_[pickNode->nodeIndex_] - pickNode->readyTime_);
        for (int j = 0; j < subGraph_->nbNodes_; ++j) {
            objExpr -= (X_[j][pickNode->nodeIndex_]* pickNode->related_Request_->dual_);
        }
    }

    objExpr -= (Vehicle_)->dual_;
//    SubProModel_.add(IloMinimize(env_, objExpr));

    // -----------------------------------------
    // defining constraints
    // -----------------------------------------

    IloExpr expr_extract(env_);

    int sourceIndex = Vehicle_->departNode_->nodeIndex_;
    int sinkIndex = Vehicle_->sinkNode_->nodeIndex_;

    std::vector<PNode> nodesWithOnboards = nodes;
    for (auto & nodeID : Vehicle_->onboards_)
        nodesWithOnboards.push_back(subGraph_->nodes_[nodeID]);
    std::vector<PNode> allNodes = nodesWithOnboards;
    allNodes.push_back(Vehicle_->sinkNode_);
    allNodes.push_back(Vehicle_->departNode_);

    for (auto & nodeObj : nodesWithOnboards) {
        if (nodeObj->type_ != SINK) {
            int fromIndex = nodeObj->nodeIndex_;

            for (auto &pickNodeObj : subGraph_->pickNodes_) {
                if (nodeObj->nodeID_ != pickNodeObj->nodeID_) {
                    int toIndex = pickNodeObj->nodeIndex_;

                    // Apply the same pruning logic from sortSuccessors
                    float minWait = (Vehicle_)->departTime_ +
                                   nodeObj->travelTimeFromSource_ +
                                   nodeObj->serviceTime_ +
                                   durationMatrix_[nodeObj->locationID_][pickNodeObj->locationID_] -
                                   pickNodeObj->readyTime_;

                    if (minWait > pickNodeObj->related_Request_->penalty_) {
                        // Mark this arc as pruned
                        expr_extract += (X_[fromIndex][toIndex]);
                    }
                }
            }
        }
    }




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

    for (auto & pickNode : subGraph_->pickNodes_) {
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
        PNode onNode = subGraph_->nodes_[onboardID];
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

        // Tighten capacity bounds
        if (node->type_ == PICKUP) {
            W_[i].setBounds(node->nbPassengers_, Vehicle_->capacity_);
        } else if (node->type_ == DROPOFF) {
            W_[i].setBounds(0, Vehicle_->capacity_);
        }

        // Tighten U bounds for pickup nodes
        if (node->type_ == PICKUP) {
            // Tighten U bounds for pickup nodes
            double minTime = std::max(node->readyTime_,
                                     Vehicle_->departTime_ +
                                     durationMatrix_[Vehicle_->departNode_->locationID_][node->locationID_]);
            double maxTime = std::min(node->related_Request_->latestPickup_,
                                     Vehicle_->endTime_ - node->serviceTime_);

            if (minTime < maxTime) {
                U_[i].setBounds(minTime, maxTime);
            }
        }

        for (auto & otherNode : allNodes) {
            j = otherNode->nodeIndex_;
            if (i != j) {
                // constraints 1h -------------------
                IloExpr expr_1h(env_);
                expr_1h = U_[i] + node->serviceTime_ + durationMatrix_[node->locationID_][otherNode->locationID_]
                          - U_[j] - 86400 * (1 - X_[i][j]);
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

// Updated solveModel method
void CPLEXSubProblem::solveModel(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    try {
        Cplex_ = IloCplex(SubProModel_);

        // Initialize tracking variables
        bestObjectiveValue_ = 1e9;  // Large positive value
        solutionFound_ = false;

        // Output settings
        Cplex_.setOut(env_.getNullStream());  // Disable output for speed
        Cplex_.setOut(std::cout);  // Enable this for debugging

        // CRITICAL: For column generation subproblems, you often need optimal or near-optimal solutions
        // Set tighter parameters for better solutions
        Cplex_.setParam(IloCplex::Param::TimeLimit, 1000);  // 30 seconds should be enough
        Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.01);  // 0.01% gap for near-optimal

        // Algorithm settings
        Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);  // Dual simplex
        Cplex_.setParam(IloCplex::Param::NodeAlgorithm, 2);  // Dual simplex
        Cplex_.setParam(IloCplex::Param::Threads, 8);

        // Emphasis on optimality for column generation
    //    Cplex_.setParam(IloCplex::Param::Emphasis::MIP, 2);  // Emphasize optimality

        // Preprocessing
 //       Cplex_.setParam(IloCplex::Param::Preprocessing::Reduce, 3);  // Aggressive
 //       Cplex_.setParam(IloCplex::Param::MIP::Strategy::Probe, 3);   // Aggressive probing

        // Cuts - moderate settings
        Cplex_.setParam(IloCplex::Param::MIP::Cuts::Gomory, 1);

        // Node selection strategy
        Cplex_.setParam(IloCplex::Param::MIP::Strategy::NodeSelect, 2);  // Best estimate

    //    addSmartMIPStart();

        // Solve for optimal/near-optimal solution first
        if (!Cplex_.solve()) {
            std::cout << "Failed to optimize the subproblem for vehicle "
                      << Vehicle_->vehicleID_ << std::endl;
            std::cout << "Status: " << Cplex_.getStatus() << std::endl;
            return;
        }

        // Store the best objective value
        bestObjectiveValue_ = Cplex_.getObjValue();
        Vehicle_->bestReducedCost_ = bestObjectiveValue_;
        solutionFound_ = true;

        std::cout << "Vehicle " << Vehicle_->vehicleID_
                  << " - Best objective: " << bestObjectiveValue_
                  << " (Status: " << Cplex_.getStatus() << ")" << std::endl;

        // Extract the optimal solution
        extractSingleSolution(pInst, availableRoutes, -1);

        // Only look for additional solutions if we found negative reduced cost
        if (bestObjectiveValue_ < -0.001) {
            // Solution pool settings for multiple solutions
            Cplex_.setParam(IloCplex::Param::MIP::Pool::Capacity, 20);
            Cplex_.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.5);  // 50% gap from best for pool
            Cplex_.setParam(IloCplex::Param::MIP::Pool::Intensity, 2);
            Cplex_.setParam(IloCplex::Param::MIP::Pool::Replace, 1);
            Cplex_.setParam(IloCplex::Param::MIP::Limits::Populate, 100);  // Limit populate effort

            // Shorter time limit for populate
            Cplex_.setParam(IloCplex::Param::TimeLimit, 500);

            try {
                Cplex_.populate();
                extractPoolSolutions(pInst, availableRoutes);
            } catch (IloException& e) {
                std::cout << "Warning: Could not populate solution pool: " << e << std::endl;
            }
        }

        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error in solveModel: " << e << std::endl;
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


// Method to extract pool solutions
void CPLEXSubProblem::extractPoolSolutions(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    int maxRoutes = 10;
    int routesExtracted = 1;  // Already extracted optimal

 //   std::cout << "Solution pool contains " << Cplex_.getSolnPoolNsolns() << " solutions" << std::endl;

    for (int s = 0; s < Cplex_.getSolnPoolNsolns() && routesExtracted < maxRoutes; ++s) {
        double objValue = Cplex_.getObjValue(s);

        // Only extract solutions with negative reduced cost
        if (objValue < -0.001 && objValue > bestObjectiveValue_ + 0.001) {
            extractSingleSolution(pInst, availableRoutes, s);
            routesExtracted++;
        }
    }

 //   std::cout << "Extracted " << routesExtracted << " routes with negative reduced cost" << std::endl;
}

// Updated extraction method
void CPLEXSubProblem::extractSingleSolution(PInstance &pInst, std::vector<PRoute> &availableRoutes, int solutionIndex) {
    try {
        IloNumArray uVal(env_);
        IloNumArray wVal(env_);

        int nbNodes = nbRequests_ * 2 + 2 + Vehicle_->onboards_.size();
        IloArray<IloNumArray> xVal(env_, nbNodes);

        double objValue;

        // Get values for specific solution
        if (solutionIndex == -1) {  // Incumbent solution
            Cplex_.getValues(uVal, U_);
            Cplex_.getValues(wVal, W_);
            objValue = Cplex_.getObjValue();

            for (int i = 0; i < nbNodes; ++i) {
                xVal[i] = IloNumArray(env_);
                Cplex_.getValues(xVal[i], X_[i]);
            }
        } else {  // Solution from pool
            Cplex_.getValues(uVal, U_, solutionIndex);
            Cplex_.getValues(wVal, W_, solutionIndex);
            objValue = Cplex_.getObjValue(solutionIndex);

            for (int i = 0; i < nbNodes; ++i) {
                xVal[i] = IloNumArray(env_);
                Cplex_.getValues(xVal[i], X_[i], solutionIndex);
            }
        }

        // Create route
        PRoute newRoute = std::make_shared<Route>(Vehicle_->vehicleID_);
        newRoute->reducedCost_ = objValue;

        // Build the route
        newRoute->addSource(Vehicle_->departNode_, Vehicle_->departTime_,
                           Vehicle_->numPassengers_);

        int currentNodeIndex = Vehicle_->departNode_->nodeIndex_;
        int maxIterations = nbNodes_;  // Prevent infinite loops
        int iterations = 0;

        while (currentNodeIndex != Vehicle_->sinkNode_->nodeIndex_ && iterations < maxIterations) {
            bool found = false;
            for (int i = 0; i < nbNodes_; ++i) {
                if (xVal[currentNodeIndex][i] > 0.5) {
                    PNode nodeSelected = pInst->instGraph_->nodes_[
                        getNode(pInst, Vehicle_->vehicleID_, i, nbRequests_)];

                    if (nodeSelected->nodeIndex_ != Vehicle_->sinkNode_->nodeIndex_) {
                        newRoute->addNode(nodeSelected);
                    }

                    currentNodeIndex = i;
                    found = true;
                    break;
                }
            }

            if (!found) {
                std::cout << "Warning: Route construction failed - no outgoing arc from node "
                          << currentNodeIndex << std::endl;
                return;
            }

            iterations++;
        }

        if (iterations >= maxIterations) {
            std::cout << "Warning: Route construction failed - maximum iterations reached" << std::endl;
            return;
        }
        newRoute->calculateTripDelay(pInst->parameters_->Wait_W1_, pInst->parameters_->Ride_W2_);

        availableRoutes.push_back(newRoute);

        // Clean up
        for (int i = 0; i < nbNodes; ++i) {
            xVal[i].end();
        }
        uVal.end();
        wVal.end();
    }
    catch (IloException& e) {
        std::cout << "Error in solution extraction: " << e << std::endl;
    }
}

void CPLEXSubProblem::solveForColumnGeneration(PInstance &pInst, std::vector<PRoute> &availableRoutes) {
    try {
        Cplex_ = IloCplex(SubProModel_);
        // Output settings
        Cplex_.setOut(env_.getNullStream());  // Disable output for speed
        Cplex_.setOut(std::cout);  // Enable this for debugging

        // Special parameters for column generation subproblems
    //    Cplex_.setParam(IloCplex::Param::MIP::Tolerances::UpperCutoff, -0.01);
        Cplex_.setParam(IloCplex::Param::Threads, 8);

        // First, try to find ANY negative reduced cost solution quickly
        Cplex_.setParam(IloCplex::Param::TimeLimit, 50);  // Quick attempt
  //      Cplex_.setParam(IloCplex::Param::Emphasis::MIP, 1);  // Feasibility
        Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.5);  // Allow 50% gap
  //      addSmartMIPStart();
        double startTime = Cplex_.getCplexTime();
        bool foundNegative = false;
        if (Cplex_.solve()) {
            foundNegative = true;
            extractSingleSolution(pInst, availableRoutes, -1);
            bestObjectiveValue_ = Cplex_.getObjValue();

            // Now try to find better solutions
            Cplex_.setParam(IloCplex::Param::TimeLimit, 1000);  // More time
            Cplex_.setParam(IloCplex::Param::Emphasis::MIP, 2);  // Optimality
            Cplex_.setParam(IloCplex::Param::MIP::Tolerances::MIPGap, 0.01);  // Tighter gap
            Cplex_.setParam(IloCplex::Param::Preprocessing::Reduce, 3);  // Aggressive
            // Algorithm settings
            Cplex_.setParam(IloCplex::Param::RootAlgorithm, 2);  // Dual simplex
            Cplex_.setParam(IloCplex::Param::NodeAlgorithm, 2);  // Dual simplex

            // Node selection strategy
            Cplex_.setParam(IloCplex::Param::MIP::Strategy::NodeSelect, 2);  // Best estimate

            // Continue solving from current state
            if (Cplex_.solve()) {
                if (Cplex_.getObjValue() < availableRoutes.back()->reducedCost_) {
  //                  availableRoutes.clear();  // Replace with better solution
                    extractSingleSolution(pInst, availableRoutes, -1);
                    bestObjectiveValue_ = Cplex_.getObjValue();
                }
            }
        }

        // If we found negative reduced cost, populate pool
        double elapsedTime = Cplex_.getCplexTime() - startTime;
        if (foundNegative) {
            Cplex_.setParam(IloCplex::Param::MIP::Pool::Capacity, 10);
            Cplex_.setParam(IloCplex::Param::MIP::Pool::RelGap, 0.5);
            Cplex_.setParam(IloCplex::Param::TimeLimit, 500);

            Cplex_.populate();
            extractPoolSolutions(pInst, availableRoutes);
        }
        std::cout << "Vehicle: " << Vehicle_->vehicleID_ << " objective: " << bestObjectiveValue_ << " runtime:" << elapsedTime << std::endl;
        Cplex_.clearModel();
    }
    catch (IloException& e) {
        std::cout << "Error: " << e << std::endl;
    }
}


void CPLEXSubProblem::setInitialIncumbent() {
    try {
        // Create arrays for the solution
        IloNumArray xStart(env_);
        IloNumArray uStart(env_);
        IloNumArray wStart(env_);

        int nbNodes = nbRequests_ * 2 + 2 + Vehicle_->onboards_.size();

        // Initialize all variables
        for (int i = 0; i < nbNodes_; ++i) {
            for (int j = 0; j < nbNodes_; ++j) {
                xStart.add(0.0);
            }
        }

        // Create a simple feasible solution: source -> sink
        int sourceIdx = Vehicle_->departNode_->nodeIndex_;
        int sinkIdx = Vehicle_->sinkNode_->nodeIndex_;

        // Set X variable for direct route
        xStart[sourceIdx * nbNodes_ + sinkIdx] = 1.0;

        // Set time variables
        for (int i = 0; i < nbNodes_; ++i) {
            if (i == sourceIdx) {
                uStart.add(Vehicle_->departTime_);
            } else if (i == sinkIdx) {
                uStart.add(Vehicle_->departTime_ + 1);  // Just after start
            } else {
                uStart.add(0.0);
            }
        }

        // Set load variables
        for (int i = 0; i < nbNodes_; ++i) {
            wStart.add(Vehicle_->numPassengers_);
        }

        // Create solution object
        IloCplex::MIPStartEffort effort = IloCplex::MIPStartNoCheck;  // Skip feasibility check

        // Add the MIP start
        Cplex_.addMIPStart(IloNumVarArray(env_), IloNumArray(env_), effort, "DummyStart");

        // Alternatively, set a solution value directly
       // Cplex_.setParam(IloCplex::Param::MIP::Strategy::StartAlgorithm, 1);

    } catch (IloException& e) {
        std::cout << "Could not set initial incumbent: " << e << std::endl;
    }
}

void CPLEXSubProblem::addSimpleMIPStart() {
    try {
        IloNumVarArray startVars(env_);
        IloNumArray startVals(env_);

        // Only set the critical variables for a simple feasible path
        int sourceIdx = Vehicle_->departNode_->nodeIndex_;
        int sinkIdx = Vehicle_->sinkNode_->nodeIndex_;

        // Just specify the direct arc from source to sink
        startVars.add(X_[sourceIdx][sinkIdx]);
        startVals.add(1.0);

        // Set arrival times for source and sink only
        startVars.add(U_[sourceIdx]);
        startVals.add(Vehicle_->departTime_);


        // Set load variables
        startVars.add(W_[sourceIdx]);
        startVals.add(Vehicle_->numPassengers_);

        startVars.add(W_[sinkIdx]);
        startVals.add(Vehicle_->numPassengers_);

        // Let CPLEX complete the rest
        Cplex_.addMIPStart(startVars, startVals, IloCplex::MIPStartAuto, "PartialStart");

        startVars.end();
        startVals.end();

    } catch (IloException& e) {
        // Silently ignore if it fails
    }
}

void CPLEXSubProblem::addSmartMIPStart() {
    try {
        IloNumVarArray startVars(env_);
        IloNumArray startVals(env_);

        // Find the best request to include based on dual values
        int pickupIdx = -1;
        int dropoffIdx = -1;
        double bestDualValue = -1e9;

        // Check all requests
        for (auto & pickNode : subGraph_->pickNodes_) {

            // Check if request is feasible (time windows, capacity)
            if (pickNode->related_Request_->dual_ > bestDualValue) {
                bestDualValue = pickNode->related_Request_->dual_;
                pickupIdx = pickNode->nodeIndex_;
                dropoffIdx = pickNode->pairNode_->nodeIndex_;
            }
        }

        if (pickupIdx == -1) {
            // No feasible request, fall back to direct route
            addSimpleMIPStart();
            return;
        }

        // Build route: source -> pickup -> dropoff -> sink
        int sourceIdx = Vehicle_->departNode_->nodeIndex_;
        int sinkIdx = Vehicle_->sinkNode_->nodeIndex_;

        // Set X variables for the path
        startVars.add(X_[sourceIdx][pickupIdx]);
        startVals.add(1.0);

        startVars.add(X_[pickupIdx][dropoffIdx]);
        startVals.add(1.0);

        startVars.add(X_[dropoffIdx][sinkIdx]);
        startVals.add(1.0);


        // Set W variables (load)
        startVars.add(W_[sourceIdx]);
        startVals.add(Vehicle_->numPassengers_);


        // Add the MIP start
        Cplex_.addMIPStart(startVars, startVals, IloCplex::MIPStartAuto, "SmartStart");


        startVars.end();
        startVals.end();

    } catch (IloException& e) {
        std::cout << "Warning: Could not add smart MIP start: " << e.getMessage() << std::endl;
        // Fall back to simple start
        addSimpleMIPStart();
    }
}



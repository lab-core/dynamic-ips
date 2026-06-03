//
// Created by Elahe Amiri on 2025-07-27.
//

#include "LagrangianSolver.h"

LagrangianSolver::LagrangianSolver(const PInstance &pInst, float objValue,
    std::vector<PRoute> &routeSolution, std::vector<PRequest> & zSolution, vector2D<PRoute> &availableRoutes) {
    // Initialize dual bounds
    lambda_lo_ = std::vector<double>(pInst->nbRequests_, 0.0);  // default: 0
    lambda_hi_ = std::vector<double>(pInst->nbRequests_, 1e6);  // or p_i if passed later
    mu_lo_ = std::vector<double>(pInst->nbVehicles_, -1e6);
    mu_hi_ = std::vector<double>(pInst->nbVehicles_, 0.0);       // optional cap

    reqLambda_.assign(pInst->nbRequests_, 0.0);  // or p_i/2 if you have penalties available
    vehMu_.assign(pInst->nbVehicles_, 0.0);

    // create the list of routes
    nbRoutes_ = 0;
    routeList_.clear();
    LambdaMax_ = 0.0;
    for (auto & vehicleObj : pInst->vehicles_) {
        routeList_.push_back(vehicleObj->currentRoute_);
        nbRoutes_++;
        for (auto & routeObj : availableRoutes[vehicleObj->vehicleID_]) {
            routeList_.push_back(routeObj);
            nbRoutes_++;
            if (routeObj->totalWait_ > LambdaMax_)
                LambdaMax_ = routeObj->totalWait_;
        }
    }
    primal_value_ = 0.0;
    for (auto & requestObj : pInst->requests_) {
        reqLambda_[requestObj->taskIndex_] = requestObj->penalty_;
        primal_value_ += requestObj->penalty_;
        if (requestObj->penalty_ > LambdaMax_)
            LambdaMax_ = requestObj->penalty_;
    }
    LambdaMax_ *=2;


    // Diagonal scaling (can be initialized with heuristics later)
    d_lambda_ = std::vector<double>(pInst->nbRequests_, 1.0);
    d_mu_ = std::vector<double>(pInst->nbVehicles_, 1.0);

    // Defaults for control values
    gamma_ = 0.1;
    theta_ = 0.2;
    alpha_ = 0.0;
    eps_rc_ = 1e-8;
    eps_gap_ = 0.01;

    // Bounds and iteration count
    dual_value_ = -1e100;
    primal_value_ = objValue;
    best_dual_ = -1e100;
    lagrangeIter_ = 0;
    max_iter_ = 3000;

    // Initialize stagnation detection parameters
    previous_best_dual_ = -1e100;
    stagnation_count_ = 0;
    max_stagnation_iterations_ = 5;  // Stop after 50 iterations without improvement
    lb_improvement_threshold_ = 1e-6; // Minimum improvement threshold (relative)```````````````````````````````````
}

void LagrangianSolver::updateReducedCostsLagrange() {

    for (auto & routeObj : routeList_){
        routeObj->reducedCost_ = routeObj->totalWait_ - vehMu_[routeObj->vehicleID_];
        for (auto & nodeObj: routeObj->routeNodes_) {
            if (nodeObj->type_ == PICKUP){
                routeObj->reducedCost_ -= reqLambda_[nodeObj->related_Request_->taskIndex_];
            }
        }
    }
}

void LagrangianSolver::solveLRAtMultipliers(const PInstance& pInst) {
    routeLSolution_.clear();
    zLSolution_.clear();
    // y_r
    for (auto &routeObj : routeList_) {
        if (routeObj->reducedCost_ < 0.0) {
            routeLSolution_.push_back(routeObj);
        }
    }
    // z_i
    for (auto & requestObj : pInst->requests_) {
        if (requestObj->committedPickTime_ == LARGE_CONSTANT) {
            if (requestObj->penalty_ - reqLambda_[requestObj->taskIndex_] < 0)
                zLSolution_.push_back(requestObj);
        }
    }
}

void LagrangianSolver::computeSubgradients(const PInstance& pInst) {
    // Initialize subgradients
    reqGradient_ = std::vector<double>(pInst->nbRequests_, 1.0);   // start at 1 for each request
    vehGradient_ = std::vector<double>(pInst->nbVehicles_, 1.0);  // start at -1 for each vehicle

    for (const auto& reqObj : zLSolution_) {
        reqGradient_[reqObj->taskIndex_] --;
    }

    for (auto & routeObj : routeLSolution_) {
        vehGradient_[routeObj->vehicleID_]--;
        for (const auto& reqObj : routeObj->routeRequests_) {
            if (routeObj->column_.test(reqObj->taskIndex_)) {
                reqGradient_[reqObj->taskIndex_] --;
            }
        }
    }
}


void LagrangianSolver::evaluateDual() {
    dual_value_ = 0.0;

    // sum_i lambda_i
    for (double li : reqLambda_) dual_value_ += li;

    // - sum_v mu_v
    for (double mv : vehMu_) dual_value_ += mv;

    // sum_r min(0, rc_r)
    for (auto &routeObj : routeLSolution_) {
        dual_value_ += routeObj->reducedCost_;
    }

    // sum over uncommitted i of min(0, p_i - lambda_i)
    for (auto &req : zLSolution_) {
        dual_value_ += req->penalty_ - reqLambda_[req->taskIndex_];
    }

    if (dual_value_ > best_dual_) best_dual_ = dual_value_;
}

void LagrangianSolver::dualStep(const PInstance& pInst) {
    const double gap = std::max(0.0, primal_value_ - best_dual_);
    // Weighted subgradient norm (D^{1/2} g)
    double denom = 1e-12;
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        denom += d_lambda_[i] * reqGradient_[i] * reqGradient_[i];
    }
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        denom += d_mu_[v] * vehGradient_[v] * vehGradient_[v];
    }

    alpha_ = (denom > 0.0) ? (theta_ * gap / denom) : 0.0;

    // Update and project λ
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        reqLambda_[i] += alpha_ * d_lambda_[i] * reqGradient_[i];
        if (reqLambda_[i] < lambda_lo_[i]) reqLambda_[i] = lambda_lo_[i];
        if (reqLambda_[i] > lambda_hi_[i]) reqLambda_[i] = lambda_hi_[i];
    }
    // Update and project μ
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        vehMu_[v] += alpha_ * d_mu_[v] * vehGradient_[v];
        if (vehMu_[v] < mu_lo_[v]) vehMu_[v] = mu_lo_[v];
        if (vehMu_[v] > mu_hi_[v]) vehMu_[v] = mu_hi_[v];
    }
}

void LagrangianSolver::dualStepLR(const PInstance& pInst) {
    const double gap = std::max(0.0, primal_value_ - best_dual_);
    // Weighted subgradient norm (D^{1/2} g)
    double normGradient = 0.0;
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        normGradient += reqGradient_[i] * reqGradient_[i];
    }
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        normGradient += vehGradient_[v] * vehGradient_[v];
    }

    alpha_ = theta_ * gap / (normGradient + 1e-8);

    // Update and project λ
    for (int i = 0; i < pInst->nbRequests_; ++i) {
        reqLambda_[i] += alpha_ * reqGradient_[i];
        if (reqLambda_[i] < lambda_lo_[i]) reqLambda_[i] = lambda_lo_[i];
        if (reqLambda_[i] > lambda_hi_[i]) reqLambda_[i] = lambda_hi_[i];
    }
    // Update and project μ
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        vehMu_[v] += alpha_ * vehGradient_[v];
        if (vehMu_[v] < mu_lo_[v]) vehMu_[v] = mu_lo_[v];
        if (vehMu_[v] > mu_hi_[v]) vehMu_[v] = mu_hi_[v];
    }
}

void LagrangianSolver::initProjectionBoxes(const PInstance& pInst) {
    for (auto &req : pInst->requests_) {
        int i = req->taskIndex_;
        if (req->committedPickTime_ == LARGE_CONSTANT) {
            // uncommitted: 0 ≤ λ_i ≤ p_i
            lambda_lo_[i] = 0.0;
            lambda_hi_[i] = req->penalty_;
        } else {
            // committed: λ_i free (equality constraint in MP), project to symmetric box
            lambda_lo_[i] = 0.0;
            lambda_hi_[i] = req->penalty_;
        }
    }
    for (int v = 0; v < pInst->nbVehicles_; ++v) {
        mu_lo_[v] = -1e8;             // ≤ 1 constraint ⇒ μ_v ≥ 0
        mu_hi_[v] = 0.0;             // cap for stability
    }
}

void LagrangianSolver::initializeErgodicDuals(const PInstance &pInst) {
    ergodic_lambda_.assign(pInst->nbRequests_, 0.0);
    ergodic_mu_.assign(pInst->nbVehicles_, 0.0);
    ergodic_count_ = 0;
}

void LagrangianSolver::updateErgodicDuals() {
    ergodic_count_++;

    if (ergodic_count_ == 1) {
        // First iteration - just copy
        ergodic_lambda_ = reqLambda_;
        ergodic_mu_ = vehMu_;
    } else {
        // Update running average
        double weight = 1.0 / ergodic_count_;
        for (int i = 0; i < reqLambda_.size(); ++i) {
            ergodic_lambda_[i] = (1.0 - weight) * ergodic_lambda_[i] + weight * reqLambda_[i];
        }
        for (int v = 0; v < vehMu_.size(); ++v) {
            ergodic_mu_[v] = (1.0 - weight) * ergodic_mu_[v] + weight * vehMu_[v];
        }
    }
}


bool LagrangianSolver::iterateOnce(const PInstance& pInst) {
    // 1) Reduced costs
    updateReducedCostsLagrange();

    // 2) Best response (y,z)
    solveLRAtMultipliers(pInst);

    // 3) Subgradients
    computeSubgradients(pInst);

    // 4) Dual value (lower bound L^k)
    evaluateDual();

    // 5) Check for lower bound improvement
    double current_best = std::max(best_dual_, dual_value_);
    double improvement = 0.0;

    if (previous_best_dual_ > -1e90) {  // Not the first iteration
        improvement = (current_best - previous_best_dual_) / std::max(1e-12, std::abs(previous_best_dual_));

        if (improvement < lb_improvement_threshold_) {
            stagnation_count_++;
        } else {
            stagnation_count_ = 0;  // Reset counter if we see improvement
        }
    }

    previous_best_dual_ = current_best;
    best_dual_ = current_best;

    // 6) Dual step (Polyak) towards U* = objValue_
    dualStepLR(pInst);  // uses objValue_ and dual_value_ internally
 //   adam_optimizer_.updateDuals(reqLambda_, vehMu_, reqGradient_, vehGradient_,
 //                              lambda_lo_, lambda_hi_, mu_lo_, mu_hi_);


    ++lagrangeIter_;
    // Termination check on lower-bound gap
    const double gap = std::max(0.0, primal_value_ - dual_value_) / primal_value_;
    best_dual_ = std::max(best_dual_, dual_value_);

    // Print iteration info including duality gap
    /*std::cout << "Iter: " << lagrangeIter_
              << " | UB: " << objValue_
              << " | LB: " << best_dual_
              << " | Gap: " << gap
              << " | Improvement: " << improvement
              << " | Stagnation: " << stagnation_count_ << "/" << max_stagnation_iterations_
              << std::endl;*/

    return (gap <= eps_gap_) ||
           (lagrangeIter_ >= max_iter_) ||
           (stagnation_count_ >= max_stagnation_iterations_);
}

void LagrangianSolver::run(const PInstance& pInst) {
    // Optional: seed projection boxes and duals
    initProjectionBoxes(pInst);
    // (Optional) warm start duals here if you want

    while (!iterateOnce(pInst)) {
    }
    for (auto &reqObj : pInst->requests_) {
        /*std::cout << "initial: " << std::setw(9) << reqObj->InitialDual_ << " | "
                  << "barrier: " << std::setw(9) << reqObj->dual_ << " | "
                  << "lagrange: "<< std::setw(9) << reqLambda_[reqObj->taskIndex_] << " | " <<
                      reqGradient_[reqObj->taskIndex_] << " | " << reqObj->committedPickTime_ << std::endl;*/
        reqObj->dual_ = reqLambda_[reqObj->taskIndex_];
    }
    for (auto &vehObj : pInst->vehicles_) {
        vehObj->dual_ = 0;
 //       vehObj->dual_ = vehMu_[vehObj->vehicleID_];
 //       std::cout << "initial: " << vehObj->InitialDual_ << " - " << "current: " << vehObj->dual_ << std::endl;
    }
}

void LagrangianSolver::runADAM(const PInstance &pInst) {
    initProjectionBoxes(pInst);

    // Initialize ADAM
    adam_optimizer_.initialize(pInst->nbRequests_, pInst->nbVehicles_);

    // You might want to set learning rate based on problem size
    double base_lr = 10.0;  // Tune this - start higher for Lagrangian
    adam_optimizer_.setLearningRate(base_lr);

    while (!iterateOnce(pInst)) {
        // Use ADAM instead of regular dual step
        // In iterateOnce(), replace dualStepLR(pInst) with:
        // dualStepADAM(pInst);
    }

    // Assign duals as before
    for (auto &reqObj : pInst->requests_) {
        std::cout << "initial: " << std::setw(9) << reqObj->InitialDual_ << " | "
                  << "barrier: " << std::setw(9) << reqObj->dual_ << " | "
                  << "lagrange: "<< std::setw(9) << reqLambda_[reqObj->taskIndex_] << " | " <<
                      reqGradient_[reqObj->taskIndex_] << " | " << reqObj->committedPickTime_ << std::endl;
        reqObj->dual_ = reqLambda_[reqObj->taskIndex_];
    }
}


void LagrangianSolver::runErgodic(const PInstance& pInst) {
    initProjectionBoxes(pInst);
    initializeErgodicDuals(pInst);  // Add this

    while (!iterateOnce(pInst)) {
        updateErgodicDuals();  // Add this after dual update
    }

    // At the end, use ergodic duals instead of raw duals:
    for (auto &reqObj : pInst->requests_) {
        std::cout << "initial: " << std::setw(9) << reqObj->InitialDual_ << " | "
                  << "barrier: " << std::setw(9) << reqObj->dual_ << " | "
                  << "lagrange: " << std::setw(9) << reqLambda_[reqObj->taskIndex_] << " | "
                  << "ergodic: " << std::setw(9) << ergodic_lambda_[reqObj->taskIndex_] << " | " <<
                  reqGradient_[reqObj->taskIndex_] << " | " << reqObj->committedPickTime_ << std::endl;

        // USE ERGODIC DUALS FOR COLUMN GENERATION:
        reqObj->dual_ = ergodic_lambda_[reqObj->taskIndex_];
    }

    // Similar for vehicles if you use vehicle duals
    for (auto &vehObj : pInst->vehicles_) {
        vehObj->dual_ = 0;  // or use ergodic_mu_[vehObj->vehicleID_] if needed
    }
}

//
// Created by Elahe Amiri on 2025-07-27.
//

#ifndef LAGRANGIAN_H
#define LAGRANGIAN_H

#include <algorithm>
#include "utilities/MyTools.h"
#include "data/Instance.h"

class ADAMOptimizer {
private:
    // ADAM parameters
    double beta1_ = 0.9;      // Momentum decay
    double beta2_ = 0.999;    // RMSprop decay
    double epsilon_ = 1e-8;   // Numerical stability
    double learning_rate_ = 0.01;  // Initial learning rate

    // First and second moment estimates
    std::vector<double> m_lambda_;  // First moment (momentum) for lambda
    std::vector<double> v_lambda_;  // Second moment (RMSprop) for lambda
    std::vector<double> m_mu_;      // First moment for mu
    std::vector<double> v_mu_;      // Second moment for mu

    int t_ = 0;  // Time step

public:
    void initialize(int nbRequests, int nbVehicles) {
        m_lambda_.assign(nbRequests, 0.0);
        v_lambda_.assign(nbRequests, 0.0);
        m_mu_.assign(nbVehicles, 0.0);
        v_mu_.assign(nbVehicles, 0.0);
        t_ = 0;
    }

    void updateDuals(std::vector<double>& reqLambda,
                     std::vector<double>& vehMu,
                     const std::vector<double>& reqGradient,
                     const std::vector<double>& vehGradient,
                     const std::vector<double>& lambda_lo,
                     const std::vector<double>& lambda_hi,
                     const std::vector<double>& mu_lo,
                     const std::vector<double>& mu_hi) {

        t_++;  // Increment time step

        // Bias-corrected learning rate
        double lr_t = learning_rate_ * std::sqrt(1.0 - std::pow(beta2_, t_)) /
                      (1.0 - std::pow(beta1_, t_));

        // Update lambda (request duals)
        for (int i = 0; i < reqLambda.size(); ++i) {
            // Update biased first moment estimate
            m_lambda_[i] = beta1_ * m_lambda_[i] + (1.0 - beta1_) * reqGradient[i];

            // Update biased second raw moment estimate
            v_lambda_[i] = beta2_ * v_lambda_[i] +
                          (1.0 - beta2_) * reqGradient[i] * reqGradient[i];

            // Update dual with ADAM
            reqLambda[i] += lr_t * m_lambda_[i] / (std::sqrt(v_lambda_[i]) + epsilon_);

            // Project to bounds
            reqLambda[i] = std::max(lambda_lo[i], std::min(reqLambda[i], lambda_hi[i]));
        }

        // Update mu (vehicle duals)
        for (int v = 0; v < vehMu.size(); ++v) {
            // Update biased first moment estimate
            m_mu_[v] = beta1_ * m_mu_[v] + (1.0 - beta1_) * vehGradient[v];

            // Update biased second raw moment estimate
            v_mu_[v] = beta2_ * v_mu_[v] +
                      (1.0 - beta2_) * vehGradient[v] * vehGradient[v];

            // Update dual with ADAM
            vehMu[v] += lr_t * m_mu_[v] / (std::sqrt(v_mu_[v]) + epsilon_);

            // Project to bounds
            vehMu[v] = std::max(mu_lo[v], std::min(vehMu[v], mu_hi[v]));
        }
    }

    void setLearningRate(double lr) { learning_rate_ = lr; }
};

class LagrangianSolver {
public:
    ADAMOptimizer adam_optimizer_;

    int nbRoutes_;
    // ---- Dual variables (Lagrange multipliers) ----
    std::vector<double> reqLambda_;      // request multipliers (size n_requests)
    std::vector<double> vehMu_;          // vehicle multipliers (size n_vehicles)

    // ---- Projection bounds for duals ----
    std::vector<double> lambda_lo_;   // usually 0 (or -Λmax if committed)
    std::vector<double> lambda_hi_;   // usually p_i (or +Λmax if committed)
    std::vector<double> mu_lo_;       // typically all zeros
    std::vector<double> mu_hi_;       // optional cap for stability (can be +inf)
    float LambdaMax_;

    // ---- Subgradients (constraint residuals) ----
    std::vector<double> reqGradient_;           // s_i = 1 - Σ_r a_i^r y_r - z_i   (size n_requests)
    std::vector<double> vehGradient_;           // t_v = Σ_{r∈R_v} y_r - 1         (size n_vehicles)

    // ---- Column pool (route set) ----
    std::vector<PRoute> routeList_;

    // ---- Current LR primal solution (best response at current multipliers) ----
    std::vector<PRoute> routeLSolution_;
    std::vector<PRequest> zLSolution_;

    double gamma_;                    // current aggregation weight

    // ---- Scaling / stepsize / VA controls ----
    std::vector<double> d_lambda_;    // diagonal scaling for lambda updates
    std::vector<double> d_mu_;        // diagonal scaling for mu updates
    double theta_;                    // VA control parameter in (0,2)
    double alpha_ ;                   // current stepsize
    double eps_rc_;                   // reduced-cost tolerance
    double eps_gap_;                  // target optimality gap

    // ---- Bounds and bookkeeping ----
    double dual_value_;               // L^k
    double primal_value_;             // U^k (from repaired primal)
    double best_dual_;                // best lower bound so far
    int lagrangeIter_;                // current iteration
    int max_iter_;                    // iteration cap

    double previous_best_dual_;
    int stagnation_count_;
    int max_stagnation_iterations_;
    double lb_improvement_threshold_;

    std::vector<double> ergodic_lambda_;  // Running average of lambda
    std::vector<double> ergodic_mu_;      // Running average of mu
    int ergodic_count_;


    // Constructor and Destructor
    explicit LagrangianSolver(const PInstance &pInst, float objValue,
        std::vector<PRoute> &routeSolution, std::vector<PRequest> & zSolution, vector2D<PRoute> &availableRoutes);

    void updateReducedCostsLagrange();

    void solveLRAtMultipliers(const PInstance &pInst);

    void computeSubgradients(const PInstance &pInst);

    void evaluateDual();

    void dualStep(const PInstance &pInst);

    void dualStepLR(const PInstance &pInst);

    void initProjectionBoxes(const PInstance &pInst);

    void initializeErgodicDuals(const PInstance& pInst);
    void updateErgodicDuals();

    bool iterateOnce(const PInstance &pInst);

    void run(const PInstance &pInst);

    void runADAM(const PInstance &pInst);

    void runErgodic(const PInstance &pInst);
};

#endif //LAGRANGIAN_H

// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/30/19
// Brief: An implementation of inverse dynamics

#include <dart/dart.hpp>
#include <nlopt.hpp>

#include "id.hpp"

// // Define Regulation Opt Params P
// Eigen::MatrixXd iddefinePReg(mWMatPose, mWMatSpeedReg,);

// // Define Regulation Opt Params b
// Eigen::MatrixXd iddefinePReg();

// // Define P for QP
Eigen::MatrixXd iddefineP(Eigen::MatrixXd mPEER,
                          Eigen::MatrixXd mPOrR,
                          Eigen::MatrixXd mPEEL,
                          Eigen::MatrixXd mPOrL,
                          Eigen::VectorXd mPBal,
                          Eigen::MatrixXd mPPose,
                          Eigen::MatrixXd mPSpeedReg,
                          Eigen::MatrixXd mPReg, int mOptDim) {}

// // Define b for QP
Eigen::MatrixXd iddefineb(Eigen::MatrixXd mbEER,
                          Eigen::MatrixXd mbOrR,
                          Eigen::MatrixXd mbEEL,
                          Eigen::MatrixXd mbOrL,
                          Eigen::VectorXd mbBal,
                          Eigen::MatrixXd mbPose,
                          Eigen::MatrixXd mbSpeedReg,
                          Eigen::MatrixXd mbReg) {}

// // opt function
double optFunc(const std::vector<double>& x, std::vector<double>& grad,
               void* my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams*>(my_func_data);
  // std::cout << "done reading optParams " << std::endl;
  // Eigen::MatrixXd X(x.data());
  size_t n = x.size();
  Eigen::VectorXd X = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; i++) X(i) = x[i];
  // std::cout << "done reading x" << std::endl;

  if (!grad.empty()) {
    Eigen::MatrixXd mGrad =
        optParams->P.transpose() * (optParams->P * X - optParams->b);
    // std::cout << "done calculating gradient" << std::endl;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
    // std::cout << "done changing gradient cast" << std::endl;
  }
  // std::cout << "about to return something" << std::endl;
  return (0.5 * pow((optParams->P * X - optParams->b).norm(), 2));
}

// // constraint function
void constraintFunc(unsigned m, double* result, unsigned n, const double* x,
                    double* grad, void* f_data) {
  OptParams* constParams = reinterpret_cast<OptParams*>(f_data);
  // std::cout << "done reading optParams " << std::endl;

  if (grad != NULL) {
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        grad[i * n + j] = constParams->P(i, j);
      }
    }
  }
  // std::cout << "done with gradient" << std::endl;

  Eigen::MatrixXd X = Eigen::VectorXd::Zero(n);
  for (size_t i = 0; i < n; i++) X(i) = x[i];
  // std::cout << "done reading x" << std::endl;

  Eigen::VectorXd mResult;
  mResult = constParams->P * X - constParams->b;
  for (size_t i = 0; i < m; i++) {
    result[i] = mResult(i);
  }
  // std::cout << "done calculating the result"
}

// // compute accelerations for joints based on id algorithm
Eigen::VectorXd computeAccelerations(int mOptDim, OptParams optParamsID,
                                     OptParams* inequalityconstraintParams,
                                     bool maxTimeSet,
                                     Eigen::VectorXd mddqBodyRef) {
  const std::vector<double> inequalityconstraintTol(mOptDim, 1e-3);
  // nlopt::opt opt(nlopt::LN_COBYLA, 30);
  nlopt::opt opt(nlopt::LD_SLSQP, mOptDim);
  double minf;
  opt.set_min_objective(optFunc, &optParamsID);
  opt.add_inequality_mconstraint(constraintFunc, &inequalityconstraintParams[0],
                                 inequalityconstraintTol);
  opt.add_inequality_mconstraint(constraintFunc, &inequalityconstraintParams[1],
                                 inequalityconstraintTol);
  opt.set_xtol_rel(1e-3);
  if (maxTimeSet) opt.set_maxtime(0.01);
  std::vector<double> ddqBodyRef_vec(mOptDim);
  Eigen::VectorXd::Map(&ddqBodyRef_vec[0], mddqBodyRef.size()) = mddqBodyRef;
  try {
    nlopt::result result = opt.optimize(ddqBodyRef_vec, minf);
  } catch (std::exception& e) {
    // std::cout << "nlopt failed: " << e.what() << std::endl;
  }
  for (int i = 0; i < mOptDim; i++) mddqBodyRef(i) = ddqBodyRef_vec[i];

  return mddqBodyRef;
}

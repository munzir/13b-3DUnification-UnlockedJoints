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
Eigen::MatrixXd iddefineP(Eigen::Matrix<double, 3, 18> mPEER,
                          Eigen::Matrix<double, 3, 18> mPOrR,
                          Eigen::Matrix<double, 3, 18> mPEEL,
                          Eigen::Matrix<double, 3, 18> mPOrL,
                          Eigen::VectorXd mPBal,
                          Eigen::Matrix<double, 18, 18> mPPose,
                          Eigen::Matrix<double, 18, 18> mPSpeedReg,
                          Eigen::Matrix<double, 18, 18> mPReg, int mOptDim) {}

// // Define b for QP
Eigen::MatrixXd iddefineb(Eigen::Matrix<double, 3, 1> mbEER,
                          Eigen::Matrix<double, 3, 1> mbOrR,
                          Eigen::Matrix<double, 3, 1> mbEEL,
                          Eigen::Matrix<double, 3, 1> mbOrL,
                          Eigen::VectorXd mbBal,
                          Eigen::Matrix<double, 18, 1> mbPose,
                          Eigen::Matrix<double, 18, 1> mbSpeedReg,
                          Eigen::Matrix<double, 18, 1> mbReg) {}

// // compute accelerations for joints based on id algorithm
Eigen::VectorXd computeAccelerations(
    int mOptDim,
    double (*optFunc)(const std::vector<double>& x, std::vector<double>& grad,
                      void* my_func_data),
    OptParams optParamsID,
    void (*constraintFunc)(unsigned m, double* result, unsigned n,
                           const double* x, double* grad, void* f_data),
    OptParams* inequalityconstraintParams, bool maxTimeSet,
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

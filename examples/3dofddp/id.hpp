// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/30/19
// Brief: An implementation of inverse dynamics

#ifndef ID_HPP
#define ID_HPP

#include <dart/dart.hpp>
#include <nlopt.hpp>

// As of right now the below gets declared in id.hpp first
// Very bad, ideally ID and IK are two classes that implement a single interace
// or come from a single abstract class
struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

// Function Prototypes

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
                          Eigen::Matrix<double, 18, 18> mPReg, int mOptDim);

// // Define b for QP
Eigen::MatrixXd iddefineb(Eigen::Matrix<double, 3, 1> mbEER,
                          Eigen::Matrix<double, 3, 1> mbOrR,
                          Eigen::Matrix<double, 3, 1> mbEEL,
                          Eigen::Matrix<double, 3, 1> mbOrL,
                          Eigen::VectorXd mbBal,
                          Eigen::Matrix<double, 18, 1> mbPose,
                          Eigen::Matrix<double, 18, 1> mbSpeedReg,
                          Eigen::Matrix<double, 18, 1> mbReg);

// // opt function
double optFunc(const std::vector<double>& x, std::vector<double>& grad,
               void* my_func_data);

// // constraint function
void constraintFunc(unsigned m, double* result, unsigned n, const double* x,
                    double* grad, void* f_data);

// // compute accelerations for joints based on id algorithm
Eigen::VectorXd computeAccelerations(int mOptDim, OptParams optParamsID,
                                     OptParams* inequalityconstraintParams,
                                     bool maxTimeSet,
                                     Eigen::VectorXd mddqBodyRef);

#endif  // ID_HPP

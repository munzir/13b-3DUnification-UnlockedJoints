// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/30/19
// Brief: An implementation of inverse kinematics

#ifndef IK_HPP
#define IK_HPP

#include <dart/dart.hpp>
#include <nlopt.hpp>

struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

// Function Prototypes
// // Define P for QP
Eigen::MatrixXd defineP(
    Eigen::Matrix<double, 3, 18> mPEER, Eigen::Matrix<double, 3, 18> mPOrR,
    Eigen::Matrix<double, 3, 18> mPEEL, Eigen::Matrix<double, 3, 18> mPOrL,
    Eigen::VectorXd mPBal, Eigen::Matrix<double, 18, 18> mPPose,
    Eigen::Matrix<double, 18, 18> mPSpeedReg,
    Eigen::Matrix<double, 18, 18> mPReg, int mOptDim);

// // Define b for QP
Eigen::MatrixXd defineb(
    Eigen::Matrix<double, 3, 1> mbEER, Eigen::Matrix<double, 3, 1> mbOrR,
    Eigen::Matrix<double, 3, 1> mbEEL, Eigen::Matrix<double, 3, 1> mbOrL,
    Eigen::VectorXd mbBal, Eigen::Matrix<double, 18, 1> mbPose,
    Eigen::Matrix<double, 18, 1> mbSpeedReg,
    Eigen::Matrix<double, 18, 1> mbReg);

// // compute speeds for joints based on ik algorithm
Eigen::VectorXd computeSpeeds(int mOptDim,
                              double (*optFunc)(const std::vector<double>& x,
                                                std::vector<double>& grad,
                                                void* my_func_data),
                              OptParams optParams, bool maxTimeSet,
                              Eigen::VectorXd mdqBodyRef);

#endif  // IK_HPP

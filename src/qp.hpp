// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/6/19
// Brief: Defining the QP for the upper body on Krang

#ifndef QP_HPP
#define QP_HPP

#include <dart/dart.hpp>

// Function Prototypes

// // Define LeftArmOptParams
void qpLeftArmOptParams();

// // Define Right Arm Opt Params
void qpRightArmOptParams();

// // Define Left Orientation Opt Params
void qpLeftOrientationOptParams();

// // Define Right Orientation Opt Params
void qpRightOrientationOptParams();

// // Define Balance Opt Params
void qpBalanceOptParams();

// // Define Regulation Opt Params
void qpRegulationOptParams(bool mInverseKinematicsOnArms,
                           Eigen::MatrixXd mqBody, Eigen::MatrixXd mqBodyInit,
                           Eigen::MatrixXd mdqBody, Eigen::MatrixXd mWMatPose,
                           Eigen::MatrixXd mWMatSpeedReg,
                           Eigen::MatrixXd mWMatReg, double mKpPose,
                           double mKvPose, double mKvSpeedReg,
                           Eigen::MatrixXd& mPPose, Eigen::VectorXd& mbPose,
                           Eigen::MatrixXd& mPSpeedReg,
                           Eigen::VectorXd& mbSpeedReg, Eigen::MatrixXd& mPReg,
                           Eigen::VectorXd& mbReg);

// Define the P matrix in the QP
Eigen::MatrixXd qpdefineP(Eigen::MatrixXd mPEER, Eigen::MatrixXd mPOrR,
                          Eigen::MatrixXd mPEEL, Eigen::MatrixXd mPOrL,
                          Eigen::VectorXd mPBal, Eigen::MatrixXd mPPose,
                          Eigen::MatrixXd mPSpeedReg, Eigen::MatrixXd mPReg,
                          int mOptDim);

// Define the b matrix in the QP
Eigen::VectorXd qpdefineb(Eigen::MatrixXd mbEER, Eigen::MatrixXd mbOrR,
                          Eigen::MatrixXd mbEEL, Eigen::MatrixXd mbOrL,
                          Eigen::VectorXd mbBal, Eigen::MatrixXd mbPose,
                          Eigen::MatrixXd mbSpeedReg, Eigen::MatrixXd mbReg);

#endif  // QP_HPP

// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/30/19
// Brief: An implementation of inverse kinematics

#include <dart/dart.hpp>
#include <nlopt.hpp>

#include "ik.hpp"

// // Define P for QP
//Eigen::MatrixXd defineP(Eigen::MatrixXd mPEER,
//                        Eigen::MatrixXd mPOrR,
//                        Eigen::MatrixXd mPEEL,
//                        Eigen::MatrixXd mPOrL,
//                        Eigen::VectorXd mPBal,
//                        Eigen::MatrixXd mPPose,
//                        Eigen::MatrixXd mPSpeedReg,
//                        Eigen::MatrixXd mPReg, int mOptDim) {
//  Eigen::MatrixXd P(mPEER.rows() + mPOrR.rows() + mPEEL.rows() + mPOrL.rows() +
//                        mPBal.rows() + mPPose.rows() + mPSpeedReg.rows() +
//                        mPReg.rows(),
//                    mOptDim);
//  P << mPEER.col(0), mPEER.topRightCorner(mPEER.rows(), mOptDim - 1),
//      mPOrR.col(0), mPOrR.topRightCorner(mPOrR.rows(), mOptDim - 1),
//      mPEEL.col(0), mPEEL.topRightCorner(mPEEL.rows(), mOptDim - 1),
//      mPOrL.col(0), mPOrL.topRightCorner(mPOrL.rows(), mOptDim - 1),
//      mPBal.col(0), mPBal.topRightCorner(mPBal.rows(), mOptDim - 1),
//      mPPose.col(0), mPPose.topRightCorner(mPPose.rows(), mOptDim - 1),
//      mPSpeedReg.col(0),
//      mPSpeedReg.topRightCorner(mPSpeedReg.rows(), mOptDim - 1), mPReg.col(0),
//      mPReg.topRightCorner(mPReg.rows(), mOptDim - 1);
//
//  return P;
//}
//
//// // Define b for QP
//Eigen::VectorXd defineb(Eigen::MatrixXd mbEER,
//                        Eigen::MatrixXd mbOrR,
//                        Eigen::MatrixXd mbEEL,
//                        Eigen::MatrixXd mbOrL,
//                        Eigen::VectorXd mbBal,
//                        Eigen::MatrixXd mbPose,
//                        Eigen::MatrixXd mbSpeedReg,
//                        Eigen::MatrixXd mbReg) {
//  Eigen::VectorXd b(mbEER.rows() + mbOrR.rows() + mbEEL.rows() + mbOrL.rows() +
//                        mbBal.rows() + mbPose.rows() + mbSpeedReg.rows() +
//                        mbReg.rows(),
//                    mbEER.cols());
//  b << mbEER, mbOrR, mbEEL, mbOrL, mbBal, mbPose, mbSpeedReg, mbReg;
//  return b;
//}

// // compute speeds for joints based on ik algorithm
Eigen::VectorXd computeSpeeds(int mOptDim, OptParams optParams, bool maxTimeSet,
                              Eigen::VectorXd mdqBodyRef) {
  nlopt::opt opt(nlopt::LD_SLSQP, mOptDim);
  double minf;
  opt.set_min_objective(optFunc, &optParams);
  opt.set_xtol_rel(1e-3);
  if (maxTimeSet) opt.set_maxtime(0.01);
  std::vector<double> dqBodyRef_vec(mOptDim);
  Eigen::VectorXd::Map(&dqBodyRef_vec[0], mdqBodyRef.size()) = mdqBodyRef;
  try {
    // nlopt::result result = opt.optimize(dqBodyRef_vec, minf);
    opt.optimize(dqBodyRef_vec, minf);
  } catch (std::exception& e) {
    std::cout << "nlopt failed: " << e.what() << std::endl;
  }

  for (int i = 0; i < mOptDim; i++) mdqBodyRef(i) = dqBodyRef_vec[i];

  return mdqBodyRef;
}

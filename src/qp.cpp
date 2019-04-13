// Author: Akash Patel (apatel435@gatech.edu)
// Date: 3/6/19
// Brief: Defining the QP for the upper body on Krang

#include "qp.hpp"

// Define LeftArmOptParams
void qpLeftArmOptParams() {}

// Define Right Arm Opt Params
void qpRightArmOptParams() {}

// Define Left Orientation Opt Params
void qpLeftOrientationOptParams() {}

// Define Right Orientation Opt Params
void qpRightOrientationOptParams() {}

// Define Balance Opt Params
void qpBalanceOptParams() {}

// Define Regulation Opt Params
void qpRegulationOptParams(bool mInverseKinematicsOnArms,
                           Eigen::MatrixXd mqBody, Eigen::MatrixXd mqBodyInit,
                           Eigen::MatrixXd mdqBody, Eigen::MatrixXd mWMatPose,
                           Eigen::MatrixXd mWMatSpeedReg,
                           Eigen::MatrixXd mWMatReg, double mKpPose,
                           double mKvPose, double mKvSpeedReg,
                           Eigen::MatrixXd& mPPose, Eigen::VectorXd& mbPose,
                           Eigen::MatrixXd& mPSpeedReg,
                           Eigen::VectorXd& mbSpeedReg, Eigen::MatrixXd& mPReg,
                           Eigen::VectorXd& mbReg) {
  if (!mInverseKinematicsOnArms) {
    mPPose = mWMatPose;
    mbPose << mWMatPose *
                  (-mKpPose * (mqBody - mqBodyInit) - mKvPose * mdqBody);

    mPSpeedReg = mWMatSpeedReg;
    mbSpeedReg << -mWMatSpeedReg * mKvSpeedReg * mdqBody;

    mPReg = mWMatReg;
    mbReg.setZero();
  } else {
    mPPose = mWMatPose;
    mbPose << mWMatPose * (-mKpPose * (mqBody - mqBodyInit));

    mPSpeedReg = mWMatSpeedReg;
    mbSpeedReg.setZero();

    mPReg = mWMatReg;
    mbReg = mWMatReg * mdqBody;
  }
}

// Define the P matrix in the QP
Eigen::MatrixXd qpdefineP(Eigen::MatrixXd mPEER, Eigen::MatrixXd mPOrR,
                          Eigen::MatrixXd mPEEL, Eigen::MatrixXd mPOrL,
                          Eigen::VectorXd mPBal, Eigen::MatrixXd mPPose,
                          Eigen::MatrixXd mPSpeedReg, Eigen::MatrixXd mPReg,
                          int mOptDim) {
  Eigen::MatrixXd P(mPEER.rows() + mPOrR.rows() + mPEEL.rows() + mPOrL.rows() +
                        mPBal.rows() + mPPose.rows() + mPSpeedReg.rows() +
                        mPReg.rows(),
                    mOptDim);
  P << mPEER.col(0), mPEER.topRightCorner(mPEER.rows(), mOptDim - 1),
      mPOrR.col(0), mPOrR.topRightCorner(mPOrR.rows(), mOptDim - 1),
      mPEEL.col(0), mPEEL.topRightCorner(mPEEL.rows(), mOptDim - 1),
      mPOrL.col(0), mPOrL.topRightCorner(mPOrL.rows(), mOptDim - 1),
      mPBal.col(0), mPBal.topRightCorner(mPBal.rows(), mOptDim - 1),
      mPPose.col(0), mPPose.topRightCorner(mPPose.rows(), mOptDim - 1),
      mPSpeedReg.col(0),
      mPSpeedReg.topRightCorner(mPSpeedReg.rows(), mOptDim - 1), mPReg.col(0),
      mPReg.topRightCorner(mPReg.rows(), mOptDim - 1);
  return P;
}

// Define the b matrix in the QP
Eigen::VectorXd qpdefineb(Eigen::MatrixXd mbEER, Eigen::MatrixXd mbOrR,
                          Eigen::MatrixXd mbEEL, Eigen::MatrixXd mbOrL,
                          Eigen::VectorXd mbBal, Eigen::MatrixXd mbPose,
                          Eigen::MatrixXd mbSpeedReg, Eigen::MatrixXd mbReg) {
  Eigen::VectorXd b(mbEER.rows() + mbOrR.rows() + mbEEL.rows() + mbOrL.rows() +
                        mbBal.rows() + mbPose.rows() + mbSpeedReg.rows() +
                        mbReg.rows(),
                    mbEER.cols());
  b << mbEER, mbOrR, mbEEL, mbOrL, mbBal, mbPose, mbSpeedReg, mbReg;
  return b;
}

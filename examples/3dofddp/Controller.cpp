/*
 * Copyright (c) 2014-2016, Humanoid Lab, Georgia Tech Research Corporation
 * Copyright (c) 2014-2017, Graphics Lab, Georgia Tech Research Corporation
 * Copyright (c) 2016-2017, Personal Robotics Lab, Carnegie Mellon University
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include "Controller.hpp"

//==============================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot,
                       dart::dynamics::BodyNode* _LeftendEffector,
                       dart::dynamics::BodyNode* _RightendEffector)
    : mRobot(_robot),
      mLeftEndEffector(_LeftendEffector),
      mRightEndEffector(_RightendEffector) {
  assert(_robot != nullptr);
  assert(_LeftendEffector != nullptr);
  assert(_RightendEffector != nullptr);

  int dof = mRobot->getNumDofs();
  // std::cout << "[controller] DoF: " << dof << std::endl;

  mForces.setZero(19);

  mSteps = 0;

  // *************** Read Initial Pose for Pose Regulation and Generate
  // reference zCOM
  Eigen::Matrix<double, 25, 1> qInit;
  Eigen::Matrix3d Rot0;
  qInit = mRobot->getPositions();
  mBaseTf = mRobot->getBodyNode(0)->getTransform().matrix();
  double psiInit = atan2(mBaseTf(0, 0), -mBaseTf(1, 0));
  Rot0 << cos(psiInit), sin(psiInit), 0, -sin(psiInit), cos(psiInit), 0, 0, 0,
      1;
  double qBody1Init =
      atan2(mBaseTf(0, 1) * cos(psiInit) + mBaseTf(1, 1) * sin(psiInit),
            mBaseTf(2, 1));
  mqBodyInit(0) = qBody1Init;
  mqBodyInit.tail(17) = qInit.tail(17);

  dart::dynamics::BodyNode* LWheel = mRobot->getBodyNode("LWheel");
  dart::dynamics::BodyNode* RWheel = mRobot->getBodyNode("RWheel");
  Eigen::Vector3d bodyCOM = Rot0 * (mRobot->getCOM() - qInit.segment(3, 3));
  bodyCOM(1) = 0;
  mInitCOMDistance = bodyCOM.norm();

  // ************** Remove position limits
  for (int i = 6; i < dof - 1; ++i)
    _robot->getJoint(i)->setPositionLimitEnforced(false);
  // std::cout << "Position Limit Enforced set to false" << std::endl;

  // ************** Set joint damping
  for (int i = 6; i < dof - 1; ++i)
    _robot->getJoint(i)->setDampingCoefficient(0, 0.5);
  // std::cout << "Damping coefficients set" << std::endl;

  mdqFilt = new filter(25, 100);

  // ************** Wheel Radius and Distance between wheels
  mR = 0.25, mL = 0.68;

  // *********************************** Tunable Parameters
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";
  const char* configFile = "../../../examples/3dofddp/controlParams.cfg";
  const char* str;
  std::istringstream stream;
  double newDouble;
  Eigen::Matrix<double, 18, 1> tauLim;

  mKpEE.setZero();
  mKvEE.setZero();
  mWEER.setZero();
  mWEEL.setZero();
  mWBal.setZero();
  mWMatPose.setZero();
  mWMatSpeedReg.setZero();
  mWMatReg.setZero();

  try {
    cfg->parse(configFile);

    // Waist Locked?
    mWaistLocked = cfg->lookupBoolean(scope, "waistLocked");

    // -- COM Angle Based Control or not
    mCOMAngleControl = cfg->lookupBoolean(scope, "COMAngleControl");
    mMaintainInitCOMDistance =
        cfg->lookupBoolean(scope, "maintainInitCOMDistance");

    // -- Torque Limits
    str = cfg->lookupString(scope, "tauLim");
    stream.str(str);
    for (int i = 0; i < 18; i++) stream >> tauLim(i);
    stream.clear();

    // -- Gains
    mKpEE(0, 0) = cfg->lookupFloat(scope, "KpEE");
    mKpEE(1, 1) = mKpEE(0, 0);
    mKpEE(2, 2) = mKpEE(0, 0);
    mKvEE(0, 0) = cfg->lookupFloat(scope, "KvEE");
    mKvEE(1, 1) = mKvEE(0, 0);
    mKvEE(2, 2) = mKvEE(0, 0);
    mKpOr(0, 0) = cfg->lookupFloat(scope, "KpOr");
    mKpOr(1, 1) = mKpOr(0, 0);
    mKpOr(2, 2) = mKpOr(0, 0);
    mKvOr(0, 0) = cfg->lookupFloat(scope, "KvOr");
    mKvOr(1, 1) = mKvOr(0, 0);
    mKvOr(2, 2) = mKvOr(0, 0);
    mKpCOM = cfg->lookupFloat(scope, "KpCOM");
    mKvCOM = cfg->lookupFloat(scope, "KvCOM");
    mKvSpeedReg = cfg->lookupFloat(scope, "KvSpeedReg");
    mKpPose = cfg->lookupFloat(scope, "KpPose");
    mKvPose = cfg->lookupFloat(scope, "KvPose");

    // -- Weights
    // Right Arm
    if (mWaistLocked)
      str = cfg->lookupString(scope, "wEERWaistLocked");
    else
      str = cfg->lookupString(scope, "wEER");
    stream.str(str);
    for (int i = 0; i < 3; i++) stream >> mWEER(i, i);
    stream.clear();
    mWOrR = cfg->lookupFloat(scope, "wOrR");

    // Left Arm
    if (mWaistLocked)
      str = cfg->lookupString(scope, "wEELWaistLocked");
    else
      str = cfg->lookupString(scope, "wEEL");
    stream.str(str);
    for (int i = 0; i < 3; i++) stream >> mWEEL(i, i);
    stream.clear();
    mWOrL = cfg->lookupFloat(scope, "wOrL");

    // Balance
    str = cfg->lookupString(scope, "wBal");
    stream.str(str);
    for (int i = 0; i < 3; i++) stream >> mWBal(i, i);
    stream.clear();
    // Regulation
    const char* s[] = {"wRegBase", "wRegWaist", "wRegTorso", "wRegKinect",
                       "wRegArm1", "wRegArm2",  "wRegArm3",  "wRegArm4",
                       "wRegArm5", "wRegArm6",  "wRegArm7"};
    for (int i = 0; i < 11; i++) {
      str = cfg->lookupString(scope, s[i]);
      stream.str(str);
      stream >> mWMatPose(i, i);
      stream >> mWMatSpeedReg(i, i);
      stream >> mWMatReg(i, i);
      if (i > 3) {
        mWMatPose(i + 7, i + 7) = mWMatPose(i, i);
        mWMatSpeedReg(i + 7, i + 7) = mWMatSpeedReg(i, i);
        mWMatReg(i + 7, i + 7) = mWMatReg(i, i);
      }
      stream.clear();
    }

  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
  }
  std::cout << "COMAngleControl: " << (mCOMAngleControl ? "true" : "false")
            << std::endl;
  std::cout << "maintainInitCOMDistance: "
            << (mMaintainInitCOMDistance ? "true" : "false") << std::endl;
  std::cout << "tauLim: " << mTauLim.transpose() << std::endl;
  std::cout << "KpEE: " << mKpEE(0, 0) << ", " << mKpEE(1, 1) << ", "
            << mKpEE(2, 2) << std::endl;
  std::cout << "KvEE: " << mKvEE(0, 0) << ", " << mKvEE(1, 1) << ", "
            << mKvEE(2, 2) << std::endl;
  std::cout << "KpCOM: " << mKpCOM << std::endl;
  std::cout << "KvCOM: " << mKvCOM << std::endl;
  std::cout << "KvSpeedReg: " << mKvSpeedReg << std::endl;
  std::cout << "KpPose: " << mKpPose << std::endl;
  std::cout << "KvPose: " << mKvPose << std::endl;
  std::cout << "wEER: " << mWEER.diagonal().transpose() << std::endl;
  std::cout << "wEEL: " << mWEEL.diagonal().transpose() << std::endl;
  // std::cout << "wBal: " << mWBal(0, 0) << ", " << mWBal(1, 1) << ", " <<
  // mWBal(2, 2) << std::endl;
  std::cout << "wBal: " << mWBal.diagonal().transpose() << std::endl;
  std::cout << "wMatPoseReg: ";
  for (int i = 0; i < 18; i++) std::cout << mWMatPose(i, i) << ", ";
  std::cout << std::endl;
  std::cout << "wMatSpeedReg: ";
  for (int i = 0; i < 18; i++) std::cout << mWMatSpeedReg(i, i) << ", ";
  std::cout << std::endl;
  std::cout << "wMatReg: ";
  for (int i = 0; i < 18; i++) std::cout << mWMatReg(i, i) << ", ";
  std::cout << std::endl;
  std::cout << "waistLocked: " << (mWaistLocked ? "true" : "false")
            << std::endl;
  cfg->destroy();

  // PBal and bBal size based on mCOMAngleControl
  if (mCOMAngleControl) {
    mPBal = Eigen::MatrixXd::Zero(1, 18);
    mbBal = Eigen::VectorXd::Zero(1);
  } else {
    mPBal = Eigen::MatrixXd::Zero(3, 18);
    mbBal = Eigen::VectorXd::Zero(3);
  }

  // *********************************** Transform Jacobians
  mJtf.topRightCorner(8, 17) = Eigen::Matrix<double, 8, 17>::Zero();
  mJtf.bottomLeftCorner(17, 3) = Eigen::Matrix<double, 17, 3>::Zero();
  mJtf.bottomRightCorner(17, 17) = Eigen::Matrix<double, 17, 17>::Identity();
  mdJtf.setZero();

  // ******************************** zero Cols
  mZeroCol.setZero();
  mZero7Col.setZero();

  // **************************** if waist locked, dimesion of decision variable
  // in QP should be reduced by one
  if (mWaistLocked)
    mOptDim = 17;
  else
    mOptDim = 18;
  mddqBodyRef = Eigen::VectorXd::Zero(mOptDim);
  mMM = Eigen::MatrixXd::Zero(mOptDim, mOptDim);
  mhh = Eigen::VectorXd::Zero(mOptDim);
  mTauLim = Eigen::VectorXd::Zero(mOptDim);
  mTauLim << tauLim(0), tauLim.tail(mOptDim - 1);
}

//==============================================================================
Controller::~Controller() {}

//==============================================================================
struct OptParams {
  Eigen::MatrixXd P;
  Eigen::VectorXd b;
};

//==============================================================================
void printMatrix(Eigen::MatrixXd A) {
  for (int i = 0; i < A.rows(); i++) {
    for (int j = 0; j < A.cols(); j++) {
      std::cout << A(i, j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

//==============================================================================
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

//==============================================================================
double optFunc(const std::vector<double>& x, std::vector<double>& grad,
               void* my_func_data) {
  OptParams* optParams = reinterpret_cast<OptParams*>(my_func_data);
  // std::cout << "done reading optParams " << std::endl;
  // Eigen::Matrix<double, 18, 1> X(x.data());
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

//==============================================================================
void Controller::updatePositions() {
  mBaseTf = mRobot->getBodyNode(0)->getTransform().matrix();
  mq = mRobot->getPositions();
  mxyz0 = mq.segment(3, 3);  // position of frame 0 in the world frame
                             // represented in the world frame
  mpsi = atan2(mBaseTf(0, 0), -mBaseTf(1, 0));
  mqBody1 = atan2(mBaseTf(0, 1) * cos(mpsi) + mBaseTf(1, 1) * sin(mpsi),
                  mBaseTf(2, 1));
  mqBody(0) = mqBody1;
  mqBody.tail(17) = mq.tail(17);
  mRot0 << cos(mpsi), sin(mpsi), 0, -sin(mpsi), cos(mpsi), 0, 0, 0, 1;
}

//==============================================================================
void Controller::updateSpeeds() {
  mdqFilt->AddSample(mRobot->getVelocities());
  mdq = mdqFilt->average;
  mdxyz0 = mBaseTf.matrix().block<3, 3>(0, 0) *
           mdq.segment(3, 3);  // velocity of frame 0 in the world frame
                               // represented in the world frame
  mdx = mdq(4) * sin(mqBody1) - mdq(5) * cos(mqBody1);
  mdqBody1 = -mdq(0);
  mdpsi = (mBaseTf.block<3, 3>(0, 0) * mdq.head(3))(2);
  mdqBody(0) = mdqBody1;
  mdqBody.tail(17) = mdq.tail(17);
  mdqMin(0) = mdx;
  mdqMin(1) = mdpsi;
  mdqMin.tail(18) = mdqBody;
  mdRot0 << (-sin(mpsi) * mdpsi), (cos(mpsi) * mdpsi), 0, (-cos(mpsi) * mdpsi),
      (-sin(mpsi) * mdpsi), 0, 0, 0, 0;
}

//==============================================================================
void Controller::updateTransformJacobian() {
  // ********************************* Transform Jacobian
  // Coordinate Transformation to minimum set of coordinates
  // dq0 = -dq_1
  // dq1 = dpsi*cos(q_1)
  // dq2 = dpsi*sin(q_1)
  // dq3 = 0
  // dq4 = dx*sin(q_1)
  // dq5 = -dx*cos(q_1)
  // dq6 = dx/R - (L/(2*R))*dpsi - dq_1
  // dq7 = dx/R + (L/(2*R))*dpsi - dq_1
  // dq8 = dq_2
  // dq9 = dq_3
  // [dq0 dq1 dq2 dq3 dq4 dq5 dq6 dq7]' = J*[dx dpsi dq_1]';
  // where

  mJtf.topLeftCorner(8, 3) << 0, 0, -1, 0, cos(mqBody1), 0, 0, sin(mqBody1), 0,
      0, 0, 0, sin(mqBody1), 0, 0, -cos(mqBody1), 0, 0, 1 / mR, -mL / (2 * mR),
      -1, 1 / mR, mL / (2 * mR), -1;

  mdJtf.topLeftCorner(8, 3) << 0, 0, 0, 0, -sin(mqBody1) * mdqBody1, 0, 0,
      cos(mqBody1) * mdqBody1, 0, 0, 0, 0, cos(mqBody1) * mdqBody1, 0, 0,
      sin(mqBody1) * mdqBody1, 0, 0, 0, 0, 0, 0, 0, 0;

  if (mSteps < 0) {
    std::cout << "Jtf: " << std::endl;
    for (int i = 0; i < mJtf.rows(); i++) {
      for (int j = 0; j < mJtf.cols(); j++) std::cout << mJtf(i, j) << ", ";
      std::cout << std::endl;
    }
    std::cout << std::endl;
    std::cout << "dJtf: " << std::endl;
    for (int i = 0; i < mdJtf.rows(); i++) {
      for (int j = 0; j < mdJtf.cols(); j++) std::cout << mdJtf(i, j) << ", ";
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

//==============================================================================
void Controller::setLeftArmOptParams(
    const Eigen::Vector3d& _LeftTargetPosition) {
  static Eigen::Vector3d xEELref, xEEL, dxEEL, ddxEELref;
  static Eigen::Matrix<double, 3, 15> JEEL_small, dJEEL_small;
  static Eigen::Matrix<double, 3, 25> JEEL_full, dJEEL_full;
  static Eigen::Matrix<double, 3, 18> JEEL, dJEEL;

  xEELref = _LeftTargetPosition;
  if (mSteps == 1) {
    std::cout << "xEELref: " << xEELref(0) << ", " << xEELref(1) << ", "
              << xEELref(2) << std::endl;
  }

  // x, dx, ddxref
  xEEL = mRot0 * (mLeftEndEffector->getTransform().translation() - mxyz0);
  dxEEL = mRot0 * (mLeftEndEffector->getLinearVelocity() - mdxyz0) +
          mdRot0 * (mLeftEndEffector->getTransform().translation() - mxyz0);
  ddxEELref = -mKpEE * (xEEL - xEELref) - mKvEE * dxEEL;

  // Jacobian
  JEEL_small = mLeftEndEffector->getLinearJacobian();
  JEEL_full << JEEL_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      JEEL_small.block<3, 2>(0, 6), mZeroCol, JEEL_small.block<3, 7>(0, 8),
      mZero7Col;
  JEEL = (mRot0 * JEEL_full * mJtf).topRightCorner(3, 18);

  // Jacobian Derivative
  dJEEL_small = mLeftEndEffector->getLinearJacobianDeriv();
  dJEEL_full << dJEEL_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      dJEEL_small.block<3, 2>(0, 6), mZeroCol, dJEEL_small.block<3, 7>(0, 8),
      mZero7Col;
  dJEEL = (mdRot0 * JEEL_full * mJtf + mRot0 * dJEEL_full * mJtf +
           mRot0 * JEEL_full * mdJtf)
              .topRightCorner(3, 18);

  // P and b
  mPEEL << mWEEL * JEEL;
  mbEEL = -mWEEL * (dJEEL * mdqBody - ddxEELref);
}

//==============================================================================
void Controller::setRightArmOptParams(
    const Eigen::Vector3d& _RightTargetPosition) {
  static Eigen::Vector3d xEERref, xEER, dxEER, ddxEERref;
  static Eigen::Matrix<double, 3, 15> JEER_small, dJEER_small;
  static Eigen::Matrix<double, 3, 25> JEER_full, dJEER_full;
  static Eigen::Matrix<double, 3, 18> JEER, dJEER;

  // x, dx, ddxref
  xEERref = _RightTargetPosition;
  if (mSteps == 1) {
    std::cout << "xEErefR: " << xEERref(0) << ", " << xEERref(1) << ", "
              << xEERref(2) << std::endl;
  }
  xEER = mRot0 * (mRightEndEffector->getTransform().translation() - mxyz0);
  dxEER = mRot0 * (mRightEndEffector->getLinearVelocity() - mdxyz0) +
          mdRot0 * (mRightEndEffector->getTransform().translation() - mxyz0);
  ddxEERref = -mKpEE * (xEER - xEERref) - mKvEE * dxEER;

  // Jacobian
  JEER_small = mRightEndEffector->getLinearJacobian();
  JEER_full << JEER_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      JEER_small.block<3, 2>(0, 6), mZeroCol, mZero7Col,
      JEER_small.block<3, 7>(0, 8);
  JEER = (mRot0 * JEER_full * mJtf).topRightCorner(3, 18);

  // Jacobian Derivative
  dJEER_small = mRightEndEffector->getLinearJacobianDeriv();
  dJEER_full << dJEER_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      dJEER_small.block<3, 2>(0, 6), mZeroCol, mZero7Col,
      dJEER_small.block<3, 7>(0, 8);
  dJEER = (mdRot0 * JEER_full * mJtf + mRot0 * dJEER_full * mJtf +
           mRot0 * JEER_full * mdJtf)
              .topRightCorner(3, 18);

  // P and b
  mPEER << mWEER * JEER;
  mbEER = -mWEER * (dJEER * mdqBody - ddxEERref);
}

//==============================================================================
void Controller::setLeftOrientationOptParams(
    const Eigen::Vector3d& _LeftTargetRPY) {
  static Eigen::Quaterniond quatRef, quat;
  static double quatRef_w, quat_w;
  static Eigen::Vector3d quatRef_xyz, quat_xyz, quatError_xyz, w, dwref;
  static Eigen::Matrix<double, 3, 15> JwL_small, dJwL_small;
  static Eigen::Matrix<double, 3, 25> JwL_full, dJwL_full;
  static Eigen::Matrix<double, 3, 18> JwL, dJwL;

  // Reference orientation (TargetRPY is assumed to be in Frame 0)
  quatRef = Eigen::Quaterniond(
      Eigen::AngleAxisd(_LeftTargetRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(_LeftTargetRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(_LeftTargetRPY(2), Eigen::Vector3d::UnitZ()));
  quatRef_w = quatRef.w();
  quatRef_xyz << quatRef.x(), quatRef.y(), quatRef.z();
  if (quatRef_w < 0) {
    quatRef_w *= -1.0;
    quatRef_xyz *= -1.0;
  }

  // Current orientation in Frame 0
  Eigen::Vector3d currentRPY = dart::math::matrixToEulerXYZ(
      mRot0 * mLeftEndEffector->getTransform().rotation());
  quat = Eigen::Quaterniond(
      Eigen::AngleAxisd(currentRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(currentRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(currentRPY(2), Eigen::Vector3d::UnitZ()));
  // quat =
  // Eigen::Quaterniond(mRot0*mLeftEndEffector->getTransform().rotation());
  quat_w = quat.w();
  quat_xyz << quat.x(), quat.y(), quat.z();
  if (pow(-quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2) <
      pow(quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2)) {
    quat_w *= -1.0;
    quat_xyz *= -1.0;
  }

  // Orientation error
  quatError_xyz =
      quatRef_w * quat_xyz - quat_w * quatRef_xyz + quatRef_xyz.cross(quat_xyz);

  // Jacobian
  JwL_small = mLeftEndEffector->getAngularJacobian();
  JwL_full << JwL_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      JwL_small.block<3, 2>(0, 6), mZeroCol, JwL_small.block<3, 7>(0, 8),
      mZero7Col;
  JwL = (mRot0 * JwL_full * mJtf).topRightCorner(3, 18);

  // Jacobian Derivative
  dJwL_small = mLeftEndEffector->getAngularJacobianDeriv();
  dJwL_full << dJwL_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      dJwL_small.block<3, 2>(0, 6), mZeroCol, dJwL_small.block<3, 7>(0, 8),
      mZero7Col;
  dJwL = (mdRot0 * JwL_full * mJtf + mRot0 * dJwL_full * mJtf +
          mRot0 * JwL_full * mdJtf)
             .topRightCorner(3, 18);

  // Current angular speed in frame 0 and Reference angular acceleration of the
  // end-effector in frame 0
  w = JwL * mdqBody;
  dwref = -mKpOr * quatError_xyz - mKvOr * w;

  // P and b
  mPOrL = mWOrL * JwL;
  mbOrL = -mWOrL * (dJwL * mdqBody - dwref);
}

//==============================================================================
void Controller::setRightOrientationOptParams(
    const Eigen::Vector3d& _RightTargetRPY) {
  static Eigen::Quaterniond quatRef, quat;
  static double quatRef_w, quat_w;
  static Eigen::Vector3d quatRef_xyz, quat_xyz, quatError_xyz, w, dwref;
  static Eigen::Matrix<double, 3, 15> JwR_small, dJwR_small;
  static Eigen::Matrix<double, 3, 25> JwR_full, dJwR_full;
  static Eigen::Matrix<double, 3, 18> JwR, dJwR;

  // Reference orientation (TargetRPY is assumed to be in Frame 0)
  quatRef = Eigen::Quaterniond(
      Eigen::AngleAxisd(_RightTargetRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(_RightTargetRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(_RightTargetRPY(2), Eigen::Vector3d::UnitZ()));
  quatRef_w = quatRef.w();
  quatRef_xyz << quatRef.x(), quatRef.y(), quatRef.z();
  if (quatRef_w < 0) {
    quatRef_w *= -1.0;
    quatRef_xyz *= -1.0;
  }

  // Current orientation in Frame 0
  Eigen::Vector3d currentRPY = dart::math::matrixToEulerXYZ(
      mRot0 * mRightEndEffector->getTransform().rotation());
  quat = Eigen::Quaterniond(
      Eigen::AngleAxisd(currentRPY(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(currentRPY(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(currentRPY(2), Eigen::Vector3d::UnitZ()));
  // quat =
  // Eigen::Quaterniond(mRot0*mRightEndEffector->getTransform().rotation());
  quat_w = quat.w();
  quat_xyz << quat.x(), quat.y(), quat.z();
  if (pow(-quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2) <
      pow(quat_w - quatRef_w, 2) + pow((-quat_xyz - quatRef_xyz).norm(), 2)) {
    quat_w *= -1.0;
    quat_xyz *= -1.0;
  }

  // Orientation error
  quatError_xyz =
      quatRef_w * quat_xyz - quat_w * quatRef_xyz + quatRef_xyz.cross(quat_xyz);

  // Jacobian
  JwR_small = mRightEndEffector->getAngularJacobian();
  JwR_full << JwR_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      JwR_small.block<3, 2>(0, 6), mZeroCol, mZero7Col,
      JwR_small.block<3, 7>(0, 8);
  JwR = (mRot0 * JwR_full * mJtf).topRightCorner(3, 18);

  // Jacobian Derivative
  dJwR_small = mRightEndEffector->getAngularJacobianDeriv();
  dJwR_full << dJwR_small.block<3, 6>(0, 0), mZeroCol, mZeroCol,
      dJwR_small.block<3, 2>(0, 6), mZeroCol, mZero7Col,
      dJwR_small.block<3, 7>(0, 8);
  dJwR = (mdRot0 * JwR_full * mJtf + mRot0 * dJwR_full * mJtf +
          mRot0 * JwR_full * mdJtf)
             .topRightCorner(3, 18);

  // Current angular speed in frame 0 and Reference angular acceleration of the
  // end-effector in frame 0
  w = JwR * mdqBody;
  dwref = -mKpOr * quatError_xyz - mKvOr * w;

  // P and b
  mPOrR = mWOrR * JwR;
  mbOrR = -mWOrR * (dJwR * mdqBody - dwref);
}

//==============================================================================
void Controller::setBalanceOptParams(double thref, double dthref,
                                     double ddthref) {
  static Eigen::Vector3d COM, dCOM, COMref, dCOMref, ddCOMref, ddCOMStar;
  static Eigen::Matrix<double, 3, 25> JCOM_full, dJCOM_full;
  static Eigen::Matrix<double, 3, 18> JCOM, dJCOM;
  static Eigen::Matrix<double, 1, 18> Jth, dJth;
  static Eigen::Matrix<double, 1, 3> thVec, dthVec;
  static double L, th, th_wrong, dth, ddthStar;

  //*********************************** Balance
  // Excluding wheels from COM Calculation
  // Eigen::Vector3d bodyCOM = Rot0*(mRobot->getCOM() - xyz0);
  // Eigen::Vector3d bodyCOMLinearVelocity =
  // Rot0*(mRobot->getCOMLinearVelocity() - dxyz0) + dRot0*(mRobot->getCOM() -
  // xyz0);

  // COM, dCOM, JCOM and dJCOM
  COM = mRot0 * (mRobot->getCOM() - mxyz0);
  dCOM = mRot0 * (mRobot->getCOMLinearVelocity() - mdxyz0) +
         mdRot0 * (mRobot->getCOM() - mxyz0);

  // x, dx, ddxStar
  COM = mRot0 * (mRobot->getCOM() - mxyz0);
  dCOM = mRot0 * (mRobot->getCOMLinearVelocity() - mdxyz0) +
         mdRot0 * (mRobot->getCOM() - mxyz0);
  if (mCOMAngleControl) {
    th = atan2(COM(0), COM(2));
    dth = (cos(th) / COM(2)) * (cos(th) * dCOM(0) - sin(th) * dCOM(2));
    ddthStar = ddthref - mKpCOM * (th - thref) - mKvCOM * (dth - dthref);
  } else {
    if (mMaintainInitCOMDistance)
      L = mInitCOMDistance;
    else
      L = pow(COM(0) * COM(0) + COM(2) * COM(2), 0.5);
    COMref << L * sin(thref), 0, L * cos(thref);
    dCOMref << (L * cos(thref) * dthref), 0.0, (-L * sin(thref) * dthref);
    ddCOMStar << (-L * sin(thref) * dthref * dthref + L * cos(thref) * ddthref),
        0.0, (-L * cos(thref) * dthref * dthref - L * sin(thref) * ddthref);
  }

  // Jacobian
  JCOM_full = mRobot->getCOMLinearJacobian();
  JCOM = (mRot0 * JCOM_full * mJtf).topRightCorner(3, 18);
  if (mCOMAngleControl) {
    thVec << cos(th), 0.0, -sin(th);
    Jth = (cos(th) * thVec * JCOM) / COM(2);
  }

  // Jacobian derivative
  dJCOM_full = mRobot->getCOMLinearJacobianDeriv();
  dJCOM = (mdRot0 * JCOM_full * mJtf + mRot0 * dJCOM_full * mJtf +
           mRot0 * JCOM_full * mdJtf)
              .topRightCorner(3, 18);
  if (mCOMAngleControl) {
    dthVec << -sin(th), 0.0, -cos(th);
    dJth = (-sin(th) * thVec * JCOM * dth + cos(th) * dthVec * JCOM * dth +
            cos(th) * thVec * dJCOM - dCOM(2) * Jth) /
           COM(2);
  }

  // P and b
  if (mCOMAngleControl) {
    mPBal << mWBal(0, 0) * Jth;
    mbBal << mWBal(0, 0) * (-dJth * mdqBody + ddthStar);
  } else {
    mPBal << mWBal * JCOM;
    mbBal << mWBal * (-dJCOM * mdqBody + ddCOMStar);
  }
}

//==============================================================================
void Controller::computeDynamics() {
  static Eigen::Matrix<double, 25, 25> M_full;
  static Eigen::Matrix<double, 20, 20> M;
  static Eigen::Matrix<double, 20, 1> h;
  static Eigen::Matrix<double, 19, 1> h_without_psi_equation;
  static double axx, alpha, beta;
  static Eigen::Matrix<double, 18, 1> axq, hh;
  static Eigen::Matrix<double, 18, 19> PP;
  static Eigen::Matrix<double, 18, 18> Aqq, A_qq, B, pre, MM;

  // ***************************** Inertia and Coriolis Matrices
  M_full = mRobot->getMassMatrix();
  M = mJtf.transpose() * M_full * mJtf;
  h = mJtf.transpose() * M_full * mdJtf * mdqMin +
      mJtf.transpose() * mRobot->getCoriolisAndGravityForces();
  h_without_psi_equation(0) = h(0);
  h_without_psi_equation.tail(18) = h.tail(18);

  axx = M(0, 0);
  axq = M.bottomLeftCorner(18, 1);
  Aqq = M.bottomRightCorner(18, 18);
  alpha = axq(0) / (mR * axx);
  beta = 1 / (1 + alpha);
  A_qq = Aqq - (1 / axx) * (axq * axq.transpose());  // AqqSTAR in derivation
  B << axq / (mR * axx), Eigen::Matrix<double, 18, 17>::Zero();
  pre = Eigen::Matrix<double, 18, 18>::Identity() - beta * B;
  PP << -pre * axq / axx, pre;
  MM = pre * A_qq;
  hh = PP * h_without_psi_equation;
  mMM << MM(0, 0), MM.topRightCorner(1, mOptDim - 1),
      MM.bottomLeftCorner(mOptDim - 1, 1),
      MM.bottomRightCorner(mOptDim - 1, mOptDim - 1);
  mhh << hh(0), hh.tail(mOptDim - 1);
  if (mSteps < 0) {
    std::cout << "axx: " << axx << std::endl;
    std::cout << "axq: ";
    for (int i = 0; i < axq.rows(); i++)
      for (int j = 0; j < axq.cols(); j++) std::cout << axq(i, j) << ", ";
    std::cout << std::endl;
    std::cout << "Aqq: ";
    for (int i = 0; i < Aqq.rows(); i++)
      for (int j = 0; j < Aqq.cols(); j++) std::cout << Aqq(i, j) << ", ";
    std::cout << std::endl;
    std::cout << "alpha: " << alpha << std::endl;
    std::cout << "beta: " << beta << std::endl;
    std::cout << "A_qq: ";
    for (int i = 0; i < A_qq.rows(); i++)
      for (int j = 0; j < A_qq.cols(); j++) std::cout << A_qq(i, j) << ", ";
    std::cout << std::endl;
    std::cout << "B: ";
    for (int i = 0; i < B.rows(); i++)
      for (int j = 0; j < B.cols(); j++) std::cout << B(i, j) << ", ";
    std::cout << std::endl;
    std::cout << "pre: ";
    for (int i = 0; i < pre.rows(); i++)
      for (int j = 0; j < pre.cols(); j++) std::cout << pre(i, j) << ", ";
    std::cout << std::endl;
    std::cout << "PP: ";
    for (int i = 0; i < PP.rows(); i++)
      for (int j = 0; j < PP.cols(); j++) std::cout << PP(i, j) << ", ";
    std::cout << std::endl;
    std::cout << "MM: ";
    for (int i = 0; i < mMM.rows(); i++)
      for (int j = 0; j < mMM.cols(); j++) std::cout << mMM(i, j) << ", ";
    std::cout << std::endl;
    std::cout << "hh: ";
    for (int i = 0; i < mhh.rows(); i++)
      for (int j = 0; j < mhh.cols(); j++) std::cout << mhh(i, j) << ", ";
    std::cout << std::endl;
  }
  // std::cout << "Size of M = " << M.rows() << "*" << M.cols() << std::endl;
  // std::cout << "Size of h = " << h.rows() << "*" << h.cols() << std::endl;
}

//==============================================================================
void Controller::update(const Eigen::Vector3d& _LeftTargetPosition,
                        const Eigen::Vector3d& _RightTargetPosition,
                        const Eigen::Vector3d& _LeftTargetRPY,
                        const Eigen::Vector3d& _RightTargetRPY, double thref,
                        double dthref, double ddthref, double tau_0) {
  // increase the step counter
  mSteps++;

  // updates mBaseTf, mq, mxyz0, mpsi, mqBody1, mqBody, mRot0
  // Needs mRobot
  updatePositions();
  // updates mdq, mdxyz0, mdx, mdqBody1, mdpsi, mdqBody, mdqMin, dRot0
  // Needs mRobot, mdqFilt, mBaseTf, mqBody1
  updateSpeeds();

  // updates mJtf and mdJtf
  // Needs mqBody1, mdqBody1, mR, mL
  updateTransformJacobian();

  // sets mPEEL and mbEEL
  // Needs mRot0, mLeftEndEffector, mxyz0, mdxyz0, mdRot0, mKpEE, mKvEE, mJtf,
  // mdJtf
  setLeftArmOptParams(_LeftTargetPosition);

  // sets mPEER and mbEER
  // Needs mRot0, mRightEndEffector, mxyz0, mdxyz0, mdRot0, mKpEE, mKvEE, mJtf,
  // mdJtf
  setRightArmOptParams(_RightTargetPosition);

  setLeftOrientationOptParams(_LeftTargetRPY);
  setRightOrientationOptParams(_RightTargetRPY);

  // sets mPBal and mbBal
  // Needs mRot0, mRobot, mxyz0, mdxyz0, mdRot0, mKpCOM, mKvCOM, mJtf, mdJtf
  setBalanceOptParams(thref, dthref, ddthref);

  // set Regulation Opt Params
  mPPose = mWMatPose;
  mbPose << mWMatPose * (-mKpPose * (mqBody - mqBodyInit) - mKvPose * mdqBody);

  mPSpeedReg = mWMatSpeedReg;
  mbSpeedReg << -mWMatSpeedReg * mKvSpeedReg * mdqBody;

  mPReg = mWMatReg;
  mbReg.setZero();

  // set mMM and mhh
  // Needs mRobot, mJtf, mdJtf, mdqMin, mR
  computeDynamics();

  // ***************************** QP
  OptParams optParams;
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

  Eigen::VectorXd b(mbEER.rows() + mbOrR.rows() + mbEEL.rows() + mbOrL.rows() +
                        mbBal.rows() + mbPose.rows() + mbSpeedReg.rows() +
                        mbReg.rows(),
                    mbEER.cols());
  b << mbEER, mbOrR, mbEEL, mbOrL, mbBal, mbPose, mbSpeedReg, mbReg;
  optParams.P = P;
  optParams.b = b;

  // mTauLim = 1.6*mTauLim;
  // const std::vector<double> inequalityconstraintTol(19, 1e-3);
  // OptParams inequalityconstraintParams[2];
  // inequalityconstraintParams[0].P = Eigen::MatrixXd::Zero(19, 18);
  // inequalityconstraintParams[1].P = Eigen::MatrixXd::Zero(19, 18);
  // inequalityconstraintParams[0].b = Eigen::VectorXd::Zero(19);
  // inequalityconstraintParams[1].b = Eigen::VectorXd::Zero(19);

  // // Upper Limits
  // inequalityconstraintParams[0].P.bottomLeftCorner(17, 18) =
  // mMM.bottomLeftCorner(17, 18); inequalityconstraintParams[0].b.tail(17) =
  // -mhh.tail(17) + mTauLim.tail(17);

  // inequalityconstraintParams[0].P.row(0) = -0.5*mMM.row(0);
  // inequalityconstraintParams[0].b(0) = mhh(0)/2 + mTauLim(0)/2 - mR/mL*tau_0;

  // inequalityconstraintParams[0].P.row(1) = -0.5*mMM.row(0);
  // inequalityconstraintParams[0].b(1) = mhh(0)/2 + mTauLim(0)/2 + mR/mL*tau_0;

  // // Lower Limits
  // inequalityconstraintParams[1].P.bottomLeftCorner(17, 18) =
  // -mMM.bottomLeftCorner(17, 18); inequalityconstraintParams[1].b.tail(17) =
  // mhh.tail(17) + mTauLim.tail(17);

  // inequalityconstraintParams[1].P.row(0) = 0.5*mMM.row(0);
  // inequalityconstraintParams[1].b(0) = -mhh(0)/2 + mTauLim(0)/2 +
  // mR/mL*tau_0;

  // inequalityconstraintParams[1].P.row(1) = 0.5*mMM.row(0);
  // inequalityconstraintParams[1].b(1) = -mhh(0)/2 + mTauLim(0)/2 -
  // mR/mL*tau_0;

  const std::vector<double> inequalityconstraintTol(mOptDim, 1e-3);
  OptParams inequalityconstraintParams[2];
  inequalityconstraintParams[0].P = mMM;
  inequalityconstraintParams[1].P = -mMM;
  inequalityconstraintParams[0].b = -mhh + mTauLim;
  inequalityconstraintParams[1].b = mhh + mTauLim;

  // nlopt::opt opt(nlopt::LN_COBYLA, 30);
  nlopt::opt opt(nlopt::LD_SLSQP, mOptDim);
  double minf;
  opt.set_min_objective(optFunc, &optParams);
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

  // ************************************ Torques
  Eigen::VectorXd bodyTorques = mMM * mddqBodyRef + mhh;
  mForces(0) = -mR / mL * tau_0 - bodyTorques(0) / 2;
  mForces(1) = mR / mL * tau_0 - bodyTorques(0) / 2;
  mForces.tail(mOptDim - 1) = bodyTorques.tail(mOptDim - 1);

  const std::vector<size_t> index{6,  7,  8,  9,  10, 11, 12, 13, 14, 15,
                                  16, 17, 18, 19, 20, 21, 22, 23, 24};
  mRobot->setForces(index, mForces);

  if (mSteps < 0) {
    std::cout << "PEER: " << mPEER.rows() << " x " << mPEER.cols() << std::endl;
    std::cout << "PEEL: " << mPEEL.rows() << " x " << mPEEL.cols() << std::endl;
    std::cout << "PBal: " << mPBal.rows() << " x " << mPBal.cols() << std::endl;
    std::cout << "PPose: " << mPPose.rows() << " x " << mPPose.cols()
              << std::endl;
    std::cout << "PSpeedReg: " << mPSpeedReg.rows() << " x "
              << mPSpeedReg.cols() << std::endl;
    std::cout << "PReg: " << mPReg.rows() << " x " << mPReg.cols() << std::endl;
    // std::cout << "PxdotReg: " << PxdotReg.rows() << " x " << PxdotReg.cols()
    // << std::endl;
    std::cout << "bEER: " << mbEER.rows() << " x " << mbEER.cols() << std::endl;
    std::cout << "bEEL: " << mbEEL.rows() << " x " << mbEEL.cols() << std::endl;
    std::cout << "bBal: " << mbBal.rows() << " x " << mbBal.cols() << std::endl;
    std::cout << "bPose: " << mbPose.rows() << " x " << mbPose.cols()
              << std::endl;
    std::cout << "bSpeedReg: " << mbSpeedReg.rows() << " x "
              << mbSpeedReg.cols() << std::endl;
    std::cout << "bReg: " << mbReg.rows() << " x " << mbReg.cols() << std::endl;
    // std::cout << "bxdotReg: " << bxdotReg.rows() << " x " << bxdotReg.cols()
    // << std::endl;
  }

  if (mSteps < 0) {
    std::cout << "ddqBodyRef: " << std::endl;
    for (int i = 0; i < 18; i++) {
      std::cout << mddqBodyRef(i) << ", ";
    }
    std::cout << std::endl;
    std::cout << "ddqBodyRef_vec: " << std::endl;
    for (int i = 0; i < 18; i++) {
      std::cout << ddqBodyRef_vec[i] << ", ";
    }
    std::cout << std::endl;
  }

  // if(mSteps%(maxTimeSet==1?30:30) == 0)
  if (false) {
    std::cout << "mForces: " << mForces(0);
    for (int i = 1; i < 19; i++) {
      std::cout << ", " << mForces(i);
    }
    std::cout << std::endl;

    // Print the objective function components
    std::cout << "EEL loss: " << pow((mPEEL * mddqBodyRef - mbEEL).norm(), 2)
              << std::endl;
    std::cout << "EER loss: " << pow((mPEER * mddqBodyRef - mbEER).norm(), 2)
              << std::endl;
    std::cout << "OrL loss: " << pow((mPOrL * mddqBodyRef - mbOrL).norm(), 2)
              << std::endl;
    std::cout << "OrR loss: " << pow((mPOrR * mddqBodyRef - mbOrR).norm(), 2)
              << std::endl;
    std::cout << "Bal loss: " << pow((mPBal * mddqBodyRef - mbBal).norm(), 2)
              << std::endl;
    std::cout << "Pose loss: " << pow((mPPose * mddqBodyRef - mbPose).norm(), 2)
              << std::endl;
    std::cout << "Speed Reg loss: "
              << pow((mPSpeedReg * mddqBodyRef - mbSpeedReg).norm(), 2)
              << std::endl;
    std::cout << "Reg loss: " << pow((mPReg * mddqBodyRef - mbReg).norm(), 2)
              << std::endl;
  }
}

//==============================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const { return mRobot; }

//==============================================================================
dart::dynamics::BodyNode* Controller::getEndEffector(
    const std::string& s) const {
  if (!s.compare("left")) {
    return mLeftEndEffector;
  } else if (!s.compare("right")) {
    return mRightEndEffector;
  }
}

//==============================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {}

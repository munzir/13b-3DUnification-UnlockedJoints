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

#ifndef EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_
#define EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

#include <Eigen/Eigen>
#include <string>
#include <dart/dart.hpp>
#include <boost/circular_buffer.hpp>
#include <nlopt.hpp>
#include <string>
#include <config4cpp/Configuration.h>
#include <iostream>

// using namespace dart;
// using namespace std;  
// using namespace config4cpp;

class filter {
  public:
    filter(const int dim, const int n)
    {
      samples.set_capacity(n);
      total = Eigen::VectorXd::Zero(dim,1);
    }
    void AddSample(Eigen::VectorXd v)
    {
      if(samples.full()) 
      {
        total -= samples.front();
      }
      samples.push_back(v);
      total += v;
      average = total/samples.size();
    }
  
    boost::circular_buffer<Eigen::VectorXd> samples;
    Eigen::VectorXd total;
    Eigen::VectorXd average;
    
};

/// \brief Operational space controller for 6-dof manipulator
class Controller {
public:
  /// \brief Constructor
  Controller( dart::dynamics::SkeletonPtr _robot,
              dart::dynamics::BodyNode* _LeftendEffector,
              dart::dynamics::BodyNode* _RightendEffector);

  /// \brief Destructor
  virtual ~Controller();

  void updatePositions();

  void updateSpeeds();

  void updateTransformJacobian();

  void setLeftArmOptParams(const Eigen::Vector3d& _LeftTargetPosition);

  void setRightArmOptParams(const Eigen::Vector3d& _RightTargetPosition);

  void setLeftOrientationOptParams(const Eigen::Vector3d& _LeftTargetRPY);

  void setRightOrientationOptParams(const Eigen::Vector3d& _RightTargetRPY);

  void setBalanceOptParams(double ddthref);

  void computeDynamics();

  /// \brief
  void update(const Eigen::Vector3d& _LeftTargetPosition,const Eigen::Vector3d& _RightTargetPosition, \
    const Eigen::Vector3d& _LeftTargetRPY, const Eigen::Vector3d& _RightTargetRPY,double ddthref, double tau_0);

  /// \brief Get robot
  dart::dynamics::SkeletonPtr getRobot() const;

  /// \brief Get end effector of the robot
  dart::dynamics::BodyNode* getEndEffector(const std::string &s) const;

  /// \brief Keyboard control
  virtual void keyboard(unsigned char _key, int _x, int _y);

//private:
  /// \brief Robot
  dart::dynamics::SkeletonPtr mRobot;

  /// \brief Left End-effector of the robot
  dart::dynamics::BodyNode* mLeftEndEffector;

  /// \brief Right End-effector of the robot
  dart::dynamics::BodyNode* mRightEndEffector;

  /// \brief Control forces
  Eigen::Matrix<double, 19, 1> mForces;

  size_t mSteps;

  Eigen::Matrix<double, 18, 1> mddqBodyRef;

  double mZCOMInit;

  Eigen::Matrix<double, 18, 1> mqBodyInit;

  filter *mdqFilt;

  double mR, mL;

  Eigen::Matrix3d mKpEE, mKpOr;
  Eigen::Matrix3d mKvEE, mKvOr;
  double mKpCOM, mKvCOM;
  double mKvSpeedReg; 
  double mKpPose, mKvPose;

  double mWEER, mWOrR, mWEEL, mWOrL, mWSpeedReg, mWReg, mWPose;
  Eigen::Matrix<double, 3, 3> mWBal;
  Eigen::Matrix<double, 18, 18> mWMatPose;
  Eigen::Matrix<double, 18, 18> mWMatSpeedReg;
  Eigen::Matrix<double, 18, 18> mWMatReg;

  Eigen::Matrix<double, 4, 4> mBaseTf;
  Eigen::Matrix<double, 25, 1> mq;
  Eigen::Vector3d mxyz0; // position of frame 0 in the world frame represented in the world frame
  double mpsi;
  double mqBody1;
  Eigen::Matrix<double, 18, 1> mqBody; 
  
  Eigen::Matrix<double, 25, 1> mdq;
  Eigen::Vector3d mdxyz0;
  double mdx, mdqBody1, mdpsi;
  Eigen::Matrix<double, 18, 1> mdqBody;  
  Eigen::Matrix<double, 20, 1> mdqMin;

  Eigen::Matrix3d mRot0, mdRot0;

  Eigen::Matrix<double, 25, 20> mJtf, mdJtf;

  Eigen::Matrix<double, 3, 18> mPEEL, mPOrL, mPEER, mPOrR, mPBal;
  Eigen::Matrix<double, 3, 1> mbEEL, mbOrL, mbEER, mbOrR, mbBal;
  Eigen::Matrix<double, 18, 18> mPPose, mPSpeedReg, mPReg;
  Eigen::Matrix<double, 18, 1> mbPose, mbSpeedReg, mbReg;

  Eigen::Matrix<double, 3, 1> mZeroCol;
  Eigen::Matrix<double, 3, 7> mZero7Col;  

  Eigen::Matrix<double, 18, 18> mMM;
  Eigen::Matrix<double, 18, 1> mhh;

  Eigen::Matrix<double, 18, 1> mTauLim;

  bool maxTimeSet = 0;
};

#endif  // EXAMPLES_OPERATIONALSPACECONTROL_CONTROLLER_HPP_

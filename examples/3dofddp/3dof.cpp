#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <boost/circular_buffer.hpp>
#include <ddp/costs.hpp>
#include <ddp/ddp.hpp>
#include <ddp/mpc.hpp>
#include <ddp/util.hpp>
#include <nlopt.hpp>
#include "Controller.hpp"
#include "krangddp.h"
#include <config4cpp/Configuration.h>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace config4cpp;

class MyWindow : public dart::gui::SimWindow {
  using Scalar = double;  
  using Dynamics = Krang3D<Scalar>;
  using DDP_Opt = optimizer::DDP<Dynamics>;
  using Cost = Krang3DCost<Scalar>;
  using TerminalCost = Krang3DTerminalCost<Scalar>;
  using StateTrajectory = typename Dynamics::StateTrajectory ;
  using ControlTrajectory= typename Dynamics::ControlTrajectory ;
  using State = typename Dynamics::State;
  using Control = typename Dynamics::Control;

  public: 
    MyWindow(const WorldPtr& world) {

      // *********************************** Tunable Parameters
      Configuration *  cfg = Configuration::create();
      const char *     scope = "";
      const char *     configFile = "/home/panda/myfolder/wholebodycontrol/13b-3DUnification-UnlockedJoints/examples/3dofddp/controlParams.cfg";
      const char * str;
      std::istringstream stream;
      double newDouble;

      try {
        cfg->parse(configFile);
        
        mInitCOMAngle = (cfg->lookupFloat(scope, "initCOMAngle"))*M_PI/180.0;

        // -- Gains
        // mKpEE(0, 0) = cfg->lookupFloat(scope, "KpEE"); mKpEE(1, 1) = mKpEE(0, 0); mKpEE(2, 2) = mKpEE(0, 0);
        // mKvEE(0, 0) = cfg->lookupFloat(scope, "KvEE"); mKvEE(1, 1) = mKvEE(0, 0); mKvEE(2, 2) = mKvEE(0, 0);
        // mKpOr(0, 0) = cfg->lookupFloat(scope, "KpOr"); mKpOr(1, 1) = mKpOr(0, 0); mKpOr(2, 2) = mKpOr(0, 0);
        // mKvOr(0, 0) = cfg->lookupFloat(scope, "KvOr"); mKvOr(1, 1) = mKvOr(0, 0); mKvOr(2, 2) = mKvOr(0, 0);
        // mKpCOM = cfg->lookupFloat(scope, "KpCOM");
        // mKvCOM = cfg->lookupFloat(scope, "KvCOM");
        // mKvSpeedReg = cfg->lookupFloat(scope, "KvSpeedReg");
        // mKpPose = cfg->lookupFloat(scope, "KpPose");
        // mKvPose = cfg->lookupFloat(scope, "KvPose");
        
        // // -- Weights
        // // Right Arm
        // mWEER = cfg->lookupFloat(scope, "wEER");
        // mWOrR = cfg->lookupFloat(scope, "wOrR");
        // // Left Arm
        // mWEEL = cfg->lookupFloat(scope, "wEEL");
        // mWOrL = cfg->lookupFloat(scope, "wOrL");
        // // Balance
        // // str = cfg->lookupString(scope, "wBal"); 
        // // stream.str(str);
        // // for(int i=0; i<3; i++) stream >> mWBal(i, i);
        // // stream.clear();
        // mWBal = cfg->lookupFloat(scope, "wBal");
        // // Regulation
        // const char* s[] = {"wRegBase", "wRegWaist", "wRegTorso", "wRegKinect", \
        //  "wRegArm1", "wRegArm2", "wRegArm3", "wRegArm4", "wRegArm5", "wRegArm6", "wRegArm7"};
        // for(int i=0; i<11; i++) {
        //   str = cfg->lookupString(scope, s[i]); 
        //   stream.str(str);
        //   stream >> mWMatPose(i, i);     
        //   stream >> mWMatSpeedReg(i, i); 
        //   stream >> mWMatReg(i, i);      
        //   if(i > 3){
        //     mWMatPose(i+7, i+7) = mWMatPose(i, i);
        //     mWMatSpeedReg(i+7, i+7) = mWMatSpeedReg(i, i); 
        //     mWMatReg(i+7, i+7) = mWMatReg(i, i); 
        //   }
        //   stream.clear();
        // }
      } catch(const ConfigurationException & ex) {
          cerr << ex.c_str() << endl;
          cfg->destroy();
      }

      cout << "8" << endl;
  
      // Attach the world passed in the input argument to the window, and fetch the robot from the world
      setWorld(world);
      mkrang = world->getSkeleton("krang");

      // Adjust Init COM Angle
      Eigen::Matrix3d baseRot; double psi, qBody1; Eigen::Transform<double, 3, Eigen::Affine> baseTf; Eigen::AngleAxisd aa; Eigen::Matrix<double, 25, 1> q;
      baseRot = mkrang->getBodyNode("Base")->getTransform().rotation();
      psi = atan2(baseRot(0, 0), -baseRot(1, 0));
      qBody1 = atan2(baseRot(2, 2), baseRot(2, 1));
      qBody1 += mInitCOMAngle;
      baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
      baseTf.prerotate(Eigen::AngleAxisd(-qBody1,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+psi,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
      aa = Eigen::AngleAxisd(baseTf.rotation());
      q << aa.angle()*aa.axis(), mkrang->getPositions().tail(22);
      mkrang->setPositions(q);

      // Initialize the simplified robot
      m3DOF = create3DOF_URDF(mkrang);
      mWorld3dof = std::make_shared<World>();
      mWorld3dof->addSkeleton(m3DOF);
      getSimple(m3DOF, mkrang); 
      

      mSteps = 0;
      mMPCSteps = -1; 
      mMPCdt = 0.01;
      mdqFilt = new filter(8, 50);
      mR = 0.25;
      mL = 0.68;//*6;
      char mpctrajfile[] = "mpc_traj.csv";
      mMPCWriter.open_file(mpctrajfile);
      cout << "9" << endl;

      computeDDPTrajectory();
      cout << "10" << endl;

      mController = new Controller(mkrang, mkrang->getBodyNode("lGripper"), mkrang->getBodyNode("rGripper") ) ;
      
      // Targets for the controller
      // baseTf = mController->mRobot->getBodyNode(0)->getTransform();
      // double psi =  atan2(baseTf(0,0), -baseTf(1,0));
      Eigen::Matrix3d Rot0; Eigen::Vector3d xyz0;
      Rot0 << cos(psi), sin(psi), 0,
              -sin(psi), cos(psi), 0,
              0, 0, 1;
      xyz0 = (mController->mRobot->getPositions()).segment(3,3);
      mLeftTargetPosition = Rot0*(mController->getEndEffector("left")->getTransform().translation() - xyz0);
      mRightTargetPosition = Rot0*(mController->getEndEffector("right")->getTransform().translation() - xyz0);
      mLeftTargetRPY = dart::math::matrixToEulerXYZ(Rot0*mController->getEndEffector("left")->getTransform().rotation());
      mRightTargetRPY = dart::math::matrixToEulerXYZ(Rot0*mController->getEndEffector("right")->getTransform().rotation());
    }

    void drawWorld() const override;

    void keyboard(unsigned char _key, int _x, int _y) override;

    SkeletonPtr create3DOF_URDF(SkeletonPtr krang);

    Eigen::Vector3d getBodyCOM(dart::dynamics::SkeletonPtr robot);

    void getSimple(SkeletonPtr& threeDOF, SkeletonPtr& krang);

    State getCurrentState();

    void computeDDPTrajectory();

    void timeStepping() override;

    ~MyWindow() {}

  protected:

    /// Full robot and low level controller
    SkeletonPtr mkrang;
    Controller* mController;
    Eigen::Vector3d mLeftTargetPosition;
    Eigen::Vector3d mRightTargetPosition;
    Eigen::Vector3d mLeftTargetRPY;
    Eigen::Vector3d mRightTargetRPY;
    double mInitCOMAngle;

    // 3DOF robot
    WorldPtr mWorld3dof;
    SkeletonPtr m3DOF;
    Eigen::Matrix<double, 2, 1> mForces;   

    // MPC DDP states
    // double psi, dpsi, qBody1, dqBody1, dthL, dthR;
    // double psiFilt, dpsiFilt, qBody1Filt, dqBody1Filt, dthLFilt, dthRFilt;
    filter *mdqFilt;
    double mR;
    double mL;
    ControlTrajectory mDDPControlTraj;
    StateTrajectory mDDPStateTraj;
    Dynamics *mDDPDynamics;
    Control mMPCControlRef;
    State mMPCStateRef;
    int mSteps;
    int mMPCSteps;
    double mMPCdt;
    CSV_writer<Scalar> mMPCWriter;
};

//====================================================================
void MyWindow::drawWorld() const {
  // Draw the target position
  if (mRI) {
    // Get Frame 0 Transformation in Tf0
    Eigen::Matrix<double, 4, 4> baseTf = mController->mRobot->getBodyNode(0)->getTransform().matrix();
    double psi =  atan2(baseTf(0,0), -baseTf(1,0));
    Eigen::Transform<double, 3, Eigen::Affine> Tf0 = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    Tf0.rotate(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d Rot = Tf0.matrix().block<3, 3>(0, 0);
    Eigen::Vector3d xyz0 = (mController->mRobot->getPositions()).segment(3,3);

    // Draw Left Target Frame
    Eigen::Matrix3d mat;
    mat =  Eigen::AngleAxisd(mLeftTargetRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(mLeftTargetRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(mLeftTargetRPY(2), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d localTranslation; 
    double axisLength = 0.1;
    double axisWidth = 0.003;
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mLeftTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mLeftTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mLeftTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();

    // Draw Right Target Frame
    mat =  Eigen::AngleAxisd(mRightTargetRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(mRightTargetRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(mRightTargetRPY(2), Eigen::Vector3d::UnitZ());
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mRightTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mRightTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot*(mRightTargetPosition + mat*localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();


    // Draw Left End-Effector Frame
    Eigen::Vector3d eeRPY = dart::math::matrixToEulerXYZ(mController->getEndEffector("left")->getTransform().rotation());
    Eigen::Vector3d eePosition = mController->getEndEffector("left")->getTransform().translation();
    
    mat =  Eigen::AngleAxisd(eeRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(eeRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(eeRPY(2), Eigen::Vector3d::UnitZ());
    axisLength = 0.08;
    axisWidth = 0.006;
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();


    // Draw Right End-Effector Frame
    eeRPY = dart::math::matrixToEulerXYZ(mController->getEndEffector("right")->getTransform().rotation());
    eePosition = mController->getEndEffector("right")->getTransform().translation();
    
    mat =  Eigen::AngleAxisd(eeRPY(0), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(eeRPY(1), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(eeRPY(2), Eigen::Vector3d::UnitZ());
    axisLength = 0.08;
    axisWidth = 0.006;
    
    localTranslation << axisLength/2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength/2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength/2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat*localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1)*180/M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2)*180/M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();


    // mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    // mRI->pushMatrix();
    // mRI->translate( \
    //   (mController->mRobot->getPositions()).segment(3,3) \
    //   + Tf0.matrix().block<3, 3>(0, 0)*mLeftTargetPosition);
    // mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    // mRI->popMatrix();    

    // mRI->setPenColor(Eigen::Vector3d(0.0, 0.4, 0.2));
    // mRI->pushMatrix();
    // mRI->translate( \
    //   (mController->mRobot->getPositions()).segment(3,3) \
    //   + Tf0.matrix().block<3, 3>(0, 0)*mRightTargetPosition);
    // mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    // mRI->popMatrix();   

    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(mWorld->getSkeleton("krang")->getCOM());
    mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    mRI->popMatrix();    
    

  }

  // Draw world
  SimWindow::drawWorld();
}

//====================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  double incremental = 0.01;
  Eigen::Matrix3d rot;
  // Eigen::IOFormat commaSep(Eigen::StreamPrecision, 0, ", ", "\n", "", "", "",  "");

  switch (_key) {
    // case 'c':  // print debug information
    //   if (mCircleTask) {
    //     std::cout << "Circle task [off]." << std::endl;
    //     mCircleTask = false;
    //   }
    //   else {
    //     std::cout << "Circle task [on]." << std::endl;
    //     mCircleTask = true;
    //   }
    //   break;

    case 'q':
      mLeftTargetPosition[0] -= incremental;
      break;
    case 'w':
      mLeftTargetPosition[0] += incremental;
      break;
    case 'a':
      mLeftTargetPosition[1] -= incremental;
      break;
    case 's':
      mLeftTargetPosition[1] += incremental;
      break;
    case 'z':
      mLeftTargetPosition[2] -= incremental;
      break;
    case 'x':
      mLeftTargetPosition[2] += incremental;
      break;
    
    case '[':
      mRightTargetPosition[0] -= incremental;
      break;
    case ']':
      mRightTargetPosition[0] += incremental;
      break;
    case ';':
      mRightTargetPosition[1] -= incremental;
      break;
    case '\'':
      mRightTargetPosition[1] += incremental;
      break;
    case '.':
      mRightTargetPosition[2] -= incremental;
      break;
    case '/':
      mRightTargetPosition[2] += incremental;
      break;
    
    case 'r':
      mLeftTargetRPY[0] -= incremental;
      break;
    case 't':
      mLeftTargetRPY[0] += incremental;
      break;
    case 'f':
      mLeftTargetRPY[1] -= incremental;
      break;
    case 'g':
      mLeftTargetRPY[1] += incremental;
      break;
    case 'c':
      mLeftTargetRPY[2] -= incremental;
      break;
    case 'b':
      mLeftTargetRPY[2] += incremental;
      break;

    case 'i':
      mRightTargetRPY[0] -= incremental;
      break;
    case 'o':
      mRightTargetRPY[0] += incremental;
      break;
    case 'j':
      mRightTargetRPY[1] -= incremental;
      break;
    case 'k':
      mRightTargetRPY[1] += incremental;
      break;
    case 'n':
      mRightTargetRPY[2] -= incremental;
      break;
    case 'm':
      mRightTargetRPY[2] += incremental;
      break;

    case 'd':
      cout << (mController->mRobot->getPositions().transpose()) << endl;
      break;

    case 'e':
      rot <<  1, 0,  0,
              0, 0, -1,
              0, 1,  0;
      mLeftTargetRPY = dart::math::matrixToEulerXYZ(rot);
      break;

    case 'p':
      rot << -1, 0, 0,
              0, 0, 1,
              0, 1, 0;
      mRightTargetRPY = dart::math::matrixToEulerXYZ(rot);      
      break;

    case 'h':
      mLeftTargetPosition[0] = (mLeftTargetPosition[0] + mRightTargetPosition[0])/2;
      mRightTargetPosition[0] = mLeftTargetPosition[0];

      mLeftTargetPosition[1] = (mLeftTargetPosition[1] - mRightTargetPosition[1])/2;
      mRightTargetPosition[1] = -mLeftTargetPosition[1];

      mLeftTargetPosition[2] = (mLeftTargetPosition[2] + mRightTargetPosition[2])/2;
      mRightTargetPosition[2] = mLeftTargetPosition[2];

      cout << "left Ref: " << mLeftTargetPosition << endl;
      cout << "right Ref: " << mRightTargetPosition << endl;
      break;
      

    default:
      // Default keyboard control
      SimWindow::keyboard(_key, _x, _y);
      break;
  }

  // Keyboard control for Controller
  mController->keyboard(_key, _x, _y);

  glutPostRedisplay();
}

//====================================================================
SkeletonPtr MyWindow::create3DOF_URDF(SkeletonPtr krang) {
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  SkeletonPtr threeDOF = 
      loader.parseSkeleton("/home/panda/myfolder/wholebodycontrol/09-URDF/3DOF-WIP/3dof.urdf");
  threeDOF->setName("m3DOF");
  
  threeDOF->getJoint(0)->setDampingCoefficient(0, 0.5);
  threeDOF->getJoint(1)->setDampingCoefficient(0, 0.5);

  return threeDOF;
}

//====================================================================
Eigen::Vector3d MyWindow::getBodyCOM(dart::dynamics::SkeletonPtr robot) {
  double fullMass = robot->getMass();
  double wheelMass = robot->getBodyNode("LWheel")->getMass();
  return (fullMass*robot->getCOM() - wheelMass*robot->getBodyNode("LWheel")->getCOM() - wheelMass*robot->getBodyNode("RWheel")->getCOM())/(fullMass - 2*wheelMass);
}

//====================================================================
void MyWindow::getSimple(SkeletonPtr& threeDOF, SkeletonPtr& krang) {
  // Load the full body with fixed wheel and set the pose q
  // dart::utils::DartLoader loader;
  // SkeletonPtr krangFixedWheel =
  //     loader.parseSkeleton("/home/krang/dart/09-URDF/KrangFixedWheels/krang_fixed_wheel.urdf");
  
  // Body Mass
  double mFull = krang->getMass(); 
  double mLWheel = krang->getBodyNode("LWheel")->getMass();
  double mRWheel = krang->getBodyNode("RWheel")->getMass();
  double mBody = mFull - mLWheel - mRWheel;


  Eigen::Vector3d bodyCOM;
  dart::dynamics::Frame* baseFrame = krang->getBodyNode("Base");
  bodyCOM = (mFull*krang->getCOM(baseFrame) - mLWheel*krang->getBodyNode("LWheel")->getCOM(baseFrame) - mLWheel*krang->getBodyNode("RWheel")->getCOM(baseFrame))/(mFull - mLWheel - mRWheel);

  // Body inertia (axis)
  double m;
  Eigen::Matrix3d iMat;
  Eigen::Matrix3d iBody = Eigen::Matrix3d::Zero();
  double ixx, iyy, izz, ixy, ixz, iyz;  
  Eigen::Matrix3d rot;
  Eigen::Vector3d t;
  Eigen::Matrix3d tMat;
  dart::dynamics::BodyNodePtr b;
  int nBodies = krang->getNumBodyNodes();
  for(int i=0; i<nBodies; i++){
    if(i==1 || i==2) continue; // Skip wheels
    b = krang->getBodyNode(i);
    b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = b->getTransform(baseFrame).rotation();
    t = krang->getCOM(baseFrame) - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
    m = b->getMass();
    iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
            ixy, iyy, iyz,
            ixz, iyz, izz;
    iMat = rot*iMat*rot.transpose(); // Inertia tensor of the body around its CoM expressed in base frame
    tMat << (t(1)*t(1)+t(2)*t(2)), (-t(0)*t(1)),          (-t(0)*t(2)),
            (-t(0)*t(1)),          (t(0)*t(0)+t(2)*t(2)), (-t(1)*t(2)),
            (-t(0)*t(2)),          (-t(1)*t(2)),          (t(0)*t(0)+t(1)*t(1));
    iMat = iMat + m*tMat; // Parallel Axis Theorem
    iBody += iMat;
  }


  // Aligning threeDOF base frame to have the y-axis pass through the CoM
  double th = atan2(bodyCOM(2), bodyCOM(1));
  rot << 1, 0, 0,
         0, cos(th), sin(th),
         0, -sin(th), cos(th);
  bodyCOM = rot*bodyCOM;
  iBody = rot*iBody*rot.transpose();

  // Set the 3 DOF robot parameters
  threeDOF->getBodyNode("Base")->setMomentOfInertia(iBody(0,0), iBody(1,1), iBody(2,2), iBody(0,1), iBody(0,2), iBody(1,2));
  threeDOF->getBodyNode("Base")->setLocalCOM(bodyCOM);
  threeDOF->getBodyNode("Base")->setMass(mBody);

  // Print them out
  // cout << "mass: " << mBody << endl;
  // cout << "COM: " << bodyCOM(0) << ", " << bodyCOM(1) << ", " << bodyCOM(2) << endl;
  // cout << "ixx: " << iBody(0,0) << ", iyy: " << iBody(1,1) << ", izz: " << iBody(2,2) << endl;
  // cout << "ixy: " << iBody(0,1) << ", ixz: " << iBody(0,2) << ", iyz: " << iBody(1,2) << endl;

  // Update 3DOF state
  // get positions
  Eigen::Matrix3d baseRot = krang->getBodyNode("Base")->getTransform().rotation();
  baseRot = baseRot*rot.transpose();
  Eigen::AngleAxisd aa(baseRot);
  Eigen::Matrix<double, 8, 1> q, dq;
  q << aa.angle()*aa.axis(), krang->getPositions().segment(3, 5);
  threeDOF->setPositions(q);

  // TODO: When joints are unlocked qBody1 of the 3DOF (= dth = COM angular speed) is not the same as qBody1 of the full robot
  dq << rot*krang->getVelocities().head(3), rot*krang->getVelocities().segment(3, 3), krang->getVelocities().segment(6, 2);
  threeDOF->setVelocities(dq);
}

//====================================================================
Krang3D<double>::State MyWindow::getCurrentState() {

  Eigen::Matrix<double, 4, 4> Tf;
  double psi, qBody1, dpsi, dpsiFilt, dqBody1, dqBody1Filt, thL, dthL, dthLFilt, thR, dthR, dthRFilt;
  Eigen::Matrix<double, 8, 1> q, dq_orig, dq;
  State currentState = Dynamics::State::Zero();

  // Read Positions, Speeds, Transform speeds to world coordinates and filter the speeds      
  Tf = m3DOF->getBodyNode(0)->getTransform().matrix();
  psi =  atan2(Tf(0,0), -Tf(1,0));
  qBody1 = atan2(Tf(0,1)*cos(psi) + Tf(1,1)*sin(psi), Tf(2,1));
  q = m3DOF->getPositions();
  dq_orig = m3DOF->getVelocities();
  dq << (Tf.block<3,3>(0,0) * dq_orig.head(3)) , (Tf.block<3,3>(0,0) * dq_orig.segment(3,3)), dq_orig(6), dq_orig(7);
  mdqFilt->AddSample(dq);

  // Calculate the quantities we are interested in
  dpsi = dq(2);
  dpsiFilt = mdqFilt->average(2);
  dqBody1 = -dq_orig(0);
  dqBody1Filt = (-mdqFilt->average(0)*sin(psi) + mdqFilt->average(1)*cos(psi));
  thL = q(6) + qBody1;
  dthL = dq(6) + dqBody1;
  dthLFilt = mdqFilt->average(6) + dqBody1Filt;
  thR = q(7) + qBody1;
  dthR = dq(7) + dqBody1;
  dthRFilt = mdqFilt->average(7) + dqBody1Filt;

  // State: x, psi, theta, dx, dpsi, dtheta, x0, y0
  currentState << mR/2 * (thL + thR), psi, qBody1, dq(3) * cos(psi) + dq(4) * sin(psi), dpsi, dqBody1, q(3), q(4); 
  return currentState;
}

//====================================================================
void MyWindow::computeDDPTrajectory() {
  cout << "9a" << endl;

  param p; 
  double ixx, iyy, izz, ixy, ixz, iyz; 
  Eigen::Vector3d com;
  Eigen::Matrix3d iMat;      
  Eigen::Matrix3d tMat;
        cout << "9b" << endl;

  dart::dynamics::Frame* baseFrame = m3DOF->getBodyNode("Base");
  p.R = 2.500000e-01; p.L = 6.000000e-01; p.g=9.800000e+00;
        cout << "9c" << endl;

  p.mw = m3DOF->getBodyNode("LWheel")->getMass(); 
        cout << "9d" << endl;

  m3DOF->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
        cout << "9e" << endl;

  p.YYw = ixx; p.ZZw = izz; p.XXw = iyy; // Wheel frame of reference in ddp dynamic model is different from the one in DART
  p.m_1 = m3DOF->getBodyNode("Base")->getMass(); 
  com = m3DOF->getBodyNode("Base")->getCOM(baseFrame);
  p.MX_1 = p.m_1*com(0); p.MY_1 = p.m_1*com(1); p.MZ_1 = p.m_1*com(2);
        cout << "9f" << endl;

  m3DOF->getBodyNode("Base")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  Eigen::Vector3d s = -com; // Position vector from local COM to body COM expressed in base frame
  iMat << ixx, ixy, ixz, // Inertia tensor of the body around its CoM expressed in body frame
          ixy, iyy, iyz,
          ixz, iyz, izz;
  tMat << (s(1)*s(1)+s(2)*s(2)), (-s(0)*s(1)),          (-s(0)*s(2)),
          (-s(0)*s(1)),          (s(0)*s(0)+s(2)*s(2)), (-s(1)*s(2)),
          (-s(0)*s(2)),          (-s(1)*s(2)),          (s(0)*s(0)+s(1)*s(1));
  iMat = iMat + p.m_1*tMat; // Parallel Axis Theorem
  p.XX_1 = iMat(0,0); p.YY_1 = iMat(1,1); p.ZZ_1 = iMat(2,2);
  p.XY_1 = iMat(0,1); p.YZ_1 = iMat(1,2); p.XZ_1 = iMat(0,2);
  p.fric_1 = m3DOF->getJoint(0)->getDampingCoefficient(0); // Assuming both joints have same friction coeff (Please make sure that is true)
        cout << "9g" << endl;

  CSV_writer<Scalar> writer;
  util::DefaultLogger logger;
  bool verbose = true;
  Scalar tf = 20;
  auto time_steps = util::time_steps(tf, mMPCdt);
  int max_iterations = 15;
  

  mDDPDynamics = new Dynamics(p);
  
  // Initial state 
  State x0 = getCurrentState();
  x0 << 0, 0, x0(2), 0, 0, 0, 0, 0; 
  cout << x0 << endl;
  // Dynamics::State xf; xf << 2, 0, 0, 0, 0, 0, 0.01, 5;
  Dynamics::State xf; xf << 5, 0, 0, 0, 0, 0, 5, 0;
  Dynamics::ControlTrajectory u = Dynamics::ControlTrajectory::Zero(2, time_steps);

  // Costs
  Cost::StateHessian Q;
  Q.setZero();
  Q.diagonal() << 0,0.1,0.1,0.1,0.1,0.1,0.1,0.1;

  Cost::ControlHessian R;
  R.setZero();
  R.diagonal() << 0.01, 0.01;

  TerminalCost::Hessian Qf;
  Qf.setZero();
  Qf.diagonal() << 0,1e4,1e4,1e4,1e4,1e4,1e4,1e4;

  Cost cp_cost(xf, Q, R);
  TerminalCost cp_terminal_cost(xf, Qf);

  // initialize DDP for trajectory planning
  DDP_Opt trej_ddp (mMPCdt, time_steps, max_iterations, &logger, verbose);

  cout << "9h" << endl;
  // Get initial trajectory from DDP
  OptimizerResult<Dynamics> DDP_traj = trej_ddp.run(x0, u, *mDDPDynamics, cp_cost, cp_terminal_cost);

  cout << "9i" << endl;

  mDDPStateTraj = DDP_traj.state_trajectory;
  mDDPControlTraj = DDP_traj.control_trajectory;

  writer.save_trajectory(mDDPStateTraj, mDDPControlTraj, "initial_traj.csv");
}

//====================================================================
void MyWindow::timeStepping() {
  mSteps++;

  getSimple(m3DOF, mkrang);
  State cur_state = getCurrentState(); 

  // MPC DDP RECEDING HORIZON CALCULATION
  int beginStep = 10; 
  int cur_mpc_steps = ((mSteps > beginStep) ? ((mSteps - beginStep) / 10) : -1);

  if (cur_mpc_steps > mMPCSteps) {
    mMPCSteps = cur_mpc_steps;
    int max_iterations = 15; 
    bool verbose = true; 
    util::DefaultLogger logger;
    int mpc_horizon = 10; 
    
    Dynamics::State target_state;
    target_state = mDDPStateTraj.col(mMPCSteps + mpc_horizon);
    Dynamics::ControlTrajectory hor_control = Dynamics::ControlTrajectory::Zero(2, mpc_horizon);
    Dynamics::StateTrajectory hor_traj_states = mDDPStateTraj.block(0, mMPCSteps, 8, mpc_horizon);
    
    DDP_Opt ddp_horizon (mMPCdt, mpc_horizon, max_iterations, &logger, verbose);
    
    Cost::StateHessian Q_mpc, Qf_mpc;
    Cost::ControlHessian ctl_R;
    
    ctl_R.setZero();
    ctl_R.diagonal() << 0.01, 0.01;
    Q_mpc.setZero();
    Q_mpc.diagonal() << 0, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    Qf_mpc.setZero();
    Qf_mpc.diagonal() << 0, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4, 1e4;
    Cost running_cost_horizon(target_state, Q_mpc, ctl_R);
    TerminalCost terminal_cost_horizon(target_state, Qf_mpc);
    
    OptimizerResult<Dynamics> results_horizon;
    results_horizon.control_trajectory = hor_control;
    
    results_horizon = ddp_horizon.run_horizon(cur_state, hor_control, hor_traj_states, *mDDPDynamics, running_cost_horizon, terminal_cost_horizon);
    mMPCControlRef = results_horizon.control_trajectory.col(0);
    mMPCStateRef = results_horizon.state_trajectory.col(1);


    mMPCWriter.save_step(cur_state, mMPCControlRef);

  }

  double tau_L = 0, tau_R = 0;
  if(mMPCSteps > -1) {

    double thref, dthref, ddthref, tau_0;
    ddthref = mMPCControlRef(0);
    tau_0 = mMPCControlRef(1);
    thref = mMPCStateRef(2);
    dthref = mMPCStateRef(5);
    mController->update(mLeftTargetPosition, mRightTargetPosition, mLeftTargetRPY, mRightTargetRPY, thref, dthref, ddthref, tau_0);

    // // *************************************** IDEA 2
    // double ddth = u(0);
    // double tau_0 = u(1);
    // State xdot = mDDPDynamics->f(cur_state, u);
    // double ddx = xdot(3);
    // double ddpsi = xdot(4);
    // Eigen::Vector3d ddq, dq;
    // ddq << ddx, ddpsi, ddth;
    // dq = cur_state.segment(3,3);
    // c_forces dy_forces = mDDPDynamics->dynamic_forces(cur_state, u);
    // //double tau_1 = (dy_forces.A.block<1,3>(2,0)*ddq) + (dy_forces.C.block<1,3>(2,0)*dq) + (dy_forces.Q(2)) - (dy_forces.Gamma_fric(2));
    // double tau_1 = dy_forces.A.block<1,3>(2,0)*ddq;
    // tau_1 += dy_forces.C.block<1,3>(2,0)*dq;
    // tau_1 += dy_forces.Q(2);
    // tau_1 -= dy_forces.Gamma_fric(2);
    // tau_L = -0.5*(tau_1+tau_0);
    // tau_R = -0.5*(tau_1-tau_0);

    // double tau_lim = 100.0;
    // if(abs(tau_L) > tau_lim | abs(tau_R) > tau_lim){
    //   cout << "step: " << steps << ", tau_0: " << tau_0 << ", tau_1: " << tau_1 << ", tau_L: " << tau_L << ", tau_R: " << tau_R << endl;
    // }

    // tau_L = min(tau_lim, max(-tau_lim, tau_L));
    // tau_R = min(tau_lim, max(-tau_lim, tau_R));
  }
  // mForces(0) = tau_L;
  // mForces(1) = tau_R;
  // const vector<size_t > index{6, 7};
  // mkrang->setForces(index, mForces);

  
  SimWindow::timeStepping();
}

//====================================================================
SkeletonPtr createFloor() {
  SkeletonPtr floor = Skeleton::create("floor");

  // Give the floor a body
  BodyNodePtr body =
      floor->createJointAndBodyNodePair<WeldJoint>(nullptr).second;
  //  body->setFrictionCoeff(1e16);

  // Give the body a shape
  double floor_width = 50;
  double floor_height = 0.05;
  std::shared_ptr<BoxShape> box(
        new BoxShape(Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode
      = body->createShapeNodeWith<VisualAspect, CollisionAspect, DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//====================================================================
dart::dynamics::SkeletonPtr createKrang() {
  
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang;
  ifstream file;
  char line [1024];
  std::istringstream stream;
  Eigen::Matrix<double, 24, 1> initPoseParams; // heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
  size_t i;
  double newDouble, headingInit, qBaseInit, qLWheelInit, qRWheelInit, qWaistInit, qTorsoInit, qKinectInit, th;
  Eigen::Vector3d xyzInit, COM;
  Eigen::Matrix<double, 7, 1> qLeftArmInit;
  Eigen::Matrix<double, 7, 1> qRightArmInit;
  Eigen::Transform<double, 3, Eigen::Affine> baseTf;
  Eigen::AngleAxisd aa;
  Eigen::Matrix<double, 25, 1> q;

  // Load the Skeleton from a file
  krang = loader.parseSkeleton("/home/panda/myfolder/wholebodycontrol/09-URDF/Krang/KrangOld.urdf");
  krang->setName("krang");

  // Read initial pose from the file
  file = ifstream("/home/panda/myfolder/wholebodycontrol/13b-3DUnification-UnlockedJoints/examples/3dofddp/defaultInit.txt");
  assert(file.is_open());
  file.getline(line, 1024);
  stream = std::istringstream(line);
  i = 0;
  while((i < 24) && (stream >> newDouble)) initPoseParams(i++) = newDouble;
  file.close();
  headingInit = initPoseParams(0);
  qBaseInit = initPoseParams(1);
  xyzInit << initPoseParams.segment(2,3);
  qLWheelInit = initPoseParams(5);
  qRWheelInit = initPoseParams(6);
  qWaistInit = initPoseParams(7);
  qTorsoInit = initPoseParams(8);
  qKinectInit = initPoseParams(9);
  qLeftArmInit << initPoseParams.segment(10, 7);
  qRightArmInit << initPoseParams.segment(17, 7);
  
  // Calculating the axis angle representation of orientation from headingInit and qBaseInit: 
  // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
  baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
  aa = Eigen::AngleAxisd(baseTf.rotation());
  
  // Set the positions and get the resulting COM angle
  q << aa.angle()*aa.axis(), xyzInit, qLWheelInit, qRWheelInit, qWaistInit, qTorsoInit, qKinectInit, qLeftArmInit, qRightArmInit; 
  krang->setPositions(q);
  COM = krang->getCOM() - xyzInit;
  th = atan2(COM(0), COM(2));
  
  // Adjust qBaseInit to bring COM on top of wheels and set the positions again 
  qBaseInit -= th;
  baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
  aa = Eigen::AngleAxisd(baseTf.rotation());
  q << aa.angle()*aa.axis(), xyzInit, qLWheelInit, qRWheelInit, qWaistInit, qTorsoInit, qKinectInit, qLeftArmInit, qRightArmInit; 
  krang->setPositions(q);

  // int joints = krang->getNumJoints();
  // for(int i=3; i < joints; i++) {
  //   krang->getJoint(i)->setActuatorType(dart::dynamics::Joint::ActuatorType::LOCKED);
  // }

  krang->getJoint(0)->setDampingCoefficient(0, 0.5);
  krang->getJoint(1)->setDampingCoefficient(0, 0.5);

  return krang;
}

//====================================================================
dart::dynamics::SkeletonPtr createTray(dart::dynamics::BodyNodePtr ee) {

  Eigen::Matrix3d EELRot, trayLocalRot, trayRot;
  Eigen::Vector3d EELPos, trayLocalTranslation, trayPos;
  Eigen::Matrix<double, 6, 1> qObject;
  
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr tray =
      loader.parseSkeleton("/home/panda/myfolder/wholebodycontrol/09-URDF/scenes/tray.urdf");
  tray->setName("tray");

  // Orientation
  EELRot = ee->getTransform().rotation();
  trayLocalRot << 1,  0, 0,
                  0,  0, 1,
                  0, -1, 0;
  trayRot = EELRot*trayLocalRot;
  Eigen::AngleAxisd aa(trayRot);
  
  // Position
  EELPos = ee->getTransform().translation();
  trayLocalTranslation << 0, -0.286303, 0.04;
  trayPos = EELPos + trayRot*trayLocalTranslation;

  // Set the position
  qObject << (aa.angle()*aa.axis()), trayPos;
  tray->setPositions(qObject);

  return tray;
}

//====================================================================
dart::dynamics::SkeletonPtr createCup(dart::dynamics::BodyNodePtr ee) {
  
  Eigen::Matrix3d EELRot, cupLocalRot, cupRot;
  Eigen::Vector3d EELPos, cupLocalTranslation, cupPos;
  Eigen::Matrix<double, 6, 1> qObject;
  
  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr cup =
      loader.parseSkeleton("/home/panda/myfolder/wholebodycontrol/09-URDF/scenes/cup.urdf");
  cup->setName("cup");

  // Orientation
  EELRot = ee->getTransform().rotation();
  cupLocalRot << 1,  0, 0,
                 0,  0, 1,
                 0, -1, 0;
  cupRot = EELRot*cupLocalRot;
  Eigen::AngleAxisd aa(cupRot);
  
  // Position
  EELPos = ee->getTransform().translation();
  cupLocalTranslation << 0, -0.286303, 0.04;
  cupPos = EELPos + cupRot*cupLocalTranslation;

  // Set the position
  qObject << (aa.angle()*aa.axis()), cupPos;
  cup->setPositions(qObject);

  return cup;
}

//====================================================================
int main(int argc, char* argv[]) {

  // create and initialize the world
  cout << "1" << endl;
  // SkeletonPtr threeDOF = create3DOF_URDF();
  cout << "2" << endl;

    // load skeletons
  SkeletonPtr floor = createFloor();
  SkeletonPtr robot = createKrang();
  cout << "3" << endl;

  SkeletonPtr tray = createTray(robot->getBodyNode("lGripper"));
  cout << "4" << endl;

  SkeletonPtr cup = createCup(robot->getBodyNode("lGripper")); //cup->setPositions(tray->getPositions());
  cout << "5" << endl;

  WorldPtr world = std::make_shared<World>();

  world->addSkeleton(floor); //add ground and robot to the world pointer
  world->addSkeleton(robot);
  world->addSkeleton(tray);
  world->addSkeleton(cup);
  cout << "6" << endl;

  // create and initialize the world
  //Eigen::Vector3d gravity(0.0,  -9.81, 0.0);
  //world->setGravity(gravity);
  // world->setTimeStep(1.0/1000);
  cout << "7" << endl; 

  MyWindow window(world);

  glutInit(&argc, argv);
  window.initWindow(1280,720, "3DOF URDF");
  glutMainLoop();

  return 0;
}
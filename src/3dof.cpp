#include <config4cpp/Configuration.h>
#include <dart/external/lodepng/lodepng.h>
#include <sys/stat.h>
#include <boost/circular_buffer.hpp>
#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <ddp/costs.hpp>
#include <ddp/ddp.hpp>
#include <ddp/mpc.hpp>
#include <ddp/util.hpp>
#include <fstream>
#include <iostream>
#include <nlopt.hpp>

#include <krang-qp/Controller.hpp>
#include "krangddp.h"

class MyWindow : public dart::gui::glut::SimWindow {
  using Scalar = double;
  using Dynamics = Krang3D<Scalar>;
  using DDP_Opt = optimizer::DDP<Dynamics>;
  using Cost = Krang3DCost<Scalar>;
  using TerminalCost = Krang3DTerminalCost<Scalar>;
  using StateTrajectory = typename Dynamics::StateTrajectory;
  using ControlTrajectory = typename Dynamics::ControlTrajectory;
  using State = typename Dynamics::State;
  using Control = typename Dynamics::Control;

 public:
  MyWindow(const dart::simulation::WorldPtr& world) {
    // Attach the world passed in the input argument to the window, and
    // fetch the robot from the world
    setWorld(world);
    mkrang = world->getSkeleton("krang");
    numDof = mkrang->getNumDofs();
    numTwipDof = 8;
    numBodyLinks = numDof - numTwipDof + 1;

    // *********************************** Tunable Parameters
    config4cpp::Configuration* cfg = config4cpp::Configuration::create();
    const char* scope = "";
    const char* configFile = "../../src/controlParams.cfg";
    const char* str;
    std::istringstream stream;
    double newDouble;

    numControls = 2;
    numStates = 8;

    mForces = Eigen::VectorXd(numControls);
    mDDPStatePenalties = Eigen::VectorXd(numStates);
    mDDPTerminalStatePenalties = Eigen::VectorXd(numStates);
    mDDPControlPenalties = Eigen::VectorXd(numControls);
    mMPCStatePenalties = Eigen::VectorXd(numStates);
    mMPCTerminalStatePenalties = Eigen::VectorXd(numStates);
    mMPCControlPenalties = Eigen::VectorXd(numControls);

    try {
      cfg->parse(configFile);

      strcpy(urdfpath3DOF, cfg->lookupString(scope, "urdfpath3DOF"));

      mLockedJoints = cfg->lookupBoolean(scope, "lockedJoints");

      mInitCOMAngle = (cfg->lookupFloat(scope, "initCOMAngle")) * M_PI / 180.0;

      str = cfg->lookupString(scope, "goalState");
      stream.str(str);
      for (int i = 0; i < numStates; i++) stream >> mGoalState(i);
      stream.clear();

      mFinalTime = cfg->lookupFloat(scope, "finalTime");

      mDDPMaxIter = cfg->lookupInt(scope, "DDPMaxIter");

      str = cfg->lookupString(scope, "DDPStatePenalties");
      stream.str(str);
      for (int i = 0; i < numStates; i++) stream >> mDDPStatePenalties(i);
      stream.clear();

      str = cfg->lookupString(scope, "DDPTerminalStatePenalties");
      stream.str(str);
      for (int i = 0; i < numStates; i++)
        stream >> mDDPTerminalStatePenalties(i);
      stream.clear();

      str = cfg->lookupString(scope, "DDPControlPenalties");
      stream.str(str);
      for (int i = 0; i < numControls; i++) stream >> mDDPControlPenalties(i);
      stream.clear();

      mBeginStep = cfg->lookupInt(scope, "beginStep");

      mMPCMaxIter = cfg->lookupInt(scope, "MPCMaxIter");

      mMPCHorizon = cfg->lookupInt(scope, "MPCHorizon");

      str = cfg->lookupString(scope, "MPCStatePenalties");
      stream.str(str);
      for (int i = 0; i < numStates; i++) stream >> mMPCStatePenalties(i);
      stream.clear();

      str = cfg->lookupString(scope, "MPCTerminalStatePenalties");
      stream.str(str);
      for (int i = 0; i < numStates; i++)
        stream >> mMPCTerminalStatePenalties(i);
      stream.clear();

      str = cfg->lookupString(scope, "MPCControlPenalties");
      stream.str(str);
      for (int i = 0; i < numControls; i++) stream >> mMPCControlPenalties(i);
      stream.clear();

      mTauLimBase = cfg->lookupFloat(scope, "tauLimBase");

      mContinuousZoom = cfg->lookupBoolean(scope, "continuousZoom");

      mWaistLocked = cfg->lookupBoolean(scope, "waistLocked");

      mCOMControlInLowLevel = cfg->lookupBoolean(scope, "COMControlInLowLevel");

    } catch (const config4cpp::ConfigurationException& ex) {
      std::cerr << ex.c_str() << std::endl;
      cfg->destroy();
    }
    std::cout << "urdfpath3DOF: " << urdfpath3DOF << std::endl;
    std::cout << "lockedJoints: " << (mLockedJoints ? "true" : "false")
              << std::endl;
    std::cout << "initCOMAngle: " << mInitCOMAngle << std::endl;
    std::cout << "goalState: " << mGoalState.transpose() << std::endl;
    std::cout << "finalTime: " << mFinalTime << std::endl;
    std::cout << "DDPMaxIter: " << mDDPMaxIter << std::endl;
    std::cout << "DDPStatePenalties: " << mDDPStatePenalties.transpose()
              << std::endl;
    std::cout << "DDPTerminalStatePenalties: "
              << mDDPTerminalStatePenalties.transpose() << std::endl;
    std::cout << "DDPControlPenalties: " << mDDPControlPenalties.transpose()
              << std::endl;
    std::cout << "beginStep: " << mBeginStep << std::endl;
    std::cout << "MPCMaxIter: " << mMPCMaxIter << std::endl;
    std::cout << "MPCHorizon: " << mMPCHorizon << std::endl;
    std::cout << "MPCStatePenalties: " << mMPCStatePenalties.transpose()
              << std::endl;
    std::cout << "MPCTerminalStatePenalties: "
              << mMPCTerminalStatePenalties.transpose() << std::endl;
    std::cout << "MPCControlPenalties: " << mMPCControlPenalties.transpose()
              << std::endl;
    std::cout << "tauLimBase: " << mTauLimBase << std::endl;
    std::cout << "continuousZoom: " << (mContinuousZoom ? "true" : "false")
              << std::endl;
    std::cout << "waistLocked: " << (mWaistLocked ? "true" : "false")
              << std::endl;
    std::cout << "COMControlInLowLevel: "
              << (mCOMControlInLowLevel ? "true" : "false") << std::endl;

    if (mLockedJoints) {
      int joints = mkrang->getNumJoints();
      for (int i = 3; i < joints; i++) {
        mkrang->getJoint(i)->setActuatorType(
            dart::dynamics::Joint::ActuatorType::LOCKED);
      }
    } else if (mWaistLocked) {
      mkrang->getJoint("JWaist")->setActuatorType(
          dart::dynamics::Joint::ActuatorType::LOCKED);
    }

    // Adjust Init COM Angle
    Eigen::Matrix3d baseRot;
    double psi, qBody1;
    Eigen::Transform<double, 3, Eigen::Affine> baseTf;
    Eigen::AngleAxisd aa;

    Eigen::VectorXd q(numDof);

    baseRot = mkrang->getBodyNode("Base")->getTransform().rotation();
    psi = atan2(baseRot(0, 0), -baseRot(1, 0));
    qBody1 = atan2(baseRot(2, 2), baseRot(2, 1));
    qBody1 += mInitCOMAngle;
    baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    baseTf.prerotate(Eigen::AngleAxisd(-qBody1, Eigen::Vector3d::UnitX()))
        .prerotate(Eigen::AngleAxisd(-M_PI / 2 + psi, Eigen::Vector3d::UnitY()))
        .prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
    aa = Eigen::AngleAxisd(baseTf.rotation());
    q << aa.angle() * aa.axis(), mkrang->getPositions().tail(numDof - 3);
    mkrang->setPositions(q);

    // Initialize the simplified robot
    m3DOF = create3DOF_URDF(mkrang, urdfpath3DOF);
    mWorld3dof = std::make_shared<dart::simulation::World>();
    mWorld3dof->addSkeleton(m3DOF);
    getSimple(m3DOF, mkrang);

    mSteps = 0;
    mMPCSteps = -1;
    mMPCdt = 0.01;
    mdqFilt = new filter(numStates, 50);
    mR = 0.25;
    mL = 0.68;  //*6;
    char mpctrajfile[] = "mpc_traj.csv";
    mMPCWriter.open_file(mpctrajfile);

    computeDDPTrajectory();

    mController = new Controller(mkrang, mkrang->getBodyNode("lGripper"),
                                 mkrang->getBodyNode("rGripper"));

    // Targets for the controller
    // baseTf = mController->mRobot->getBodyNode(0)->getTransform();
    // double psi =  atan2(baseTf(0,0), -baseTf(1,0));
    Eigen::Matrix3d Rot0;
    Eigen::Vector3d xyz0;
    Rot0 << cos(psi), sin(psi), 0, -sin(psi), cos(psi), 0, 0, 0, 1;
    xyz0 = (mController->mRobot->getPositions()).segment(3, 3);
    mLeftTargetPosition =
        Rot0 *
        (mController->getEndEffector("left")->getTransform().translation() -
         xyz0);
    mRightTargetPosition =
        Rot0 *
        (mController->getEndEffector("right")->getTransform().translation() -
         xyz0);
    mLeftTargetRPY = dart::math::matrixToEulerXYZ(
        Rot0 * mController->getEndEffector("left")->getTransform().rotation());
    mRightTargetRPY = dart::math::matrixToEulerXYZ(
        Rot0 * mController->getEndEffector("right")->getTransform().rotation());

    // Camera View
    mTrackBallRot << 0.370763, 0.897987, -0.236967, -0.273914, 0.349534,
        0.895989, 0.887415, -0.267292, 0.375566;
    mTrackBall.setQuaternion(Eigen::Quaterniond(mTrackBallRot));
    mZoom = 0.25;
  }

  void drawWorld() const override;

  void keyboard(unsigned char _key, int _x, int _y) override;

  dart::dynamics::SkeletonPtr create3DOF_URDF(dart::dynamics::SkeletonPtr krang,
                                              const char* urdfpath);

  Eigen::Vector3d getBodyCOM(dart::dynamics::SkeletonPtr robot);

  void getSimple(dart::dynamics::SkeletonPtr& threeDOF,
                 dart::dynamics::SkeletonPtr& krang);

  State getCurrentState();

  void computeDDPTrajectory();

  void timeStepping() override;

  void render() override;

  bool screenshot() override;

  ~MyWindow() {}

 protected:
  char urdfpath3DOF[1024];

  /// Full robot and low level controller
  dart::dynamics::SkeletonPtr mkrang;
  Controller* mController;
  Eigen::Vector3d mLeftTargetPosition;
  Eigen::Vector3d mRightTargetPosition;
  Eigen::Vector3d mLeftTargetRPY;
  Eigen::Vector3d mRightTargetRPY;
  double mInitCOMAngle;
  bool mLockedJoints;
  double mTauLimBase;
  bool mWaistLocked;

  // 3DOF robot
  dart::simulation::WorldPtr mWorld3dof;
  dart::dynamics::SkeletonPtr m3DOF;
  Eigen::VectorXd mForces;

  // MPC DDP states
  // double psi, dpsi, qBody1, dqBody1, dthL, dthR;
  // double psiFilt, dpsiFilt, qBody1Filt, dqBody1Filt, dthLFilt, dthRFilt;
  filter* mdqFilt;
  double mR;
  double mL;
  ControlTrajectory mDDPControlTraj;
  StateTrajectory mDDPStateTraj;
  Dynamics* mDDPDynamics;
  Control mMPCControlRef;
  State mMPCStateRef;
  int mSteps, mBeginStep;
  int mMPCSteps;
  double mMPCdt;
  CSV_writer<Scalar> mMPCWriter;
  State mGoalState;
  double mFinalTime;
  int mDDPMaxIter;
  Eigen::VectorXd mDDPStatePenalties;
  Eigen::VectorXd mDDPTerminalStatePenalties;
  Eigen::VectorXd mDDPControlPenalties;
  int mMPCMaxIter;
  int mMPCHorizon;
  Eigen::VectorXd mMPCStatePenalties;
  Eigen::VectorXd mMPCTerminalStatePenalties;
  Eigen::VectorXd mMPCControlPenalties;
  double mthref, mdthref;

  int numBodyLinks;
  int numControls;
  int numStates;

  int numDof;
  int numTwipDof;

  // Camera motion
  Eigen::Matrix3d mTrackBallRot;
  bool mContinuousZoom;

  bool mCOMControlInLowLevel;
};

//==============================================================================
void MyWindow::drawWorld() const {
  // Draw the target position
  if (mRI) {
    // Get Frame 0 Transformation in Tf0
    Eigen::Matrix<double, 4, 4> baseTf =
        mController->mRobot->getBodyNode(0)->getTransform().matrix();
    double psi = atan2(baseTf(0, 0), -baseTf(1, 0));
    Eigen::Transform<double, 3, Eigen::Affine> Tf0 =
        Eigen::Transform<double, 3, Eigen::Affine>::Identity();
    Tf0.rotate(Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()));
    Eigen::Matrix3d Rot = Tf0.matrix().block<3, 3>(0, 0);
    Eigen::Vector3d xyz0 = (mController->mRobot->getPositions()).segment(3, 3);

    // Draw Left Target Frame
    Eigen::Matrix3d mat;
    mat = Eigen::AngleAxisd(mLeftTargetRPY(0), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(mLeftTargetRPY(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(mLeftTargetRPY(2), Eigen::Vector3d::UnitZ());
    Eigen::Vector3d localTranslation;
    double axisLength = 0.1;
    double axisWidth = 0.003;

    localTranslation << axisLength / 2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot * (mLeftTargetPosition + mat * localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength / 2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot * (mLeftTargetPosition + mat * localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength / 2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(xyz0 + Rot * (mLeftTargetPosition + mat * localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mLeftTargetRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mLeftTargetRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mLeftTargetRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();

    // Draw Right Target Frame
    mat = Eigen::AngleAxisd(mRightTargetRPY(0), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(mRightTargetRPY(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(mRightTargetRPY(2), Eigen::Vector3d::UnitZ());

    localTranslation << axisLength / 2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 +
                   Rot * (mRightTargetPosition + mat * localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength / 2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(xyz0 +
                   Rot * (mRightTargetPosition + mat * localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength / 2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(xyz0 +
                   Rot * (mRightTargetPosition + mat * localTranslation));
    mRI->rotate(Eigen::Vector3d::UnitZ(), psi * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitX(), mRightTargetRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), mRightTargetRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), mRightTargetRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();

    // Draw Left End-Effector Frame
    Eigen::Vector3d eeRPY = dart::math::matrixToEulerXYZ(
        mController->getEndEffector("left")->getTransform().rotation());
    Eigen::Vector3d eePosition =
        mController->getEndEffector("left")->getTransform().translation();

    mat = Eigen::AngleAxisd(eeRPY(0), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(eeRPY(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(eeRPY(2), Eigen::Vector3d::UnitZ());
    axisLength = 0.08;
    axisWidth = 0.006;

    localTranslation << axisLength / 2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat * localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength / 2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat * localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength / 2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat * localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisWidth, axisLength));
    mRI->popMatrix();

    // Draw Right End-Effector Frame
    eeRPY = dart::math::matrixToEulerXYZ(
        mController->getEndEffector("right")->getTransform().rotation());
    eePosition =
        mController->getEndEffector("right")->getTransform().translation();

    mat = Eigen::AngleAxisd(eeRPY(0), Eigen::Vector3d::UnitX()) *
          Eigen::AngleAxisd(eeRPY(1), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(eeRPY(2), Eigen::Vector3d::UnitZ());
    axisLength = 0.08;
    axisWidth = 0.006;

    localTranslation << axisLength / 2, 0, 0;
    mRI->setPenColor(Eigen::Vector3d(0.8, 0.2, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat * localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisLength, axisWidth, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, axisLength / 2, 0;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.8, 0.2));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat * localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2) * 180 / M_PI);
    mRI->drawCube(Eigen::Vector3d(axisWidth, axisLength, axisWidth));
    mRI->popMatrix();

    localTranslation << 0, 0, axisLength / 2;
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(eePosition + mat * localTranslation);
    mRI->rotate(Eigen::Vector3d::UnitX(), eeRPY(0) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitY(), eeRPY(1) * 180 / M_PI);
    mRI->rotate(Eigen::Vector3d::UnitZ(), eeRPY(2) * 180 / M_PI);
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

    // Draw COM
    mRI->setPenColor(Eigen::Vector3d(0.2, 0.2, 0.8));
    mRI->pushMatrix();
    mRI->translate(mWorld->getSkeleton("krang")->getCOM());
    mRI->drawEllipsoid(Eigen::Vector3d(0.05, 0.05, 0.05));
    mRI->popMatrix();

    // Draw Scale along x-axis
    mRI->setPenColor(Eigen::Vector3d(0.9, 0.7, 0.7));
    mRI->pushMatrix();
    mRI->translate(Eigen::Vector3d(2.5, 0.0, 0.0));
    mRI->drawCube(Eigen::Vector3d(5, 0.01, 0.01));
    mRI->popMatrix();

    mRI->setPenColor(Eigen::Vector3d(0.9, 0.7, 0.7));
    for (int i = 0; i <= 5; i++) {
      mRI->pushMatrix();
      mRI->translate(Eigen::Vector3d(i, 0.0, 0.0));
      mRI->drawCube(Eigen::Vector3d(0.01, 1, 0.01));
      mRI->popMatrix();
    }
  }

  // Draw world
  SimWindow::drawWorld();
}

//==============================================================================
void MyWindow::keyboard(unsigned char _key, int _x, int _y) {
  double incremental = 0.01;
  Eigen::Matrix3d rot;
  // Eigen::IOFormat commaSep(Eigen::StreamPrecision, 0, ", ", "\n", "", "",
  // "",  "");

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

    case '-':
      mRightTargetPosition[0] -= incremental;
      break;
    case '=':
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
    case 'B':
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
      std::cout << (mController->mRobot->getPositions().transpose())
                << std::endl;
      break;

    case 'e':
      rot << 1, 0, 0, 0, 0, -1, 0, 1, 0;
      mLeftTargetRPY = dart::math::matrixToEulerXYZ(rot);
      break;

    case 'p':
      rot << -1, 0, 0, 0, 0, 1, 0, 1, 0;
      mRightTargetRPY = dart::math::matrixToEulerXYZ(rot);
      break;

    case 'h':
      mLeftTargetPosition[0] =
          (mLeftTargetPosition[0] + mRightTargetPosition[0]) / 2;
      mRightTargetPosition[0] = mLeftTargetPosition[0];

      mLeftTargetPosition[1] =
          (mLeftTargetPosition[1] - mRightTargetPosition[1]) / 2;
      mRightTargetPosition[1] = -mLeftTargetPosition[1];

      mLeftTargetPosition[2] =
          (mLeftTargetPosition[2] + mRightTargetPosition[2]) / 2;
      mRightTargetPosition[2] = mLeftTargetPosition[2];

      std::cout << "left Ref: " << mLeftTargetPosition << std::endl;
      std::cout << "right Ref: " << mRightTargetPosition << std::endl;
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

//==============================================================================
dart::dynamics::SkeletonPtr MyWindow::create3DOF_URDF(
    dart::dynamics::SkeletonPtr krang, const char* urdfpath) {
  char fullpath[1024];

  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr threeDOF = loader.parseSkeleton(urdfpath);
  threeDOF->setName("m3DOF");

  threeDOF->getJoint(0)->setDampingCoefficient(0, 0.5);
  threeDOF->getJoint(1)->setDampingCoefficient(0, 0.5);

  return threeDOF;
}

//==============================================================================
Eigen::Vector3d MyWindow::getBodyCOM(dart::dynamics::SkeletonPtr robot) {
  double fullMass = robot->getMass();
  double wheelMass = robot->getBodyNode("LWheel")->getMass();
  return (fullMass * robot->getCOM() -
          wheelMass * robot->getBodyNode("LWheel")->getCOM() -
          wheelMass * robot->getBodyNode("RWheel")->getCOM()) /
         (fullMass - 2 * wheelMass);
}

//==============================================================================
void MyWindow::getSimple(dart::dynamics::SkeletonPtr& threeDOF,
                         dart::dynamics::SkeletonPtr& krang) {
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
  bodyCOM = (mFull * krang->getCOM(baseFrame) -
             mLWheel * krang->getBodyNode("LWheel")->getCOM(baseFrame) -
             mLWheel * krang->getBodyNode("RWheel")->getCOM(baseFrame)) /
            (mFull - mLWheel - mRWheel);

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
  for (int i = 0; i < nBodies; i++) {
    if (i == 1 || i == 2) continue;  // Skip wheels
    b = krang->getBodyNode(i);
    b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = b->getTransform(baseFrame).rotation();
    t = krang->getCOM(baseFrame) -
        b->getCOM(baseFrame);  // Position vector from local COM to body COM
                               // expressed in base frame
    m = b->getMass();
    iMat << ixx, ixy, ixz,  // Inertia tensor of the body around its CoM
                            // expressed in body frame
        ixy, iyy, iyz, ixz, iyz, izz;
    iMat = rot * iMat * rot.transpose();  // Inertia tensor of the body around
                                          // its CoM expressed in base frame
    tMat << (t(1) * t(1) + t(2) * t(2)), (-t(0) * t(1)), (-t(0) * t(2)),
        (-t(0) * t(1)), (t(0) * t(0) + t(2) * t(2)), (-t(1) * t(2)),
        (-t(0) * t(2)), (-t(1) * t(2)), (t(0) * t(0) + t(1) * t(1));
    iMat = iMat + m * tMat;  // Parallel Axis Theorem
    iBody += iMat;
  }

  // Aligning threeDOF base frame to have the y-axis pass through the CoM
  double th = atan2(bodyCOM(2), bodyCOM(1));
  rot << 1, 0, 0, 0, cos(th), sin(th), 0, -sin(th), cos(th);
  bodyCOM = rot * bodyCOM;
  iBody = rot * iBody * rot.transpose();

  // Set the 3 DOF robot parameters
  threeDOF->getBodyNode("Base")->setMomentOfInertia(iBody(0, 0), iBody(1, 1),
                                                    iBody(2, 2), iBody(0, 1),
                                                    iBody(0, 2), iBody(1, 2));
  threeDOF->getBodyNode("Base")->setLocalCOM(bodyCOM);
  threeDOF->getBodyNode("Base")->setMass(mBody);

  // Print them out
  // std::cout << "mass: " << mBody << std::endl;
  // std::cout << "COM: " << bodyCOM(0) << ", " << bodyCOM(1) << ", " <<
  // bodyCOM(2)
  // << std::endl; std::cout << "ixx: " << iBody(0,0) << ", iyy: " << iBody(1,1)
  // << ", izz: " << iBody(2,2) << std::endl; std::cout << "ixy: " << iBody(0,1)
  // << ", ixz: "
  // << iBody(0,2) << ", iyz: " << iBody(1,2) << std::endl;

  // Update 3DOF state
  // get positions
  Eigen::Matrix3d baseRot =
      krang->getBodyNode("Base")->getTransform().rotation();
  baseRot = baseRot * rot.transpose();
  Eigen::AngleAxisd aa(baseRot);
  Eigen::VectorXd q(numStates);
  Eigen::VectorXd dq(numStates);
  q << aa.angle() * aa.axis(), krang->getPositions().segment(3, 5);
  threeDOF->setPositions(q);

  // TODO: When joints are unlocked qBody1 of the 3DOF (= dth = COM angular
  // speed) is not the same as qBody1 of the full robot
  dq << rot * krang->getVelocities().head(3),
      rot * krang->getVelocities().segment(3, 3),
      krang->getVelocities().segment(6, 2);
  threeDOF->setVelocities(dq);
}

//==============================================================================
Krang3D<double>::State MyWindow::getCurrentState() {
  Eigen::Matrix<double, 4, 4> Tf;
  double psi, qBody1, dpsi, dpsiFilt, dqBody1, dqBody1Filt, thL, dthL, dthLFilt,
      thR, dthR, dthRFilt;
  Eigen::VectorXd q(numStates);
  Eigen::VectorXd dq_orig(numStates);
  Eigen::VectorXd dq(numStates);
  State currentState = Dynamics::State::Zero();

  // Read Positions, Speeds, Transform speeds to world coordinates and filter
  // the speeds
  Tf = m3DOF->getBodyNode(0)->getTransform().matrix();
  psi = atan2(Tf(0, 0), -Tf(1, 0));
  qBody1 = atan2(Tf(0, 1) * cos(psi) + Tf(1, 1) * sin(psi), Tf(2, 1));
  q = m3DOF->getPositions();
  dq_orig = m3DOF->getVelocities();
  dq << (Tf.block<3, 3>(0, 0) * dq_orig.head(3)),
      (Tf.block<3, 3>(0, 0) * dq_orig.segment(3, 3)), dq_orig(6), dq_orig(7);
  mdqFilt->AddSample(dq);

  // Calculate the quantities we are interested in
  dpsi = dq(2);
  dpsiFilt = mdqFilt->average(2);
  dqBody1 = -dq_orig(0);
  dqBody1Filt =
      (-mdqFilt->average(0) * sin(psi) + mdqFilt->average(1) * cos(psi));
  thL = q(6) + qBody1;
  dthL = dq(6) + dqBody1;
  dthLFilt = mdqFilt->average(6) + dqBody1Filt;
  thR = q(7) + qBody1;
  dthR = dq(7) + dqBody1;
  dthRFilt = mdqFilt->average(7) + dqBody1Filt;

  // State: x, psi, theta, dx, dpsi, dtheta, x0, y0
  currentState << mR / 2 * (thL + thR), psi, qBody1,
      dq(3) * cos(psi) + dq(4) * sin(psi), dpsi, dqBody1, q(3), q(4);
  return currentState;
}

//==============================================================================
void MyWindow::computeDDPTrajectory() {
  param p;
  double ixx, iyy, izz, ixy, ixz, iyz;
  Eigen::Vector3d com;
  Eigen::Matrix3d iMat;
  Eigen::Matrix3d tMat;

  dart::dynamics::Frame* baseFrame = m3DOF->getBodyNode("Base");
  p.R = mR;
  p.L = mL;
  p.g = 9.800000e+00;

  p.mw = m3DOF->getBodyNode("LWheel")->getMass();

  m3DOF->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz,
                                                   iyz);

  p.YYw = ixx;
  p.ZZw = izz;
  p.XXw = iyy;  // Wheel frame of reference in ddp dynamic model is different
                // from the one in DART
  p.m_1 = m3DOF->getBodyNode("Base")->getMass();
  com = m3DOF->getBodyNode("Base")->getCOM(baseFrame);
  p.MX_1 = p.m_1 * com(0);
  p.MY_1 = p.m_1 * com(1);
  p.MZ_1 = p.m_1 * com(2);

  m3DOF->getBodyNode("Base")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  Eigen::Vector3d s = -com;  // Position vector from local COM to body COM
                             // expressed in base frame
  iMat << ixx, ixy, ixz,     // Inertia tensor of the body around its CoM
                             // expressed in body frame
      ixy, iyy, iyz, ixz, iyz, izz;
  tMat << (s(1) * s(1) + s(2) * s(2)), (-s(0) * s(1)), (-s(0) * s(2)),
      (-s(0) * s(1)), (s(0) * s(0) + s(2) * s(2)), (-s(1) * s(2)),
      (-s(0) * s(2)), (-s(1) * s(2)), (s(0) * s(0) + s(1) * s(1));
  iMat = iMat + p.m_1 * tMat;  // Parallel Axis Theorem
  p.XX_1 = iMat(0, 0);
  p.YY_1 = iMat(1, 1);
  p.ZZ_1 = iMat(2, 2);
  p.XY_1 = iMat(0, 1);
  p.YZ_1 = iMat(1, 2);
  p.XZ_1 = iMat(0, 2);
  p.fric_1 = m3DOF->getJoint(0)->getDampingCoefficient(
      0);  // Assuming both joints have same friction coeff (Please make sure
           // that is true)

  CSV_writer<Scalar> writer;
  util::DefaultLogger logger;
  bool verbose = true;
  Scalar tf = mFinalTime;
  auto time_steps = util::time_steps(tf, mMPCdt);
  int max_iterations = mDDPMaxIter;

  mDDPDynamics = new Dynamics(p);

  // Initial state
  State x0 = getCurrentState();
  x0 << 0, 0, x0(2), 0, 0, 0, 0, 0;
  std::cout << "initState: " << x0.transpose() << std::endl;
  // Dynamics::State xf; xf << 2, 0, 0, 0, 0, 0, 0.01, 5;
  // Dynamics::State xf; xf << 5, 0, 0, 0, 0, 0, 5, 0;
  Dynamics::ControlTrajectory u =
      Dynamics::ControlTrajectory::Zero(numControls, time_steps);

  // Costs
  Cost::StateHessian Q;
  Q.setZero();
  Q.diagonal() << mDDPStatePenalties;

  Cost::ControlHessian R;
  R.setZero();
  R.diagonal() << mDDPControlPenalties;

  TerminalCost::Hessian Qf;
  Qf.setZero();
  Qf.diagonal() << mDDPTerminalStatePenalties;

  Cost cp_cost(mGoalState, Q, R);
  TerminalCost cp_terminal_cost(mGoalState, Qf);

  // initialize DDP for trajectory planning
  DDP_Opt trej_ddp(mMPCdt, time_steps, max_iterations, &logger, verbose);

  // Get initial trajectory from DDP
  OptimizerResult<Dynamics> DDP_traj =
      trej_ddp.run(x0, u, *mDDPDynamics, cp_cost, cp_terminal_cost);

  mDDPStateTraj = DDP_traj.state_trajectory;
  mDDPControlTraj = DDP_traj.control_trajectory;

  writer.save_trajectory(mDDPStateTraj, mDDPControlTraj, "initial_traj.csv");
}

//==============================================================================
void MyWindow::timeStepping() {
  mSteps++;

  // Camera View Update (should move with the base frame)
  if (mContinuousZoom) {
    mZoom = std::min(
        0.25, std::max(0.11, 0.25 + ((0.25 - 0.11) / (0 - 3200)) * mSteps));
    glutPostRedisplay();
  } else {
    if (mSteps == 800) {
      mZoom = 0.215;
      glutPostRedisplay();
    } else if (mSteps == 1600) {
      mZoom = 0.18;
      glutPostRedisplay();
    } else if (mSteps == 2400) {
      mZoom = 0.145;
      glutPostRedisplay();
    } else if (mSteps == 3200) {
      mZoom = 0.11;
      glutPostRedisplay();
    }
  }

  // if((mSteps-1)%10 == 1) {
  //   std::cout << "mTrans: " << mTrans.transpose() << std::endl;
  //   std::cout << "mEye: " << mEye.transpose() << std::endl;
  //   std::cout << "mUp: " << mUp.transpose() << std::endl;
  //   std::cout << "mZoom: " << mZoom << std::endl;
  //   std::cout << "mPersp: " << mPersp << std::endl;
  //   std::cout << "mRotate: " << mRotate << std::endl;
  //   std::cout << "mTranslate: " << mTranslate << std::endl;
  //   std::cout << "mZooming: " << mZooming << std::endl;
  //   std::cout << "Trackball Center: " << mTrackBall.getCenter().transpose()
  //   << std::endl; std::cout << "Trackball Quaternion: " << std::endl <<
  //   mTrackBall.getCurrQuat().toRotationMatrix() << std::endl; std::cout <<
  //   "Trackball Radius: " << mTrackBall.getRadius() << std::endl; std::cout <<
  //   "Trackball Rotation Matrix: " << std::endl <<
  //   mTrackBall.getRotationMatrix() << std::endl;
  // }

  getSimple(m3DOF, mkrang);
  State cur_state = getCurrentState();

  // MPC DDP RECEDING HORIZON CALCULATION
  int cur_mpc_steps =
      ((mSteps > mBeginStep) ? ((mSteps - mBeginStep) / 10) : -1);

  if (cur_mpc_steps > mMPCSteps) {
    mMPCSteps = cur_mpc_steps;
    int max_iterations = mMPCMaxIter;
    bool verbose = true;
    util::DefaultLogger logger;
    int mpc_horizon = mMPCHorizon;

    Dynamics::State target_state;
    target_state = mDDPStateTraj.col(mMPCSteps + mpc_horizon);
    Dynamics::ControlTrajectory hor_control =
        Dynamics::ControlTrajectory::Zero(numControls, mpc_horizon);
    Dynamics::StateTrajectory hor_traj_states =
        mDDPStateTraj.block(0, mMPCSteps, numStates, mpc_horizon);

    DDP_Opt ddp_horizon(mMPCdt, mpc_horizon, max_iterations, &logger, verbose);

    Cost::StateHessian Q_mpc, Qf_mpc;
    Cost::ControlHessian ctl_R;

    ctl_R.setZero();
    ctl_R.diagonal() << mMPCControlPenalties;
    Q_mpc.setZero();
    Q_mpc.diagonal() << mMPCStatePenalties;
    Qf_mpc.setZero();
    Qf_mpc.diagonal() << mMPCTerminalStatePenalties;
    Cost running_cost_horizon(target_state, Q_mpc, ctl_R);
    TerminalCost terminal_cost_horizon(target_state, Qf_mpc);

    OptimizerResult<Dynamics> results_horizon;
    results_horizon.control_trajectory = hor_control;

    results_horizon = ddp_horizon.run_horizon(
        cur_state, hor_control, hor_traj_states, *mDDPDynamics,
        running_cost_horizon, terminal_cost_horizon);
    mMPCControlRef = results_horizon.control_trajectory.col(0);
    mMPCStateRef = results_horizon.state_trajectory.col(0);
    mthref = mMPCStateRef(2);
    mdthref = mMPCStateRef(5);

    mMPCWriter.save_step(cur_state, mMPCControlRef);
  }

  updateStruct updateOutput;

  if (mMPCSteps > -1) {
    if (!mLockedJoints) {
      double ddthref, tau_0;
      ddthref = mMPCControlRef(0);
      tau_0 = mMPCControlRef(1);
      mdthref += ddthref * mWorld->getTimeStep();
      if (mSteps == 20) {
        std::cout << "world's dt: " << mWorld->getTimeStep() << std::endl;
      }
      mthref += mdthref * mWorld->getTimeStep();
      updateOutput = mController->update(
          mLeftTargetPosition, mRightTargetPosition, mLeftTargetRPY,
          mRightTargetRPY, mthref, mdthref, ddthref, tau_0);

      // /  / Set forces based on the output speeds from the update function
      Eigen::VectorXd mdqBodyRef = updateOutput.mdqBodyRef;
      Eigen::VectorXd mdqBody = updateOutput.mdqBody;
      int numBodyLinks = updateOutput.numBodyLinks;
      int numArmJoints = updateOutput.numArmJoints;
      Eigen::MatrixXd mKvJoint = updateOutput.mKvJoint;
      int mOptDim = updateOutput.mOptDim;
      Eigen::VectorXd torqueLow = updateOutput.torqueLow;
      Eigen::VectorXd torqueHigh = updateOutput.torqueHigh;
      int numActuators = updateOutput.numActuators;

      Eigen::VectorXd dqL;
      Eigen::VectorXd dqR;
      Eigen::VectorXd opt_torque_cmdL;
      Eigen::VectorXd opt_torque_cmdR;
      Eigen::VectorXd lmtd_torque_cmdL(numArmJoints);
      Eigen::VectorXd lmtd_torque_cmdR(numArmJoints);

      // mRobot is replaced with mkrang

      // Get angular velocities of left and right arm joints respectively

      dqL = mdqBody.segment(numBodyLinks - 2 * numArmJoints, numArmJoints);
      dqR = mdqBody.segment(numBodyLinks - numArmJoints, numArmJoints);

      // Calculate opt_torque_cmd
      opt_torque_cmdL =
          -mKvJoint *
          (dqL - mdqBodyRef.segment(mOptDim - 2 * numArmJoints, numArmJoints));
      opt_torque_cmdR =
          -mKvJoint *
          (dqR - mdqBodyRef.segment(mOptDim - numArmJoints, numArmJoints));

      // Set lmtd_torque_cmd
      for (int i = 0; i < 7; i++) {
        lmtd_torque_cmdL(i) =
            std::max(torqueLow(i), std::min(torqueHigh(i), opt_torque_cmdL(i)));
        lmtd_torque_cmdR(i) =
            std::max(torqueLow(i), std::min(torqueHigh(i), opt_torque_cmdR(i)));
      }

      // Set Forces
      std::vector<std::string> left_arm_joint_names = {
          "LJ1", "LJ2", "LJ3", "LJ4", "LJ5", "LJ6", "LJFT"};
      std::vector<std::string> right_arm_joint_names = {
          "RJ1", "RJ2", "RJ3", "RJ4", "RJ5", "RJ6", "RJFT"};
      for (int i = 0; i < 7; i++) {
        mkrang->getJoint(left_arm_joint_names[i])
            ->setForce(0, lmtd_torque_cmdL(i));
        mkrang->getJoint(right_arm_joint_names[i])
            ->setForce(0, lmtd_torque_cmdR(i));
      }

      std::vector<std::string> lower_body_joint_names = {
          "JLWheel", "JRWheel", "JWaist", "JTorso", "JKinect"};
      for (int i = 2; i < numActuators - 2 * numArmJoints; i++)
        mkrang->getJoint(lower_body_joint_names[i])->setVelocity(0, 0.0);

      // End setting forces based on output speeds from Controller::update(*)
    }

    if (mLockedJoints | !mCOMControlInLowLevel) {
      double ddth, tau_0, ddx, ddpsi, tau_1, tau_L, tau_R;
      State xdot;
      Eigen::Vector3d ddq, dq;
      c_forces dy_forces;

      // Control input from High-level Control
      ddth = mMPCControlRef(0);
      tau_0 = mMPCControlRef(1);

      // ddq
      xdot = mDDPDynamics->f(cur_state, mMPCControlRef);
      ddx = xdot(3);
      ddpsi = xdot(4);
      ddq << ddx, ddpsi, ddth;

      // dq
      dq = cur_state.segment(3, 3);

      // A, C, Q and Gamma_fric
      dy_forces = mDDPDynamics->dynamic_forces(cur_state, mMPCControlRef);

      // tau_1
      // tau_1 = (dy_forces.A.block<1,3>(2,0)*ddq) +
      // (dy_forces.C.block<1,3>(2,0)*dq) + (dy_forces.Q(2)) -
      // (dy_forces.Gamma_fric(2));
      tau_1 = dy_forces.A.block<1, 3>(2, 0) * ddq;
      tau_1 += dy_forces.C.block<1, 3>(2, 0) * dq;
      tau_1 += dy_forces.Q(2);
      tau_1 -= dy_forces.Gamma_fric(2);

      // Wheel Torques
      tau_L = -0.5 * (tau_1 + tau_0);
      tau_R = -0.5 * (tau_1 - tau_0);
      if (abs(tau_L) > mTauLimBase / 2 | abs(tau_R) > mTauLimBase / 2) {
        std::cout << "step: " << mSteps << ", tau_0: " << tau_0
                  << ", tau_1: " << tau_1 << ", tau_L: " << tau_L
                  << ", tau_R: " << tau_R << std::endl;
      }
      tau_L = std::min(mTauLimBase / 2, std::max(-mTauLimBase / 2, tau_L));
      tau_R = std::min(mTauLimBase / 2, std::max(-mTauLimBase / 2, tau_R));
      mForces(0) = tau_L;
      mForces(1) = tau_R;
      const std::vector<size_t> index{6, 7};
      mkrang->setForces(index, mForces);
    }
  }

  SimWindow::timeStepping();
}

//==============================================================================
void MyWindow::render() {
  // Camera View Update (should move with the base frame)
  int steps;
  if (mPlayFrame) {
    steps = mPlayFrame;
  } else {
    steps = mWorld->getTime() * 1000;
  }

  if (mContinuousZoom) {
    mZoom = std::min(
        0.25, std::max(0.11, 0.25 + ((0.25 - 0.11) / (0 - 3200)) * steps));
    glutPostRedisplay();
  } else {
    if (steps < 800) {
      mZoom = 0.25;
      glutPostRedisplay();
    } else if (steps < 1600) {
      mZoom = 0.215;
      glutPostRedisplay();
    } else if (steps < 2400) {
      mZoom = 0.18;
      glutPostRedisplay();
    } else if (steps < 3200) {
      mZoom = 0.145;
      glutPostRedisplay();
    } else if (steps > 3200) {
      mZoom = 0.11;
      glutPostRedisplay();
    }
  }

  SimWindow::render();
}

//==============================================================================
bool MyWindow::screenshot() {
  static int count = 0;
  const char directory[8] = "frames";
  const char fileBase[8] = "Capture";
  char fileName[32];

  // create frames directory if not exists
  using Stat = struct stat;
  Stat buff;

  if (stat(directory, &buff) != 0) mkdir(directory, 0777);

  if (!S_ISDIR(buff.st_mode)) {
    dtwarn << "[Window::screenshot] 'frames' is not a directory, "
           << "cannot write a screenshot\n";
    return false;
  }

  // png
  std::snprintf(fileName, sizeof(fileName), "%s%s%s%.4d.png", directory, "/",
                fileBase, mPlayFrame);

  int tw = glutGet(GLUT_WINDOW_WIDTH);
  int th = glutGet(GLUT_WINDOW_HEIGHT);

  glReadPixels(0, 0, tw, th, GL_RGBA, GL_UNSIGNED_BYTE, &mScreenshotTemp[0]);

  // reverse temp2 temp1
  for (int row = 0; row < th; row++) {
    memcpy(&mScreenshotTemp2[row * tw * 4],
           &mScreenshotTemp[(th - row - 1) * tw * 4], tw * 4);
  }

  unsigned result = lodepng::encode(fileName, mScreenshotTemp2, tw, th);

  // if there's an error, display it
  if (result) {
    std::cout << "lodepng error " << result << ": "
              << lodepng_error_text(result) << std::endl;
    return false;
  } else {
    std::cout << "wrote screenshot " << fileName << "\n";
    return true;
  }
}

//==============================================================================
dart::dynamics::SkeletonPtr createFloor() {
  dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor");

  // Give the floor a body
  dart::dynamics::BodyNodePtr body =
      floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr)
          .second;
  //  body->setFrictionCoeff(1e16);

  // Give the body a shape
  double floor_width = 50;
  double floor_height = 0.05;
  std::shared_ptr<dart::dynamics::BoxShape> box(new dart::dynamics::BoxShape(
      Eigen::Vector3d(floor_width, floor_width, floor_height)));
  auto shapeNode =
      body->createShapeNodeWith<dart::dynamics::VisualAspect,
                                dart::dynamics::CollisionAspect,
                                dart::dynamics::DynamicsAspect>(box);
  shapeNode->getVisualAspect()->setColor(dart::Color::Blue());

  // Put the body into position
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = Eigen::Vector3d(0.0, 0.0, -floor_height / 2.0);
  body->getParentJoint()->setTransformFromParentBodyNode(tf);

  return floor;
}

//==============================================================================
dart::dynamics::SkeletonPtr createKrang(const char* urdfpath) {
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr krang;
  std::ifstream file;
  char line[1024];
  std::istringstream stream;

  // Load the Skeleton from a file
  krang = loader.parseSkeleton(urdfpath);
  krang->setName("krang");

  Eigen::VectorXd initPoseParams(24);
  initPoseParams << 0.0, -1.047, 0.0, 0.0, 0.264, 0.0, 0.0, -4.2976, 0.053232,
      -0.0575697, -1.36631, -0.495357, 0.969689, -1.55801, -0.421576, -1.27307,
      -1.35663, 1.2217, 0.606397, -0.91889, 1.50091, 0.516969, 1.31059,
      1.26479;  // heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso,
                // qKinect, qLArm0, ... qLArm6, qRArm0, ..., qRArm6
  size_t i;
  double newDouble, headingInit, qBaseInit, qLWheelInit, qRWheelInit,
      qWaistInit, qTorsoInit, qKinectInit, th;
  Eigen::Vector3d xyzInit, COM;
  Eigen::VectorXd qLeftArmInit(7);
  Eigen::VectorXd qRightArmInit(7);
  Eigen::Transform<double, 3, Eigen::Affine> baseTf;
  Eigen::AngleAxisd aa;
  Eigen::VectorXd qBase(6);

  // Read initial pose from the file
  file = std::ifstream("../../src/defaultInit.txt");
  assert(file.is_open());
  file.getline(line, 1024);
  stream = std::istringstream(line);
  i = 0;
  while ((i < 24) && (stream >> newDouble)) initPoseParams(i++) = newDouble;
  file.close();
  headingInit = initPoseParams(0);
  qBaseInit = initPoseParams(1);
  xyzInit << initPoseParams.segment(2, 3);
  qLWheelInit = initPoseParams(5);
  qRWheelInit = initPoseParams(6);
  qWaistInit = initPoseParams(7);
  qTorsoInit = initPoseParams(8);
  qKinectInit = initPoseParams(9);
  qLeftArmInit << initPoseParams.segment(10, 7);
  qRightArmInit << initPoseParams.segment(17, 7);

  krang->getJoint("JLWheel")->setPosition(0, qLWheelInit);
  krang->getJoint("JRWheel")->setPosition(0, qRWheelInit);
  krang->getJoint("JWaist")->setPosition(0, qWaistInit);
  krang->getJoint("JTorso")->setPosition(0, qTorsoInit);
  std::vector<std::string> left_arm_joint_names = {"LJ1", "LJ2", "LJ3", "LJ4",
                                                   "LJ5", "LJ6", "LJFT"};
  std::vector<std::string> right_arm_joint_names = {"RJ1", "RJ2", "RJ3", "RJ4",
                                                    "RJ5", "RJ6", "RJFT"};
  for (int i = 0; i < 7; i++) {
    krang->getJoint(left_arm_joint_names[i])->setPosition(0, qLeftArmInit(i));
    krang->getJoint(right_arm_joint_names[i])->setPosition(0, qRightArmInit(i));
  }

  // Calculating the axis angle representation of orientation from headingInit
  // and qBaseInit: RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
  baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit, Eigen::Vector3d::UnitX()))
      .prerotate(
          Eigen::AngleAxisd(-M_PI / 2 + headingInit, Eigen::Vector3d::UnitY()))
      .prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  aa = Eigen::AngleAxisd(baseTf.rotation());

  // Set the positions and get the resulting COM angle
  qBase << aa.angle() * aa.axis(), xyzInit;
  krang->setPositions({0, 1, 2, 3, 4, 5}, qBase);
  COM = krang->getCOM() - xyzInit;
  th = atan2(COM(0), COM(2));

  // Adjust qBaseInit to bring COM on top of wheels and set the positions
  // again
  qBaseInit -= th;
  baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit, Eigen::Vector3d::UnitX()))
      .prerotate(
          Eigen::AngleAxisd(-M_PI / 2 + headingInit, Eigen::Vector3d::UnitY()))
      .prerotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX()));
  aa = Eigen::AngleAxisd(baseTf.rotation());
  qBase << aa.angle() * aa.axis(), xyzInit;
  krang->setPositions({0, 1, 2, 3, 4, 5}, qBase);

  krang->getJoint(0)->setDampingCoefficient(0, 0.5);
  krang->getJoint(1)->setDampingCoefficient(0, 0.5);

  return krang;
}

//==============================================================================
dart::dynamics::SkeletonPtr createTray(dart::dynamics::BodyNodePtr ee,
                                       const char* urdfpath) {
  Eigen::Matrix3d EELRot, trayLocalRot, trayRot;
  Eigen::Vector3d EELPos, trayLocalTranslation, trayPos;
  Eigen::VectorXd qObject(6);
  char fullpath[1024];

  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr tray = loader.parseSkeleton(urdfpath);
  tray->setName("tray");

  // Orientation
  EELRot = ee->getTransform().rotation();
  trayLocalRot << 1, 0, 0, 0, 0, 1, 0, -1, 0;
  trayRot = EELRot * trayLocalRot;
  Eigen::AngleAxisd aa(trayRot);

  // Position
  EELPos = ee->getTransform().translation();
  trayLocalTranslation << 0, -0.286303, 0.04;
  trayPos = EELPos + trayRot * trayLocalTranslation;

  // Set the position
  qObject << (aa.angle() * aa.axis()), trayPos;
  tray->setPositions(qObject);

  return tray;
}

//==============================================================================
dart::dynamics::SkeletonPtr createCup(dart::dynamics::BodyNodePtr ee,
                                      const char* urdfpath) {
  Eigen::Matrix3d EELRot, cupLocalRot, cupRot;
  Eigen::Vector3d EELPos, cupLocalTranslation, cupPos;
  Eigen::VectorXd qObject(6);
  char fullpath[1024];

  // Load the Skeleton from a file
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr cup = loader.parseSkeleton(urdfpath);
  cup->setName("cup");

  // Orientation
  EELRot = ee->getTransform().rotation();
  cupLocalRot << 1, 0, 0, 0, 0, 1, 0, -1, 0;
  cupRot = EELRot * cupLocalRot;
  Eigen::AngleAxisd aa(cupRot);

  // Position
  EELPos = ee->getTransform().translation();
  cupLocalTranslation << 0, -0.286303, 0.04;
  cupPos = EELPos + cupRot * cupLocalTranslation;

  // Set the position
  qObject << (aa.angle() * aa.axis()), cupPos;
  cup->setPositions(qObject);

  return cup;
}

//==============================================================================
int main(int argc, char* argv[]) {
  // To load tray and cup or not
  char urdfpathKrang[1024];
  char urdfpathTray[1024];
  char urdfpathCup[1024];
  bool loadTray, loadCup;
  double trayCupFriction;
  config4cpp::Configuration* cfg = config4cpp::Configuration::create();
  const char* scope = "";
  const char* configFile = "../../src/controlParams.cfg";
  try {
    cfg->parse(configFile);
    loadTray = cfg->lookupBoolean(scope, "tray");
    loadCup = cfg->lookupBoolean(scope, "cup");
    trayCupFriction = cfg->lookupFloat(scope, "trayCupFriction");
    strcpy(urdfpathKrang, cfg->lookupString(scope, "urdfpathKrang"));
    strcpy(urdfpathTray, cfg->lookupString(scope, "urdfpathTray"));
    strcpy(urdfpathCup, cfg->lookupString(scope, "urdfpathCup"));
  } catch (const config4cpp::ConfigurationException& ex) {
    std::cerr << ex.c_str() << std::endl;
    cfg->destroy();
  }
  std::cout << "loadTray: " << (loadTray ? "true" : "false") << std::endl;
  std::cout << "loadCup: " << (loadCup ? "true" : "false") << std::endl;
  std::cout << "trayCupFriction: " << trayCupFriction << std::endl;
  std::cout << "urdfpathKrang: " << urdfpathKrang << std::endl;
  std::cout << "urdfpathTray: " << urdfpathTray << std::endl;
  std::cout << "urdfpathCup: " << urdfpathCup << std::endl;

  // Create world
  dart::simulation::WorldPtr world =
      std::make_shared<dart::simulation::World>();

  // add floor and robot to the world pointer
  // Load Floor
  dart::dynamics::SkeletonPtr floor = createFloor();
  world->addSkeleton(floor);

  // Load robot
  dart::dynamics::SkeletonPtr robot = createKrang(urdfpathKrang);
  world->addSkeleton(robot);

  // Load Tray
  if (loadTray) {
    dart::dynamics::SkeletonPtr tray =
        createTray(robot->getBodyNode("lGripper"), urdfpathTray);
    world->addSkeleton(tray);
    tray->getBodyNode(0)->setFrictionCoeff(trayCupFriction);
    std::cout << "tray surface friction: "
              << tray->getBodyNode(0)->getFrictionCoeff() << std::endl;
  }

  // Load Cup
  if (loadCup) {
    dart::dynamics::SkeletonPtr cup =
        createCup(robot->getBodyNode("lGripper"),
                  urdfpathCup);  // cup->setPositions(tray->getPositions());
    world->addSkeleton(cup);
    cup->getBodyNode(0)->setFrictionCoeff(trayCupFriction);
    std::cout << "cup surface friction: "
              << cup->getBodyNode(0)->getFrictionCoeff() << std::endl;
  }

  // Create window
  MyWindow window(world);

  // Run the world
  glutInit(&argc, argv);
  window.initWindow(1280, 720, "3DOF URDF");
  glutMainLoop();

  return 0;
}

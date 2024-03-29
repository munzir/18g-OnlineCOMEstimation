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


//==========================================================================
Controller::Controller(dart::dynamics::SkeletonPtr _robot, Eigen::MatrixXd _trajectory)
  : mRobot(_robot), mTrajectory(_trajectory)
   {
  assert(_robot != nullptr);
  int dof = mRobot->getNumDofs();
  std::cout << "[controller] DoF: " << dof << std::endl;
  mForces.setZero(19);
  mSteps = 0;
  mdt = mRobot->getTimeStep();


    // *************** Initialize Pose Counter
    // Used to determine where we are in our trajectory
    mCurrTargetPose = 0;

    // *************** Initialize Robot to first Target Pose
    mRobot->setPositions(mTrajectory.row(mCurrTargetPose));

    mIsBalanced = false;


  // *************** Initialize Guess Robot
  mGuessRobot = mRobot->clone();
  mGuessRobot->setName("GuessRobot");
  mVirtualWorld = new dart::simulation::World;
  mVirtualWorld->addSkeleton(mGuessRobot);
  mGuessRobot->setPositions(mRobot->getPositions());

  int bodyParams = 4; double minXCOMError = 0.02, maxDeviation = 0.50, maxOffset = 0.50;
  changeRobotParameters(mGuessRobot, bodyParams, minXCOMError, maxDeviation, maxOffset);

    // Used as initial parameters to learn with
    mGuessRobotParameters = getGuessRobotParameters();

  // ************** Lock joints
  int joints = mRobot->getNumJoints();
  for(int i=3; i < joints; i++) {
    mRobot->getJoint(i)->setActuatorType(dart::dynamics::Joint::ActuatorType::LOCKED);
    mGuessRobot->getJoint(i)->setActuatorType(dart::dynamics::Joint::ActuatorType::LOCKED);
  }
  mdqFilt = new filter(25, 100);

  // ************** Wheel Radius and Distance between wheels
  mR = 0.25, mL = 0.68;

  // *********************************** Tunable Parameters
  Configuration *  cfg = Configuration::create();
  const char *     scope = "";
  const char *     configFile = "../controlParams.cfg";

  try {
    cfg->parse(configFile);

    // -- com error
    mInitialComAngle = M_PI*(cfg->lookupFloat(scope, "initialComAngle")/180);
  } catch(const ConfigurationException & ex) {
      cerr << ex.c_str() << endl;
      cfg->destroy();
  }
  cfg->destroy();

  // ********************************** Introduce CoM error
  qBody1Change(mRobot, mInitialComAngle);
  qBody1Change(mGuessRobot, mInitialComAngle);
  Eigen::Vector3d com = mRobot->getCOM() - mRobot->getPositions().segment(3,3);
  cout << "robot com: " << com << endl;
  cout << "robot com angle: " << atan2(com(0), com(2))*180.0/M_PI << endl;
  com = mGuessRobot->getCOM() - mGuessRobot->getPositions().segment(3,3);
  cout << "guess robot com: " << com << endl;
  cout << "guess com angle: " << atan2(com(0), com(2))*180.0/M_PI << endl;

  //*********************************** Read Initial Pose for Pose Regulation
  Eigen::Matrix<double, 25, 1> qInit = mRobot->getPositions();
  mBaseTf = mRobot->getBodyNode(0)->getTransform().matrix();
  double psiInit =  atan2(mBaseTf(0,0), -mBaseTf(1,0));
  double qBody1Init = atan2(mBaseTf(0,1)*cos(psiInit) + mBaseTf(1,1)*sin(psiInit), mBaseTf(2,1));
  mqBodyInit(0) = qBody1Init;
  mqBodyInit.tail(17) = qInit.tail(17);

  //*********************************** Initialize Extended State Observer
  initializeExtendedStateObservers();

  // **************************** Torque Limits
  mTauLim << 120, 740, 370, 10, 370, 370, 175, 175, 40, 40, 9.5, 370, 370, 175, 175, 40, 40, 9.5;

  // **************************** Data Output
  mOutFile.open("output.csv");
}

//=========================================================================
Controller::~Controller() {
  mOutFile.close();
}

//=========================================================================
void printMatrix(Eigen::MatrixXd A){
  for(int i=0; i<A.rows(); i++){
    for(int j=0; j<A.cols(); j++){
      std::cout << A(i,j) << ", ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

// ==========================================================================
dart::dynamics::SkeletonPtr Controller::qBody1Change(dart::dynamics::SkeletonPtr robot, double change) {

  // Get qBody1
  Eigen::Matrix<double, 4, 4> baseTf = robot->getBodyNode(0)->getTransform().matrix();
  double psi =  atan2(baseTf(0,0), -baseTf(1,0));
  double qBody1 = atan2(baseTf(0,1)*cos(psi) + baseTf(1,1)*sin(psi), baseTf(2,1));

  // Change qBody1
  qBody1 += change;

  // Convert qBody1 to axis angle
  Eigen::Transform<double, 3, Eigen::Affine> newBaseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
  newBaseTf.prerotate(Eigen::AngleAxisd(-qBody1, Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+psi,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd aa(newBaseTf.matrix().block<3,3>(0,0));

  // Set the axis angle as new robot position
  Eigen::VectorXd q = robot->getPositions();
  q.head(3) = aa.angle()*aa.axis();
  robot->setPositions(q);

  return robot;
}

// ==========================================================================
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

// ==========================================================================
void Controller::changeRobotParameters(dart::dynamics::SkeletonPtr robot, int bodyParams, double minXCOMError, double maxDeviation, double maxOffset) {

  dart::dynamics::SkeletonPtr originalRobot = robot->clone();

  int numBodies = robot->getNumBodyNodes();
  dart::dynamics::BodyNodePtr bodyi;
  double mi, pert_mi;
  double mxi, pert_mxi;
  double myi, pert_myi;
  double mzi, pert_mzi;

  // Generate perturbed parameters
  double deviation;
  double offset;
  double xCOM = 0;
  while (abs(xCOM) < minXCOMError) {
      for (int i = 0; i < numBodies; i++) {
          bodyi = originalRobot->getBodyNode(i);
          mi = bodyi->getMass();
          mxi = mi * bodyi->getLocalCOM()(0);
          myi = mi * bodyi->getLocalCOM()(1);
          mzi = mi * bodyi->getLocalCOM()(2);

          pert_mi = mi;

          deviation = fRand(-maxDeviation, maxDeviation);
          offset = fRand(-maxOffset, maxOffset);
          pert_mxi = mxi + deviation * mxi + offset;

          deviation = fRand(-maxDeviation, maxDeviation);
          offset = fRand(-maxOffset, maxOffset);
          pert_myi = myi + deviation * myi + offset;

          deviation = fRand(-maxDeviation, maxDeviation);
          offset = fRand(-maxOffset, maxOffset);
          pert_mzi = mzi + deviation * mzi + offset;

          robot->getBodyNode(i)->setMass(pert_mi);
          robot->getBodyNode(i)->setLocalCOM(Eigen::Vector3d(pert_mxi, pert_myi, pert_mzi)/pert_mi);
      }
      xCOM = robot->getCOM()(0);
      cout << "xCOM: " << xCOM << endl;
  }
}
// ==========================================================================
Eigen::Vector3d Controller::getBodyCOM(dart::dynamics::SkeletonPtr robot) {
  double fullMass = robot->getMass();
  double wheelMass = robot->getBodyNode("LWheel")->getMass();
  return (fullMass*robot->getCOM() - wheelMass*robot->getBodyNode("LWheel")->getCOM() - wheelMass*robot->getBodyNode("RWheel")->getCOM())/(fullMass - 2*wheelMass);
}

// ==========================================================================
void Controller::updatePositions(){
  mBaseTf = mRobot->getBodyNode(0)->getTransform().matrix();
  mq = mRobot->getPositions();
  mxyz0 = mq.segment(3,3); // position of frame 0 in the world frame represented in the world frame
  mpsi =  atan2(mBaseTf(0,0), -mBaseTf(1,0));
  mqBody1 = atan2(mBaseTf(0,1)*cos(mpsi) + mBaseTf(1,1)*sin(mpsi), mBaseTf(2,1));
  mqBody(0) = mqBody1;
  mqBody.tail(17) = mq.tail(17);
  mRot0 << cos(mpsi), sin(mpsi), 0,
          -sin(mpsi), cos(mpsi), 0,
          0, 0, 1;
  mthWheel += mdthWheel*mdt;
  mGuessRobot->setPositions(mRobot->getPositions());
  Eigen::Vector3d bodyCOM = mRot0*(getBodyCOM(mGuessRobot)-mxyz0);
  mthCOM = atan2(bodyCOM(0), bodyCOM(2));
  bodyCOM = mRot0*(getBodyCOM(mRobot)-mxyz0);
  mthCOM_true = atan2(bodyCOM(0), bodyCOM(2)); // for plotting purposes
}

// ==========================================================================
void Controller::updateSpeeds(){
  mdqFilt->AddSample(mRobot->getVelocities());
  mdq = mdqFilt->average;
  mdxyz0 = mBaseTf.matrix().block<3,3>(0,0)*mdq.segment(3,3); // velocity of frame 0 in the world frame represented in the world frame
  mdx = mdq(4)*sin(mqBody1) - mdq(5)*cos(mqBody1);
  mdqBody1 = -mdq(0);
  mdpsi = (mBaseTf.block<3,3>(0,0) * mdq.head(3))(2);
  mdqBody(0) = mdqBody1;
  mdqBody.tail(17) = mdq.tail(17);
  mdqMin(0) = mdx;
  mdqMin(1) = mdpsi;
  mdqMin.tail(18) = mdqBody;
  mdRot0 << (-sin(mpsi)*mdpsi), (cos(mpsi)*mdpsi), 0,
           (-cos(mpsi)*mdpsi), (-sin(mpsi)*mdpsi), 0,
           0, 0, 0;
  mdthL = mdq(6) + mdqBody1;
  mdthR = mdq(7) + mdqBody1;
  mdthWheel = (mdthL + mdthR)/2;
  mdthCOM = mdqBody1;
}

// ==========================================================================
void Controller::initializeExtendedStateObservers() {

  updatePositions();
  updateSpeeds();

  mthWheel = 0.0;
  mthWheel_hat = 0.0;
  mdthWheel_hat = 0.0;
  mf_thWheel = 0.0;

  mGuessRobot->setPositions(mRobot->getPositions());
  Eigen::Vector3d COM = mRot0*(getBodyCOM(mGuessRobot)-mxyz0);
  mthCOM_hat = atan2(COM(0), COM(2));
  mdthCOM_hat = mdqBody1;
  mf_thCOM = 0.0;
}

void Controller::updateExtendedStateObserverParameters() {

  // ********************* Extracting Required Parameters from DART URDF

  // Wheeled Inverted Pendulum Parameters (symbols taken from the paper)
  double I_ra = 0;
  double gamma = 1.0;
  double g = 9.81;
  double c_w = 0.1;
  double r_w = mR;
  double m_w;
  double I_wa;
  double M_g;
  double l_g;
  double I_yy;
  double delta, c1, c2; // Intermediate Parameters

  // Our intermediate Variables
  double ixx, iyy, izz, ixy, ixz, iyz;
  Eigen::Vector3d COM;
  int nBodies;
  Eigen::Matrix3d iMat;
  Eigen::Matrix3d iBody;
  Eigen::Matrix3d rot;
  Eigen::Vector3d t;
  Eigen::Matrix3d tMat;
  dart::dynamics::BodyNodePtr b;
  dart::dynamics::Frame* baseFrame;
  double m;

  // Wheel Mass
  m_w = mRobot->getBodyNode("LWheel")->getMass();

  // Wheel inertia (axis)
  mRobot->getBodyNode("LWheel")->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
  I_wa = ixx;

  // Body Mass
  M_g = mRobot->getMass() - 2*m_w;

  // Distance to body COM
  COM = mRot0*(getBodyCOM(mRobot) - mxyz0); COM(1) = 0;
  l_g = COM.norm();

  // Body inertia (axis)
  nBodies = mRobot->getNumBodyNodes();
  iBody = Eigen::Matrix3d::Zero();
  baseFrame = mRobot->getBodyNode("Base");
  for(int i=0; i<nBodies; i++){
    if(i==1 || i==2) continue; // Skip wheels
    b = mRobot->getBodyNode(i);
    b->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
    rot = b->getTransform(baseFrame).rotation();
    t = mRobot->getCOM(baseFrame) - b->getCOM(baseFrame) ; // Position vector from local COM to body COM expressed in base frame
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
  I_yy = iBody(0, 0);

  // Intermediate Parameters
  delta = (M_g*l_g+I_yy+pow(gamma,2)*I_ra)*(M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)-pow(M_g*r_w*l_g-I_ra*pow(gamma,2),2);
  c1 = (M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2)+M_g*r_w*l_g+I_ra*pow(gamma,2);
  c2 = M_g*r_w*l_g+M_g*pow(l_g,2)+I_yy;

  // ******************** Robot dynamics for LQR Gains (Not used)
  Eigen::Matrix<double, 4, 4> A;
  Eigen::Matrix<double, 4, 1> B;
    Eigen::Matrix<double, 4, 4> Q;
    Eigen::Matrix<double, 1, 1> R;

  A << 0, 0, 1, 0,
       0, 0, 0, 1,
       ((M_g+m_w)*pow(r_w,2)+I_wa+I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, -c1*c_w/delta, c1*c_w/delta,
       (M_g*r_w*l_g-I_ra*pow(gamma,2))*M_g*g*l_g/delta, 0, c2*c_w/delta, -c2*c_w/delta;

  B << 0,
       0,
       -c1/delta,
       c2/delta;

    // Q and R from Matlab simulation code
    // Need to make sure these are correct
    // TODO im p sure they are not
    Q << 1,   0,  0, 0,
         0, 320,  0, 0,
         0,   0, 100, 0,
         0,   0,  0, 300;
    Q = 300 * Q;

    R << 500;

  if(mSteps == 1) {
    cout << A << endl;
    cout << B << endl;
        cout << Q << endl;
        cout << R << endl;
  }

    Eigen::MatrixXd F;

    //lqr(A, B, Q, R, &F);
    //cout << F << endl;

  // A << 0, 0, 1, 0,
  //      0, 0, 0, 1,
  //      17.7828531704201,  0, -0.00858417221300891,  0.00858417221300891,
  //      47.8688365622367,  0, 0.0288387521185202,  -0.0288387521185202;
  // B << 0,
  //      0,
  //      -0.0858417221300890,
  //      0.288387521185202;

  // ********************** Observer Dynamics
  mA_ << 0, 1, 0,
         0, 0, 1,
         0, 0, 0;
  mB_1 << 0,
         c2/delta,
         0;
  mB_2 << 0,
         -c1/delta,
         0;
  // mB_1 << 0,
  //        0.288387521185202,
  //        0;
  // mB_2 << 0,
  //        -0.0858417221300890,
  //        0;


  // ***************** Observer Feedback Gains
  // mL_1 << 1760.00000000439,
  //        269438.396409193,
  //        21501434.2537036;
  // mL_2 << 1760.00000000439,
  //        269438.396409193,
  //        21501434.2537036;

  mL_1 << 1159.99999999673,173438.396407957,1343839.4084839;
  mL_2 = mL_1;
}

// ==========================================================================
void Controller::updateExtendedStateObserverStates() {

  // Integrate Dynamics
  // mA_, mL_1, mB_1, mL_2, mB_2;
  Eigen::Vector3d x, xdot;

  x << mthWheel_hat, mdthWheel_hat, mf_thWheel;
  xdot = mA_*x + mL_1*(mthWheel - mthWheel_hat) + mB_1*mu_thWheel;
  x += xdot*mdt; mthWheel_hat = x(0); mdthWheel_hat = x(1); mf_thWheel = x(2);

  x << mthCOM_hat, mdthCOM_hat, mf_thCOM;
  xdot = mA_*x + mL_2*(mthCOM - mthCOM_hat) + mB_2*mu_thCOM;
  x += xdot*mdt; mthCOM_hat = x(0); mdthCOM_hat = x(1); mf_thCOM = x(2);
}

// ==========================================================================
double Controller::activeDisturbanceRejectionControl() {

  // ****************** The following F comes from Bodgan and Nathan's code
  Eigen::Matrix<double, 4, 1> F;
  // F << -444.232838220247,   -13.8564064605507,   -111.681669536162,   -26.3837189119712;
  // F << -1848.03012979190,    -13.8564064605509,   -269.597433286135,   -18.3661533292315;
  // Working at 0 Tilt
  // F << -2443.80123228903,    -21.9089023002056,   -366.198685841371,   -28.8559866982220;
  // F << -500, -20, -400, -30;
  // Break Dance Moves
  // F << -510.449550243191,  -0.244948974282393,  -110.528023245082, -1.14116859943037;
  // F << -502.507542429414, -0.182574185839181,  -109.159318118851, -0.994601287695807;
  // Indefinite Drift
  // F << -504.249727091444,  -0.199999999999294,  -109.459542459413, -1.02733232811016;
  // F << -504.249793519900, -0.200000000000233,  -109.459675610606, -1.02733243214781;
  // Explodes ???
  // F << -505.168127552260,  -0.200000000003292,  -109.618042362170, -1.04301620674174;
  // F << -500.481166823486,  -0.173205080751090,  -108.809840324047, -0.958459823947375;
  // GOOD
  // F << -500.4811784741, -0.1732050808, -108.8098652890, -0.9584598411;
  // F << -3617.30655560822,    -69.2820323027686,   -676.022987559394,   -95.4128736402033;
  // F << -6802.85419804756, -69.2820323027654,   -928.328202601923,   -86.9485793475169;
  // F << -2669.67918242245,   -21.9089023002072,   -371.814613562732,   -28.2110280320200;
  // F << -2549.11511950484,   -21.9089023002060,   -368.094666786882,   -28.5125368751267;
  // F << -2443.80123228903,    0.0,   0.0,   0.0;
  // Munzir (from MATLAB?)
  F << -953.5142, -13.8564, -330.9289, -29.5582; // By Munzir, based on A, B matrices calculated in updateESOParameters() function
  // TODO need to confirm values
  // Akash (using c++/octave)
  // F << -220.684, 0, -0.942423, -0.0970048;


  // Observer Control Gains
  double F_thWheel = F(1);
  double F_dthWheel = F(3);
  double F_thCOM = F(0);
  double F_dthCOM = F(2);

  // Observer Control Update
  mu_thWheel = -F_thWheel*(mthWheel_hat - 0) - F_dthWheel*(mdthWheel_hat - 0);
  mu_thCOM = -F_thCOM*(mthCOM_hat - 0) - F_dthCOM*(mdthCOM_hat - 0);

  // Active Disturbance Rejection Control
  return mu_thWheel + mu_thCOM - mf_thWheel/mB_1(1) - mf_thCOM/mB_2(1);

  // Simple LQR
  // Eigen::VectorXd ut1(2); ut1 << F[1], F[3];
  // Eigen::VectorXd ut2(2); ut2 << xFull[1]-xDesired[1], xFull[3]-xDesired[3];
  // Eigen::VectorXd up1(2); up1 << F[0], F[2];
  // Eigen::VectorXd up2(2); up2 << xFull[0]-xDesired[0], xFull[2]-xDesired[2];
  // u_theta = -ut1.transpose()*ut2;
  // u_psi = -up1.transpose()*up2;
  // tau_w = u_psi + u_theta;
  // tauL = 0.5*tau_w;
  // tauR = 0.5*tau_w;
}

// ==========================================================================
// TODO
void Controller::updateBalanceState() {

    //cout << mRobot->getVelocities().transpose() << endl;
    //cout << "\rRobot XCoM" << mRobot->getCOM()(0);

    double maxRobotVel = mRobot->getVelocities().cwiseAbs().maxCoeff();
    //cout << "\rRobot max vel: " << maxRobotVel << "Balance: " << mIsBalanced << " \t ";

    // TODO
    // condition in if statement should be if xcom is < than some value (1mm)
    // How to get the xcom value in the right frame tho?
    // Need to fix this condition since robot is learning when we first start
    // the simulation(at the unbalanced position)
    double eps = 1e-4;
    if (maxRobotVel < eps) {
        mIsBalanced = true;
    } else {
        mIsBalanced = false;
    }
}

// ==========================================================================
void Controller::updateGuessRobotParameters() {

    if (mIsBalanced && mCurrTargetPose != 0) {
        learnRobotParameters();
        cout << "\rBeta: " << mGuessRobotParameters.block(0, 0, 1, 12);
        //TODO
        //setGuessRobotParameters(mGuessRobotParameters, 4);
    }
}

// ==========================================================================
void Controller::learnRobotParameters() {
    Eigen::MatrixXd balancedPose = mRobot->getPositions().transpose();

    Eigen::MatrixXd phiVec = genPhiMatrix(mRobot, balancedPose, 4);
    int n = 300;
    double xCOMPred = (phiVec * mGuessRobotParameters.transpose())(0, 0);
    Eigen::MatrixXd delta = phiVec;

    mGuessRobotParameters = mGuessRobotParameters - (n * xCOMPred * delta);

}

// ==========================================================================
Eigen::MatrixXd Controller::getGuessRobotParameters() {

    int numBodies = mGuessRobot->getNumBodyNodes();
    int bodyParams = 4;
    Eigen::MatrixXd parameters(1, numBodies * bodyParams);
    double mi, xi, yi, zi;

    for (int i = 0; i < numBodies; i++) {
        BodyNodePtr bodyi = mGuessRobot->getBodyNode(i);
        mi = bodyi->getMass();
        xi = bodyi->getLocalCOM()(0);
        yi = bodyi->getLocalCOM()(1);
        zi = bodyi->getLocalCOM()(2);

        parameters(0, i * bodyParams + 0) = mi;
        parameters(0, i * bodyParams + 1) = mi * xi;
        parameters(0, i * bodyParams + 2) = mi * yi;
        parameters(0, i * bodyParams + 3) = mi * zi;

    }

    return parameters;
}

// ==========================================================================
void Controller::setGuessRobotParameters(Eigen::MatrixXd parameters, int bodyParams) {
    Eigen::Vector3d bodyMCOM;
    double mi;
    int numBodies = parameters.cols()/bodyParams;
    for (int i = 0; i < numBodies; i++) {
        mi = parameters(0, i * bodyParams);
        bodyMCOM(0) = parameters(0, i * bodyParams + 1);
        bodyMCOM(1) = parameters(0, i * bodyParams + 2);
        bodyMCOM(2) = parameters(0, i * bodyParams + 3);

        mGuessRobot->getBodyNode(i)->setMass(mi);
        mGuessRobot->getBodyNode(i)->setLocalCOM(bodyMCOM/mi);
    }
}

// ==========================================================================
void Controller::updateRobotPose() {
    //TODO
    // Get trajectories to work
    // for now just jump to target pose
    // Figure out how to move to mCurrTargetPose
    if (mIsBalanced) {
        // 24th pose goes crazy maybe the LQR gains are off for it?
        // The ones before are p fine
        //cout << "Here" << endl;
        //cout << mTrajectory.row(mCurrTargetPose) << endl;
        mCurrTargetPose++;
        mRobot->setPositions(mTrajectory.row(mCurrTargetPose));
        mGuessRobot->setPositions(mRobot->getPositions());
        // TODO
        // Set velocities to track the trajectory
        //mRobot->setVelocities((mTrajectory.row(mCurrTargetPose) - mTrajectory.row(mCurrTargetPose + 1))/1);
        //mGuessRobot->setVelocities(mRobot->getVelocities());
        mCurrTargetPose++;
    }

    //// Check to see if next pose is reached
    //// Target pose is listed twice in a row in trajectory file
    if ((mRobot->getPositions().transpose()).isApprox(mTrajectory.row(mCurrTargetPose + 1))) {
        // Since a target pose is written twice in a row, need to make sure the
        // next target pose after this one is counted from the right location
        // Basically this moves to the next duplicate entry
        mCurrTargetPose++;
    } else {
        //mRobot->setVelocities((mTrajectory.row(mCurrTargetPose) - mTrajectory.row(mCurrTargetPose + 1))/1);
        //mGuessRobot->setVelocities(mRobot->getVelocities());
    }
}

//=========================================================================
void Controller::update(const Eigen::Vector3d& _LeftTargetPosition,const Eigen::Vector3d& _RightTargetPosition) {

  // increase the step counter
  mSteps++;

    // updates mBaseTf, mq, mxyz0, mpsi, mqBody1, mqBody, mRot0
    // Needs mRobot
    updatePositions();

    // updates mdq, mdxyz0, mdx, mdqBody1, mdpsi, mdqBody, mdqMin, dRot0
    // Needs mRobot, mdqFilt, mBaseTf, mqBody1
    updateSpeeds();

    // Apply the Control
    double wheelsTorque;
    wheelsTorque = activeDisturbanceRejectionControl();
    mForces(0) = 0.5*wheelsTorque;
    mForces(1) = 0.5*wheelsTorque;
    const vector<size_t > index{6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24};
    mRobot->setForces(index, mForces);

    // Update Extended State Observer
    updateExtendedStateObserverParameters();
    updateExtendedStateObserverStates();

    // TODO:
    // Update if the robot is balanced or not
    updateBalanceState();

    // TODO
    // Update robot system parameters if balanced
    // Update robot's system id via learning from input pose
    updateGuessRobotParameters();

    // update that goes to the next pose if needed
    // TODO
    updateRobotPose();

    //cout << mIsBalanced << endl;
    cout << "\r" << mCurrTargetPose;

  // TODO
  // If balanced go to next pose
  //
  // How do i move to next pose while also obeying timestepping in simulation
  // for RRTs
  //
  // As of right now lets just go to next pose instantly
  //
  // Need to LQR gain it may cause crazy sim results
  //
  // Actually we should only apply ADRC when we reach the pose not before then
  // but we still want the timestepping to happen pondering
  //
  // Actually we can have regular controller from pose to pose and when we reach
  // target pose we turn off regular controller and turn on ADRC so we dont have
  // conflicts and timestepping resumes normally
  //
  // but next thing is what is/ do we have a controller for going pose to pose
  // RRT provides target nodes but how to implement that within a Controller.cpp
  // file

  // Dump data
  mOutFile << mthCOM_true << ", " << mthCOM << endl;
}

//=========================================================================
dart::dynamics::SkeletonPtr Controller::getRobot() const {
  return mRobot;
}

//=========================================================================
void Controller::keyboard(unsigned char /*_key*/, int /*_x*/, int /*_y*/) {
}

#include <iostream>
#include <Utilities/Timer.h>

#include "ConvexMPCLocomotion.h"
#include "convexMPC_interface.h"
#include "../../../../common/FootstepPlanner/GraphSearch.h"
#include <Utilities/pseudoInverse.h>
#include "Gait.h"
#include <cmath>

#define USE_TERRAIN_A
#define USE_PF_FIX
#define USE_ROS_DEBUG
////////////////////
// Controller
////////////////////

ConvexMPCLocomotion::ConvexMPCLocomotion(float _dt, int _iterations_between_mpc, MIT_UserParameters* parameters) :
        iterationsBetweenMPC(_iterations_between_mpc),
        horizonLength(10),//10
        dt(_dt),
        standing(horizonLength, Vec4<int>(0,0,0,0),Vec4<int>(10,10,10,10),"Standing"),//0
        trotting(int(horizonLength*1.2), Vec4<int>(0,6,6,0), Vec4<int>(6,6,6,6),"Trotting"),//4
        walking(int(horizonLength*3.2), Vec4<int>(0,16,16,0), Vec4<int>(24,24,24,24), "Walking"),//5
        pacing(horizonLength, Vec4<int>(5,0,5,0),Vec4<int>(5,5,5,5),"Pacing"),//6
        jumping(horizonLength, Vec4<int>(0,0,0,0), Vec4<int>(5,5,5,5), "Jumping"),//7
        galloping(horizonLength, Vec4<int>(0,3,6,9),Vec4<int>(4,4,4,4),"Gallopsing"),//8
        bounding(horizonLength, Vec4<int>(5,5,0,0),Vec4<int>(5,5,5,5),"Bounding"),//9

        pronking(horizonLength*1.2, Vec4<int>(0,0,0,0),Vec4<int>(5,5,5,5),"Pronking"),
        trotRunning(horizonLength*1.6, Vec4<int>(0,10,10,0),Vec4<int>(6,6,6,6),"Trot Running"),
        walking2(horizonLength, Vec4<int>(0,5,5,0), Vec4<int>(7,7,7,7), "Walking2"),
        random(horizonLength, Vec4<int>(9,13,13,9), 0.4, "Flying nine thirteenths trot"),
        random2(horizonLength, Vec4<int>(8,16,16,8), 0.5, "Double Trot")
{
  _parameters = parameters;
  dtMPC = dt * iterationsBetweenMPC;
  default_iterations_between_mpc = iterationsBetweenMPC;
  weights << 1.25, 1.25, 2, 2, 2, 50, 0.2, 0.2, 0.3, 2, 2, 0.2;

  printf("[Convex MPC] dt: %.3f iterations: %d, dtMPC: %.3f\n", dt, iterationsBetweenMPC, dtMPC);
  setup_problem(dtMPC, horizonLength, 0.3, 120);
  
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;

  for(int i = 0; i < 4; i++)
    firstSwing[i] = true;
  initSparseMPC();

  pBody_des.setZero();
  vBody_des.setZero();
  aBody_des.setZero();

  step_height = 0.05f;

  wPla.resize(4,3);
  wPla.setZero();
  wPla(0,0)=1; wPla(1,0)=1; wPla(2,0)=1; wPla(3,0)=1;
  wPlaInv.setZero();
  zFoot.setZero();
  aPla.setZero();
  rPla.setZero();
  rpyPlane.setZero();
  rpyDesFin.setZero();
  this->node = std::make_shared<rclcpp::Node>("mpc_visual_node");
  std::cout<<"mpc_debug_node create success"<<std::endl;
  this->vtraj_pub = this->node->create_publisher<ros_bridge::msg::VTraj>("vtraj_data", 10);
  this->debug_pub = this->node->create_publisher<std_msgs::msg::Float64MultiArray>("debug_data", 10);
  this->weights_pub = this->node->create_publisher<std_msgs::msg::Float64MultiArray>("weights_data", 10);
  this->vstate_pub = this->node->create_publisher<ros_bridge::msg::VState>("vstate_data", 10);
  this->vpf_pub = this->node->create_publisher<ros_bridge::msg::VPf>("vpf_data", 10);

  this->qweight_sub = this->node->create_subscription<ros_bridge::msg::Qweights>
                    ("qweight_data", 10, std::bind(&ConvexMPCLocomotion::qweightCallback, this, std::placeholders::_1));
  vpf_msg.pfx_1 = 0;
  vpf_msg.pfy_1 = 0;
  vpf_msg.pfz_1 = 0;

  vpf_msg.pfx_2 = 0;
  vpf_msg.pfy_2 = 0;
  vpf_msg.pfz_2 = 0;

  vpf_msg.pfx_3 = 0;
  vpf_msg.pfy_3 = 0;
  vpf_msg.pfz_3 = 0;

  vpf_msg.pfx_4 = 0;
  vpf_msg.pfy_4 = 0;
  vpf_msg.pfz_4 = 0;

  yaw_des_count = 0;
  yaw_error_ = 0;
  debug_msg.data = {0,0,0,0,0,0,0,0,0,0};
  weights_msg.data = {weights[0],weights[1],weights[2],
                      weights[3],weights[4],weights[5],
                      weights[6],weights[7],weights[8],
                      weights[9],weights[10],weights[11],
                      0,0,0,
                      0,0,0,
                      0,0,0,
                      0,0,0};

  des_roll = MovingWindowFilter(5);
  des_pitch = MovingWindowFilter(5);
  des_yaw  = MovingWindowFilter(5);
  z_force_  = MovingWindowFilter(50);
  mass_fix = MovingWindowFilter(5);
  z_force = 8.3644*9.81;
  mass_ = 8.3644;
}

void ConvexMPCLocomotion::qweightCallback(const ros_bridge::msg::Qweights msg)
{
  weights[0] = msg.roll;
  weights[1] = msg.pitch;
  weights[2] = msg.yaw;
  weights[3] = msg.x;
  weights[4] = msg.y;
  weights[5] = msg.z;
  weights[6] = msg.omega_roll;
  weights[7] = msg.omega_pitch;
  weights[8] = msg.omega_yaw;
  weights[9] = msg.v_x;
  weights[10] = msg.v_y;
  weights[11] = msg.v_z;
  //std::cout<<"sucess"<<std::endl;
}

void ConvexMPCLocomotion::initialize(){
  for(int i = 0; i < 4; i++) firstSwing[i] = true;
  firstRun = true;
}

void ConvexMPCLocomotion::recompute_timing(int iterations_per_mpc) {
  iterationsBetweenMPC = iterations_per_mpc;
  dtMPC = dt * iterations_per_mpc;
}

void ConvexMPCLocomotion::_SetupCommand(ControlFSMData<float> & data){
  
  if(data._quadruped->_robotType == RobotType::MINI_CHEETAH){
    _body_height = 0.25;
  }else if(data._quadruped->_robotType == RobotType::CHEETAH_3){
    _body_height = 0.45;
  }else{
    assert(false);
  }
  if(firstRun)
  {
    yaw_des_count = data._stateEstimator->getResult().rpy[2];
  }
  float x_vel_cmd, y_vel_cmd;
  float filter(0.3);
  if(data.controlParameters->use_rc){
    const rc_control_settings* rc_cmd = data._desiredStateCommand->rcCommand;
    data.userParameters->cmpc_gait = rc_cmd->variable[0];
    _yaw_turn_rate = -rc_cmd->omega_des[2];
    if(yaw_des_count* data._stateEstimator->getResult().rpy[2]<0)
    {
      yaw_des_count = data._stateEstimator->getResult().rpy[2];
    }
    yaw_des_count -= dt * rc_cmd->omega_des[2];

    x_vel_cmd = rc_cmd->v_des[0];
    y_vel_cmd = rc_cmd->v_des[1] * 0.5;
    _body_height += rc_cmd->height_variation * 0.08;
    step_height = rc_cmd->variable[1];
    if(step_height < 0.01f)
      step_height = 0.05f;
    else if(step_height > 0.12f)
      step_height = 0.12f;
  //  step_height += ((float)rpyDesFin(1))*-.4f-0.003;
    _pitch_des = rc_cmd->omega_des[1];
    isprint = rc_cmd->isPrint;
  }else{
    _yaw_turn_rate = data._desiredStateCommand->rightAnalogStick[0];
    x_vel_cmd = data._desiredStateCommand->leftAnalogStick[1];
    y_vel_cmd = data._desiredStateCommand->leftAnalogStick[0];
  }
  _x_vel_des = _x_vel_des*(1-filter) + x_vel_cmd*filter;
  _y_vel_des = _y_vel_des*(1-filter) + y_vel_cmd*filter;
  // ------ FIX YAW ------//
  float yaw_error = yaw_des_count - data._stateEstimator->getResult().rpy[2];
  //yaw_error_ = yaw_error;
  // if(std::abs(yaw_error)>0.05)
  //_yaw_turn_rate += yaw_error;
  // ------ FIX YAW END -------//

  _yaw_des =  data._stateEstimator->getResult().rpy[2] + dt * _yaw_turn_rate;//data._stateEstimator->getResult().rpy[2]
  
  _roll_des = 0.;
}

template<>
void ConvexMPCLocomotion::run(ControlFSMData<float>& data) {


  bool omniMode = false;
  rclcpp::spin_some(this->node);
  // Command Setup
  _SetupCommand(data);
  gaitNumber = data.userParameters->cmpc_gait;
  if(gaitNumber >= 10) {
    gaitNumber -= 10;
    omniMode = true;
  }

  auto& seResult = data._stateEstimator->getResult();
  
  // Check if transition to standing
  if(((gaitNumber == 0) && current_gait != 0) || firstRun)
  {
    stand_traj[0] = seResult.position[0];
    stand_traj[1] = seResult.position[1];
    stand_traj[2] = 0.25;
    stand_traj[3] = 0;
    stand_traj[4] = 0;
    stand_traj[5] = seResult.rpy[2];
    world_position_desired[0] = stand_traj[0];
    world_position_desired[1] = stand_traj[1];
  }

  // pick gait
  Gait* gait = &standing;
  if(gaitNumber == 4)
    gait = &trotting;
  else if(gaitNumber == 5)
    gait = &walking;
  else if(gaitNumber == 6)
    gait = &pacing;
  else if(gaitNumber == 7)
    gait = &jumping;
  else if(gaitNumber == 8)
    gait = &galloping;
  else if(gaitNumber == 9)
    gait = &bounding;
  current_gait = gaitNumber;

  gait->setIterations(iterationsBetweenMPC, iterationCounter);
  jumping.setIterations(iterationsBetweenMPC, iterationCounter);

  jumping.setIterations(27/2, iterationCounter);

  jump_state.trigger_pressed(jump_state.should_jump(jumping.getCurrentGaitPhase()),
                             data._desiredStateCommand->trigger_pressed);
  // check jump action
  if(jump_state.should_jump(jumping.getCurrentGaitPhase())) {
    gait = &jumping;
    recompute_timing(27/2);
    _body_height = _body_height_jumping;
    currently_jumping = true;

  } else {
    recompute_timing(default_iterations_between_mpc);
    currently_jumping = false;
  }

  if(_body_height < 0.02) {
    _body_height = 0.25;
  }
  // integrate position setpoint
  Vec3<float> v_des_robot(_x_vel_des, _y_vel_des, 0);
  Vec3<float> v_des_world =
          omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
  Vec3<float> v_robot = seResult.vWorld;

  if(fabs(v_robot[0]) > .2)   //avoid dividing by zero
  {
    rpy_int[1] += dt*(_pitch_des - seResult.rpy[1])/v_robot[0];
  }
  if(fabs(v_robot[1]) > 0.1)
  {
    rpy_int[0] += dt*(_roll_des - seResult.rpy[0])/v_robot[1];
  }

  rpy_int[0] = fminf(fmaxf(rpy_int[0], -.25), .25);
  rpy_int[1] = fminf(fmaxf(rpy_int[1], -.25), .25);
  rpy_comp[1] = v_robot[0] * rpy_int[1];
  rpy_comp[0] = v_robot[1] * rpy_int[0] * (gaitNumber!=8);  //turn off for pronking


  for(int i = 0; i < 4; i++) {
    pFoot[i] = seResult.position +
               seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) +
                                             data._legController->datas[i].p);
  }

  if(gait != &standing) {
    world_position_desired += dt * Vec3<float>(v_des_world[0], v_des_world[1], 0);
  }

  // some first time initialization
  if(firstRun)
  {
    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.rpy[2];

    for(int i = 0; i < 4; i++)
    {

      footSwingTrajectories[i].setHeight(0.05);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);

    }
    firstRun = false;
  }

  // foot placement
  for(int l = 0; l < 4; l++)
    swingTimes[l] = gait->getCurrentSwingTime(dtMPC, l);

  float side_sign[4] = {-1, 1, -1, 1};//-1, 1, -1, 1
  //------ FIX FOOT P ------//
  // float f_r[4] = {2, 2, -2, -2};
  //------ FIX FOOT END ------//
  float interleave_y[4] = {-0.03, 0.03, 0.03, -0.03};
  float interleave_gain = -0.2;//-0.2before
  //float v_abs = std::fabs(seResult.vBody[0]);
  float v_abs = std::fabs(v_des_robot[0]);

  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;// 
    }
  
    footSwingTrajectories[i].setHeight(step_height);
    Vec3<float> offset(0.03, side_sign[i] * 0.065, 0);

    Vec3<float> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);

    pRobotFrame[1] += interleave_y[i] * v_abs * interleave_gain;
    float stance_time = gait->getCurrentStanceTime(dtMPC, i);
    Vec3<float> pYawCorrected =
            coordinateRotation(CoordinateAxis::Z, -_yaw_turn_rate* stance_time / 2) * pRobotFrame;

    Vec3<float> des_vel;
    des_vel[0] = _x_vel_des;
    des_vel[1] = _y_vel_des;
    des_vel[2] = 0.0;

    Vec3<float> Pf = seResult.position + seResult.rBody.transpose() * (pYawCorrected
                                                                       + des_vel * swingTimeRemaining[i]);
    // ----- FIX PF ------ // 
#ifdef USE_PF_FIX                                                              
    float p_rel_max = 0.1f;

    // Using the estimated velocity is correct
    float pfx_rel = seResult.vWorld[0] * (.5 + _parameters->cmpc_bonus_swing) * stance_time* dtMPC +//* dtMPC
                    .03f*(seResult.vWorld[0]-v_des_world[0]) +
                    (0.5f*seResult.position[2]/9.81f) * (seResult.vWorld[1]*_yaw_turn_rate);
    float pfy_rel = seResult.vWorld[1] * .5 * stance_time * dtMPC +//.5 * stance_time * dtMPC 
                    .03f*(seResult.vWorld[1]-v_des_world[1]) +
                    (0.5f*seResult.position[2]/9.81f) * (-seResult.vWorld[0]*_yaw_turn_rate);
    pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
    
    Pf[0] +=  pfx_rel;//0 * f_r[i]
    Pf[1] +=  pfy_rel;//0 * f_r[i]
#endif
    Pf[2] =   -0.002;
#ifdef USE_PF_FIX 
    if(((float)rpyDesFin(1)) < 0)
    {
      if(i==0||i==1)
      {
        Pf[2] += ((float)rpyDesFin(1))*-.5f;//-0.003rpyDesFin(1)*-.164*2
      }
      else{
        Pf[2] -= ((float)rpyDesFin(1))*-.1f;
        if(Pf[2]<-0.003)
        {
          Pf[2] = -0.003;
        }
      }
    }
#endif
    // ----- FIX PF END------ // 

    // ----- SET FOOT SWING POINT ------ // 
    footSwingTrajectories[i].setFinalPosition(Pf);
    // ----- SET FOOT SWING POINT END------ // 
#ifdef USE_ROS_DEBUG
    // ----- ROS DEBUG PF ------ // 
    if(i==0)
    {
      vpf_msg.pfx_1 = Pf[0];
      vpf_msg.pfy_1 = Pf[1];
      vpf_msg.pfz_1 = Pf[2];
    }else if(i==1)
    {
      vpf_msg.pfx_2 = Pf[0];
      vpf_msg.pfy_2 = Pf[1];
      vpf_msg.pfz_2 = Pf[2];
    }else if(i==2)
    {
      vpf_msg.pfx_3 = Pf[0];
      vpf_msg.pfy_3 = Pf[1];
      vpf_msg.pfz_3 = Pf[2];
    }else if(i==3)
    {
      vpf_msg.pfx_4 = Pf[0];
      vpf_msg.pfy_4 = Pf[1];
      vpf_msg.pfz_4 = Pf[2];
    }
#endif
  }
#ifdef USE_ROS_DEBUG
  vpf_pub->publish(vpf_msg);
  // ----- ROS DEBUG PF END------ // 
#endif

  // ----- GET GAIT STATE------ //
  Vec4<float> contactStates = gait->getContactState();
  Vec4<float> swingStates = gait->getSwingState();
  // ----- GET GAIT STATE END------ //

  // ----- COMPUTE TERRAIN ------ //
#ifdef USE_TERRAIN_A
  for(int i = 0; i < 4; i++){
    if(swingStates[i] == 0 && gaitNumber != 0){
      wPla(i, 1) = pFoot[i](0);
      wPla(i, 2) = pFoot[i](1);
      zFoot(i) = pFoot[i](2);
      pseudoInverse(wPla, 0.001, wPlaInv);
      aPla = (1 - 0.4) * aPla + (0.4) * wPlaInv * zFoot;
    }
  }

  Vec3<float> rPla_1(-aPla(1), -aPla(2), 1);
  Vec3<float> rPla_2(aPla(2), -aPla(1), 0);
  rPla_1.normalize();
  rPla_2.normalize();
  Vec3<float> rPla_3 = rPla_2.cross(rPla_1);
  rPla.block(0, 0, 3, 1) = rPla_3;
  rPla.block(0, 1, 3, 1) = rPla_2;
  rPla.block(0, 2, 3, 1) = rPla_1;
  rpyPlane = rotationMatrixToRPY(rPla);
  rpyPlane(2) = 0;
  rPla = rpyToRotMat(rpyPlane);
  debug_msg.data[0] = (double)rpyPlane[0];
  debug_msg.data[1] = (double)rpyPlane[1];
  debug_msg.data[2] = (double)rpyPlane[2];
  rpyPlane(0) = des_roll.CalculateAverage(rpyPlane(0));
  rpyPlane(1) = des_pitch.CalculateAverage(rpyPlane(1));
  rpyPlane(2) = des_yaw.CalculateAverage(rpyPlane(2));
  rPla = rpyToRotMat(rpyPlane);
#endif
  // ----- COMPUTE TERRAIN END------ //

  // ----- ROS DEBUG TERRAIN ------ //
  debug_msg.data[3] = (double)rpyPlane[0];
  debug_msg.data[4] = (double)rpyPlane[1];
  debug_msg.data[5] = (double)rpyPlane[2];
  debug_pub->publish(debug_msg);
  // ----- ROS DEBUG TERRAIN END------ //

  
  // calc gait
  iterationCounter++;

  // load LCM leg swing gains
  Kp << 700, 0, 0,//700
          0, 700, 0,//700
          0, 0, 150;//150
  Kp_stance = 0*Kp;


  Kd <<   7, 0, 0,//7,0,0,
          0, 7, 0,//0,7,0,
          0, 0, 7;//0,0,7;
  Kd_stance = Kd;

  // ----- GET MPC TABLE & UPDATE MPC ------ //
  int* mpcTable = gait->getMpcTable();
  updateMPCIfNeeded(mpcTable, data, omniMode);
  // ----- GET MPC TABLE & UPDATE MPC END------ //

  Vec4<float> se_contactState(0,0,0,0);

  for(int foot = 0; foot < 4; foot++)
  {
    float contactState = contactStates[foot];
    float swingState = swingStates[foot];
    if(swingState > 0) // foot is in swing
    {
      if(firstSwing[foot])
      {
        firstSwing[foot] = false;
        footSwingTrajectories[foot].setInitialPosition(pFoot[foot]);
        //pFoot_last_contact[foot] = pFoot[foot];
      }

      footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position)
                            - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);

      // Update for WBC
      pFoot_des[foot] = pDesFootWorld;
      vFoot_des[foot] = vDesFootWorld;
      aFoot_des[foot] = footSwingTrajectories[foot].getAcceleration();

      if(!data.userParameters->use_wbc){
        // Update leg control command regardless of the usage of WBIC
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp;
        data._legController->commands[foot].kdCartesian = Kd;
      }
    }
    else // foot is in stance
    {
      firstSwing[foot] = true;

      Vec3<float> pDesFootWorld = footSwingTrajectories[foot].getPosition();
      Vec3<float> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
      Vec3<float> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) - data._quadruped->getHipLocation(foot);
      Vec3<float> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
      //cout << "Foot " << foot << " relative velocity desired: " << vDesLeg.transpose() << "\n";

      if(!data.userParameters->use_wbc){
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;

        data._legController->commands[foot].forceFeedForward = f_ff[foot];
        data._legController->commands[foot].kdJoint = Mat3<float>::Identity() * 0.2;
      }else{ // Stance foot damping
        data._legController->commands[foot].pDes = pDesLeg;
        data._legController->commands[foot].vDes = vDesLeg;
        data._legController->commands[foot].kpCartesian = 0.*Kp_stance;
        data._legController->commands[foot].kdCartesian = Kd_stance;
      }
      se_contactState[foot] = contactState;

      // Update for WBC
      //Fr_des[foot] = -f_ff[foot];
    }
  }

  
  data._stateEstimator->setContactPhase(contactStates);
  // CONTACT DETECT
  data._stateEstimator->setSwingPhase(swingStates);
  // ----- UPDATE FOR WBC ------ //
  pBody_des[0] = world_position_desired[0];
  pBody_des[1] = world_position_desired[1];
  pBody_des[2] = _body_height;

  vBody_des[0] = v_des_world[0];
  vBody_des[1] = v_des_world[1];
  vBody_des[2] = 0.;

  aBody_des.setZero();

  pBody_RPY_des[0] = rpyDesFin[0];
  pBody_RPY_des[1] = rpyDesFin[1];
  pBody_RPY_des[2] = _yaw_des;//_yaw_des

  vBody_Ori_des[0] = 0.;
  vBody_Ori_des[1] = 0.;
  vBody_Ori_des[2] = _yaw_turn_rate;//

  //contact_state = gait->getContactState();
  contact_state = gait->getContactState();
  // ----- UPDATE FOR WBC END------ //

  // ----- DEBUG MPC STATE ------ //
  // static int step = 0;
  // if(isprint)
  // {
  //   step ++;
  //   std::cout.precision(4);
  //   if(step % 100 == 0)
  //   {
  //     std::cout <<"[MPC] current position : " << seResult.position[0] << " " << seResult.position[1] << " " << seResult.position[2] << std::endl;
  //     std::cout <<"[MPC] current velocity : " << seResult.vBody[0] << " " << seResult.vBody[1] << " " << seResult.vBody[2] << std::endl;
  //   }
  //   if(step > 200)
  //   {
  //     step = 0;
  //   }
  // }
  // ----- DEBUG MPC STATE ------ //

}


template<>
void ConvexMPCLocomotion::run(ControlFSMData<double>& data) {
  (void)data;
  printf("call to old CMPC with double!\n");

}

void ConvexMPCLocomotion::updateMPCIfNeeded(int *mpcTable, ControlFSMData<float> &data, bool omniMode) {
  //iterationsBetweenMPC = 30;
  if((iterationCounter % iterationsBetweenMPC) == 0)
  {
    auto seResult = data._stateEstimator->getResult();
#ifdef USE_ROS_DEBUG
    // ----- ROS DEBUG ROBOTSTATE ------ //
    float* p = seResult.position.data();
    vstate_msg.roll = seResult.rpy[0];
    vstate_msg.pitch = seResult.rpy[1];
    vstate_msg.yaw = seResult.rpy[2];
    vstate_msg.x = seResult.position[0];
    vstate_msg.y = seResult.position[1];
    vstate_msg.z = seResult.position[2];
    vstate_msg.roll_rate = seResult.omegaBody[0];
    vstate_msg.pitch_rate = seResult.omegaBody[1];
    vstate_msg.yaw_rate = seResult.omegaBody[2];
    vstate_msg.v_x = seResult.vBody[0];
    vstate_msg.v_y = seResult.vBody[1];
    vstate_msg.v_z = seResult.vBody[2];
    vstate_msg.a_x = seResult.aBody[0];
    vstate_msg.a_y = seResult.aBody[1];
    vstate_msg.a_z = seResult.aBody[2];
    vstate_pub->publish(vstate_msg);
#endif
    // ----- ROS DEBUG ROBOTSTATE END------ //
    Vec3<float> v_des_robot(_x_vel_des, _y_vel_des,0);
    Vec3<float> v_des_world = omniMode ? v_des_robot : seResult.rBody.transpose() * v_des_robot;
    // ----- PREDICT TRAJ FOR MPC------ //
    if(current_gait == 0)
    {
      float trajInitial[12] = {
              _roll_des,
              _pitch_des /*-hw_i->state_estimator->se_ground_pitch*/,
              (float)stand_traj[5]/*+(float)stateCommand->data.stateDes[11]*/,
              (float)stand_traj[0]/*+(float)fsm->main_control_settings.p_des[0]*/,
              (float)stand_traj[1]/*+(float)fsm->main_control_settings.p_des[1]*/,
              (float)_body_height/*fsm->main_control_settings.p_des[2]*/,//+rpyDesFin(1)*-.164*0.5
              0,0,0,0,0,0};

      for(int i = 0; i < horizonLength; i++)
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];
    }
    else
    {
      const float max_pos_error = .1;
      float xStart = world_position_desired[0];
      float yStart = world_position_desired[1];

      if(xStart - p[0] > max_pos_error) xStart = p[0] + max_pos_error;
      if(p[0] - xStart > max_pos_error) xStart = p[0] - max_pos_error;

      if(yStart - p[1] > max_pos_error) yStart = p[1] + max_pos_error;
      if(p[1] - yStart > max_pos_error) yStart = p[1] - max_pos_error;

      world_position_desired[0] = xStart;
      world_position_desired[1] = yStart;

      Eigen::Matrix<float,3,1> oriDes;
      Eigen::Matrix<float,4,1> quatDes;
      oriDes << _roll_des, _pitch_des, _yaw_des + dtMPC * _yaw_turn_rate;
      quatDes = rotationMatrixToQuaternion(rpyToRotMat(oriDes) * rPla.transpose());
      rpyDesFin = quatToRPY(quatDes);

      float trajInitial[12] = {
              (float)rpy_comp[0]+ rpyDesFin(0),
              (float)rpy_comp[1]+ rpyDesFin(1),
              _yaw_des,//_yaw_des,5月7日
              xStart,
              yStart,
              (float)_body_height,//+rpyDesFin(1)*-.164*0.5
              0,
              0,
              _yaw_turn_rate,
              v_des_world[0],
              v_des_world[1],
              0
      };
      
      for(int i = 0; i < horizonLength; i++)
      {
        for(int j = 0; j < 12; j++)
          trajAll[12*i+j] = trajInitial[j];

        if(i == 0) // start at current position  TODO consider not doing this
        {
          trajAll[2] = seResult.rpy[2];
        }
        else
        {
          trajAll[12*i + 3] = trajAll[12 * (i - 1) + 3] + dtMPC * v_des_world[0];
          trajAll[12*i + 4] = trajAll[12 * (i - 1) + 4] + dtMPC * v_des_world[1];
          trajAll[12*i + 2] = trajAll[12 * (i - 1) + 2] + dtMPC * _yaw_turn_rate;
        }
      }
      // ----- PREDICT TRAJ FOR MPC END------ //
#ifdef USE_ROS_DEBUG
      // ----- ROS DEBUG TRAJ------ //
      vtraj_msg.des_roll = trajInitial[0];
      vtraj_msg.des_pitch = trajInitial[1];
      vtraj_msg.des_yaw = trajInitial[2];
      vtraj_msg.des_x = trajInitial[3];
      vtraj_msg.des_y = trajInitial[4];
      vtraj_msg.des_z = trajInitial[5];
      vtraj_msg.des_roll_rate = trajInitial[6];
      vtraj_msg.des_pitch_rate = trajInitial[7];
      vtraj_msg.des_yaw_rate = trajInitial[8];
      vtraj_msg.v_des_x = trajInitial[9];
      vtraj_msg.v_des_y = trajInitial[10];
      vtraj_msg.v_des_z = trajInitial[11];
      vtraj_pub->publish(vtraj_msg);
      // ----- ROS DEBUG TRAJ END------ //
#endif
    }

    
    Timer solveTimer;
    // ----- SOLVER MPC------ //
    if(_parameters->cmpc_use_sparse > 0.5) {
      solveSparseMPC(mpcTable, data);
    } else {
      solveDenseMPC(mpcTable, data);
    } 
    // ----- SOLVER MPC END------ //
  }

}

void ConvexMPCLocomotion::solveDenseMPC(int *mpcTable, ControlFSMData<float> &data) {
  auto seResult = data._stateEstimator->getResult();
  // ----- MPC QWEIGHT ------ //
  static float Q[12] = {1, 1, 1, 2, 2, 50, 0, 0, 0.1, 1.5, 1.5, 0.2};
  // ----- MPC QWEIGHT END ------ //
  
  // ----- MPC STATE ------ //
  float yaw = seResult.rpy[2];//seResult.rpy[2]
  float* weight = Q;
  float alpha = 4e-5; // make setting eventually
  //float alpha = 4e-7; // make setting eventually: DH
  float* p = seResult.position.data();
  float* v = seResult.vWorld.data();
  float* w = seResult.omegaWorld.data();
  float* q = seResult.orientation.data();
  float r[12];
  for(int i = 0; i < 12; i++)
    r[i] = pFoot[i%4][i/4]  - seResult.position[i/4];
  if(alpha > 1e-4) {
    std::cout << "Alpha was set too high (" << alpha << ") adjust to 1e-5\n";
    alpha = 1e-5;
  }
  // ----- MPC STATE END------ //

  Vec3<float> pxy_act(p[0], p[1], 0);
  Vec3<float> pxy_des(world_position_desired[0], world_position_desired[1], 0);
  //Vec3<float> pxy_err = pxy_act - pxy_des;
  float pz_err = p[2] - _body_height;
  Vec3<float> vxy(seResult.vWorld[0], seResult.vWorld[1], 0);

  Timer t1;
  dtMPC = dt * iterationsBetweenMPC;
  setup_problem(dtMPC,horizonLength,0.4,120);
  //setup_problem(dtMPC,horizonLength,0.4,650); //DH
  update_x_drag(x_comp_integral);
  if(vxy[0] > 0.3 || vxy[0] < -0.3) {
    //x_comp_integral += _parameters->cmpc_x_drag * pxy_err[0] * dtMPC / vxy[0];
    x_comp_integral += _parameters->cmpc_x_drag * pz_err * dtMPC / vxy[0];
  }

  //printf("pz err: %.3f, pz int: %.3f\n", pz_err, x_comp_integral);

  update_solver_settings(_parameters->jcqp_max_iter, _parameters->jcqp_rho,
                         _parameters->jcqp_sigma, _parameters->jcqp_alpha, _parameters->jcqp_terminate, _parameters->use_jcqp);
  //t1.stopPrint("Setup MPC");

  Timer t2;
  //cout << "dtMPC: " << dtMPC << "\n";
  update_problem_data_floats(p,v,q,w,r,yaw,weight,trajAll,alpha,mpcTable);
  //t2.stopPrint("Run MPC");
  //printf("MPC Solve time %f ms\n", t2.getMs());

  for(int leg = 0; leg < 4; leg++)
  {
    Vec3<float> f;
    for(int axis = 0; axis < 3; axis++)
      f[axis] = get_solution(leg*3 + axis);

    f_ff[leg] = -seResult.rBody * f;
    // Update for WBC
    Fr_des[leg] = f;

  }
}

void ConvexMPCLocomotion::solveSparseMPC(int *mpcTable, ControlFSMData<float> &data) {
  // X0, contact trajectory, state trajectory, feet, get result!
  (void)mpcTable;
  (void)data;
  auto seResult = data._stateEstimator->getResult();
  std::vector<ContactState> contactStates;
  for(int i = 0; i < horizonLength; i++) {
    contactStates.emplace_back(mpcTable[i*4 + 0], mpcTable[i*4 + 1], mpcTable[i*4 + 2], mpcTable[i*4 + 3]);
  }

  for(int i = 0; i < horizonLength; i++) {
    for(u32 j = 0; j < 12; j++) {
      _sparseTrajectory[i][j] = trajAll[i*12 + j];
    }
  }

  Vec12<float> feet;
  for(u32 foot = 0; foot < 4; foot++) {
    for(u32 axis = 0; axis < 3; axis++) {
      feet[foot*3 + axis] = pFoot[foot][axis] - seResult.position[axis];
    }
  }
  // ----- DO OP WEIGHT ALGORITHM -----//
  // //----- CUREENT STATE -----//
  // Vec12<double> cur_state;
  // cur_state[0] = seResult.position[0];
  // cur_state[1] = seResult.position[1];
  // cur_state[2] = seResult.position[2];

  // cur_state[3] = seResult.vWorld[0];
  // cur_state[4] = seResult.vWorld[1];
  // cur_state[5] = seResult.vWorld[2];

  // cur_state[6] = seResult.rpy[0];
  // cur_state[7] = seResult.rpy[1];
  // cur_state[8] = seResult.rpy[2];

  // cur_state[9] = seResult.omegaWorld[0];
  // cur_state[10] = seResult.omegaWorld[1];
  // cur_state[11] = seResult.omegaWorld[2];
  // //----- CUREENT STATE END-----//
  // Vec12<double> des_state;
  // des_state[0] = trajAll[3];
  // des_state[1] = trajAll[4];
  // des_state[2] = trajAll[5];

  // des_state[3] = trajAll[9];
  // des_state[4] = trajAll[10];
  // des_state[5] = trajAll[11];

  // des_state[6] = trajAll[0];
  // des_state[7] = trajAll[1];
  // des_state[8] = trajAll[2];

  // des_state[9] = trajAll[6];
  // des_state[10] = trajAll[7];
  // des_state[11] = trajAll[8];
  // if(flag == firstSwing[0])
  // {
  //   weights = fweights.create_weights(true,cur_state,des_state,weights);
  // }else{
  //   weights = fweights.create_weights(false,cur_state,des_state,weights);
  //   flag = firstSwing[0];
  // }
  // weights_msg.data = {weights[0],weights[1],weights[2],
  //                   weights[3],weights[4],weights[5],
  //                   weights[6],weights[7],weights[8],
  //                   weights[9],weights[10],weights[11],
  //                   des_state[0],des_state[1],des_state[2],
  //                   des_state[3],des_state[4],des_state[5],
  //                   des_state[6],des_state[7],des_state[8],
  //                   des_state[9],des_state[10],des_state[11]};
  // weights_pub->publish(weights_msg);
  _sparseCMPC.setWeights(weights, 4e-5);
  // ----- DO OP WEIGHT ALGORITHM END -----//
  _sparseCMPC.setX0(seResult.position, seResult.vWorld, seResult.orientation, seResult.omegaWorld);
  _sparseCMPC.setContactTrajectory(contactStates.data(), contactStates.size());
  _sparseCMPC.setStateTrajectory(_sparseTrajectory);
  _sparseCMPC.setFeet(feet);
  _sparseCMPC.run();

  Vec12<float> resultForce = _sparseCMPC.getResult();

  for(u32 foot = 0; foot < 4; foot++) {
    // ------ compute z_force ----- //
    // if(firstSwing[foot])
    // {
    //   z_force += resultForce[foot*3 + 2];
    // }else{
    //   z_force += 0;
    // }
    // ------ compute z_force end ----- //
    Vec3<float> force(resultForce[foot*3], resultForce[foot*3 + 1], resultForce[foot*3 + 2]);
    f_ff[foot] = -seResult.rBody * force;
    Fr_des[foot] = force;
  }
  //------ ADAPTIVE LOAD ----- //
  // if(trajAll[11]+trajAll[10]+trajAll[9] <= 0.01 )
  // {
    // if(z_force<50)
    // {
    //   z_force = debug_msg.data[8]*9.81;
    // }
    // debug_msg.data[7] = z_force/9.81;
    // debug_msg.data[8] = z_force_.CalculateAverage(z_force/9.81);
    // Mat3<double> baseInertia;
    // baseInertia <<  0.07641, 0, 0,
    //         0, 0.17843, 0,
    //         0, 0, 0.197579;
    // double mass = debug_msg.data[8];//9//8.3644
    // //mass -= mass_fix.CalculateAverage((seResult.position[2] - trajAll[5]) * 1000 + (seResult.vBody[2] - trajAll[11]) * 100);
    // double maxForce = 150;
    // _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
    // z_force = 0;
  // }
    // Mat3<double> baseInertia;
    // baseInertia <<  0.07641, 0, 0,
    //         0, 0.17843, 0,
    //         0, 0, 0.197579;
    // mass_ -= mass_fix.CalculateAverage((seResult.position[2] - trajAll[5]) * 10);
    // debug_msg.data[8] = mass_;
    // double maxForce = 150;
    // _sparseCMPC.setRobotParameters(baseInertia, mass_, maxForce);
  // ------ ADAPTIVE LOAD end ----- //
  Eigen::VectorXd tau_fl(3);
  Eigen::VectorXd tau_fr(3);
  Eigen::VectorXd tau_rl(3);
  Eigen::VectorXd tau_rr(3);
  tau_fl<<  data._legController->_lowState->motorState[0].tauEst,
            data._legController->_lowState->motorState[1].tauEst,
            data._legController->_lowState->motorState[2].tauEst;
  tau_fr<<  data._legController->_lowState->motorState[3].tauEst,
            data._legController->_lowState->motorState[4].tauEst,
            data._legController->_lowState->motorState[5].tauEst;
  tau_rl<<  data._legController->_lowState->motorState[6].tauEst,
            data._legController->_lowState->motorState[7].tauEst,
            data._legController->_lowState->motorState[8].tauEst;
  tau_rr<<  data._legController->_lowState->motorState[9].tauEst,
            data._legController->_lowState->motorState[10].tauEst,
            data._legController->_lowState->motorState[11].tauEst;
  Eigen::MatrixXd iTJ_fl = data._legController->datas[0].J.cast<double>().transpose().inverse();
  Eigen::MatrixXd iTJ_fr = data._legController->datas[1].J.cast<double>().transpose().inverse(); 
  Eigen::MatrixXd iTJ_rl = data._legController->datas[2].J.cast<double>().transpose().inverse(); 
  Eigen::MatrixXd iTJ_rr = data._legController->datas[3].J.cast<double>().transpose().inverse();

  Eigen::VectorXd grf_fl = iTJ_fl * tau_fl;
  Eigen::VectorXd grf_fr = iTJ_fr * tau_fr;
  Eigen::VectorXd grf_rl = iTJ_rl * tau_rl;
  Eigen::VectorXd grf_rr = iTJ_rr * tau_rr; 
  z_force = grf_fl[2] + grf_fr[2]+ grf_rl[2] + grf_rr[2];

#ifdef USE_ROS_DEBUG  
 for(u32 foot = 0; foot < 4; foot++) {
  debug_msg.data[foot+6] = firstSwing[foot]+foot*2;//z_force/9.81
  }
#endif
  // if(debug_msg.data[7]<8)
  // {
  //   debug_msg.data[7] = 8.3644;
  // }
  // if(debug_msg.data[7]>9)
  // {
  //   debug_msg.data[7] = 8.3644;
  // }
  z_force = 0;

}

void ConvexMPCLocomotion::initSparseMPC() {
  Mat3<double> baseInertia;
  baseInertia <<  0.07641, 0, 0,
          0, 0.17843, 0,
          0, 0, 0.197579;
  double mass = 8.3644;//9//8.3644
  double maxForce = 150;

  std::vector<double> dtTraj;
  for(int i = 0; i < horizonLength; i++) {
    dtTraj.push_back(dtMPC);
  }
  _sparseCMPC.setRobotParameters(baseInertia, mass, maxForce);
  _sparseCMPC.setFriction(0.3);
  _sparseCMPC.setWeights(weights, 4e-5);
  _sparseCMPC.setDtTrajectory(dtTraj);

  _sparseTrajectory.resize(horizonLength);
}

#include <rt/rt_srv.h>
#include <iostream>

static pthread_mutex_t lcm_get_set_mutex =
        PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                             LCM */

// Controller Settings
rc_control_settings srv_control;

template<typename T1, typename T2>
inline T1 max(const T1 a, const T2 b){
  return (a > b ? a : b);
}

template<typename T1, typename T2>
inline T1 min(const T1 a, const T2 b){
  return (a < b ? a : b);
}

void get_srv_control_settings(void *settings) {
  pthread_mutex_lock(&lcm_get_set_mutex);
  v_memcpy(settings, &srv_control, sizeof(rc_control_settings));
  pthread_mutex_unlock(&lcm_get_set_mutex);
}

SrvCtrl::SrvCtrl(){
    userCmd = UserCommand::NONE;
    userValue.setZero();
    this->node = std::make_shared<rclcpp::Node>("SrvCtrl");
    cmd_srv = this->node->create_service<ros_bridge::srv::UsrCmd>("usr_cmd", std::bind(&SrvCtrl::cmd_srv_request, this, std::placeholders::_1, std::placeholders::_2));
    pthread_create(&_tid, NULL, runSrvCtrl, (void*)this);
}

SrvCtrl::~SrvCtrl(){
  pthread_cancel(_tid);
  pthread_join(_tid, NULL);
}

void SrvCtrl::rosspin()
{
    rclcpp::spin_some(this->node);
}
void SrvCtrl::cmd_srv_request(const  std::shared_ptr<ros_bridge::srv::UsrCmd::Request> request,
                                std::shared_ptr<ros_bridge::srv::UsrCmd::Response> response)
{
    //std::cout<<"111";
    switch (request->gait_type)
    {
    case 0:
        userCmd = UserCommand::PASSIVE;
    break;
    case 1:
        userCmd = UserCommand::RECOVERY_STAND;
    break;
    case 2:
        userCmd = UserCommand::BALANCE_STAND;
    break;
    case 3:
        userCmd = UserCommand::LOCOMOTION;
    break;
    default:
        userCmd = UserCommand::NONE;
    break;
    }
    userValue.forward_vel = request->cmd_velx;
    userValue.lateral_vel = request->cmd_vely;
    userValue.rotation_yaw = request->cmd_yaw;
    userValue.rotation_pitch = request->cmd_pitch;
    userValue.gait_type = request->gait_type;
    userValue.step_height = request->foot_height;
    response->cmd_result = true;
} 

void* SrvCtrl::runSrvCtrl(void *arg){
    ((SrvCtrl*)arg)->run(NULL);
    return NULL;
}

void* SrvCtrl::run(void *arg){
  (void) arg;
  while(1){
    switch (userCmd){
      case UserCommand::PASSIVE:
        srv_control.zeros();
        srv_control.mode = RC_mode::OFF;
        userValue.setZero();
        break;
      case UserCommand::RECOVERY_STAND:
        srv_control.zeros();
        srv_control.mode = RC_mode::RECOVERY_STAND;
        userValue.setZero();
        break;
      case UserCommand::BALANCE_STAND:
        srv_control.zeros();
        srv_control.mode = RC_mode::QP_STAND;
        userValue.setZero();
        break;
      case UserCommand::LOCOMOTION:
        srv_control.mode = RC_mode::LOCOMOTION;
        userValue.setZero();
        break;
      case UserCommand::NONE:
        if(srv_control.mode == RC_mode::LOCOMOTION){
          srv_control.v_des[0] = userValue.forward_vel;
          srv_control.v_des[1] = userValue.lateral_vel;
          srv_control.v_des[2] = 0.f;
          srv_control.omega_des[0] = 0.f;
          srv_control.omega_des[1] = userValue.rotation_pitch;
          srv_control.omega_des[2] = userValue.rotation_yaw;
          srv_control.variable[0] = userValue.gait_type;
          srv_control.variable[1] = userValue.step_height;
          srv_control.isPrint = userValue.isprint;
        }
        break;
      default:
        break;
    }

    usleep(1000);
  }
  return NULL;
}
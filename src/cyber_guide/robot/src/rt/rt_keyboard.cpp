
#include <rt/rt_keyboard.h>
#include <iostream>

static pthread_mutex_t lcm_get_set_mutex =
        PTHREAD_MUTEX_INITIALIZER; /**< mutex to protect gui settings coming over
                             LCM */

// Controller Settings
rc_control_settings keyboard_control;

template<typename T1, typename T2>
inline T1 max(const T1 a, const T2 b){
  return (a > b ? a : b);
}

template<typename T1, typename T2>
inline T1 min(const T1 a, const T2 b){
  return (a < b ? a : b);
}

void get_keyboard_control_settings(void *settings) {
  pthread_mutex_lock(&lcm_get_set_mutex);
  v_memcpy(settings, &keyboard_control, sizeof(rc_control_settings));
  pthread_mutex_unlock(&lcm_get_set_mutex);
}

KeyBoard::KeyBoard(){
  userCmd = UserCommand::NONE;
  userValue.setZero();

  tcgetattr( fileno( stdin ), &_oldSettings );
  _newSettings = _oldSettings;
  _newSettings.c_lflag &= (~ICANON & ~ECHO);
  tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

  pthread_create(&_tid, NULL, runKeyBoard, (void*)this);
}

KeyBoard::~KeyBoard(){
  pthread_cancel(_tid);
  pthread_join(_tid, NULL);
  tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
  switch (_c){
    case '0':
      return UserCommand::PASSIVE;
    case '1':
      return UserCommand::RECOVERY_STAND;
    case '2':
      return UserCommand::BALANCE_STAND;
    case '3':
      return UserCommand::LOCOMOTION;
    case ' ':
      userValue.setZero();
      return UserCommand::NONE;
    default:
      return UserCommand::NONE;
  }
}

void KeyBoard::changeValue(){
  switch (_c){
    case 'w':case 'W':
      userValue.forward_vel = min<float>(userValue.forward_vel+sensitivityLeft, 0.6);
      break;
    case 's':case 'S':
      userValue.forward_vel = max<float>(userValue.forward_vel-sensitivityLeft, -0.6);
      break;
    case 'a':case 'A':
      userValue.lateral_vel = min<float>(userValue.lateral_vel+sensitivityLeft, 1.0);
      break;
    case 'd':case 'D':
      userValue.lateral_vel = max<float>(userValue.lateral_vel-sensitivityLeft, -1.0);
      break;



    case 'i':case 'I':
      userValue.rotation_pitch = min<float>(userValue.rotation_pitch+sensitivityRight, 1.0);
      userValue.des_pitch = min<float>(userValue.des_pitch+sensitivityRight, 1.0);
      break;
    case 'k':case 'K':
      userValue.rotation_pitch = max<float>(userValue.rotation_pitch-sensitivityRight, -1.0);
      userValue.des_pitch = max<float>(userValue.des_pitch-sensitivityRight, -1.0);
      break;
    case 'l':case 'L':
      userValue.rotation_yaw = min<float>(userValue.rotation_yaw+sensitivityRight, 1.0);
      userValue.des_yaw = min<float>(userValue.des_yaw+sensitivityRight, 1.0);
      break;
    case 'j':case 'J':
      userValue.rotation_yaw = max<float>(userValue.rotation_yaw-sensitivityRight, -1.0);
      userValue.des_yaw = max<float>(userValue.des_yaw-sensitivityRight, -1.0);
      break;
    case 'u':case 'U':
      userValue.des_roll = min<float>(userValue.des_roll+sensitivityRight, 1.0);
      break;
    case 'o':case 'O':
      userValue.des_roll = max<float>(userValue.des_roll-sensitivityRight, -1.0);
      break;



    case 'v':case 'V':
      userValue.step_height = max<float>(userValue.step_height-sensitivityStep, 0.04);
      break;
    case 'b':case 'B':
      userValue.step_height = min<float>(userValue.step_height+sensitivityStep, 0.12);
      break;
    case 'p':case 'P':
      userValue.isprint = !userValue.isprint;
      break;
    case '4':
      userValue.gait_type = 4; // trot
      break;
    case '5':
      userValue.gait_type = 5; // walk
      break;
    case '6':
      userValue.gait_type = 6; // pace
      break;
    case '7':
      userValue.gait_type = 7; // jumping
      break;
    case '8':
      userValue.gait_type = 8; // gallop
      break;
    case '9':
      userValue.gait_type = 9; // bounding
      break;
    default:
      break;
  }
}

void* KeyBoard::runKeyBoard(void *arg){
  ((KeyBoard*)arg)->run(NULL);
  return NULL;
}

void* KeyBoard::run(void *arg){
  (void) arg;
  while(1){
    FD_ZERO(&set);
    FD_SET( fileno( stdin ), &set );

    res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

    if(res > 0){
      ret = read( fileno( stdin ), &_c, 1 );
      userCmd = checkCmd();
      if(userCmd == UserCommand::NONE)
        changeValue();
      _c = '\0';
    }

    switch (userCmd){
      case UserCommand::PASSIVE:
        keyboard_control.zeros();
        keyboard_control.mode = RC_mode::OFF;
        userValue.setZero();
        break;
      case UserCommand::RECOVERY_STAND:
        keyboard_control.zeros();
        keyboard_control.mode = RC_mode::RECOVERY_STAND;
        userValue.setZero();
        break;
      case UserCommand::BALANCE_STAND:
        keyboard_control.zeros();
        keyboard_control.mode = RC_mode::QP_STAND;
        userValue.setZero();
        break;
      case UserCommand::LOCOMOTION:
        keyboard_control.mode = RC_mode::LOCOMOTION;
        userValue.setZero();
        break;
      case UserCommand::NONE:
        if(keyboard_control.mode == RC_mode::LOCOMOTION){
          keyboard_control.v_des[0] = userValue.forward_vel;
          keyboard_control.v_des[1] = userValue.lateral_vel;
          keyboard_control.v_des[2] = 0.f;
          keyboard_control.omega_des[0] = 0.f;
          keyboard_control.omega_des[1] = userValue.rotation_pitch;
          keyboard_control.omega_des[2] = userValue.rotation_yaw;
          keyboard_control.variable[0] = userValue.gait_type;
          keyboard_control.variable[1] = userValue.step_height;
          keyboard_control.isPrint = userValue.isprint;
        }else if( keyboard_control.mode == RC_mode::QP_STAND )
        {
          keyboard_control.rpy_des[0] = userValue.des_roll;
          keyboard_control.rpy_des[1] = userValue.des_pitch;
          keyboard_control.rpy_des[2] = userValue.des_yaw;
        }
        break;
      default:
        break;
    }
    //debug
    std::cout<<"-----------------------"<<std::endl;
    std::cout<<"line x : "<< keyboard_control.v_des[0]<<std::endl;
    std::cout<<"line y : "<< keyboard_control.v_des[1]<<std::endl;
    std::cout<<"line z : "<< keyboard_control.v_des[2]<<std::endl;

    std::cout<<"-----------------------"<<std::endl;
    std::cout<<"roll : "<< keyboard_control.omega_des[0]<<std::endl;
    std::cout<<"pitch : "<< keyboard_control.omega_des[1]<<std::endl;
    std::cout<<"yaw : "<<  keyboard_control.omega_des[2]<<std::endl;

    std::cout<<"-----------------------"<<std::endl;
    std::cout<<"step_height : "<< keyboard_control.variable[1]<<std::endl;

    usleep(1000);
  }
  return NULL;
}
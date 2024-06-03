#ifndef SRC_RC_KEYBOARD_H
#define SRC_RC_KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "rt_rc_interface.h"
#include <pthread.h>

struct UserValue{
    float forward_vel;
    float lateral_vel;
    float rotation_yaw;
    float rotation_pitch;
    float step_height;
    int gait_type;
    bool isprint;
    float des_roll;
    float des_pitch;
    float des_yaw;
    UserValue(){
      setZero();
    }
    void setZero(){
      forward_vel = 0.f;
      lateral_vel = 0.f;
      rotation_yaw = 0.f;
      rotation_pitch = 0.f;
      step_height = 0.05f;
      gait_type = 0;
      isprint = false;
      des_roll = 0.f;
      des_pitch = 0.f;
      des_yaw= 0.f;
    }
};

enum class UserCommand{
    // EXIT,
    NONE,
    PASSIVE,
    RECOVERY_STAND,
    BALANCE_STAND,
    LOCOMOTION,
};

void get_keyboard_control_settings(void* settings);

class CmdPanel{
public:
    CmdPanel(){}
    virtual ~CmdPanel(){}
    UserCommand getUserCmd(){return userCmd;}
    UserValue getUserValue(){return userValue;}
    void setPassive(){userCmd = UserCommand::PASSIVE;}
    void setZero(){userValue.setZero();}
protected:
    virtual void* run(void *arg){ (void) arg; return NULL;}
    UserCommand userCmd;
    UserValue userValue;
};

class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
private:
    static void* runKeyBoard(void *arg);
    void* run(void *arg);
    UserCommand checkCmd();
    void changeValue();

    pthread_t _tid;
    float sensitivityLeft = 0.05;
    float sensitivityRight = 0.05;
    float sensitivityStep = 0.01;
    struct termios _oldSettings, _newSettings;
    fd_set set;
    int res;
    int ret;
    char _c;
};


#endif //SRC_RC_KEYBOARD_H

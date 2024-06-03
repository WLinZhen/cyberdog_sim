#ifndef CYBER_GUIDE_USERPARAMETERS_H
#define CYBER_GUIDE_USERPARAMETERS_H

#include "interface/mathTypes.h"
#include "interface/mathTools.h"
#include <iostream>

struct Mpc_Parameters{

    float orientation_Weight[3];
    float position_Weight[3];
    float omega_Weight[3];
    float velocity_Weight[3];

    void zeros(){

        for(int i = 0; i < 3; i ++){

            orientation_Weight[i] = 0.f;
            position_Weight[i] = 0.f;
            omega_Weight[i] = 0.f;
            velocity_Weight[i] = 0.f;

        }
    }
};

struct Rl_Parameters{

    float Kp_joint[3];
    float Kd_joint[3];

    void zeros(){

        for(int i = 0; i < 3; i ++){

            Kp_joint[i] = 0.f;
            Kd_joint[i] = 0.f;

        }
    }

};

struct Wbc_Parameters{

    float Kp_joint[3];
    float Kd_joint[3];

    float Kp_ori[3];
    float Kd_ori[3];

    float Kp_foot[3];
    float Kd_foot[3];

    float Kp_body[3];
    float Kd_body[3];

    void zeros(){

        for(int i = 0; i < 3; i ++){

            Kp_joint[i] = 0.f;
            Kd_joint[i] = 0.f;

            Kp_ori[i] = 0.f;
            Kd_ori[i] = 0.f;

            Kp_foot[i] = 0.f;
            Kd_foot[i] = 0.f;

            Kp_body[i] = 0.f;
            Kd_body[i] = 0.f;

        }
    }

};

struct User_Parameters{

    Mpc_Parameters mpc_paras;
    Rl_Parameters rl_paras;
    Wbc_Parameters wbc_paras;

    bool mpcEnable;
    bool rlEnable;
    bool wbcEnable;

    void zeros(){

        mpc_paras.zeros();
        rl_paras.zeros();
        wbc_paras.zeros();
        mpcEnable = false;
        rlEnable = false;
        wbcEnable = false;

    }

};
#endif //CYBER_GUIDE_USERPARAMETERS_H

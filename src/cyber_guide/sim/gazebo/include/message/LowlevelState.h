
#ifndef CYBER_GUIDE_LOWLEVELSTATE_H
#define CYBER_GUIDE_LOWLEVELSTATE_H

#include "interface/mathTypes.h"
#include "interface/mathTools.h"
#include <iostream>

struct MotorState
{
    unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];    // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];

    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMatd getRotMat(){
        Quatd quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3d getAcc(){
        Vec3d acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3d getGyro(){
        Vec3d gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quatd getQuat(){
        Quatd q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu;
    MotorState motorState[12];
    //UserCommand userCmd;
    //UserValue userValue;

    Vec34d getQ(){
        Vec34d qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].q;
            qLegs.col(i)(1) = motorState[3*i + 1].q;
            qLegs.col(i)(2) = motorState[3*i + 2].q;
        }
        return qLegs;
    }

    Vec34d getQd(){
        Vec34d qdLegs;
        for(int i(0); i < 4; ++i){
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }

    RotMatd getRotMat(){
        return imu.getRotMat();
    }

    Vec3d getAcc(){
        return imu.getAcc();
    }

    Vec3d getGyro(){
        return imu.getGyro();
    }

    Vec3d getAccGlobal(){
        return getRotMat() * getAcc();
    }

    Vec3d getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    double getYaw(){
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw(){
        return getGyroGlobal()(2);
    }

    void setQ(Vec12d q){
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);
        }
    }
};

#endif //CYBER_GUIDE_LOWLEVELSTATE_H

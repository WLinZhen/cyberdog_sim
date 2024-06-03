
#ifndef CYBER_GUIDE_LOWLEVELCMD_H
#define CYBER_GUIDE_LOWLEVELCMD_H

#include "interface/mathTypes.h"
#include "interface/mathTools.h"

struct MotorCmd{
    unsigned int mode;
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowlevelCmd{
    MotorCmd motorCmd[12];

    void setQ(Vec12d q){
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3d qi){
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }
    void setQd(Vec12d qd){
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3d qdi){
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }
    void setTau(Vec12d tau, Vec2d torqueLimit = Vec2d(-48, 48)){
        for(int i(0); i<12; ++i){
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }

    void setTau(int legID, Vec3d tau, Vec2d torqueLimit = Vec2d(-48, 48)){
        (void) torqueLimit;
        motorCmd[legID*3+0].tau = tau(0);
        motorCmd[legID*3+1].tau = tau(1);
        motorCmd[legID*3+2].tau = tau(2);
    }

    void setKp(Vec12d kp){
        for(int i(0); i<12; ++i){
            motorCmd[i].Kp = kp(i);
        }
    }
    void setKp(int legID, Vec3d Kp){
        motorCmd[legID*3+0].Kp = Kp(0);
        motorCmd[legID*3+1].Kp = Kp(1);
        motorCmd[legID*3+2].Kp = Kp(2);
    }

    void setKd(Vec12d kd){
        for(int i(0); i<12; ++i){
            motorCmd[i].Kd = kd(i);
        }
    }
    void setKd(int legID, Vec3d Kd){
        motorCmd[legID*3+0].Kd = Kd(0);
        motorCmd[legID*3+1].Kd = Kd(1);
        motorCmd[legID*3+2].Kd = Kd(2);
    }

    void setZeroQ(int legID){
        motorCmd[legID*3+0].q = 0;
        motorCmd[legID*3+1].q = 0;
        motorCmd[legID*3+2].q = 0;
    }
    void setZeroQ(){
        for(int i(0); i<4; ++i){
            setZeroQ(i);
        }
    }

    void setZeroDq(int legID){
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setZeroTau(){
        for(int i(0); i<4; ++i){
            setZeroTau(i);
        }
    }

};

#endif //CYBER_GUIDE_LOWLEVELCMD_H

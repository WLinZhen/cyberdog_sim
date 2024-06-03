

#ifndef _ACTUATOR_HPP__
#define _ACTUATOR_HPP__

#include <math.h>
#include <stdio.h>

#include "utilities.h"

namespace gazebo
{
  class Actuator
  {
  public:
    /**
     * @brief 构造一个新的 Actuator 对象
     * 
     * @param n1  : TN曲线的速度
     * @param n2  : TN曲线的速度
     * @param n3  : TN曲线的速度
     * @param tau_max : 最大扭矩
     * @param it_curve_a   : I-tau曲线的系数
     * @param it_curve_b   : I-tau曲线的系数
     * @param it_curve_c   : I-tau曲线的系数
     */
    Actuator(const double &n1 = 11.52, const double &n2 = 29.32, const double &n3 = 60.0, const double &tau_max = 12.001,  
             const double &it_curve_a = 0.0022, const double &it_curve_b = -0.0067, const double &it_curve_c = 1.1246)
    {
      n1_ = n1;     
      n2_ = n2;     
      n3_ = n3;
      tau_max_ = tau_max;    
      it_curve_a_ = it_curve_a;     
      it_curve_b_ = it_curve_b;     
      it_curve_c_ = it_curve_c; 
    }

    ~Actuator(){};

    /*!
    * 计算实际执行器扭矩，考虑摩擦（干摩擦和阻尼摩擦）、电压限制和扭矩限制
    * @param tauDes : 期望扭矩
    * @param qd : 当前执行器速度（在关节处）
    * @return 实际产生的扭矩
    */
    double GetTorque(double tau_des, double qd){
      double tau_act_ = tau_des;

      // TN curve
        if ( qd < -n3_ ) {
            tau_act_ = 0;
            //printf( "actuator speed less than min\n" );
        }

        if ( qd >= -n3_ && qd < -n2_ ) {
            if ( tau_act_ > tau_max_ ) {
                tau_act_ = tau_max_;
            }
            if ( tau_act_ < 0 ) {
                tau_act_ = 0;
            }
        }

        if ( qd >= -n2_ && qd < -n1_ ) {
            double tau_act_min = ( tau_max_ ) / ( n1_ - n2_ ) * qd - ( -( ( tau_max_ ) * n2_ ) / ( n1_ - n2_ ) );
            if ( tau_act_ > tau_max_ ) {
                tau_act_ = tau_max_;
            }
            if ( tau_act_ < tau_act_min ) {
                tau_act_ = tau_act_min;
            }
        }

        if ( qd >= -n1_ && qd < n1_ ) {
            if ( tau_act_ > tau_max_ ) {
                tau_act_ = tau_max_;
            }
            if ( tau_act_ < -tau_max_ ) {
                tau_act_ = -tau_max_;
            }
        }

        if ( qd >= n1_ && qd < n2_ ) {
            double tau_act_max = ( tau_max_ ) / ( n1_ - n2_ ) * qd + ( -( ( tau_max_ ) * n2_ ) / ( n1_ - n2_ ) );
            if ( tau_act_ > tau_act_max ) {
                tau_act_ = tau_act_max;
            }
            if ( tau_act_ < -tau_max_ ) {
                tau_act_ = -tau_max_;
            }
        }

        if ( qd >= n2_ && qd < n3_ ) {
            if ( tau_act_ < -tau_max_ ) {
                tau_act_ = -tau_max_;
            }
            if ( tau_act_ > 0 ) {
                tau_act_ = 0;
            }
        }

        if ( qd >= n3_ ) {
            tau_act_ = 0;
            //printf( "actuator speed over max\n" );
        }

      return tau_act_;
    }

    double CerrentLoopResponse(double tauDes,double qd, int i){
      if(!first_in_[i]) {
        first_in_[i] = true;
        double I_motor = getActuatorI(tauDes, qd);
        i_last_[i] = I_motor;
        tau_last_[i] = tauDes;
        
        return tauDes;
      }
      else{
        double I_motor = getActuatorI(tauDes, qd);
        double d_I = I_motor-i_last_[i];

        if(d_I < 3.0 && d_I > -3.0){
          i_last_[i] = I_motor;
          tau_last_[i] = tauDes;
          return tauDes;
        }
        
        if(d_I > 3.0)
        {
          d_I = 3.0;
        }
        if(d_I < -3.0)
        {
          d_I = -3.0;
        }
        I_motor = i_last_[i]+d_I; 
        i_last_[i] = I_motor;
        tauDes = getActuatorT(I_motor, tau_last_[i]);
        tau_last_[i] = tauDes;
        return tauDes;
      }
    }

    /**
     * @brief 获取执行器电流
     * 
     * @param tauDes : 期望扭矩
     * @param qd     : 电机速度
     * @return double 
     */
    double getActuatorI(double tauDes, double qd){
      
      double I_motor;
      I_motor = it_curve_a_ * tauDes * tauDes * tauDes + it_curve_b_ * tauDes * tauDes + it_curve_c_ * tauDes;
      
      return I_motor;
    }

    /**
     * @brief 获取执行器扭矩
     * 
     * @param I_motor  : 电机电流
     * @param tau_last_ : 上一步的扭矩
     * @return double 
     */
    double getActuatorT(double I_motor, double tau_last_){      
      double k_tau = 3*it_curve_a_*tau_last_*tau_last_+2*it_curve_b_*tau_last_+it_curve_c_;
      tau_last_ = tau_last_ - (it_curve_a_ * tau_last_ * tau_last_ * tau_last_ + it_curve_b_ * tau_last_ * tau_last_ + it_curve_c_ * tau_last_-I_motor)/k_tau;
      while(fabs(it_curve_a_ * tau_last_ * tau_last_ * tau_last_ + it_curve_b_ * tau_last_ * tau_last_ + it_curve_c_ * tau_last_-I_motor)>1e-5) {
          k_tau = 3*it_curve_a_*tau_last_*tau_last_+2*it_curve_b_*tau_last_+it_curve_c_;
          tau_last_ = tau_last_ - (it_curve_a_ * tau_last_ * tau_last_ * tau_last_ + it_curve_b_ * tau_last_ * tau_last_ + it_curve_c_ * tau_last_-I_motor)/k_tau;
      }
              
      return tau_last_;
    }



  private:
    double tau_max_;         // 最大扭矩
    double n1_;              // TN曲线的速度
    double n2_;              // TN曲线的速度
    double n3_;              // TN曲线的速度
    double it_curve_a_;      // I-tau曲线的系数
    double it_curve_b_;      // I-tau曲线的系数
    double it_curve_c_;      // I-tau曲线的系数
    bool first_in_[12];
    double i_last_[12];
    double tau_last_[12];
    
  };

}

#endif //_ACTUATOR_HPP__
#ifndef _RobotState
#define _RobotState

#include <eigen3/Eigen/Dense>
#include "common_types.h"

using Eigen::Matrix;
using Eigen::Quaternionf;

#include "common_types.h"
class RobotState
{
    public:
        void set(flt* p, flt* v, flt* q, flt* w, flt* r, flt yaw);
        //void compute_rotations();
        void print();
        Matrix<fpt,3,1> p,v,w;  // Position-world , Linear velocity-world ， 
        Matrix<fpt,3,4> r_feet; // four foot for body-origin
        Matrix<fpt,3,3> R;      
        Matrix<fpt,3,3> R_yaw;  
        Matrix<fpt,3,3> I_body; // I-whole body
        Quaternionf q;          // body ori
        fpt yaw;
        fpt m = 6.6996;//8.3644//6.6996(身体+头部+4髋+4大腿)
        //fpt m = 50.236; //DH
    //private:
};
#endif

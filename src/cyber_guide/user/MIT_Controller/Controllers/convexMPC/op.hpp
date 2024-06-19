#include <cassert>
#include <iostream>
#include <utility>
#include <deque>
#include <math.h>
#include <cmath>
#include <algorithm>
#include <vector>
#include "cppTypes.h"
using namespace std;

class op
{
public:
    op(){}
    op(Vec12<double> weights_) : weights(weights_)
    {
        //----- FITNESS INIT -----//
        rpy_fitness = 0;
        position_fitness = 0;
        v_fitness = 0;
        omega_fitness = 0;
        fitness = 0;
        //----- FITNESS INIT END -----//
        a = 2.0;
        a_decay = 2.0 / 100;
        best_whale = weights_;
    }

    //----- FITNESS COMPUTE -----//
    void comfitness()
    {
        // 实现你的适应度计算
        // 示例: 简单的适应度总和
        position_fitness += (abs(cur_state[0]-des_state[0])* best_whale[3]+ abs(cur_state[1]-des_state[1]) * best_whale[4]+ abs(cur_state[2]-des_state[2]) * best_whale[5]); // 计算位置适应度
        v_fitness += (abs(cur_state[3]-des_state[3]) * best_whale[9] + abs(cur_state[4]-des_state[4]) * best_whale[10] + abs(cur_state[5]-des_state[5]) * best_whale[11]); // 计算线速度适应度
        rpy_fitness += (abs(cur_state[6]-des_state[6]) * best_whale[0] + abs(cur_state[7]-des_state[7])* best_whale[1] + abs(cur_state[8]-des_state[8]) * best_whale[2]); // 计算姿态适应度
        omega_fitness += (abs(cur_state[9]-des_state[9]) * best_whale[6] + abs(cur_state[9]-des_state[9]) * best_whale[7] + abs(cur_state[9]-des_state[9]) * best_whale[8]);; // 计算角速度适应度
    }
    //----- FITNESS COMPUTE END -----//
    void fitnesszero()
    {
        //----- FITNESS INIT -----//
        rpy_fitness = 0;
        position_fitness = 0;
        v_fitness = 0;
        omega_fitness = 0;
        //----- FITNESS INIT END -----//
    }
    //----- UPDATE ROBOT STATE -----//
    void updatestate(Vec12<double> cur_state_, Vec12<double> des_state_)
    {
        cur_state = cur_state_;
        des_state = des_state_;
        comfitness();
    }
    //----- UPDATE ROBOT STATE END -----//

    //----- CREATE NEW WEIGHT -----//
    Vec12<double> create_weights(bool flag, Vec12<double> cur_state_, Vec12<double> des_state_, Vec12<double> weights_)
    {
        if (flag)
        {
            weights = weights_;
            updatestate(cur_state_, des_state_);
            return weights;
        }
        else
        {
            weights = weights_;
            updatestate(cur_state_, des_state_);
            // 更新最好的鲸鱼
            if(fitness == 0)
            {
                best_whale = weights;
                fitness = position_fitness + v_fitness + rpy_fitness + omega_fitness;
                position_fitness_o = position_fitness;
                v_fitness_o = v_fitness;
                rpy_fitness_o = rpy_fitness;
                omega_fitness_o = omega_fitness;
            }else{
                double newfitness = position_fitness + v_fitness + rpy_fitness + omega_fitness;
                if(fitness>newfitness)
                {
                    if(rpy_fitness_o > rpy_fitness)
                    {
                        best_whale[0] = weights_[0];
                        best_whale[1] = weights_[1];
                        best_whale[2] = weights_[2];
                    }
                    if(position_fitness_o > position_fitness)
                    {
                        best_whale[3] = weights_[3];
                        best_whale[4] = weights_[4];
                        best_whale[5] = weights_[5];
                    }
                    if(omega_fitness_o > omega_fitness)
                    {
                        best_whale[6] = weights_[6];
                        best_whale[7] = weights_[7];
                        best_whale[8] = weights_[8];
                    }
                    if(v_fitness_o > v_fitness)
                    {
                        best_whale[9] = weights_[9];
                        best_whale[10] = weights_[10];
                        best_whale[11] = weights_[11];
                    }
                    fitness = newfitness;
                }
            }
            fitnesszero();
            // 生成新的weights
            Vec12<double> new_weights;
            double r = (static_cast<double>(rand()) / RAND_MAX); // [0,1] 之间的随机数
            double A = 2.0 * a * r - a; // 方程 (3.3)
            double C = 2.0 * r; // 方程 (3.4)
            

            for (int j = 0; j < 12; ++j)
            {
                double p = static_cast<double>(rand()) / RAND_MAX; // [0,1] 之间的随机数
                if (p < 0.5) // 50% 概率
                {
                    double D = C * abs(best_whale[j] - weights[j]); // 方程 (3.5)
                    new_weights[j] = best_whale[j] - A * D; // 方程 (3.6)
                    new_weights[j] =  best_whale[j] * 0.95;
                }
                else
                {
                    // 螺旋更新位置
                    double b = 1.0; // 一个常数
                    double l = (static_cast<double>(rand()) / RAND_MAX) * 2.0 - 1.0; // [-1,1] 之间的随机数
                    double distance_to_best = abs(best_whale[j] - weights[j]);
                    new_weights[j] = distance_to_best * exp(b * l) * cos(2 * M_PI * l) + best_whale[j]; // 方程 (3.12)
                    new_weights[j] =  best_whale[j] * 1.05;
                }
            }

            // 边界检查（确保权重在所需范围内）
            for (int j = 0; j < 12; ++j)
            {
                if (new_weights[j] < 0.01) new_weights[j] = 0.01;
                if (new_weights[j] > 100) new_weights[j] = 100;
            }

            return new_weights;
        }
    }
    //----- CREATE NEW WEIGHT END -----//
public:
    double position_fitness;    // 位置
    double v_fitness;           // 线速度
    double rpy_fitness;         // 姿态
    double omega_fitness;       // 角速度
    double position_fitness_o;    // 位置
    double v_fitness_o;           // 线速度
    double rpy_fitness_o;         // 姿态
    double omega_fitness_o;       // 角速度
    double a;
    double a_decay;
    double fitness;     
    Vec12<double> weights;      // 权重 roll，pitch, yaw, x, y, z, omega_roll, omega_pitch, yaw_pitch, vx, vy, vz
    Vec12<double> best_whale;
    deque<Vec12<double>> whales; 
    Vec12<double> cur_state;
    Vec12<double> des_state;
};

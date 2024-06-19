#include <cassert>
#include <iostream>
#include <utility>
#include <deque>
#include <math.h>
#include <cmath>
// 一个稳定的 O(1) 移动滤波器，用于处理输入数据流。实现了 Neumaier 算法来计算移动窗口平均值，这是数值稳定的。
class MovingWindowFilter {
 public:

  MovingWindowFilter() {}

  // 构造函数，指定窗口大小
  MovingWindowFilter(int window_size) : window_size_(window_size) {
    assert(window_size_ > 0);
    sum_ = 0.0;
    correction_ = 0.0;
  }

  // 计算移动窗口平均值
  double CalculateAverage(double new_value) {
    if (value_deque_.size() < window_size_) {
      // 如果队列大小小于窗口大小，不进行计算
    } else {
      // 需要先从移动和中减去最左边的值
      UpdateNeumaierSum(-value_deque_.front());
      value_deque_.pop_front();
    }
    // 添加新值
    UpdateNeumaierSum(new_value);
    value_deque_.push_back(new_value);

    // 返回平均值
    return (sum_ + correction_) / double(window_size_);
  }

  // 返回值队列
  std::deque<double> GetValueQueue() {
    return value_deque_;
  }
 private:
  long long int window_size_; // 窗口大小
  double sum_, correction_; // 移动和和修正值
  std::deque<double> value_deque_; // 值队列

  // 使用 Neumaier 算法更新移动窗口和
  void UpdateNeumaierSum(double value) {
    double new_sum = sum_ + value;
    if (std::abs(sum_) >= std::abs(value)) {
      // 如果前一个和更大，低位数字会丢失
      correction_ += (sum_ - new_sum) + value;
    } else {
      correction_ += (value - new_sum) + sum_;
    }
    sum_ = new_sum;
  }
};

class KrFilter {
 public:

  KrFilter() {}

  // 构造函数，指定过程噪声和测量噪声大小
  KrFilter(double P_Q_,double M_R_) : P_Q(P_Q_),M_R(M_R_){
    p_last = 0;
    x_last = 0;
  }

  // 计算输出值
  double CalculateAns(double new_value) {
    double R = M_R;
    double Q = P_Q;
 
    double x_mid = x_last;
    double x_now;
 
    double p_mid;
    double p_now;
 
    double kg;
 
    //这里p_last 等于 kalmanFilter_A 的p直接取0
    x_mid=x_last;                       //x_last=x(k-1|k-1),x_mid=x(k|k-1)
    p_mid=p_last+Q;                     //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声
 
    /*
      *  卡尔曼滤波的五个重要公式
      */
    kg=p_mid/(p_mid+R);                 //kg为kalman filter，R 为噪声
    x_now=x_mid+kg*(new_value-x_mid);   //估计出的最优值
    p_now=(1-kg)*p_mid;                 //最优值对应的covariance
    p_last = p_now;                     //更新covariance 值
    x_last = x_now;                     //更新系统状态值
 
    return x_now;
  }
  void setqr(double P_Q_,double M_R_)
  {
    P_Q=P_Q_;
    M_R=M_R_;
  }

 private:
  double p_last;

  double x_last;

  double P_Q; //过程噪音Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏

  double M_R; //测量噪音R:测量噪声，R增大，动态响应变慢，收敛稳定性变好

};
class RCFilter
{
public: 
  RCFilter(){}
  RCFilter(float fc_):fc(fc_)
  {
    a = seta(0.002,fc);
    old_value = 0;
  }
  float seta(float dt,float fc_)
  {
    fc = fc_;
    return dt/ (dt + 1 / (2 * M_PI * fc));
  }
  float CalculateAns(float new_value)
  {
    a = seta(0.002,fc);
    float ans = old_value * (1 - a) + new_value * a;
    old_value = new_value;
    return ans;
  }
private:
  float old_value;  
  float fc; // 截至频率
  float a;  // 比例系数
};

class LowPassfilter
{
public:
  LowPassfilter(){}
  LowPassfilter(double sample_rate, double cutoff_frequency)
  {
    double dt = 1.0 / sample_rate;
    double RC = 1.0 / (cutoff_frequency * 2.0 * M_PI);
    alpha_ = dt / (dt + RC);
    prev_output_ = 0.0;
  }
  // 更新滤波器输出
  double update(double input) 
  {
    double output = alpha_ * input + (1.0 - alpha_) * prev_output_;
    prev_output_ = output;
    return output;
  }
public:
  double alpha_;
  double prev_output_;
};

class IirFilter
{
public:
  IirFilter(){}
  IirFilter(double fc_,double fs_,double Q)
  {
    wc = 2.0 *M_PI * fc_ / fs_;
    alpha = sin(wc)/(2 * Q);
    // 计算滤波器系数
    a0 = 1.0 + alpha;
    a1 = -2.0 * cos(wc);
    a2 = 1.0 -alpha;
    b1 = 1.0 - cos(wc);
    b2 = (1.0 - cos(wc)) / 2.0;
    // 状态变量初始化
    x1 = 0;
    x2 = 0;
    y1 = 0;
    y2 = 0;
  }
  double update(double input)
  {
    double output = a0 * input + a1 * x1 + a2 * x2 - b1 * y1 - b2 * y2;
    x2 = x1;
    x1 = input;
    y2 = y1;
    y1 = output;
    return output;
  }
public:
  double a0,a1,a2,b1,b2;//
  double x1,x2,y1,y2;
  double wc;
  double alpha;
};
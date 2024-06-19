/*! @file OrientationEstimator.h
 *  @brief All Orientation Estimation Algorithms
 *
 *  This file will contain all orientation algorithms.
 *  Orientation estimators should compute:
 *  - orientation: a quaternion representing orientation
 *  - rBody: coordinate transformation matrix (satisfies vBody = Rbody * vWorld)
 *  - omegaBody: angular velocity in body frame
 *  - omegaWorld: angular velocity in world frame
 *  - rpy: roll pitch yaw
 */
#ifndef PROJECT_ORIENTATIONESTIMATOR_H
#define PROJECT_ORIENTATIONESTIMATOR_H

#include "Controllers/StateEstimatorContainer.h"
#include "Controllers/wfilter.hpp"
/*!
 * "Cheater" estimator for orientation which always returns the correct value in simulation
 */
template <typename T>
class CheaterOrientationEstimator : public GenericEstimator<T> {
 public:
  virtual void run();
  virtual void setup() {}
};

/*!
 * Estimator for the VectorNav IMU.  The VectorNav provides an orientation already and
 * we just return that.
 */
template <typename T>
class VectorNavOrientationEstimator : public GenericEstimator<T> {
 public:
  virtual void run();
  virtual void setup() {}
  VectorNavOrientationEstimator()
  {
    ori_w = LowPassfilter(500,50);
    ori_x = LowPassfilter(500,50);
    ori_y = LowPassfilter(500,50);
    ori_z = LowPassfilter(500,50);

    omg_x = LowPassfilter(500,50);
    omg_y = LowPassfilter(500,50);
    omg_z = LowPassfilter(500,50);

    acc_x = LowPassfilter(500,50);
    acc_y = LowPassfilter(500,50);
    acc_z = LowPassfilter(500,50);
  }
 protected:
  bool _b_first_visit = true;
  Quat<T> _ori_ini_inv;
  LowPassfilter ori_w;
  LowPassfilter ori_x;
  LowPassfilter ori_y;
  LowPassfilter ori_z;

  LowPassfilter omg_x;
  LowPassfilter omg_y;
  LowPassfilter omg_z;

  LowPassfilter acc_x;
  LowPassfilter acc_y;
  LowPassfilter acc_z;
};


#endif  // PROJECT_ORIENTATIONESTIMATOR_H

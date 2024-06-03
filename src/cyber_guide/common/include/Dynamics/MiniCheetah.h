// #ifndef PROJECT_MINICHEETAH_H
// #define PROJECT_MINICHEETAH_H

// #include "FloatingBaseModel.h"
// #include "Quadruped.h"

// /*!
//  * Generate a Quadruped model of Mini Cheetah
//  */
// template <typename T>
// Quadruped<T> buildMiniCheetah() {
//   Quadruped<T> cheetah;
//   cheetah._robotType = RobotType::MINI_CHEETAH;

//   cheetah._bodyMass = 4.03;
//   cheetah._bodyLength = 0.164 * 2;
//   cheetah._bodyWidth = 0.042 * 2;
//   cheetah._bodyHeight = 0.109;
//   cheetah._abadGearRatio = 1;
//   cheetah._hipGearRatio = 1;
//   cheetah._kneeGearRatio = 1;
//   cheetah._abadLinkLength = 0.094;
//   cheetah._hipLinkLength = 0.12;
//   cheetah._kneeLinkY_offset = 0;
//   cheetah._kneeLinkLength = 0.173+0.019;
//   cheetah._maxLegLength = 0.29;


//   cheetah._motorTauMax = 12.f;//3.f;
//   cheetah._batteryV = 36;
//   cheetah._motorKT = .05;  // this is flux linkage * pole pairs
//   cheetah._motorR = 0.173;
//   cheetah._jointDamping = .01;
//   cheetah._jointDryFriction = .2;


//   // rotor inertia if the rotor is oriented so it spins around the z-axis
//   Mat3<T> rotorRotationalInertiaZ;
//   //  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
//   rotorRotationalInertiaZ << 25, 0, 0, 0, 47, 0, 0, 0, 25;
//   rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

//   Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
//   Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
//   Mat3<T> rotorRotationalInertiaX =
//           RY * rotorRotationalInertiaZ * RY.transpose();
//   Mat3<T> rotorRotationalInertiaY =
//           RX * rotorRotationalInertiaZ * RX.transpose();

//   // spatial inertias
//   Mat3<T> abadRotationalInertia;
//   abadRotationalInertia << 190, -27, 0,
//           -27, 276, 0,
//           0, 0, 233;
//   abadRotationalInertia = abadRotationalInertia * 1e-6;
//   Vec3<T> abadCOM(-0.00392, -0.008752, -0.000102);  // LEFT  ?!??????????????????
//   SpatialInertia<T> abadInertia(0.354, abadCOM, abadRotationalInertia);

//   Mat3<T> hipRotationalInertia;
//   hipRotationalInertia << 1010, 22, -38,
//           22, 983, 199,
//           -38, 199, 347;
//   hipRotationalInertia = hipRotationalInertia * 1e-6;
//   Vec3<T> hipCOM(-0.00212, -0.0212, -0.0184);
//   SpatialInertia<T> hipInertia(0.482, hipCOM, hipRotationalInertia);

//   Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
//   kneeRotationalInertiaRotated << 668, 0, 23,
//           0, 674, 0,
//           23, 0, 15;
//   kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
//   kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
//   Vec3<T> kneeCOM(0.0006, -4.72e-05, -0.0893);
//   SpatialInertia<T> kneeInertia(0.116, kneeCOM, kneeRotationalInertia);

//   Vec3<T> rotorCOM(0, 0, 0);
//   SpatialInertia<T> rotorInertiaX(0.0567, rotorCOM, rotorRotationalInertiaX);
//   SpatialInertia<T> rotorInertiaY(0.0567, rotorCOM, rotorRotationalInertiaY);

//   Mat3<T> bodyRotationalInertia;
//   bodyRotationalInertia << 16812.282,     229,      -294.8,
//           229,          63009,    41,
//           -294.82,      41,       71654.5;
//   bodyRotationalInertia = bodyRotationalInertia * 1e-6;
//   Vec3<T> bodyCOM(0.0223, 0.002, -0.0005);
//   SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
//                                 bodyRotationalInertia);

//   cheetah._abadInertia = abadInertia;
//   cheetah._hipInertia = hipInertia;
//   cheetah._kneeInertia = kneeInertia;
//   cheetah._abadRotorInertia = rotorInertiaX;
//   cheetah._hipRotorInertia = rotorInertiaY;
//   cheetah._kneeRotorInertia = rotorInertiaY;
//   cheetah._bodyInertia = bodyInertia;

//   // locations
//   //  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
//   cheetah._abadRotorLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
//   cheetah._abadLocation =
//           Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
//   cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
//   cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
//   cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
//   cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

//   return cheetah;
// }

// #endif  // PROJECT_MINICHEETAH_H

#ifndef PROJECT_MINICHEETAH_H
#define PROJECT_MINICHEETAH_H

#include "FloatingBaseModel.h"
#include "Quadruped.h"

/*!
 * Generate a Quadruped model of Mini Cheetah
 */
template <typename T>
Quadruped<T> buildMiniCheetah() {
  Quadruped<T> cheetah;
  cheetah._robotType = RobotType::MINI_CHEETAH;

  cheetah._bodyMass = 4.03;
  cheetah._bodyLength = 0.164 * 2;
  cheetah._bodyWidth = 0.042 * 2;
  cheetah._bodyHeight = 0.109;
  cheetah._abadGearRatio = 7.75;
  cheetah._hipGearRatio = 7.75;
  cheetah._kneeGearRatio = 7.75;
  cheetah._abadLinkLength = 0.094;
  cheetah._hipLinkLength = 0.12;
  cheetah._kneeLinkY_offset = 0;
  cheetah._kneeLinkLength = 0.17390999999999998+0.019;
  cheetah._maxLegLength = 0.29;


  cheetah._motorTauMax = 12.f;//3.f;
  cheetah._batteryV = 36;
  cheetah._motorKT = .05;  // this is flux linkage * pole pairs
  cheetah._motorR = 0.173;
  cheetah._jointDamping = .01;
  cheetah._jointDryFriction = .2;


  // rotor inertia if the rotor is oriented so it spins around the z-axis
  Mat3<T> rotorRotationalInertiaZ;
  //  rotorRotationalInertiaZ << 33, 0, 0, 0, 33, 0, 0, 0, 63;
  rotorRotationalInertiaZ << 2.53e-05, 0, 0, 0, 2.53e-05, 0, 0, 0, 4.78e-05;
//   rotorRotationalInertiaZ = 1e-6 * rotorRotationalInertiaZ;

  Mat3<T> RY = coordinateRotation<T>(CoordinateAxis::Y, M_PI / 2);
  Mat3<T> RX = coordinateRotation<T>(CoordinateAxis::X, M_PI / 2);
  Mat3<T> rotorRotationalInertiaX =
          RY * rotorRotationalInertiaZ * RY.transpose();
  Mat3<T> rotorRotationalInertiaY =
          RX * rotorRotationalInertiaZ * RX.transpose();

  // spatial inertias
  Mat3<T> abadRotationalInertia;
  abadRotationalInertia << 0.00019, 2.7e-05, -3.44e-07, 2.7e-05, 0.000276, -1.95e-06, -3.44e-07, -1.95e-06, 0.000233;
  //abadRotationalInertia = abadRotationalInertia * 1e-6;
  Vec3<T> abadCOM(-0.00392, -0.015, -0.000306);  // LEFT  ?!??????????????????
  SpatialInertia<T> abadInertia(0.354, abadCOM, abadRotationalInertia);

  Mat3<T> hipRotationalInertia;
  hipRotationalInertia << 0.00101, -2.23e-05, 3.85e-05, -2.23e-05, 0.000983, 0.000199, 3.85e-05, 0.000199, 0.000347;
  //hipRotationalInertia = hipRotationalInertia * 1e-6;
  Vec3<T> hipCOM(-0.00212, -0.0212, -0.0184);
  SpatialInertia<T> hipInertia(0.482, hipCOM, hipRotationalInertia);

  Mat3<T> kneeRotationalInertia, kneeRotationalInertiaRotated;
  kneeRotationalInertiaRotated << 0.000668, 3e-09, -2.37e-05, 3e-09, 0.000674, -6.03e-07, -2.37e-05, -6.03e-07, 1.54e-05;
  //kneeRotationalInertiaRotated = kneeRotationalInertiaRotated * 1e-6;
  kneeRotationalInertia = RY * kneeRotationalInertiaRotated * RY.transpose();
  Vec3<T> kneeCOM(-0.0006, 4.72e-05, -0.0893);
  SpatialInertia<T> kneeInertia(0.116, kneeCOM, kneeRotationalInertia);

  Vec3<T> rotorCOM(0, 0, 0);
  SpatialInertia<T> rotorInertiaX(0.0567, rotorCOM, rotorRotationalInertiaX);
  SpatialInertia<T> rotorInertiaY(0.0567, rotorCOM, rotorRotationalInertiaY);

  Mat3<T> bodyRotationalInertia;
  bodyRotationalInertia << 0.0185, -0.000173, -0.0102, -0.000173, 0.0517, -2.83e-05, -0.0102, -2.83e-05, 0.0483;
  //bodyRotationalInertia = bodyRotationalInertia * 1e-6;
  Vec3<T> bodyCOM(0.0273, -0.000242, 0.0143);
  SpatialInertia<T> bodyInertia(cheetah._bodyMass, bodyCOM,
                                bodyRotationalInertia);

  cheetah._abadInertia = abadInertia;
  cheetah._hipInertia = hipInertia;
  cheetah._kneeInertia = kneeInertia;
  cheetah._abadRotorInertia = rotorInertiaX;
  cheetah._hipRotorInertia = rotorInertiaY;
  cheetah._kneeRotorInertia = rotorInertiaY;
  cheetah._bodyInertia = bodyInertia;

  // locations
  //  cheetah._abadRotorLocation = Vec3<T>(0.125, 0.049, 0);
  cheetah._abadRotorLocation = Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._abadLocation =
          Vec3<T>(cheetah._bodyLength, cheetah._bodyWidth, 0) * 0.5;
  cheetah._hipLocation = Vec3<T>(0, cheetah._abadLinkLength, 0);
  cheetah._hipRotorLocation = Vec3<T>(0, 0.04, 0);
  cheetah._kneeLocation = Vec3<T>(0, 0, -cheetah._hipLinkLength);
  cheetah._kneeRotorLocation = Vec3<T>(0, 0, 0);

  return cheetah;
}

#endif  // PROJECT_MINICHEETAH_H


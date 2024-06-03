/*! @file LegController.cpp
 *  @brief Common Leg Control Interface
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#include "Controllers/LegController.h"

/*!
 * Zero the leg command so the leg will not output torque
 */
template <typename T>
void LegControllerCommand<T>::zero() {
  tauFeedForward = Vec3<T>::Zero();
  forceFeedForward = Vec3<T>::Zero();
  qDes = Vec3<T>::Zero();
  qdDes = Vec3<T>::Zero();
  pDes = Vec3<T>::Zero();
  vDes = Vec3<T>::Zero();
  kpCartesian = Mat3<T>::Zero();
  kdCartesian = Mat3<T>::Zero();
  kpJoint = Mat3<T>::Zero();
  kdJoint = Mat3<T>::Zero();
}

/*!
 * Zero the leg data
 */
template <typename T>
void LegControllerData<T>::zero() {
  q = Vec3<T>::Zero();
  qd = Vec3<T>::Zero();
  p = Vec3<T>::Zero();
  v = Vec3<T>::Zero();
  J = Mat3<T>::Zero();
  tauEstimate = Vec3<T>::Zero();
}

/*!
 * Zero all leg commands.  This should be run *before* any control code, so if
 * the control code is confused and doesn't change the leg command, the legs
 * won't remember the last command.
 */
template <typename T>
void LegController<T>::zeroCommand() {
  for (auto& cmd : commands) {
    cmd.zero();
  }
  _legsEnabled = false;
}

/*!
 * Set the leg to edamp.  This overwrites all command data and generates an
 * emergency damp command using the given gain. For the mini-cheetah, the edamp
 * gain is Nm/(rad/s), and for the Cheetah 3 it is N/m. You still must call
 * updateCommand for this command to end up in the low-level command data!
 */
template <typename T>
void LegController<T>::edampCommand(RobotType robot, T gain) {
  zeroCommand();
  if (robot == RobotType::CHEETAH_3) {
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdCartesian(axis, axis) = gain;
      }
    }
  } else {  // mini-cheetah
    for (int leg = 0; leg < 4; leg++) {
      for (int axis = 0; axis < 3; axis++) {
        commands[leg].kdJoint(axis, axis) = gain;
      }
    }
  }
}

/*!
 * Update the "leg data" from a SPIne board message
 */
template <typename T>
void LegController<T>::updateData(const SpiData* spiData) {
  (void) spiData;
  Mat3<T> normal_scale;
  normal_scale = Vec3<T>(1, -1, -1).asDiagonal();

    for (int leg = 0; leg < 4; leg++) {
        if(std::isnan(_lowState->motorState[3*leg    ].q))
        {
            printf("sim leg q error !!!!!!!!!!\n");
            datas[leg].q(0) =  datas[leg].q(0);
            datas[leg].q(1) =  datas[leg].q(1);
            datas[leg].q(2) =  datas[leg].q(2);

            // qd
            datas[leg].qd(0) =  datas[leg].qd(0) ;//*1.25;
            datas[leg].qd(1) =  datas[leg].qd(1) ;//*1.25;
            datas[leg].qd(2) =  datas[leg].qd(2) ;//*1.25;
        }else {
            // q:
            datas[leg].q(0) = _lowState->motorState[3*leg    ].q;
            datas[leg].q(1) = _lowState->motorState[3*leg + 1].q;
            datas[leg].q(2) = _lowState->motorState[3*leg + 2].q;
            datas[leg].q = normal_scale * datas[leg].q;
            // qd
            datas[leg].qd(0) = _lowState->motorState[3*leg    ].dq;//*1.25;
            datas[leg].qd(1) = _lowState->motorState[3*leg + 1].dq;//*1.25;
            datas[leg].qd(2) = _lowState->motorState[3*leg + 2].dq;//*1.25;
            datas[leg].qd = normal_scale * datas[leg].qd;
        }

        //添加的实际扭矩
        datas[leg].tauActuatual(0) = _lowState->motorState[3*leg    ].tauEst;
        datas[leg].tauActuatual(1) = _lowState->motorState[3*leg + 1].tauEst;
        datas[leg].tauActuatual(2) = _lowState->motorState[3*leg + 2].tauEst;
        datas[leg].tauActuatual = normal_scale * datas[leg].tauActuatual;
        //    if(leg == 3)
        //        printf("leg tau: %.2f\t%.2f\t%.2f\n",datas[leg].tauActuatual(0),datas[leg].tauActuatual(1),datas[leg].tauActuatual(2));
        //    // J and p
        computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &(datas[leg].J),
                                         &(datas[leg].p), leg);

        // v
        datas[leg].v = datas[leg].J * datas[leg].qd;

        Mat3<T> inv_JT = datas[leg].J.transpose().inverse();
        datas[leg].footforceDesired = inv_JT*commands[leg].tauFeedForward;
        datas[leg].footforceActuatual = inv_JT*datas[leg].tauActuatual;
    }
}

/*!
 * Update the "leg data" from a TI Board message
 */
template <typename T>
void LegController<T>::updateData(const TiBoardData* tiBoardData) {
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      datas[leg].q(joint) = tiBoardData[leg].q[joint];
      datas[leg].qd(joint) = tiBoardData[leg].dq[joint];
      datas[leg].p(joint) = tiBoardData[leg].position[joint];
      datas[leg].v(joint) = tiBoardData[leg].velocity[joint];

      // J and p
      computeLegJacobianAndPosition<T>(_quadruped, datas[leg].q, &datas[leg].J,
                                       nullptr, leg);
      datas[leg].tauEstimate[joint] = tiBoardData[leg].tau[joint];
    }
    //printf("%d leg, position: %f, %f, %f\n", leg, datas[leg].p[0], datas[leg].p[1], datas[leg].p[2]);
    //printf("%d leg, velocity: %f, %f, %f\n", leg, datas[leg].v[0], datas[leg].v[1], datas[leg].v[2]);
  }
}

/*!
 * Update the "leg command" for the SPIne board message
 */
template <typename T>
void LegController<T>::updateCommand(SpiCommand* spiCommand) {
  (void) spiCommand;
    Vec3<T> kp, kd, q_des, qd_des;
    int mode = 0;

    Mat3<T> normal_scale;
    normal_scale = Vec3<T>(1, -1, -1).asDiagonal();

    for (int leg = 0; leg < 4; leg++) {
        // tauFF
        Vec3<T> legTorque = commands[leg].tauFeedForward;

        // forceFF
        Vec3<T> footForce = commands[leg].forceFeedForward;

        // cartesian PD
        footForce +=
                commands[leg].kpCartesian * (commands[leg].pDes - datas[leg].p);
        footForce +=
                commands[leg].kdCartesian * (commands[leg].vDes - datas[leg].v
                );

        // Torque
        legTorque += datas[leg].J.transpose() * footForce;
        legTorque = normal_scale * legTorque;
        _lowCmd->setTau(leg, legTorque.template cast<double>());
        // joint space pd
        // joint space PD
        kd[0] = commands[leg].kdJoint(0, 0);
        kd[1] = commands[leg].kdJoint(1, 1);
        kd[2] = commands[leg].kdJoint(2, 2);
        _lowCmd->setKd(leg, kd.template cast<double>());

        kp[0] = commands[leg].kpJoint(0, 0);
        kp[1] = commands[leg].kpJoint(1, 1);
        kp[2] = commands[leg].kpJoint(2, 2);
        _lowCmd->setKp(leg, kp.template cast<double>());

        q_des[0] = commands[leg].qDes(0);
        q_des[1] = commands[leg].qDes(1);
        q_des[2] = commands[leg].qDes(2);
        q_des = normal_scale * q_des;
        _lowCmd->setQ(leg, q_des.template cast<double>());

        qd_des[0] = commands[leg].qdDes(0);
        qd_des[1] = commands[leg].qdDes(1);
        qd_des[2] = commands[leg].qdDes(2);
        qd_des = normal_scale * qd_des;
        _lowCmd->setQd(leg, qd_des.template cast<double>());

        // estimate torque
        datas[leg].tauEstimate =
                legTorque +
                commands[leg].kpJoint * (commands[leg].qDes - datas[leg].q) +
                commands[leg].kdJoint * (commands[leg].qdDes - datas[leg].qd);
    }
    mode = _legsEnabled ? 10 : 0;
    //mode = _legsEnabled ? 0 : 10;

    for(int i = 0; i < 12; i++)
    {
        _lowCmd->motorCmd[i].mode = mode;
    }
}

constexpr float CHEETAH_3_ZERO_OFFSET[4][3] = {{1.f, 4.f, 7.f},
                                               {2.f, 5.f, 8.f},
                                               {3.f, 6.f, 9.f}};
/*!
 * Update the "leg command" for the TI Board
 */
template <typename T>
void LegController<T>::updateCommand(TiBoardCommand* tiBoardCommand) {
  for (int leg = 0; leg < 4; leg++) {
    Vec3<T> tauFF = commands[leg].tauFeedForward.template cast<T>();


    for (int joint = 0; joint < 3; joint++) {
      tiBoardCommand[leg].kp[joint] = commands[leg].kpCartesian(joint, joint);
      tiBoardCommand[leg].kd[joint] = commands[leg].kdCartesian(joint, joint);
      tiBoardCommand[leg].tau_ff[joint] = tauFF[joint];
      tiBoardCommand[leg].position_des[joint] = commands[leg].pDes[joint];
      tiBoardCommand[leg].velocity_des[joint] = commands[leg].vDes[joint];
      tiBoardCommand[leg].force_ff[joint] =
          commands[leg].forceFeedForward[joint];
      tiBoardCommand[leg].q_des[joint] = commands[leg].qDes[joint];
      tiBoardCommand[leg].qd_des[joint] = commands[leg].qdDes[joint];
      tiBoardCommand[leg].kp_joint[joint] = commands[leg].kpJoint(joint, joint);
      tiBoardCommand[leg].kd_joint[joint] = commands[leg].kdJoint(joint, joint);
      tiBoardCommand[leg].zero_offset[joint] = CHEETAH_3_ZERO_OFFSET[leg][joint];
    }

    // please only send 1 or 0 here or the robot will explode.
    tiBoardCommand[leg].enable = _legsEnabled ? 1 : 0;
    tiBoardCommand[leg].max_torque = _maxTorque;
    tiBoardCommand[leg].zero = _zeroEncoders ? 1 : 0;
    if(_calibrateEncoders) {
      tiBoardCommand[leg].enable = _calibrateEncoders + 1;
    }

    if(_zeroEncoders) {
      tiBoardCommand[leg].enable = 0;
    }

  }
}



template struct LegControllerCommand<double>;
template struct LegControllerCommand<float>;

template struct LegControllerData<double>;
template struct LegControllerData<float>;

template class LegController<double>;
template class LegController<float>;

/*!
 * Compute the position of the foot and its Jacobian.  This is done in the local
 * leg coordinate system. If J/p are NULL, the calculation will be skipped.
 */
template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg) {
  T l1 = quad._abadLinkLength;
  T l2 = quad._hipLinkLength;
  T l3 = quad._kneeLinkLength;
  T l4 = quad._kneeLinkY_offset;
  T sideSign = quad.getSideSign(leg);

  T s1 = std::sin(q(0));
  T s2 = std::sin(q(1));
  T s3 = std::sin(q(2));

  T c1 = std::cos(q(0));
  T c2 = std::cos(q(1));
  T c3 = std::cos(q(2));

  T c23 = c2 * c3 - s2 * s3;
  T s23 = s2 * c3 + c2 * s3;

  if (J) {
    J->operator()(0, 0) = 0;
    J->operator()(0, 1) = l3 * c23 + l2 * c2;
    J->operator()(0, 2) = l3 * c23;
    J->operator()(1, 0) = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1;
    J->operator()(1, 1) = -l3 * s1 * s23 - l2 * s1 * s2;
    J->operator()(1, 2) = -l3 * s1 * s23;
    J->operator()(2, 0) = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1;
    J->operator()(2, 1) = l3 * c1 * s23 + l2 * c1 * s2;
    J->operator()(2, 2) = l3 * c1 * s23;
  }

  if (p) {
    p->operator()(0) = l3 * s23 + l2 * s2;
    p->operator()(1) = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1;
    p->operator()(2) = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2;
  }
  //printf("current q %d : %.6f  %.6f  %.6f\n",  leg,q(0),q(1),q(2));
  //printf("current p %d : %.6f  %.6f  %.6f\n",  leg,p->operator()(0),p->operator()(1),p->operator()(2));
}

template void computeLegJacobianAndPosition<double>(Quadruped<double>& quad,
                                                    Vec3<double>& q,
                                                    Mat3<double>* J,
                                                    Vec3<double>* p, int leg);
template void computeLegJacobianAndPosition<float>(Quadruped<float>& quad,
                                                   Vec3<float>& q,
                                                   Mat3<float>* J,
                                                   Vec3<float>* p, int leg);

/*! @file LegController.h
 *  @brief Common Leg Control Interface and Leg Control Algorithms
 *
 *  Implements low-level leg control for Mini Cheetah and Cheetah 3 Robots
 *  Abstracts away the difference between the SPIne and the TI Boards (the low level leg control boards)
 *  All quantities are in the "leg frame" which has the same orientation as the
 * body frame, but is shifted so that 0,0,0 is at the ab/ad pivot (the "hip
 * frame").
 */

#ifndef PROJECT_LEGCONTROLLER_H
#define PROJECT_LEGCONTROLLER_H

#include "cppTypes.h" // a

#include "Dynamics/Quadruped.h"// a
#include "SimUtilities/SpineBoard.h"// a
#include "SimUtilities/ti_boardcontrol.h"// a

#include "message/LowlevelState.h"
#include "message/LowlevelCmd.h"

/*!
 * Data sent from the control algorithm to the legs.
 */
template <typename T>
struct LegControllerCommand {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LegControllerCommand() { zero(); }

  void zero();

  Vec3<T> tauFeedForward, forceFeedForward, qDes, qdDes, pDes, vDes;
  Mat3<T> kpCartesian, kdCartesian, kpJoint, kdJoint;
};

/*!
 * Data returned from the legs to the control code.
 */
template <typename T>
struct LegControllerData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LegControllerData() { zero(); }

    void setQuadruped(Quadruped<T>& quad) { quadruped = &quad; }

    void zero();

    Vec3<T> q, qd, p, v;
    Mat3<T> J;
    Vec3<T> tauEstimate;
    Vec3<T> tauActuatual;
    Vec3<T> footforceDesired;
    Vec3<T> footforceActuatual;
    Quadruped<T>* quadruped;
};

/*!
 * Controller for 4 legs of a quadruped.  Works for both Mini Cheetah and Cheetah 3
 */
template <typename T>
class LegController {
 public:
    LegController ( Quadruped<T>& quad, LowlevelCmd* lowCmd, LowlevelState* lowState) : _quadruped ( quad ), _lowCmd(lowCmd), _lowState(lowState)
    {
        for ( auto& data : datas ) {
            data.setQuadruped ( _quadruped );
         //Init();
        }
    }
    ~LegController(){
        delete &_quadruped;
        delete _lowCmd;
        delete _lowState;
    }

  void zeroCommand();
  void edampCommand(RobotType robot, T gain);
  void updateData(const SpiData* spiData);
  void updateData(const TiBoardData* tiBoardData);
  void updateCommand(SpiCommand* spiCommand);
  void updateCommand(TiBoardCommand* tiBoardCommand);
  void setEnabled(bool enabled) { _legsEnabled = enabled; };

  /*!
   * Set the maximum torque.  This only works on cheetah 3!
   */
  void setMaxTorqueCheetah3(T tau) { _maxTorque = tau; }

  LegControllerCommand<T> commands[4];
  LegControllerData<T> datas[4];
  Quadruped<T>& _quadruped;
  bool _legsEnabled = false;
  T _maxTorque = 0;
  bool _zeroEncoders = false;
  u32 _calibrateEncoders = 0;

  LowlevelCmd *_lowCmd;
  LowlevelState *_lowState;
};

template <typename T>
void computeLegJacobianAndPosition(Quadruped<T>& quad, Vec3<T>& q, Mat3<T>* J,
                                   Vec3<T>* p, int leg);

#endif  // PROJECT_LEGCONTROLLER_H

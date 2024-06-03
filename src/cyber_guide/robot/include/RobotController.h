/*!
 * @file RobotController.h
 * @brief Parent class of user robot controllers.
 * This is an interface between the control code and the common hardware code
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Controllers/LegController.h"            // a
#include "Dynamics/FloatingBaseModel.h"           // a
#include "Controllers/StateEstimatorContainer.h"  // a
#include "Controllers/DesiredStateCommand.h"      // a
#include "SimUtilities/VisualizationData.h"       // a

#include "interface/IOInterface.h"                // a
#include "message/LowlevelState.h"                // a
#include "message/LowlevelCmd.h"                  // a
#include "message/UserParameters.h"               // a

/*!
 * Parent class of user robot controllers
 */
class RobotController{
  friend class RobotRunner;
public:
  RobotController(IOInterface *ioInter):ioInter(ioInter){}
  virtual ~RobotController(){}

  virtual void initializeController() = 0;
/**
 * Called one time every control loop 
 */
  virtual void runController() = 0;
  virtual void updateVisualization() = 0;
  virtual ControlParameters* getUserControlParameters() = 0;
  virtual void Estop() {}

protected:
  Quadruped<float>* _quadruped = nullptr;
  FloatingBaseModel<float>* _model = nullptr;
  LegController<float>* _legController = nullptr;
  StateEstimatorContainer<float>* _stateEstimator = nullptr;
  StateEstimate<float>* _stateEstimate = nullptr;
  RobotControlParameters* _controlParameters = nullptr;
  DesiredStateCommand<float>* _desiredStateCommand = nullptr;

  VisualizationData* _visualizationData = nullptr;
  RobotType _robotType;

  LowlevelCmd *lowCmd;
  LowlevelState *lowState;
  IOInterface *ioInter;
  User_Parameters *userParas;
};

#endif

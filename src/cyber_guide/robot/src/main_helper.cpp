/*!
 * @file main.cpp
 * @brief Main Function for the robot program
 *
 * The main function parses command line arguments and starts the appropriate
 * driver.
 */

#include <cassert>
#include <iostream>


#include "SimulationBridge.h"
#include "main_helper.h"
#include "RobotController.h"

MasterConfig gMasterConfig;

/*!
 * Print a message describing the command line flags for the robot program
 */
void printUsage() {
  printf(
      "Usage: robot [robot-id] [sim-or-robot] [parameters-from-file]\n"
      "\twhere robot-id:     3 for cheetah 3, m for mini-cheetah\n"
      "\t      sim-or-robot: s for sim, r for robot\n"
      "\t      param-file:   f for loading parameters from file, l (or nothing) for LCM\n"
      "                      this option can only be used in robot mode\n");
}

/*!
 * Setup and run the given robot controller
 */
SimulationBridge *simulationBridge;//(gMasterConfig._robot, ctrl);
int main_helper(RobotController* ctrl) {

  static int flag_ini=0;
  if(flag_ini==0)
  {
    //        gMasterConfig._robot=RobotType::CYBER_160;
    gMasterConfig._robot=RobotType::MINI_CHEETAH;
    simulationBridge = new SimulationBridge(gMasterConfig._robot, ctrl);
    flag_ini= 1;
  }
  simulationBridge->run();
  return 0;
}

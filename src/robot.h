/*
 * robot.h - A header file for the robot object class.
 */
#include "Aria.h"

class Robot {
 public:
  int ret;
  ArRobot robot;
  ArSick sick;
  ArSerialConnection laserCon;
  ArSerialConnection con;
  std::string str;
  initRobot();
};
// EOF

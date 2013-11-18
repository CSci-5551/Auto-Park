/*
MobileRobots Advanced Robotics Interface for Applications (ARIA)
Copyright (C) 2004, 2005 ActivMedia Robotics LLC
Copyright (C) 2006, 2007, 2008, 2009 MobileRobots Inc.

     This program is free software; you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation; either version 2 of the License, or
     (at your option) any later version.

     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.

     You should have received a copy of the GNU General Public License
     along with this program; if not, write to the Free Software
     Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

If you wish to redistribute ARIA under different terms, contact 
MobileRobots for information about a commercial version of ARIA at 
robots@mobilerobots.com or 
MobileRobots Inc, 10 Columbia Drive, Amherst, NH 03031; 800-639-9481
*/
#include "Aria.h"

/*
This program will let you joystick the robot around, and take logs for
the mapper while you drive, automatically... 

Control: 

Attach an analog joystick to the joyport on the robot, not on the
computer.  Now calibrate the joystick: Leaving the stick centered,
press the trigger button for a second or so, then release it. Now
rotate the stick around its extremes two or three times, holding it in
each corner for a second or two. You are now ready to drive.  You can
then drive the robot by holding the trigger and moving the joystick.
To make goals you can press the top button on the joystick (this
requires AROS1_5).

You could also attach USB joystick attached to robot computer (this
depends on having a robot equiped with an accessible usb port): To
drive the robot just press the trigger button and then move the
joystick how you wish the robot to move.  You can use the throttle on
the side of the joystick to control the maximum (and hence scaled)
speed at which the robot drives.  To make a goal you can press any of
the other buttons on the joystick itself (button 2, 3, or 4).

You could run this with the keyboard, but not very versatile.  Use the
arrow keys to control the robot, and press g to make a goal.
*/


int main(int argc, char **argv)
{

  // mandatory init
  Aria::init();

  // set up our parser
  ArArgumentParser parser(&argc, argv);
  // set up our simple connector
  ArSimpleConnector simpleConnector(&parser);

  // robot
  ArRobot robot;
  // the laser
  ArSick sick;
  // the serial connection (robot)
  ArSerialConnection serConn;
  // tcp connection (sim)
  ArTcpConnection tcpConn;
  // Laser connection
  ArSerialConnection laserCon;
  // whether or not we're using the sim (auto determined);
  bool useSim = false;
  // a key handler so we can do our key handling
  ArKeyHandler keyHandler;

  // let the global aria stuff know about it
  Aria::setKeyHandler(&keyHandler);
  // toss it on the robot
  robot.attachKeyHandler(&keyHandler);

#ifdef WIN32
  printf("Pausing 5 seconds so you can disconnect VNC if you are using it.\n");
  ArUtil::sleep(5000);
#endif

  // attach the laser to the robot
  robot.addRangeDevice(&sick);

  // make an analog gyro object (it won't do anything if we don't have
  // one), if we have soemthing it'll take care of it itself
  ArAnalogGyro analogGyro(&robot);

  // try to load up the old school inertials
  if (!useSim && 
      ArModuleLoader::load("libArInertial", &robot, NULL, true) == 0)
  {
    printf("Loaded the base inertial library\n");
    if (ArModuleLoader::load("ISense_Mod", &robot, (void *)"2", true) == 0)
    {
      printf("The ISense inertial module has been loaded and should be correcting heading now.\n");
    }	
    else if (
    ArModuleLoader::load("ISIS_Mod", &robot,(void *)ArUtil::COM2, false) == 0)
    {
      printf("The ISIS inertial module has been loaded and should be correcting heading now.\n");
    }
  }

  
  // load the default arguments 
  parser.loadDefaultArguments();
  // add our right increments and degrees as a deafult
  parser.addDefaultArgument("-laserDegrees 180 -laserIncrement half");
  // parse the command line... fail and print the help if the parsing fails
  // or if the help was requested
  if (!simpleConnector.parseArgs() || !parser.checkHelpAndWarnUnparsed(1))
  {    
    simpleConnector.logOptions();
    keyHandler.restore();
    exit(1);
  }

  std::string filename = "1scans.2d";
  if (argc > 1)
    filename = argv[1];
  printf("Logging to file %s\n", filename.c_str());
  
  
  
  // set up the robot for connecting
  if (!simpleConnector.connectRobot(&robot))
  {
    printf("Could not connect to robot... exiting\n");
    Aria::shutdown();
    keyHandler.restore();
    return 1;
  }

  // make the group to drive the robot (mostly for the USB)

  ArActionGroupRatioDriveUnsafe group(&robot);
  group.activateExclusive();

  // run the robot, true here so that the run will exit if connection lost
  robot.runAsync(true);

  simpleConnector.setupLaser(&sick);
  // This must be created after the robot is connected so that it'll
  // get the right laser pos
  ArSickLogger logger(&robot, &sick, 300, 25, filename.c_str(),
		      false);

  // now that we're connected to the robot, connect to the laser
  sick.runAsync();

  // connect the laser if it was requested
  if (!sick.blockingConnect())
  {
    printf("Could not connect to laser... exiting\n");
    Aria::shutdown();
    keyHandler.restore();
    return 1;
  }
  

  // enable the motors, disable amigobot sounds
  robot.comInt(ArCommands::SONAR, 0);
  robot.comInt(ArCommands::ENABLE, 1);
  robot.comInt(ArCommands::SOUND, 32);
  robot.comInt(ArCommands::SOUNDTOG, 0);
  // enable the joystick driving from the one connected to the microcontroller
  robot.comInt(ArCommands::JOYDRIVE, 1);
  // just hang out and wait for the end
  robot.waitForRunExit();
  sick.lockDevice();
  sick.disconnect();
  sick.unlockDevice();
  // now exit
  Aria::shutdown();
  return 0;
}

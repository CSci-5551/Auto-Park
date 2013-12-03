/*
 * autoPark.cpp
 *  -  The main program to run the parking function.
 */
#include "Aria.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace std;

// Global variables for robot and laser
ArRobot robot;
ArSick sick;

/*
 * initialize
 * - A function to initialize the robot.
 */
int initialize(int *argc, char **argv) {
    
    // Manditory init call
    Aria::init();

    // Add the laser device
    robot.addRangeDevice(&sick);
    
    // Setup connector
    ArSimpleConnector connector(argc, argv);
    connector.parseArgs();

    // Try to connect, exit on failure
	if (!connector.connectRobot(&robot))
	{
		printf("Could not connect to robot... exiting\n");
		Aria::shutdown();
		return 1;
	}
    printf("Robot: Connected\n");
    
    // Start the robot running so that if we lose connection the run stops
	robot.runAsync(true);
    
    // Set up the laser
    sick.configureShort(false,ArSick::BAUD38400,ArSick::DEGREES180,ArSick::INCREMENT_ONE);
    connector.setupLaser(&sick);
    sick.runAsync();
    
    // Do a blocking connect, if it fails exit
    if (!sick.blockingConnect())
    {
        printf("Could not connect to SICK laser... exiting\n");
        Aria::shutdown();
        return 1;
    }
    printf("SICK: Connected\n");
  
    // Return 0 for successful initialization
    return 0;
}

/*
 * scanForSpace
 * - A function to search for an open space using the SICK laser.
 */
void scanForSpace() {
  int t, cnt;
  double laser_dist[900];
  double laser_angle[900];
  std::list<ArSensorReading *> *readings;
  std::list<ArSensorReading *>::iterator it;

  cnt = 0;
    while(cnt<10000)
	{
		readings=(list<ArSensorReading *,allocator<ArSensorReading *> > *)sick.getRawReadings();//CurrentBuffer..
		while (readings==NULL) 	readings=(list<ArSensorReading *,allocator<ArSensorReading *> > *)sick.getRawReadings();
        
        t=0;
        for(it=readings->begin();it!=readings->end(); it++)
        {
            laser_dist[t]=(*it)->getRange();
            laser_angle[t]=-90+t;
            //cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] <<" "<<"\n";
            t++;
        }
        cnt++;
        
	} //end while 1<2
    for (t=0; t<181; t++){
        cout << "laser angle: " << laser_angle[t] << " laser dist.: " << laser_dist[t] <<" "<<"\n";
    }
}

/*
 * main
 * - Main function for the parking program.
 */
int main(int argc, char **argv) {
    
    // Initialize the Robot
  if (initialize(&argc, argv) == 0) {
        printf("Robot: Initialized\n");
        printf("SICK: Initialized\n");
    }
    else {
        printf("Initialization failed\n");
    }
  
    // Scan for parking space
    scanForSpace();
  
    // Log scan data
  
    // When parking space is found, execute park function

    // Log parameters

    // Log status
    
    // Shutdown the robot
    robot.waitForRunExit();
    Aria::shutdown();
    return 0;
}

// EOF

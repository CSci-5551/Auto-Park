/*
 * autoPark.cpp
 * - The main program to run the parking function.
 */
#include "Aria.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <iomanip>

using namespace std;

// Constants
#define TURNING_RADIUS 1000
#define ROBOT_RADIUS 227.5
#define DEPTH_BOUND 1000
#define MAR_ERR 50
#define VMAX 500
#define LASER_ANGLE 90
#define OMEGA_MAX 2.618

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
    if (!connector.connectRobot(&robot)) {
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
    if (!sick.blockingConnect()) {
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
    double laser_dist;
    double laser_angle;
    double dist_bound;
    double init_dist;
    bool depth_ok;
    bool length_ok;
    std::list<ArSensorReading *> *readings;
    std::list<ArSensorReading *>::iterator it;
    
    // Set laser angle
    readings=(list<ArSensorReading *,allocator<ArSensorReading *> > *)sick.getRawReadings(); // Current buffer
    laser_angle = LASER_ANGLE;
    depth_ok = false;
    length_ok = false;

    // TODO: Begin moving forward
    
    // Start taking in readings
    it = readings->begin();

    // Set initial distance
    init_dist = (*it)->getRange();
    
    while (it != readings->end()) {
        // Get distance
	laser_dist=(*it)->getRange();
        
	// Check distance against boundaries
        if (laser_dist > (DEPTH_BOUND+MAR_ERR+init_dist)) {
            depth_ok = true;
            // TODO: Set a marker at current distance, check if depth
            // is still ok at current_distance + 1500 (length of spot).
        }
	it++;
    }
    printf("\nFound acceptable parking space\n");
    return;
}


/*
 * parkRobot()
 * - Function to park the robot.
 */
void parkRobot() {
    // TODO: Calculate center of circle one
    
    // TODO: Calculate triangle angle
    
    // TODO: Calculate x distance to move for alignment
    
    // TODO: Back up proper distance
    
    // TODO: First circle turn
    
    // TODO: Second circle turn
    
    return;
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
  
    // When parking space is found, execute park function
    parkRobot();
    
    // Shutdown the robot
    robot.waitForRunExit();
    Aria::shutdown();
    return 0;
}

// EOF

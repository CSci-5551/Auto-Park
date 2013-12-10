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
#define TRUE 1
#define FALSE 0

struct reading {
    double angle;
    double distance;
};

// Global variables for robot and laser
ArRobot robot;
ArSick sick;
reading reading_array[500];
reading first_corner;
reading second_corner;
reading third_corner;
reading fourth_corner;
reading fifth_corner;
FILE *logfp;

/*
 * initialize
 * - A function to initialize the robot.
 */
int initialize(int *argc, char **argv) {
    int ret;
    std::string str;
    ArSerialConnection laserCon;
    ArSerialConnection serCon;
    ArArgumentParser parser(argc, argv);
    ArSimpleConnector connector(&parser);
    
    // Manditory init call
    Aria::init();
    
    // Load the default arguments
    parser.loadDefaultArguments();
    
    // Add our right increments and degrees as a deafult
    parser.addDefaultArgument("-laserDegrees 180 -laserIncrement one");
    
    // Parse the command line
    if (!connector.parseArgs() || !parser.checkHelpAndWarnUnparsed(1))
    {
        connector.logOptions();
        exit(1);
    }
    
    // Add the laser device
    robot.addRangeDevice(&sick);
    
    // Try to connect to the robot, if we fail exit
    if (!connector.connectRobot(&robot))
    {
        printf("Robot: Could not connect...exiting\n");
        Aria::shutdown();
        return 1;
    }
    printf("Robot: Connected\n");
    
    // Set robot to stop the run if the connection is broken
    robot.runAsync(true);
    
    // Setup laser
    connector.setupLaser(&sick);
    
    // Set laser to stop the run if connection is broken
    sick.runAsync();
    
    // Do a blocking connect, exit on failure
    if (!sick.blockingConnect())
    {
        printf("Laser: Could not connect...exiting\n");
        Aria::shutdown();
        return 1;
    }
    printf("Laser: Connected\n");

    return 0;
}


/*
 * takeReadings
 * - A function to search for an open space using the SICK laser.
 */
void takeReadings() {
    int i;
    double dist, angle;
    std::list<ArPoseWithTime *> *readings;
    std::list<ArPoseWithTime *>::iterator it;

    // Initialize vars
    i = 0;
    printf("Scanning...");
    ArUtil::sleep(500);
    
    // Lock the laser
    sick.lockDevice();
        
    // Current closest reading within a degree range
    dist = sick.currentReadingPolar(-90, 90, &angle);
    if (dist < sick.getMaxRange())
        fprintf(logfp, "Closest reading %.2f mm away at %.2f degrees\n\n", dist, angle);
    else
        fprintf(logfp, "No close reading.\n\n");
    
    // Take readings and store angle and distance results in reading array
    readings = sick.getCurrentBuffer();
    for (it = readings->begin(); it != readings->end(); it++) {
        reading_array[i].distance = (*it)->findDistanceTo(ArPose(0, 0));
        reading_array[i].angle = (*it)->findAngleTo(ArPose(0, 0));
        fprintf(logfp, "Reading %d:\tLaser Dist: %f\tAngle: %f\n",
                i, reading_array[i].distance, reading_array[i].angle);
        i++;
    }
    
    // Unlock laser and return
    sick.unlockDevice();
    ArUtil::sleep(100);
    puts("");
    fprintf(logfp, "\n");
    printf("done\n");
    return;
}


/*
 * findCorners
 * - A function to find the corners of a parking space
 */
void findCorners() {
    // Find the first corner
    int i = 0;
    double average;
    reading current;
    reading next;
    
    next = reading_array[0];
    while (next.distance != NULL) {
        current = reading_array[i];
        next = reading_array[i+1];
        if (next.distance < current.distance && first_corner.distance == NULL) {
            first_corner.distance = current.distance;
            first_corner.angle = current.angle;
            fprintf(logfp, "First Corner: Distance: %f\tAngle: %f\n",
                    first_corner.distance, first_corner.angle);
        }
        i++;
    }
    // TODO: Find the second corner
    
    // TODO: Find the third corner
    
    // TODO: Find the fourth courner
    
    // TODO: Find the fifth corner
    
    return;
    
}


/*
 * parkRobot
 * - Function to park the robot.
 */
void parkRobot() {
    // TODO: Execute movements using calculated corners.
    
    return;
}


/*
 * openLogFile
 * - Function to open a logfile and write header.
 */
void openLogFile() {
    logfp = fopen("logfile.txt", "w");
    fprintf(logfp, "######################################################\n");
    fprintf(logfp, "## AUTO-PARK LOGFILE                                ##\n");
    fprintf(logfp, "## - This file contains data for a run of Auto-Park ##\n");
    fprintf(logfp, "######################################################\n\n");
    return;
}


/*
 * main
 * - Main function for the parking program.
 */
int main(int argc, char **argv) {
    
    // Open the logfile
    openLogFile();

    // Initialize the Robot
    fprintf(logfp, "## INITIALIZATION ##\n");
    if (initialize(&argc, argv) == 0) {
        fprintf(logfp, "Robot: Initialized\n");
        fprintf(logfp, "SICK: Initialized\n\n");
    }
    else {
        fprintf(logfp, "Initialization failed\n\n");
    }
  
    // Take readings
    fprintf(logfp, "## SCAN FOR SPACE ##\n");
    takeReadings();
    
    // Calcuate corner angles and distances
    fprintf(logfp, "## CORNERS ##\n");
    findCorners();
  
    // When parking space is found, execute park function
    parkRobot();
    
    // Shutdown the robot
    //robot.waitForRunExit();
    Aria::shutdown();
    fclose(logfp);
    return 0;
}

// EOF

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
#define DEPTH_BOUND 500 //Adjust depending on expected depth
#define MAR_ERR 50
#define VMAX 500
#define LASER_ANGLE 90
#define OMEGA_MAX 2.618
#define PI 3.14159265
#define TRUE 1
#define FALSE 0

struct reading {
    double angle;
    double distance;
};

// Global variables for robot and laser
ArRobot robot;
ArSick sick;
reading reading_array[400];
reading first_corner;
reading second_corner;
reading third_corner;
double found_depth, found_width;
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
    parser.addDefaultArgument("-laserDegrees 180 -laserIncrement half");
    
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
    std::list<ArPoseWithTime *> *readings;
    std::list<ArPoseWithTime *>::iterator it;

    // Initialize vars
    i = 0;
    printf("Scanning...");
    ArUtil::sleep(500);
    
    // Lock the laser
    sick.lockDevice();
   
    // Take readings from 90-180 degrees and store angle and distance results in reading array
    readings = sick.getCurrentBuffer();
    for (it = readings->begin(); it != readings->end(); it++) {
	if((*it)->findAngleTo(ArPose(0, 0)) > 89.9) {
		reading_array[i].distance = (*it)->findDistanceTo(ArPose(0, 0));
		reading_array[i].angle = (*it)->findAngleTo(ArPose(0, 0));
		fprintf(logfp, "Reading %d:\tLaser Dist: %f\tAngle: %f\n",
		        i, reading_array[i].distance, reading_array[i].angle);
	}
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
    int i = 0;
    reading current;
    reading next;
    reading nextnext;
    bool behind_car = 0; //TODO add logic to find corners assuming starting behind first car

    nextnext = reading_array[0];
    while (nextnext.distance != NULL) {
        current = reading_array[i];
        next = reading_array[i+1];
	nextnext = reading_array[i+2]; //check against 2 readings instead of 1

	//This function occasionally fails and doesn't give the corners. 
	//When it fails the data looks fine... so I'm not sure what's going on.

	//1st corner assuming starting right next to car #1 && we ccan see the botton corner of car2
	if (((current.distance + DEPTH_BOUND) < next.distance) &&
		((current.distance + DEPTH_BOUND) < nextnext.distance) && first_corner.distance == NULL) {
            first_corner.distance = current.distance;
            first_corner.angle = current.angle;
            fprintf(logfp, "First Corner: Distance: %f\tAngle: %f\n",
                    first_corner.distance, first_corner.angle);
        }
	//2nd corner assuming starting right next to car #1
	if (current.distance > next.distance && current.distance > nextnext.distance
		&& first_corner.distance != NULL && second_corner.distance == NULL) {
            second_corner.distance = current.distance;
            second_corner.angle = current.angle;
            fprintf(logfp, "Second Corner: Distance: %f\tAngle: %f\n",
                    second_corner.distance, second_corner.angle);
        }
	//3rd corner assuming starting right next to car #1
	if (current.distance < next.distance && current.distance < nextnext.distance
		&& first_corner.distance != NULL && second_corner.distance != NULL) {
            third_corner.distance = current.distance;
            third_corner.angle = current.angle;
            fprintf(logfp, "Third Corner: Distance: %f\tAngle: %f\n",
                    third_corner.distance, third_corner.angle);
	    break; //Got all the corners no need to check the other values
        }
        i++;
    }

	


	/*
        // Find the first corner if behind first car
        if (current.distance > next.distance && first_corner.distance == NULL) {
            first_corner.distance = current.distance;
            first_corner.angle = current.angle;
            fprintf(logfp, "First Corner: Distance: %f\tAngle: %f\n",
                    first_corner.distance, first_corner.angle);
        }
	*/
	return;

}

/*
 * getDimensions
 * - Function to get Depth and Width using law of Cosines
 */
void getDimensions() {
	
	found_depth = sqrt(pow(second_corner.distance,2.0) + pow(third_corner.distance,2.0) 
			- 2.0 * second_corner.distance * third_corner.distance 
			* cos((third_corner.angle - second_corner.angle) * PI / 180));

	fprintf(logfp, "Depth: %f\n", found_depth);

	found_width = sqrt(pow(first_corner.distance,2.0) + pow(third_corner.distance,2.0) 
			- 2.0 * first_corner.distance * third_corner.distance 
			* cos((third_corner.angle - first_corner.angle) * PI / 180));
	fprintf(logfp, "Width: %f\n", found_width);
    
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

    // Use corners to get dimension of parking spot
    getDimensions();
  
    // When parking space is found, execute park function
    parkRobot();
    
    // Shutdown the robot
    //robot.waitForRunExit();
    Aria::shutdown();
    fclose(logfp);
    return 0;
}

// EOF

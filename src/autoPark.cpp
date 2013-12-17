
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
#define MAX_MOVES 5 // Maximum times to move MOVE_DISTANCE and check for new spot
#define MAX_SCANS 3 // Maximum times to scan for corners at each "initial" location
#define MOVE_DISTANCE 300.0 // Distance to move before attempting to find corners again
#define DEPTH_BOUND 100.0 // Adjust depending on expected depth
#define WIDTH_BOUND 605.0 // Adjust depending on expected width
#define TURNING_RADIUS 525.0
#define ROBOT_RADIUS 227.5
#define ROBOT_BACK 425.0
#define WHEEL_BASE 320.0
#define MAR_ERR 50.0
#define VMAX 300.0
#define LASER_ANGLE 90.0
#define OMEGA_MAX 2.618
#define PI 3.14159265
#define TRUE 1
#define FALSE 0

// A struct for laser reading data
struct reading {
    double angle;
    double distance;
};

// Global variables for robot and laser
ArRobot robot;
ArSick sick;
reading reading_array[400];
reading first_corner, second_corner, third_corner, ahead_corner;
double found_depth, found_width;
int scan_times;
FILE *logfp;

/*
 * initialize
 * - A function to initialize the robot.
 */
int initialize(int *argc, char **argv) {
    std::string str;
    ArSerialConnection laserCon;
    ArSerialConnection serCon;
    ArArgumentParser parser(argc, argv);
    ArSimpleConnector connector(&parser);
    
    // Set max trans velocity
    robot.setAbsoluteMaxTransVel(VMAX);
    
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
    
	robot.enableMotors();
    
    return 0;
}


/*
 * takeReadings
 * - A function to search for an open space using the SICK laser.
 */
void takeReadings() {
    std::list<ArPoseWithTime *> *readings;
    std::list<ArPoseWithTime *>::iterator it;
    double tempAngle;
    double tempDist;
    int numReadings;
    int i;
    int j;

    // Initialize data
    numReadings = 0;
    first_corner.distance = 0;
    second_corner.distance = 0;
    third_corner.distance = 0;
    ahead_corner.distance = 0;
    for (i = 0; i < 400; i++)
        reading_array[i].distance = 0;
    
    // Print some info
    printf("Scanning...");
    fprintf(logfp, "## LASER READINGS %d ##\n", scan_times);
    
    ArUtil::sleep(500);
    sick.lockDevice();
    
    // Take readings from 90-180 degrees and store results in reading array
    readings = sick.getCurrentBuffer();
    for (it = readings->begin(); it != readings->end(); it++) {
        if ((*it)->findAngleTo(ArPose(0, 0)) > 89.9) {
            reading_array[i].distance = (*it)->findDistanceTo(ArPose(0, 0));
            reading_array[i].angle = (*it)->findAngleTo(ArPose(0, 0));
            numReadings++;
        }
        i++;
    }
    
    // Reverse array if first value is 180 instead of 90
    if(reading_array[0].angle > 100.0) {
        for (j = 0; j <= numReadings; j++) {
            tempAngle = reading_array[numReadings-j].angle;
            tempDist = reading_array[numReadings-j].distance;
            reading_array[numReadings-j].angle = reading_array[j].angle;
            reading_array[numReadings-j].distance = reading_array[j].distance;
            reading_array[j].angle = tempAngle;
            reading_array[j].distance = tempDist;
        }
    }
    
    // Print readings to log file
    for (int k = 0; k < numReadings; k++) {
        fprintf(logfp, "Reading %d:\tLaser Dist: %f\tAngle: %f\n",
                k, reading_array[k].distance, reading_array[k].angle);
    }
    
    // Unlock laser and return
    sick.unlockDevice();
    ArUtil::sleep(100);
    puts("");
    fprintf(logfp, "\n");
    printf("done\n");
    scan_times++;
    return;
}


/*
 * findCorners
 * - A function to find the corners of a parking space
 */
void findCorners() {
    int i;
    reading current;
    reading next;
    reading nextnext;
    
    i = 0;
    nextnext = reading_array[0];
    
    // Loop through the reading array
    while (nextnext.distance != 0) {
        current = reading_array[i];
        next = reading_array[i+1];
        nextnext = reading_array[i+2];
        
        // Find first corner
        if (((current.distance + DEPTH_BOUND) < next.distance) &&
            ((current.distance + DEPTH_BOUND) < nextnext.distance) &&
            first_corner.distance == 0) {
            
            first_corner.distance = current.distance;
            first_corner.angle = current.angle;
        }
        
        // Find second corner
        if (current.distance > next.distance && current.distance > nextnext.distance
            && first_corner.distance != 0 && second_corner.distance == 0) {
            second_corner.distance = current.distance;
            second_corner.angle = current.angle;

        }
        
        // Find third corner
        if (current.distance < next.distance && current.distance < nextnext.distance
            && first_corner.distance != 0 && second_corner.distance != 0) {
            third_corner.distance = current.distance;
            third_corner.angle = current.angle;
            break; //Got all the corners no need to check the other values
        }
        i++;
    }

    return;
}

/*
 * findAheadCorner
 * - A function to find the corners of the car ahead of the robot.
 */
void findAheadCorner() {
    int i = 0;
    reading current;
    reading next;
    reading nextnext;
    
    nextnext = reading_array[0];
    
    // Scan for rightmost corner
    while (nextnext.distance != 0) {
        current = reading_array[i];
        next = reading_array[i+1];
        nextnext = reading_array[i+2];
        
        if (current.distance > next.distance && current.distance > nextnext.distance
            && ahead_corner.distance == 0) {
            ahead_corner.distance = current.distance;
            ahead_corner.angle = current.angle;
            fprintf(logfp, "\n## FORWARD MOVE CORRECTION ##\n");
            fprintf(logfp, "Ahead Corner: Distance: %f\tAngle: %f\n",
                    ahead_corner.distance, ahead_corner.angle);
        }
    }
    return;
}

/*
 * getDimensions
 * - Function to get Depth and Width using Cosines
 */
void getDimensions() {
    found_depth = sqrt(pow(second_corner.distance,2.0) + pow(third_corner.distance,2.0)
                       - 2.0 * second_corner.distance * third_corner.distance
                       * cos((third_corner.angle - second_corner.angle) * PI / 180));
    found_width = sqrt(pow(first_corner.distance,2.0) + pow(third_corner.distance,2.0)
                       - 2.0 * first_corner.distance * third_corner.distance
                       * cos((third_corner.angle - first_corner.angle) * PI / 180));
    
    // Print results to logfile
    fprintf(logfp, "Depth: %f\n", found_depth);
    fprintf(logfp, "Width: %f\n\n", found_width);
    
    return;
}


/*
 * parkRobot
 * - Function to park the robot.
 */
void parkRobot() {
    double first_car_x, ahead_car_x;
    double circle1_x, circle1_y;
    double circle2_x, circle2_y;
    double xtangent, wall_y, wheel_ratio;
    double turn_angle, turn_time;
    double right_vel;
    double distance_forward;
    
    // Calculate values
    first_car_x = -cos(first_corner.angle * PI /180.0) * first_corner.distance;
    wall_y = -sin(second_corner.angle * PI /180.0) * second_corner.distance;
    circle1_x = first_car_x + ROBOT_BACK;
    circle1_y = wall_y + ROBOT_RADIUS + TURNING_RADIUS + 30.0;  //50 = wiggle room, mm
    circle2_y = -TURNING_RADIUS;
    xtangent = circle1_x+sqrt(pow(TURNING_RADIUS,2.0)-pow(((circle2_y-circle1_y)/2),2.0));
    circle2_x = (2.0 * xtangent) - circle1_x;
    wheel_ratio = ((2.0*TURNING_RADIUS/WHEEL_BASE)+1.0)/((2.0*TURNING_RADIUS/WHEEL_BASE)-1.0);
    right_vel = -(VMAX * 2.0) / (1 + wheel_ratio);
    turn_angle = atan2(circle1_y - circle2_y, circle2_x - circle1_x);
    turn_time = 1000 * abs(((PI/2)-A)/((rightVel - wheel_ratio * right_vel)/WHEEL_BASE));
    
    // Print calculations to logfile
    fprintf(logfp, "## CALCULATIONS ##\n");
    fprintf(logfp, "first_car_x: %f\n", first_car_x);
    fprintf(logfp, "wall_y: %f\n", wall_y);
    fprintf(logfp, "circle1_x: %f\n", circle1_x);
    fprintf(logfp, "circle1_y: %f\n", circle1_y);
    fprintf(logfp, "circle2_y: %f\n", circle2_y);
    fprintf(logfp, "xtangent: %f\n", xtangent);
    fprintf(logfp, "circle2_x: %f\n", circle2_x);
    fprintf(logfp, "wheel_ratio: %f\n", wheel_ratio);
    fprintf(logfp, "right_vel: %f\n", right_vel);
    fprintf(logfp, "turn_angle: %f\n", turn_angle);
    fprintf(logfp, "turn_time: %f\n", turn_time);
    
    fprintf(logfp, "## MOVE INFO ##\n");
    
    // Move to starting location for parking
    printf("Moving forward");
    fprintf(logfp, "Forward move distance: %f mm\n", circle2_x);
    robot.lock();
    robot.move(circle2_x-150);
    robot.unlock();
    
    // Wait until move is completed
    ArUtil::sleep(5000);
    while(robot.isMoveDone() == false) {}
    
    // Make first turn
    printf("Peforming first turn");
    fprintf(logfp, "First turn velocity: (%f, %f)\n", right_vel, wheel_ratio * right_vel);
    robot.lock();
    robot.setVel2((wheel_ratio * right_vel), right_vel);
    robot.unlock();
    ArUtil::sleep(turn_time);
    
    // Make second turn
    printf("Performing second turn");
    fprintf(logfp, "Second turn velocity: (%f, %f)\n", wheel_ratio * right_vel, right_vel);
    robot.lock();
    robot.setVel2(right_vel, (wheel_ratio * right_vel));
    robot.unlock();
    ArUtil::sleep(turn_time);
    
    // Stop the robot
    robot.lock();
    robot.stop();
    robot.unlock();
    ArUtil::sleep(2000);
    
    // Take new readings
    takeReadings();
    
    // Find the corner of the car ahead
    findAheadCorner();
    
    // Calculate distance to move forward
    ahead_car_x = sin(ahead_corner.angle * PI /180.0) * ahead_corner.distance;
    
    // Move forward if still far away from ahead car
    if (ahead_car_x > (ROBOT_RADIUS + 200)) {
        printf("Moving forward");
        fprintf(logfp, "Forward move correction: %f\n", ROBOT_RADIUS);
        robot.lock();
        robot.setVel2(100,100);
        robot.move(ROBOT_RADIUS);
        robot.unlock();
        ArUtil::sleep(2000);
        while(robot.isMoveDone() == false) {}
    }
    
    return;
}


/*
 * openLogFile
 * - Function to open a logfile and write header.
 */
void openLogFile() {
    logfp = fopen("logfile.txt", "w");
    fprintf(logfp, "######################################################\n");
    fprintf(logfp, "## AUTO-PARK LOGFILE ##\n");
    fprintf(logfp, "## - This file contains data for a run of Auto-Park ##\n");
    fprintf(logfp, "######################################################\n\n");
    return;
}


/*
 * main
 * - Main function for the parking program.
 */
int main(int argc, char **argv) {
    int max_tries;
    int max_move;
    bool found_spot;
    
    max_tries = MAX_SCANS;
    max_move = MAX_MOVES;
    found_spot = false;
    scan_times = 1;
    
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
    
    // Find the corners
    while (!found_spot && max_move > 0) {
        
        // Try to find corners from current position
        while((third_corner.distance == 0 || first_corner.angle > 150.0 )&& max_tries > 0) {
            takeReadings();
            findCorners();
            if(third_corner.distance != 0 && first_corner.angle < 150.0)
                found_spot = true;
            max_tries--;
        }
        
        // Move the robot forward and try again
        robot.lock();
        robot.move(MOVE_DISTANCE);
        robot.unlock();
        
        ArUtil::sleep(2000);
        while(robot.isMoveDone() == false) {}
        
        // Reset position to 0,0
        robot.lock();
        robot.moveTo(ArPose(0,0,0), true);
        robot.unlock();
        
        ArUtil::sleep(200);
        max_move--;
    }
    
    // Print corners to logfile
    fprintf(logfp, "## CORNERS ##\n");
    fprintf(logfp, "First Corner: Distance: %f\tAngle: %f\n",
            first_corner.distance, first_corner.angle);
    fprintf(logfp, "Second Corner: Distance: %f\tAngle: %f\n",
            second_corner.distance, second_corner.angle);
    fprintf(logfp, "Third Corner: Distance: %f\tAngle: %f\n\n",
            third_corner.distance, third_corner.angle);
    
    // Use corners to get dimension of parking spot
    fprintf(logfp, "## SPACE DIMENSIONS ##\n");
    getDimensions();
    
    // If parking space fits our requirements, execute park function
    if (found_width > WIDTH_BOUND && found_spot)
        parkRobot();
    else
        printf("Adequate spot not found.\n");
    
    // Shutdown the robot
    Aria::shutdown();
    fclose(logfp);
    return 0;
}

// EOF

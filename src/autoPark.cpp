
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
#define MAX_MOVES 5        //Maximum times to move MOVE_DISTANCE and check for new spot
#define MAX_SCANS 3 //Maximum times to scan for corners at each "initial" location
#define MOVE_DISTANCE 300.0 //Distance to move before attempting to find corners again
#define TURNING_RADIUS 525.0
#define ROBOT_RADIUS 227.5
#define ROBOT_BACK 425.0
#define DEPTH_BOUND 120.0 //Adjust depending on expected depth
#define WHEEL_BASE 320.0
#define MAR_ERR 50.0
#define VMAX 300.0
#define LASER_ANGLE 90.0
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
    std::string str;
    ArSerialConnection laserCon;
    ArSerialConnection serCon;
    ArArgumentParser parser(argc, argv);
    ArSimpleConnector connector(&parser);

    robot.setAbsoluteMaxTransVel(VMAX);

    //initialize distances to 0 to avoid NULL checks
        
    
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
        int i;
        std::list<ArPoseWithTime *> *readings;
        std::list<ArPoseWithTime *>::iterator it;

        // Initialize vars
        i = 0;
        printf("Scanning...");
        ArUtil::sleep(500);

        //Initialize readings to 0
        first_corner.distance = 0;
        second_corner.distance = 0;
        third_corner.distance = 0;
        for(int i = 0; i<400; i++) {
                reading_array[i].distance = 0;
        }

        // Lock the laser
        sick.lockDevice();

        // Take readings from 90-180 degrees and store angle and distance results in reading array
        readings = sick.getCurrentBuffer();
        int numReadings = 0;
        for (it = readings->begin(); it != readings->end(); it++) {
                if((*it)->findAngleTo(ArPose(0, 0)) > 89.9) {
                        reading_array[i].distance = (*it)->findDistanceTo(ArPose(0, 0));
                        reading_array[i].angle = (*it)->findAngleTo(ArPose(0, 0));
                        numReadings++;
                }
                i++;
        }

        //reverse array if first value is 180 instead of 90
        if(reading_array[0].angle > 100.0) {
                for(int j = 0; j <= numReadings; j++) {
                        double tempAngle = reading_array[numReadings-j].angle;
                        double tempDist = reading_array[numReadings-j].distance;
                        reading_array[numReadings-j].angle = reading_array[j].angle;
                        reading_array[numReadings-j].distance = reading_array[j].distance;
                        reading_array[j].angle = tempAngle;
                        reading_array[j].distance = tempDist;
                }
        }

                
        //print readings to log file
        for(int k = 0; k < numReadings; k++) {
        fprintf(logfp, "Reading %d:\tLaser Dist: %f\tAngle: %f\n",
                 k, reading_array[k].distance, reading_array[k].angle);
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
    //bool behind_car = 0; //TODO add logic to find corners assuming starting behind first car

    nextnext = reading_array[0];
    
    while (nextnext.distance != 0) {
        current = reading_array[i];
        next = reading_array[i+1];
        nextnext = reading_array[i+2]; //check against 2 readings instead of 1

        //This function occasionally fails and doesn't give the corners.
        //When it fails the data looks fine... so I'm not sure what's going on.

        //1st corner assuming starting right next to car #1 && we ccan see the botton corner of car2
        if (((current.distance + DEPTH_BOUND) < next.distance) &&
                ((current.distance + DEPTH_BOUND) < nextnext.distance) && first_corner.distance == 0) {
            first_corner.distance = current.distance;
            first_corner.angle = current.angle;
            fprintf(logfp, "First Corner: Distance: %f\tAngle: %f\n",
                    first_corner.distance, first_corner.angle);
        }
        //2nd corner assuming starting right next to car #1
        if (current.distance > next.distance && current.distance > nextnext.distance
                && first_corner.distance != 0 && second_corner.distance == 0) {
            second_corner.distance = current.distance;
            second_corner.angle = current.angle;
            fprintf(logfp, "Second Corner: Distance: %f\tAngle: %f\n",
                    second_corner.distance, second_corner.angle);
        }
        //3rd corner assuming starting right next to car #1
        if (current.distance < next.distance && current.distance < nextnext.distance
                && first_corner.distance != 0 && second_corner.distance != 0) {
            third_corner.distance = current.distance;
            third_corner.angle = current.angle;
            fprintf(logfp, "Third Corner: Distance: %f\tAngle: %f\n\n",
                    third_corner.distance, third_corner.angle);
         break; //Got all the corners no need to check the other values
        }
        i++;
    }
        /*
			Find the first corner if behind first car
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
* - Function to get Depth and Width using Cosines
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
    // TODO: Combine variables once we know they are individually correct

    double first_car_x = -cos(first_corner.angle * PI /180.0) * first_corner.distance;
    fprintf(logfp, "first_car_x: %f\n", first_car_x);

    double wall_y = -sin(second_corner.angle * PI /180.0) * second_corner.distance;
    fprintf(logfp, "wall_y %f\n", wall_y);

    double circle1_x = first_car_x + ROBOT_BACK;
    fprintf(logfp, "circle1_x: %f\n", circle1_x);

    double circle1_y = wall_y + ROBOT_RADIUS + TURNING_RADIUS + 50;  //50 = wiggle room, mm
    fprintf(logfp, "circle1_y %f\n", circle1_y);

    double circle2_y = -TURNING_RADIUS;
    fprintf(logfp, "circle2_y %f\n", circle2_y);

    double xtangent = circle1_x + sqrt(pow(TURNING_RADIUS,2.0) - pow(((circle2_y - circle1_y)/2),2.0));
    fprintf(logfp, "xtangent %f\n", xtangent);

    double circle2_x = (2.0 * xtangent) - circle1_x;
    fprintf(logfp, "circle2_x %f\n", circle2_x);

    double wheel_ratio = ((2.0 * TURNING_RADIUS/WHEEL_BASE) + 1.0)/((2.0 * TURNING_RADIUS/WHEEL_BASE) - 1.0);
    fprintf(logfp, "wheel_ratio %f\n", wheel_ratio);

    double rightVel = -(VMAX * 2.0) / (1 + wheel_ratio);
    fprintf(logfp, "rightVel %f\n", rightVel);

    double A = atan2(circle1_y - circle2_y, circle2_x - circle1_x);
    fprintf(logfp, "turnAngle %f\n", A);

    double t_turn = 1000 * abs(((PI/2)-A)/((rightVel - wheel_ratio * rightVel)/WHEEL_BASE));
    fprintf(logfp, "turn_time %f\n", t_turn);

    //move to starting location for parking
    cout << "Moving forward " << circle2_x << " mm." << endl;
    robot.lock();
    robot.move(circle2_x-150);
    robot.unlock();
    ArUtil::sleep(5000);
    while(robot.isMoveDone() == false) {} //don't do anything until the move is done

    //This does the general parking motion, although far from perfect
    //the ratio seems a bit off because it needs to turn almost 90 degrees to get deep enough into the spot
    //need to calculate sleep amount or find another way to follow circular path

    cout << "1st turning velocity: (" << rightVel << "," << (wheel_ratio * rightVel) << ")" << endl;
    robot.lock();
    robot.setVel2((wheel_ratio * rightVel), rightVel);
    robot.unlock();
    ArUtil::sleep(t_turn);

    cout << "2nd turning velocity: (" << (wheel_ratio * rightVel) << "," << rightVel << ")" << endl;
    robot.lock();
    robot.setVel2(rightVel, (wheel_ratio * rightVel));
    robot.unlock();
    ArUtil::sleep(t_turn);
    
    robot.lock();
    robot.stop();
    robot.unlock();

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

    int max_tries; //Didn't find corners? try a few more times.
        int max_move = MAX_MOVES;
        bool found_spot = false;
        while(!found_spot && max_move > 0) {
                max_tries = MAX_SCANS;
                while((third_corner.distance == 0 || first_corner.angle > 150.0 )&& max_tries > 0) {
                        takeReadings();
                        findCorners();
                        if(third_corner.distance != 0 && first_corner.angle < 150.0)
                                found_spot = true;
                        max_tries--;
                }
                robot.lock();
                robot.move(MOVE_DISTANCE);
                robot.unlock();
                ArUtil::sleep(2000);
                while(robot.isMoveDone() == false) {}
                robot.lock();
                robot.moveTo(ArPose(0,0,0), true); //resets pose to 0,0 for new position
                robot.unlock();
                ArUtil::sleep(200);
                max_move--;
        }
                

    // Use corners to get dimension of parking spot
    getDimensions();
        
    // When parking space is found, execute park function
        if((found_width > (ROBOT_RADIUS *2 + 150)) && found_spot)
            parkRobot();
        else
                cout << "Adequate spot not found." << endl;
                cout << found_spot << endl;
    
    // Shutdown the robot
    //robot.waitForRunExit();
    Aria::shutdown();
    fclose(logfp);
    return 0;
}

// EOF
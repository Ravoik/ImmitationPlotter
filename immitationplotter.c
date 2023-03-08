#include "hitechnic-accelerometer.h"
#define ACC_PORT S1
#define GYRO_PORT S3
const int MAX_SIZE = 127;
const int SENS = 10;
const int BRUSH_SIZE = 2;
const int CENTER_X = 89;
const int CENTER_Y = 64;
const int PRECISION = 7;
const int DISTANCE_MULTIPLIER = 1;
const int MAX_POINTS = 100;
const int HIT_THRESHOLD = 50;
const int INVERT_THRESHOLD = 0;
struct Point
{
    int x;
    int y;
};
int recordEtchASketch(Point *pointarray);
void drawSketch(Point *pointarray, int points);
void angleRobot(float angle, int position);
void calibrateGyro();
task main()
{
    calibrateGyro();
    // Draw rectangle beforehand
    drawRect(0, 0, 0, 0);
    // Turn brakes off so that the user can
    // sketch using the wheels (otherwise they have resistance)
    setMotorBrakeMode(motorA, 0);
    setMotorBrakeMode(motorD, 0);
    // Set up accelerometer
   SensorType[ACC_PORT] = sensorI2CCustom;
   struct Point pointarray[MAX_SIZE];
   // Begin recording drawing into the pointarray
   int points = recordEtchASketch(pointarray);
   // Reset gyro (might not be needed) and drawSketch from pointarray
   resetGyro(GYRO_PORT);
   drawSketch(pointarray, points);
}

void calibrateGyro()
{
   SensorType[GYRO_PORT] = sensorEV3_Gyro;
   wait1Msec(50);
   SensorMode[GYRO_PORT] = modeEV3Gyro_Calibration;
   wait1Msec(50);
   SensorMode[GYRO_PORT] = modeEV3Gyro_RateAndAngle;
   wait1Msec(50);
}

void storePoint(Point *pointarray, int pointNum, int x, int y)
{
    // store a point for every tenth point drawn on screen
    Point p;
    p.x = x;
    p.y = y;
    pointarray[pointNum].x = p.x;
    pointarray[pointNum].y = p.y;
}

int recordEtchASketch(Point *pointarray)
{
    // Accel Init
    // Create struct to hold sensor data
    tHTAC accelerometer;
    // Initialise and configure struct and port
    initSensor(&accelerometer, S1);
    if (!readSensor(&accelerometer))
    {
        displayTextLine(4, "ERROR!!");
        sleep(2000);
        return 0;
    }
    // Reset wheels' motor encoders
    nMotorEncoder[motorA] = 0;
    nMotorEncoder[motorD] = 0;
    // Prev X & Y are initialized to the origin
    int prevX = 0;
    int prevY = 0;
    // Number of points stored so far
    int pointNum = 0;
    // While loop - stops recording if Enter is pressed
    while(!getButtonPress(buttonEnter))
    {
        readSensor(&accelerometer);
        if (accelerometer.z > INVERT_THRESHOLD)
        {
            eraseDisplay();
            for(int i = 0; i < MAX_SIZE; i++)
            {
                pointarray[i].x = 0;
                pointarray[i].y = 0;
            }
            return recordEtchASketch(pointarray);
        }
 
        // Record the x and y based on encoder counts
        // Dividing by sensitivty allows easier control of the drawing
        int x = nMotorEncoder[motorA] / SENS;
        int y = nMotorEncoder[motorD] / SENS;
        // Draw rectangles
        drawRect(x+CENTER_X, y+CENTER_Y,
                x+BRUSH_SIZE+CENTER_X, y+BRUSH_SIZE+CENTER_Y);
        // If the difference between the current
        // drawn point and the previous one are >= 10
        // then store the point in the array, and iterate pointNum
        if (abs(x - prevX) >= PRECISION || abs(y - prevY) >= PRECISION)
        {
        storePoint(pointarray, pointNum, x, y);
        pointNum++;
        prevX = x;
        prevY = y;
        }
        // Return if we hit max number of points
        if (pointNum == MAX_POINTS)
        {
        // Turn the brakes back on so the robot can draw properly
        setMotorBrakeMode(motorA, 1);
        setMotorBrakeMode(motorD, 1);
        return pointNum;
        }
    }
        // Turn the brakes back on so the robot can draw properly
        setMotorBrakeMode(motorA, 1);
        setMotorBrakeMode(motorD, 1);
 
        // Returns the number of points needed to be
        // drawn by the robot
        return pointNum;
}

float distanceToPoint(Point p1, Point p2)
{
     // a^2
     int a = pow(p2.y - p1.y, 2);
    // b^2
    int b = pow(p2.x - p1.x, 2);
    // a^2 + b^2 = c^2
    return sqrt(a + b);
}

float getArcTan(Point p1, Point p2)
{
     // This function is for the first couple points since vectors cannot
    // properly be drawn between them
    // Calculate tan of angle within triangle determined by two points
    int rise = p2.y - p1.y;
    int run = p2.x - p1.x;
    return atan2(rise, run)*(180/PI);
}
float angleToPoint(Point p1, Point p2, Point p3, int &position)
{
    // Use lin alg to find the angle determined by two vectors
    // This is neccessary becuase we need
    // to have the context of what direction
    // we are facing before we change angles
    // Vector 1 is the vector between p1 and p2
    Point vector1;
    vector1.x = p2.x - p1.x;
    vector1.y = p2.y - p1.y;
    // Vector 2 is the vector between p2 and p3
    Point vector2;
    vector2.x = p3.x - p2.x;
    vector2.y = p3.y - p2.y;
    // Find the dot product
    float num = vector1.x * vector2.x + vector1.y * vector2.y;
    // Find the norm of each and multilpy them
    float den = sqrt(vector1.x*vector1.x + vector1.y*vector1.y)
    * sqrt(vector2.x*vector2.x + vector2.y*vector2.y);
    // Figure out whether to turn left or right based
    // on the determinant of the vectors
    position = ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x));
    // If position is on the right, we get a negative position
    // If position is on the left, we get a positive position
    if (position > 0)
    {
        position = 1;
    }
    else
    {
        position = -1;
    }
    return acos(num/den)*(180/PI)*position;
}

void pLoop(float angle, bool negative)
{
     int neg = 1;
    if (negative)
        neg = -1;
    else
        neg = 1;
    motor[motorA] = neg*abs((angle-getGyroDegrees(GYRO_PORT))*0.5)+(1*neg);
    motor[motorD] = -neg*abs((angle-getGyroDegrees(GYRO_PORT))*0.5)-(1*neg);
 
    if (getGyroDegrees(GYRO_PORT) > angle)
    {
        motor[motorA] = -neg*abs((angle-getGyroDegrees(GYRO_PORT))*0.5)+(1*neg);
        motor[motorD] = neg*abs((angle-getGyroDegrees(GYRO_PORT))*0.5)-(1*neg);
    }
}

void angleRobot(float angle, int position)
{
    // Zero the gyro
    resetGyro(GYRO_PORT);
    if (angle == 0)
        angle = angle;
    else if (position == -1)
        angle += 3;
    else if (position == 1)
        angle -= 3;
        // Wait to hit angle
    while(abs(getGyroDegrees(GYRO_PORT)) < abs(angle))
    {
        // Turn right or left based on + or - angle
        // This also corrects if the robot overshoots
        // the angle becaue of high speeds
        if(angle > 0)
        {
            pLoop(angle, 0);
        }
        else if(angle < 0)
        {
            pLoop(angle, 1);
        }
    }
}

void driveSpeed(int speed)
{
    // Dumb function
    motor[motorA] = motor[motorD] = speed;
}

bool detectHit(tHTAC &accelerometer)
{
    if (!readSensor(&accelerometer))
    {
    displayTextLine(4, "ERROR!!");
    sleep(2000);
    return true;
    }
 
    else if (accelerometer.x > HIT_THRESHOLD)
    {
    eraseDisplay();
    displayTextLine(4, "ERROR!!");
    sleep(2000);
    return true;
    }
 return false;
}

bool driveRobotDistance(Point p1, Point p2)
{
    // Find the distance between two points
    float distance = distanceToPoint(p1, p2);
    // ENC_MULTIPLIER to convert distance to cm
    const int ENC_MULTIPLIER = 180 / (2 * PI * 2.75);
    // Reset motor encoders
    nMotorEncoder[motorA] = 0;
    // Init accelerometer
    tHTAC accelerometer;
    // Drive for set distance
    driveSpeed(25);
    while(nMotorEncoder[motorA] <
    (distance * ENC_MULTIPLIER * DISTANCE_MULTIPLIER))
    {
        // If the robot hits something, stop all tasks and
        // end the program
        if (detectHit(accelerometer))
        {
            return false;
        };
    }
 
    // Stop driving
    driveSpeed(0);
    eraseDisplay();
    return true;
}

void drawSketch(Point *pointarray, int points)
{
     // Use the third motor to put the marker down
    motor[motorB] = 2;
    wait1Msec(1000);
    motor[motorB] = 0;
    calibrateGyro();
    // Use zero point as a reference for the first point
    Point zero;
    zero.x = 0;
    zero.y = 0;
    // Store first point
    Point initPoint;
    initPoint.x = pointarray[0].x;
    initPoint.y = pointarray[0].y;
    // Store second point
    Point secPoint;
    secPoint.x = pointarray[1].x;
    secPoint.y = pointarray[1].y;
    // Find the angle between the x-axis and the first point
    float angle1 = getArcTan(zero, initPoint);
    // Angle the robot to the first point, then drive the distance to it
    angleRobot(angle1, 0);
    driveRobotDistance(zero, initPoint);
    // Find the angle between the first point and the second point
    float angle2 = getArcTan(initPoint, secPoint);
    // Angle the robot to the second point, then drive the distance to it
    angleRobot(angle2, 0);
    driveRobotDistance(initPoint, secPoint);
    // Initialize three points to be read from the point array
    Point p1;
    Point p2;
    Point p3;
    // Once the above work has been done,
    // we can draw vectors between each point and its previous point
    // as well as the point and the next point
    // Afterward we can use lin alg to find the angle between the vectors
    for (int point = 2; point < points; point++)
    {
        // Read each point from the array
        p1.x = pointarray[point-2].x;
        p1.y = pointarray[point-2].y;
        p2.x = pointarray[point-1].x;
        p2.y = pointarray[point-1].y;
        p3.x = pointarray[point].x;
        p3.y = pointarray[point].y;
        // Find the angle determined by vector p1p2 and p2p3
        int position = 0;
        float angle = angleToPoint(p1, p2, p3, position);
        // Angle the robot said angle
        angleRobot(angle, position);
        // Drive the distance between p2 and p3
        if (!driveRobotDistance(p2, p3))
        {
        return;
        };
    }
    // Put marker back up
    motor[motorB] = -3;
    wait1Msec(1000);
    motor[motorB] = 0; 
}
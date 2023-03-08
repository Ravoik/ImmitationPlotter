



Final Project Report - Imitation Plotter









A Report Prepared For:
MTE 121



Prepared By Group 61:
Ryan Chan 20930530
Adel Chinoy 20930814
Tia Bariciak 20900016
Elbert Chen 20938700



November 24, 2021








Introduction
The etch-a-sketch is a fun toy that allows people of all ages to express their artistic creativity. Art is meant to be shared with others and this project was aimed at making that easier. Our imitation plotter robot will allow the user to draw on the screen using the two wheels as knobs that can be rotated to move the pen across the screen, functioning exactly like an etch-a-sketch. To extend upon this idea, we wanted to be able to project the user’s drawing onto a larger surface, so that it would be easier for a group of people to see. Since the robot is able to move, it is able to plot a scaled-up version of the user’s drawing onto a whiteboard so that it can be shared with others. In summary, the robot will allow the user to draw on the screen, and then plot the drawing onto a whiteboard.
Software Design and Implementation 

Overall Program Design 

We began by writing a main function where we wrote our main logic using blocks/tasks we hadn’t written yet. This informed us of what we needed to work on for the rest of our project and gave us a better understanding of where and what our blocks/tasks would be used for. Similarly, each of our blocks/tasks were broken into smaller functions, where we approached it the same way by using unwritten functions, then using that logic to inform our design decisions.

We chose the two primary functions, recordEtchASketch and drawSketch because they most naturally broke up our project into its two main stages. The first handles the portion where the user sketches whatever they like using the robot’s wheels, and the second handles the robot’s drawing of the user’s sketch.

From there, we broke up our code further into a few basic/trivial functions such as driveSpeed and calibrateGyro, as well as a few more functions fundamental to our code. The most important of these functions were angleToPoint and driveRobotDistance. The angleToPoint function handles our vector math to figure out the angle between the point the robot just came from, the current point, and the next point. This was necessary so that the robot had context of what direction it was facing when trying to turn to the next point, but also allowed us to reset the gyro instead of accumulating gyro error throughout our drawing.

Updated List of Tasks 

Select the correct program
As soon as the program begins, your sketch is recorded as points
Rotate the drive wheels to draw a sketch on the screen
Flip the robot to erase your sketch from the screen and begin a new sketch
Once you have completed your sketch remove the cap from the dry erase marker and place the robot in the centre of the whiteboard
After positioning the robot press the enter button to begin drawing your sketch on the whiteboard
Once the sketch is completed and all points are drawn the program ends automatically
Restart the program and make another sketch 
While the robot is drawing on the whiteboard test the obstacle detection function by placing your foot in front of the robot

The main change to our task list was the addition of obstacle detection. This allows our program to react to unexpected changes in the environment. If an obstacle is detected while the robot is drawing on the whiteboard, all motors will be set to 0 and the robot will stop. 

This is achieved by monitoring the accelerometer values in the x direction. If a large deceleration is detected, we know that the robot has hit an obstacle and all motors will be set to 0. The addition of obstacle detection improves the safety of our design as the robot will come to a stop if a person steps onto the whiteboard during the drawing period.  

Description of each function (parameters, return type, flow chart, who wrote the function)

Function: recordEtchASketch

Prototype: int recordEtchASketch(Point *pointarray);
Description:
This is one of our primary functions, encapsulating the entirety of the first stage of our project where the user uses the wheels to input their drawing.
How it works:
The function begins with various initializations relating to the accelerometer, for our invert to erase feature, the motor encoders, and a few variables for the coordinate system. It also makes sure that the brakes are off so we can spin the motors manually as input.
The function then goes into a while loop which checks the accelerometer to invert to erase, then draws a rectangle on screen based on the motor encoders, stores every fifth point drawn into the pointarray which was passed as a parameter, making sure that we do not store the same point more than once. Then, if we have reached our MAX_POINTS limit, we return the number of points. If the enter button is pressed, we break out of the while loop and return the number of points. Whenever we return, we also turn the brakes back on so the robot can drive properly.
Within the while loop, if the robot is inverted, determined by the accelerometer, then we reset the point array, wipe the screen, and then call recordEtchASketch within the function itself, and use it’s return value as the pointNum return value. This is our invert to erase feature.

Function: drawSketch

Prototype: void drawSketch(Point *pointarray, int points);
Description:
This is our second primary function which handles the robot’s drawing functionality, where it lowers the marker and draws onto a whiteboard.
How it works:
The function gets passed the point array which is the array of every point that needs to be drawn on screen, as well as the number of points needed to be drawn. It first initializes an origin point which the function uses to figure out the first angle the robot needs to drive to. The function then runs the third motor to put the marker down onto the whiteboard. It then stores the first and second points whose angles are calculated using arctan instead of vectors, as we cannot draw vectors between them. This works because it is only the first two angles where the robot does not need context for what direction the robot is going in. It then initializes three points to be used in the for loop, where it finds vectors between these points and draws lines between them using our angle and driving functions.

Function: detectHit

Prototype: void detectHit(tHTAC & accelerometer);
Description:
This handles the collision with unexpected objects
How it works:
This function gets passed a reference to the accelerometer, which is then used in the readSensor function given by the hitechnic library. Similar to the example code provided by RobotC, the function checks if the sensor can be read, and exits if there is an error. Then, if the accelerometer’s x direction is above 50, we determine that there has been a collision, and stop the program to prevent marker ink getting everywhere.

Function: angleToPoint

Prototype: float angleToPoint(Point p1, Point p2, Point p3, int &position);
Description:
This determines the vectors between p1 p2, and p2 p3, and returns the angle between them
How it works:
The function calculates the two vectors, and stores them in the same Point struct being used in the rest of the program, as the vector only has two entries as well. We then perform the formulas for determining the angle between them which is shown in appendix A.1. We also use the determinant of these two vectors to determine whether the query point, p3, is on the left or right of the p1 p2 vector.

Function: pLoop

Prototype: void pLoop(float angle, bool negative);
Description:
This function handles the the turning and the turn speed of the robot
How it works:
We first determine whether this angle is negative or positive, and assign a variable to it. We then refine the motor speeds based on how close we are to completing our turn, allowing us to have more accurate turning. This function also corrects if we are at high speeds and happen to overshoot our angle.

Function: storePoint

Prototype: void storePoint(Point *pointarray, int pointNum, int x, int y);
Description:
This function stores a point into the point array
How it works:
The function makes a point struct which stores the current x and y which are passed as parameters and stores them into the correct index of point array determined by pointNum

Data Storage

We store our points using an array of point structures. The point structure stores two integers, an “x” and “y” value. During the recordEtchASketch function, Points are printed to screen every ten encoder values because of the set sensitivity which is a constant variable. Every 10 points we store both x and y values into our Point array which is later used in the drawSketch function to determine the general distance and angle that the drawing was made at. These points are accessed and using a combination of trigonometry and vector math, distances and angles are derived from the points which are then translated into motor movements for the robot. 

Software design decisions: Trade-offs

The main tradeoff of our project lay in our point array which stored every point which would be physically drawn on the whiteboard. Having our array initialized within task main meant that we were limited by memory, maxing out at 127 points in the array. These are enough points for a simpler drawing, but wouldn’t be sufficient for a very complex drawing. We made the decision to have the robot draw one point for every five drawn on screen, which lowered the resolution of our drawing when drawn by the robot. This made curves look more boxy, while still resembling the user’s original drawing. We would have liked to have higher resolution drawings, but came to the conclusion that this solution was the best balance between resolution, memory, and the speed of the robot’s drawing as well.

Program Testing Methods

Throughout the development of our program, we performed unit tests for each function to evaluate the performance of the function before it was added into our main program. 

We started by writing the function that translates the rotation of the two drive wheels to drawing points on the screen. During the testing of this function, we noticed that the points being plotted on screen were far too small to see. Thus, we changed our logic from displaying a single pixel for each point to drawing a rectangle that is centred about the specific point. 

Next, we began to record the points that were drawn on screen as a struct in an array. To test that the x and y values of the points drawn on screen were being stored in the array correctly, we used the displayString() function to output the x and y coordinates of the points to the screen. 

See Table 1 for some of the key unit tests that were run.

Table 1: Program Testing Methods
Test #
Input Data
Reason
Expected Outcome
Actual Outcome
Pass/Fail
Improvements 
1.1
Rotation of drive wheels
Check that the rotation of the wheels was depicted as a sketch on the EV3 screen
Sketch displayed on screen 
Sketch was displayed on screen but points were too small to see 
Fail
Change the display of each point from setPixel() to drawRectangle() so the points would be visible
1.2
Rotation of drive wheels
Ensure that points drawn on screen are visible
Sketch displayed on screen using rectangles instead of individual pixels
Sketch was displayed on screen using rectangles
Pass


2.1
Rotation of drive wheels
Check that points being stored in array are correct
After enter button is pressed, x and y coordinates of all points are displayed to the screen
The x and y coordinates of all points were displayed to the screen
Pass


3.1 
x and y coordinates stored in point array
Use atan2 to determine the angle between points and the distance between them
Output the angle that the robot should turn to and the distance to drive for each point
The angle calculated for each set of points was not accurate as it did not take into account the orientation of the robot 
Fail
Look into different techniques to calculate the angle between two points that will consider the orientation of the robot prior to determining the angle to turn to
3.2 
x and y coordinates stored in point array
Use linear algebra to determine the angle between two vectors in the point array 
Output the required angle to turn to the console in a C++ program to refine the angleToPoint function before including the function in the robot code
The angle calculated for each set of points was correct for all test cases and considered the orientation of the robot
Pass
Include this new method of angle calculation in the robot code 
4.1 
Accelerometer values in the x direction  
Place an obstacle in front of the robot while drawing on the whiteboard to test collision detection function 
After hitting the obstacle, the robot should come to a stop (set all motor power values to 0)
After contacting the obstacle, the robot came to a full stop 
Pass





Discussion of any significant problems encountered and how they were resolved

During the planning phase for our angle finding function, we thought it would be sufficient to just use atan2 to find the angles between our points and then get our robot to drive to those angles using the gyro sensor. Unfortunately what we realized was that since the gyro is zeroed after each turn the robot did not know the angle that it was already facing and thus had no reference point to find the next angle. After encountering this problem we came to the realization we had to use vectors to find the angle while maintaining the direction. Thus we now took three points, generated two vectors and found the angle between them using *THIS FORMULA*. However, even with this angle, the robot cannot determine whether to turn left or right. That's where finding the determinant between the two vectors comes in, by finding the determinant between the two vectors the robot can determine whether or not it should rotate left or right. If it's on the right the determinant will be negative, if left the opposite is true. The determinant formula is shown in appendix A.2. 
Aside from this issue another major issue we encountered was the poor quality of the gyro sensor. At first we were turning at 50 motor power, which made our robot overshoot its turns by a very large margin, we first just tried reducing this power to improve the accuracy of the turns. We tried 20 then 10 then 5 and finally 2 motor power, but we were still off by a few degrees after every turn. These small errors would add up since the gyro is reset after every turn, this is why we implemented a p-loop into our motor power. By comparing the gyro value to the angle we need to get to we can calculate a variable motor power and have it slow down almost to 0 as it reaches the correct angle. Even after implementing this PID esque system our angles were still slightly off. Thus we hoped to just subtract three from every angle given to the robot depending of course on whether it was turning left or right. This ultimately fixed the vast majority of the error and it is barely noticeable when run. Sometimes the gyro would also just start counting up by like 10 or 100 counts, in these cases we had to unplug and plug the gyro back in to fix it.
Finally we also encountered many problems when trying to implement the accelerometer into our code. The documentation was just very hard to understand and the sample programs were not compiling in our robotC IDE’s at all. To resolve this we went to professor Hull’s office hours and just asked her for help and by doing this realized that we needed to move the entirety of HiTech’s include file into the program directory so that RobotC and our code would recognize it. After getting the sample program to work and reading the documentation, implementing the accelerometer was quite easy.
Conclusions and Recommendations
If given more time, we would try to improve the turning accuracy of the robot by changing the current jerk correction code from a p-loop to actual PID. We would also want to take more points between the pixels to smoothen out curves. Finally we would have wanted to try to find a way to calculate the first two turns in our point array without using the getArcTan function and instead using vectors and math similar to the equations used in our angleToPoint function.

If the project was used in industry it would more or less remain the same, the code has already been documented with comments fairly thoroughly so the constants should be easy to change. The most important constant that could be changed is the DISTANCE MULTIPLIER. This multiplier will change the distance the robot travels between points. This is useful if the robot was implemented in industry because this multiplier should be changed depending on the whiteboard below the robot. With a larger whiteboard it would look better if the robot traveled further between points and produced a larger drawing. On the other hand, if the whiteboard was smaller the robot can also be instructed to draw smaller by reducing this multiplier. 
Back Matter

Appendix A
Formulas

Formula A.1 - Formula for the angle determined by two vectors (See Figure 1).
![image](https://user-images.githubusercontent.com/63760716/223803373-1a627417-da0b-4659-92ff-4dc6a7a108fe.png)

Formula A.2 - Formula for the determinant of two vectors (See Figure 2).
![image](https://user-images.githubusercontent.com/63760716/223803408-ac8a4d4d-56e1-46f5-b543-6888dc0db0c3.png)

Appendix B
Source Code

Source File B.1 - Source code for the main RobotC file attached below report

Appendix C
Flow Charts

Flow Chart C.1 - Flow charts for all functions attached below report; Titled at the beginning of each page	

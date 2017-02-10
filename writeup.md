## Comprobo Project 1
###### Sean Carter and Paul Krusell

#Todo: Include a picture of the neato here

###### Project Overview:
The goal of this project was to get a neato (pictured above) to execute a number of basic robotic behaviors. We had access to all of the basic information that the neato generates, which included:
- Four bump sensors (to physically detect obstacles)
- A LIDAR scanner
- Odometers in the wheels
- A camera (not used for these behaviors)

To control the robot, we could set velocity and rotation speed based on logic written in Python.

For each behavior, describe the problem at a high-level.  Include any relevant diagrams that help explain your approach.  Discuss your strategy at a high-level and include any tricky decisions that had to be made to realize a successful implementation.

###### Teleoperation:
We believe that we will absolutely do this. Eventually. It will totally work.

###### Driving in a Square:
The first behavior that we implemented was to drive the robot in a square. We chose to use the robot's odometry data to determine whether or not we had completed part of the square, since we believed that it would give us greater accuracy and flexibility than timing based control.

We split moving the robot into two parts: translation and rotation. With our first function, the robot could be commanded to move a specific distance within the odometry frame of reference. This was fairly successful - as far as we could see, there wasn't any problem with our strategy yet.

The problems were more apparent while trying to rotate the robot an arbitrary number of degrees. The lag in communications was a big problem - while the odometry is surprisingly accurate, the lag between the robot publishing its orientation and receiving a command to stop meant that it consistently overshot the angle we commanded it to. To drive in a perfect square, we would have needed to run a calculation to stop the robot before we knew that it had reached its destination. Given the unpredictability of the delay, we decided to move on to the next challenge, and accept the flaws in the behavior.

###### Wall Following:

For the wall follower, we aimed for our robot to smoothly and consistently track a wall through the hallway. In order to do this, we decided to use a combination of our LIDAR and proportional control. We took LIDAR measurements at two different places along the wall at 45 degrees to the front and to the back of the robot. We then used proportional control to keep the two distances the same, while doing the same to keep the robot at a constant distance from the wall. 

For steps we used to build this program, we first started by measuring exactly 2 points at 45 degrees in front of and behind the robot. [do equation] The equation proportionally accounts for a difference between the two measurements. Once we got this working, we realized that sometimes the LIDAR would miss single points, and decided to take an average of points to reduce the chance of error. This was simple, and we just averaged out the distances from all the points.

Our code is structured with a wallFollower class, which has different functions to accomplish the various tasks needed to wall-follow. The seperate functions were organized into the three categories of "sense, think, act." The processLaser function subscribes to the LaserScan topic and sorts the data to remove noise, select the correct points, and average the data. It returns data to the doMath function, which acts as the think part of the robot.

The doMath function simply computes the rotational velocity required to balance out the front and back points, and the rotational velocity required to keep the robot at the correct distance from the wall. The run function takes the two velocities, adds them together, and then publishes a movement command in the form of a twist message to the "cmd_vel" topic.

###### Person Following:
Now that we had a firm understanding of the LaserScan data,

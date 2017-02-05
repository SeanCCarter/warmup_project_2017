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

###### Person Following:
Now that we had a firm understanding of the LaserScan data,

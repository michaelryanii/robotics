# CS485
##Robotics Code

#### Most code is under the Develop branch.  Only the initial commit resides in the Master branch

####Project 1 - Flockbot project 1
*   Using the Arduino IDE
*   We implemented a wall follow for the flockbot to keep a wall always to its right and if the wall disappeared from the side and angled infrared sensors, it would reaquire a wall to follow.
*   The final program found an orange cone with the camera attached to the raspberry pi and moved toward it unless an obstacle appeared in between the orange cone and the flockbot, then it implemented the wall follower and followed the wall until the cone could be reaquired, then the flockbot continued to the cone.

####Project 2 - Flockbot project 2
*   In this project, C code was used to develop a PD controller that used front and angled Infrared sensors that detected objects in front of the flockbot.  The PD controller was set up to increase reverse velocity when the object got closer to the flockbot (increasing speed with the closer the object became), and forward velocity as the object moved away from our ideal distance (about half a meter).
*   We then used the flockbot's camera attached to a raspberry pi to scan the environment for barcodes sheathed over small cans.  Various trigonometric functions were implemented to determine distance the flockbot stood from each of these cans, and then trinagulated the relative center between the cans.

####Project 3 - Darwin Humanoid soccer project
*   Lua and C were used in thsi project.  A state machine was used for each task.  The darwin was made to do a series of tasks leading up to its final goal.  The first task was made to find it's relative placement on a grid (a soccer field) and to walk to a specified area on the field.  
*   Darwin also located a ball, walked to the ball, walked around the ball to line up a shot toward some goal (either a location on the field or the goals set up on the grid) and then kicked it.
*   Darwin was also programmed to find it's location on the field, find another location on the field, walk to it, wait for the ball to present itself within 1 meter, walk to the ball and around it to line up a shot at another target that the darwin had to calculate the angle to, and then kick the ball to that target location.

####Project 4 - Pioneer 3DX extra credit project using ROS and Catkin
*   To mimic the behaviors of many upper end Mercedes and Lexus cars, we programmed the Pioneer 3DX to drive past several obstacles and then parallel park between two obstacles.  
#cs485

# UCF Baxter w/ psmove

## Requirements:
Ubuntu 14.04 w/ Bluetooth

Sony PlayStation Move motion controller

[PSmove API install](https://github.com/thp/psmoveapi)

[Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)

[Baxter simulator install](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)

[Rivz](http://wiki.ros.org/rviz/UserGuide)

## Instructions:
Ensure the tracker is properly calibrated by running
> ./test_tracker

in the psmove/build directory. Connect the psmove via bluetooth and run the baxter simulator and Rviz. 

Run the launch file:
> roslaunch psmove psmove_start.launch

This code will tuck both arms away then move the mirrored arm to neutral. Allowing you to move one are with out the other getting in the way.

## move.py

Can be run without the launch file:
> rosrun psmove move.py

Allows baxter to mirror a users movements with the psmove motion controller. Requires a calibration of the move to get your relationship to the robot and to the camera. Make sure the move is tracking, make sure you are far enough away from the camera so that the psmove is never out of view and follow the instructions on the screen to correctly calibrate move. 
Publishes poses to 'psmove' topic and sends the '/psmove' transform to Rviz.

## Buttons:
* Move: resets orientation
* Circle: move arm to neutral
* Square: Locks and unlocks position
* Cross: Locks and unlocks orientation
* PS: Closes the program and safely shuts down the robot
* Triangle: Takes position readings for calibration (used only in the calibration phase)

## show_pose.py

Prints the baxters current pose to the screen. Useful for debugging.

## Information:
[PSmove API documentation](https://thp.io/2012/thesis/thesis.pdf)

[Baxter_interface python API](http://api.rethinkrobotics.com/)

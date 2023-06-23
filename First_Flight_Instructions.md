# First Flight Instructions
https://github.com/PX4/PX4-user_guide/blob/v1.13/en/ros/external_position_estimation.md/
## First Flight

After setting up one of the (specific) systems described above you should now be ready to test.
The instructions below show how to do so for MoCap and VIO systems

### Check external estimate

Be sure to perform the following checks before your first flight:

* Set the PX4 parameter `MAV_ODOM_LP` to 1.
  PX4 will then stream back the received external pose as MAVLink [ODOMETRY](https://mavlink.io/en/messages/common.html#ODOMETRY) messages.
* You can check these MAVLink messages with the *QGroundControl* [MAVLink Inspector](https://docs.qgroundcontrol.com/en/analyze_view/mavlink_inspector.html)
  In order to do this, **yaw the vehicle until the quaternion of the `ODOMETRY` message is very close to a unit quaternion. (w=1, x=y=z=0)**
* At this point the body frame is aligned with the reference frame of the external pose system.
  If you do not manage to get a quaternion close to the unit quaternion without rolling or pitching your vehicle, your frame probably still have a pitch or roll offset.
  Do not proceed if this is the case and check your coordinate frames again.
* **Once aligned you can pick the vehicle up from the ground and you should see the position's z coordinate decrease.
  Moving the vehicle in forward direction, should increase the position's x coordinate.
  While moving the vehicle to the right should increase the y coordinate.**
  In the case you send also linear velocities from the external pose system, you should also check the linear velocities.
  Check that the linear velocities are in expressed in the *FRD* body frame reference frame.
* Set the PX4 parameter `MAV_ODOM_LP` back to 0. PX4 will stop streaming this message back.

If those steps are consistent, you can try your first flight.

**Put the robot on the ground and start streaming MoCap feedback (roslaunch vrpn_mavros)**.
Lower your left (throttle) stick and **arm the motors.**

At this point, with the left stick at the lowest position, switch to position control.
You should have a green light.
The green light tells you that position feedback is available and position control is now activated.

Put your left stick at the middle, this is the dead zone.
With this stick value, the robot maintains its altitude;
raising the stick will increase the reference altitude while lowering the value will decrease it.
Same for right stick on x and y.

Increase the value of the left stick and the robot will take off,
put it back to the middle right after. Check if it is able to keep its position.

After you are done with your first flight try [Offboard Setpoint Position](https://github.com/Alopez6991/offboard_py/tree/main)

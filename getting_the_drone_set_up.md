# Nomenclature 
- The computer that the mocap system is running on will be called `MocapComp`.
- The onboard computer on the drone will be called `DroneComp`
- the offboard computer will be called `OffboardComp`

# Set-up (physical)
## Cable Config

# Set-up (Code)
On the DroneComp and the OffboardComp install the following. This installs ros and all the dependencies that will be needed for runing future code (you can skip thsi step if your system is already sert up).\
https://github.com/uf-reef-avl/reef_auto_install
- Clone Repository
- cd into Repository
- Run ``./autoinstall.sh``
- Follow prompts
	
https://ardupilot.org/dev/docs/ros-install.html
- ``sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras``
- ``wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh``
- ``chmod a+x install_geographiclib_datasets.sh``
- ``sudo ./install_geographiclib_datasets.sh``
- ``sudo apt-get install ros-noetic-rqt ros-noetic-rqt-common-plugins ros-noetic-rqt-robot-plugins``

https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html
- ``git clone https://github.com/PX4/PX4-Autopilot.git --recursive``
- ``bash ./PX4-Autopilot/Tools/setup/ubuntu.sh``
	
**Restart computer**

If your DronComp doesnt already have a catkin workspace make one.\
https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Create-Catkin-Workspace.html
- ``cd ~/``
- ``mkdir --parents catkin_ws/src``
- ``cd catkin_ws``
- ``catkin init``
- ``catkin build``
- ``gedit ~/.bashrc``
- ``source ~/catkin_ws/devel/setup.bash``
	
**try this when things wont build first**
* ``catkin clean``
* ``ls``
* ``catkin build``
* ``ls``

# QGroundControl
Set up QGroundControl on the OffboardComp.

## Download
 https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
 - Download QGroundControl
 - ``chmod +x ./QGroundControl.AppImage``
 - ``./QGroundControl.AppImage``\
**To make launching QGround control easier I recommend you creat an alias in .bashrc**
- ``cd``
- ``nano .bashrc``
- add in something like ``alias QGC=./QGroundControl.AppImage``

## Flashing with px4
- plug your pixhawk into the your OffboardComp via micro USB to USB
- launch QGC
- wait for it to say connected (may take a while)
- click on the Q in the upper left corner
- Vehicle Setup
- Firmware
- 

## Calibrate (

## Uploading Parameters
To get the proper comunication channels up and running we need to set perameters in QGC.\
https://github.com/PX4/PX4-user_guide/blob/v1.13/en/ros/external_position_estimation.md
- click on the Q in the upper left corner
- Vehicle Setup
- Perameters\
**Method 1(recomended):**
- download [parameters](https://github.com/Alopez6991/2023_Drone_AVL_UNR/blob/main/params/REEF_PARAMS.params)
- tools (top right)
- upload parameters\
**Method 2 (hand set the parameters yourself):**\
After setting each parameter reatart the drone by going to parameters, tools, restart vehicale
- MAV_1_CONFIG: 102 (Telem 2)
- MAV_USEHILGPS: 1 (Enabled)
- EKF2_HGT_MODE: 3 (Vision)
- EKF2_AID_MASK: 24 (vision position fusion, vision yaw fusion)



# Mocap to DroneComp
go to the below repo and clone it into your catkin workspace.on the DroneComp\
https://github.com/Alopez6991/ros_vrpn_client
- ``cd``
- ``cd catkin_ws/``
- ``cd src/``
- ``git clone https://github.com/Alopez6991/ros_vrpn_client.git``
- ``cd ../``
- ``catkin build``
- ``roscd ros_vrpn_client``
- ``cd launch``
- ``nano test.launch``
- edit ``<arg name="name" default="magCheck" />`` (line 2) with the name of your object in mocap. e.g. ``<arg name="name" default="flybot" />``

## setting up telemetry
You should have telemetry plugged into telem1 on your pixhawk board. Find its matching transponder and plug it into your OffboardComp. On the OffboardComp run the following.\ 
https://arduino.stackexchange.com/questions/74714/arduino-dev-ttyusb0-permission-denied-even-when-user-added-to-group-dialout-o
- ``sudo usermod -a -G dialout your-username``
- ``sudo apt purge modemmanager``
# Mocap to Mavros
Clone the mocap to mavros package now. 
- ``cd``
- ``cd catkin_ws/``
- ``cd src/``
- ``git clone https://github.com/Alopez6991/vrpn_mavros.git``
- ``cd ../``
- ``catkin build``
- ``roscd vrpn_mavros``
- ``cd launch``
- ``nano test.launch``
- edit ``<arg name="name" default="platypus" />`` (line 2) with the name of your object in mocap. e.g. ``<arg name="name" default="flybot" />``
- edit ``<arg name="fcu_url" default="/dev/ttyUSB0:921600" />`` (line 3) with the dev path to your telem2 cable.
	- **Note that you need to have the pixhawk connected to the DroneComp via [telem2 and USB](#cable-config) otherwise it wont use MAVROS and MAVLINK**

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
* Once aligned you can pick the vehicle up from the ground and you should see the position's z coordinate decrease.
  Moving the vehicle in forward direction, should increase the position's x coordinate.
  While moving the vehicle to the right should increase the y coordinate.
  In the case you send also linear velocities from the external pose system, you should also check the linear velocities.
  Check that the linear velocities are in expressed in the *FRD* body frame reference frame.
* Set the PX4 parameter `MAV_ODOM_LP` back to 0. PX4 will stop streaming this message back.

If those steps are consistent, you can try your first flight.

**Put the robot on the ground and start streaming MoCap feedback (roslaunch vrpn_mavros)**.
Lower your left (throttle) stick and arm the motors.

At this point, with the left stick at the lowest position, switch to position control.
You should have a green light.
The green light tells you that position feedback is available and position control is now activated.

Put your left stick at the middle, this is the dead zone.
With this stick value, the robot maintains its altitude;
raising the stick will increase the reference altitude while lowering the value will decrease it.
Same for right stick on x and y.

Increase the value of the left stick and the robot will take off,
put it back to the middle right after. Check if it is able to keep its position.


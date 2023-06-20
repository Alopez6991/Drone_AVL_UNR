# Nomenclature 
- The computer that the mocap system is running on will be called `MocapComp`.
- The onboard computer on the drone will be called `DroneComp`
- the offboard computer will be called `OffboardComp`

# Cable Config

# Set-up
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

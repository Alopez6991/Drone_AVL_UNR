# Nomenclature 
- The computer that the mocap system is running on will be called `MocapComp`.
- The onboard computer on the drone will be called `DroneComp`
- the offboard computer will be called `OffboardComp`

# Set-up
On the DroneComp and the OffboardComp install the following. This installs ros and all the dependencies that will be needed for runing future code.\
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
go to the below repo and clone it into your catkin workspace.\
https://github.com/Alopez6991/ros_vrpn_client
- ``cd``
- ``cd catkin_ws/``
- ``cd src/``
- ``git clone https://github.com/Alopez6991/ros_vrpn_client.git``
- ``cd ../``
- ``catkin build``

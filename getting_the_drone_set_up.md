# Nomenclature 
- The computer that the mocap system is running on will be called `MocapComp`.
- The onboard computer on the drone will be called `DroneComp`
- the offboard computer will be called `OffboardComp`

# Set-up (physical)

## Bill of Material

| Item                              | Quantity | Link                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    | Price (for 1) | Use                                             |
| --------------------------------- | -------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- | ----------------------------------------------- |
| S500 V2 Frame Kit                 | 1        | [Holybro](https://holybro.com/products/s500-v2-kit?variant=42724497391805)                                                                                                                                                                                                                                                                                                                                                                                                                              | $42           | Frame Assembly                                  |
| NUC Computer                      | 1        | [Amazon](https://www.amazon.com/dp/B0BT7J7M4V?ref=cm_sw_r_cp_ud_dp_399B34ARB8ES1JM5YT8J&ref_=cm_sw_r_cp_ud_dp_399B34ARB8ES1JM5YT8J&social_share=cm_sw_r_cp_ud_dp_399B34ARB8ES1JM5YT8J&skipTwisterOG=1&th=1)                                                                                                                                                                                                                                                                                             | $649.99       | Collect data, control drone                     |
| Motor                             | 4        | [T-MOTOR](https://store.tmotor.com/product/mn3508-motor-navigator-type.html)                                                                                                                                                                                                                                                                                                                                                                                                                            | $69.90        | Spin the propellers                             |
| Motor Adapters                    | 4        | [RC-Innovations](https://rc-innovations.es/shop/adaptador-helices-t-motor-mn-ccw-m6#attr=)                                                                                                                                                                                                                                                                                                                                                                                                              | $11.51        | Attach propellers to the motor                  |
| ESC                               | 4        | [T-MOTOR](https://store.tmotor.com/product/air-40a-6s-esc.html)                                                                                                                                                                                                                                                                                                                                                                                                                                         | $39.99        | Control motors                                  |
| Flight Controller (The Cube Blue) | 1        | [IR-LOCK](https://irlock.com/products/pixhawk2-1-blue-cube)                                                                                                                                                                                                                                                                                                                                                                                                                                             | $600          | Includes IMU, firmware for estimation.          |
| Boost Converter                   | 1        | [Amazon](https://www.amazon.com/dp/B0CJ2S13YN?ref_=cm_sw_r_ud_dp_WCVA9QBVCH5QRGZ27S3X)                                                                                                                                                                                                                                                                                                                                                                                                                  | $8.99         | Voltage control - battery to NUC                |
| Power Brick Mini                  | 1        | [IR-LOCK](https://irlock.com/products/power-brick-mini)                                                                                                                                                                                                                                                                                                                                                                                                                                                 | $30           | Voltage control - battery to flight controller  |
| Input Board (Mini Carrier Board)  | 1        | [IR-LOCK](https://irlock.com/products/mini-carrier-board)                                                                                                                                                                                                                                                                                                                                                                                                                                               | $85           | Flight controller input board for sensors, etc. |
| RC telemetry                      | 1        | [Amazon](https://www.amazon.com/dp/B00RCAHHFM)                                                                                                                                                                                                                                                                                                                                                                                                                                                          | $38.99        | Send data to and from RC controller             |
| Modem                             | 1        | [IR-LOCK](https://irlock.com/products/rfd900x-us-modem-bundle-fcc-approved)                                                                                                                                                                                                                                                                                                                                                                                                                             | $226          | Send data from the drone to a computer          |
| Battery                           | 1        | [GetFPV](https://www.getfpv.com/lumenier-10000mah-4s-20c-lipo-battery.html)                                                                                                                                                                                                                                                                                                                                                                                                                             | $139.99       | Power the drone                                 |
| Battery Charger                   | 1        | [Amazon](https://www.amazon.com/dp/B01IDNJ5PG?ref_=cm_sw_r_cp_ud_dp_Z3REY5PDZKTT66CDGGE0)                                                                                                                                                                                                                                                                                                                                                                                                               | $139.97       | Charge batteries                                |
| USB to TTL                        | 1        | [Amazon](https://www.amazon.com/dp/B06ZYPLFNB?ref=cm_sw_r_cp_ud_dp_X786SY764PNK2JPQVCCP&ref_=cm_sw_r_cp_ud_dp_X786SY764PNK2JPQVCCP&social_share=cm_sw_r_cp_ud_dp_X786SY764PNK2JPQVCCP&skipTwisterOG=1)                                                                                                                                                                                                                                                                                                  | $14.99        | Connect NUC to The Cube Blue for data           |
| Power Pigtail Cables              | 1        | [Amazon](https://www.amazon.com/CERRXIAN-DC5521-Pigtails-Security-Black-M/dp/B0BNPXN8RD/ref=sr_1_2?th=1)                                                                                                                                                                                                                                                                                                                                                                                                | $9.99         | Connect battery to electronics                  |
| Wires and Connectors              | 1        | [Amazon](https://www.amazon.com/Pre-Crimped-Connectors-Pixhawk2-Pixracer-Silicone/dp/B07PBHN7TM/ref=mp_s_a_1_5?keywords=pixhawk+cables&qid=1651876086&sr=8-5)                                                                                                                                                                                                                                                                                                                                           | $18.99        | General assembly                                |
| Propellers                        | 2        | [APC Propellers](https://www.apcprop.com/product/11x4-7sf/)                                                                                                                                                                                                                                                                                                                                                                                                                                             | $3.99         | Flying                                          |
| Propellers (Reverse Rotation)     | 2        | [APC Propellers](https://www.apcprop.com/product/11x4-7sfp/)                                                                                                                                                                                                                                                                                                                                                                                                                                            | $3.99         | Flying                                          |
| M2 Standoffs and Screws           | 1        | [Amazon](https://www.amazon.com/M2-Motherboard-Standoffs-Male-Female-Electronic/dp/B0BP6MT7RP?th=1)                                                                                                                                                                                                                                                                                                                                                                                                     | $12.98        | General Assembly                                |
| M3 Standoffs and Screws           | 1        | [Amazon](https://www.amazon.com/Csdtylh-Male-Female-Standoff-Stainless-Assortment/dp/B06Y5TJXY1/ref=sr_1_3)                                                                                                                                                                                                                                                                                                                                                                                             | $13.98        | General Assembly                                |
| Rubber Padding                    | 1        | [Amazon](https://www.amazon.com/dp/B09H3GSL13?th=1)                                                                                                                                                                                                                                                                                                                                                                                                                                                     | $30.99        | Prevent the IMU from vibrating                  |
| Velcro Adhesive                   | 1        | [Amazon](https://www.amazon.com/Strips-Adhesive-Indoor-Outdoor-Sticky/dp/B07QCWFYC9/ref=sr_1_2?th=1)                                                                                                                                                                                                                                                                                                                                                                                                    | $10.97        | Attach stuff                                    |
| Zip Ties                          | 1        | [Amazon](https://www.amazon.com/HAVE-ME-TD-Cable-Ties/dp/B08TVLYB3Q/ref=sr_1_4?crid=HYOMGXA6HRZR&dib=eyJ2IjoiMSJ9.8X8UYFTZg2-fC4c9eC2sSKXEnzuz4AwnhcJWCBJe9HAY6JRY8k_Zpb22Qu4amolbLslQ4m8_yeo9Lz5Df_BKsHsc5Ie0GroUEoajsO_xxQ6X8mBOzBBJvI5Ed9ZUfhrsgZx57cPZMqGuubatu3r7wB_aurlI0r4Z71tLb4jSlIE_b1X564YBUgl0YlMzKn9rdxyGrj5FkV_kUDJKoJFgZWMlppHkZjNY_ElPDopCMKo.R_SBXJJP3-mln6LhFW_Yt5DcAXQK14qSka29QDzsT_U&dib_tag=se&keywords=cable%2Bties&qid=1715361157&sprefix=cable%2Bties%2Caps%2C131&sr=8-4&th=1) | $5.99         | Cable organization                              |
| Hot Glue Gun                      | 1        | [Amazon](https://www.amazon.com/dp/B0C2VFZ2XL?th=1)                                                                                                                                                                                                                                                                                                                                                                                                                                                     | $24.99        | General Assembly                                |
| 1/8" Wooden Dowels                | 1        | [Amazon](https://www.amazon.com/dp/B07ZP41KTY?th=1)                                                                                                                                                                                                                                                                                                                                                                                                                                                     | $8.99         | Motion Capture                                  |
| Pearl IR Markers                  | 30       | -                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       | -             | Motion Capture                                  |
#### Notes

A cheaper alternative to The Cube Blue flight controller used here is [The Cube Orange](https://irlock.com/products/cube-orange?variant=17907643908147&currency=USD&utm_medium=product_sync&utm_source=google&utm_content=sag_organic&utm_campaign=sag_organic&utm_campaign=gs-2018-09-19&utm_source=google&utm_medium=smart_campaign&gad_source=1&gclid=Cj0KCQjwrp-3BhDgARIsAEWJ6SxkUAurjztkKEPH3sd8yNxrrjXkz0NEETTFCJsIXDJiQHnI6eSvJ9EaAus8EALw_wcB). It is similar quality to The Cube Blue but is not D.o.D. compliant  
## 3D Printed Parts

| Filename                 | Quantity | Use          |
| ------------------------ | -------- | ------------ |
| QuadBatteryMountSide.stl | 2        | Hold battery |
| QuadBatteryFrontSide.stl | 1        | Hold Battery |
<-## Tools Required-!>
 
<-## Cable Config-!>
Here is a link to the [pin layout for the Cube Blue mini board](https://docs.cubepilot.org/user-guides/carrier-boards/mini-carrier-board).

#### TELEM1
Set this up to one end of a radio temelemetry. connet the paired other end of the telemetry to your OffboardComp. 
#### TELEM2
you need a cable that goes from TELEM2 to usb on the OffboardComp. This cable is an FTDI Cable. This cable is nescisary for getting any ROS and MAVROS comunications working. You will probably need to make this cable. I would start by buying an [FTDI cable](https://www.amazon.com/Ximimark-FT232RL-Serial-Adapter-Arduino/dp/B07T8YHBH1) **CUT THE POWER CABLE (RED) BEFORE PLUGGING IT IN**

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
<!-- - --> 

<!-- ## Calibrate (Pixhawk)
- 
--> 
## Calibrate (Telemetry)
- click on the Q in the upper left corner
- Vehicle Setup
- Radio
- Calibrate
- Follow prompts

## Uploading Parameters
To get the proper comunication channels up and running we need to set perameters in QGC.

- click on the Q in the upper left corner
- Vehicle Setup
- Perameters\

**Method 1 [(v1.13)](https://github.com/PX4/PX4-user_guide/blob/v1.13/en/ros/external_position_estimation.md)(hand set the parameters yourself):**
After setting each parameter reatart the drone by going to parameters, tools, restart vehicale
- MAV_1_CONFIG: 102 (Telem 2)
- MAV_USEHILGPS: 1 (Enabled)
- EKF2_HGT_MODE: 3 (Vision)
- EKF2_AID_MASK: 24 (vision position fusion, vision yaw fusion)

**Method 1 [(v1.14)](https://github.com/PX4/PX4-user_guide/blob/v1.14/en/ros/external_position_estimation.md) (hand set the parameters yourself):**
After setting each parameter reatart the drone by going to parameters, tools, restart vehicale
- MAV_1_CONFIG: 102 (Telem 2)
- EKF2_EV_CTRL: 15 (Horizontal position, vertical position, 3D velocity, yaw)
- EKF2_HGT_REF: Vision

**Method 1 [(v1.15)](https://github.com/PX4/PX4-user_guide/blob/v1.15/en/ros/external_position_estimation.md) (hand set the parameters yourself):**
After setting each parameter reatart the drone by going to parameters, tools, restart vehicale
- MAV_1_CONFIG: 102 (Telem 2)
- EKF2_EV_CTRL: 15 (Horizontal position, vertical position, 3D velocity, yaw)
- EKF2_HGT_REF: Vision

**Method 2(recomended if you have a param file already and its from a firmware version that matched your current):**
- download [parameters](https://github.com/Alopez6991/2023_Drone_AVL_UNR/blob/main/params/REEF_PARAMS_06_23.params) **for v1.13 only**
- tools (top right)
- upload parameters

**Always good to reboot the cube after changing any parameters**
- click on the Q in the upper left corner
- Vehicle Setup
- Perameters
- tools
- reboot



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
You should have telemetry plugged into telem1 on your pixhawk board. Find its matching transponder and plug it into your OffboardComp. On the OffboardComp run the following.

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

# Tie it all Together
- Toss enough mocap dots on your system such that there is no symetry
- Creat a ridged body in your mocap softwear and give it a unique name
- Update the launch files to reflect the name of your mocap object
- Place the drone in the flight space
- On the DroneComp run: ``roscore``
- On the DroneComp run: ``roslaunch vrpn_mavros test.launch``
- ``rostopic echo /mavros/local_position/pose``
	- check that there is data being published to this topic
- Move the drone around and make sure the values you see make sense
- When you are ready to do your [First Flight](https://github.com/Alopez6991/2023_Drone_AVL_UNR/blob/main/First_Flight_Instructions.md) pop over there.


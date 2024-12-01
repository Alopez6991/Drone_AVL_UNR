# vrpn_mavros
Repository to take pose from ros_vrpn_client and publish to PX4 via mavros


# Requires

* Motive
* Mavros
* ros_vrpn_client
* PX4 device

# Setup
## On Motive:

* Create a Rigid Body on motive and name it to name in launch file.
* Stream the position over VRPN using the up axis as Y.


## On ROS:

1. Clone this package to workspace
2. Clone our ros_vrpn_client to workspace

install mavros and mavros_extras
 
```
sudo apt install ros-melodic-mavros ros-melodic-mavros-extras
```
## Launch Configuration
1. Change "name" to name in motive 
2. Change "_vrpn_server_ip"in launch file to correct ip address (in our case its 192.168.1.104)
3. If using EKF2 (highly recommended), remap /vehicle/mocap/pose to /mavros/vision_pose/pose
4. If using LPE (HIGHLY not recommended) remap /vehicle/mocap/pose to /mavros/mocap/pose
5. If using Position, Altitude, Orbit, Hold, Return, Mission, Takeoff, Land, Follow Me, or Offboard modes
-- remap /vehicle/fake_gps/mocap/pose to /mavros/fake_gps/mocap/pose
-- in launch/config.yaml, set fake_gps -> satellites_visible to >6  

## Pixhawk Parameters
1. MAV_1_CONFIG: 102 (Telem 2)
2. MAV_USEHILGPS: 1 (Enabled)
3. EKF2_HGT_MODE: 3 (Vision)
4. EKF2_AID_MASK: 24 (vision position fusion, vision yaw fusion)

## Usage

### vrpn_mavros node:
The vrpn mavros node subscribes the following topics:
 * /mocap/nwu/pose_stamped [geometry_msgs/PoseStamped] - Vehicle pose in NWU Frame

The node publishes the following messages:
* /vehicle/mocap/pose [geometry_msgs/PoseStamped] - Vehicle pose in ENU Frame 
* /vehicle/fake_gps/mocap/pose [geometry_msgs/PoseStamped] - Vehicle pose in ENU Frame 



# Test

There are several ways to echo back position data
1. ```rostopic echo /mavros/local_position/pose``` (for position in ENU) 
2. In QGC -> Analyze Tools -> MAVLink Console:
    - ```listener vehicle_mocap_odometry``` or ```listener vehicle_visual_odometry``` (for position in NED)
3. In QGC -> Analyze Tools -> MAVLink Console:
    - ```mavlink stream -d /dev/ttyS1 -s LOCAL_POSITION_NED``` (for position in NED; device may be ttySX)
    - Go to QGC -> Analyze Tools -> Mavlink Inspector








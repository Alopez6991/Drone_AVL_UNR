# The Code
You can clone my repo for off board into you catkin_ws from [here]()

* ```cd```
* ```cd catkin_ws/src```
* ```git clone _________```
* ```cd ../```
* ```catkin build```\
or
* ```catkin_make```

# QGroundControl
To run position setpoints you need to set one of your switched on the RC controller to be offboard mode. you can do this by:
* Launch QGC
* Wait for the drone to connect
* Clck the Q in the top cornner
* Click Vehicle setup
* Click flight modes
* Selsect offboard mode to be mapped to your Radio controller switch of choice. **its a good idea to have manual flight be the oposit state of your offboard mode**

# Running everything
* ```roslaunch vrpn_mavros test.launch```
* ```roslaunch offboard offboard.py```
* arm the drone
* take off in manual flight mode
* flip into offboard mode
* validate as the drone flies to your targeted set point

# Future steps
Once you have validated offboard mode you can now jump to set point [velocity mode](). 

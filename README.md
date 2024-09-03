# Drone AVL UNR
collaboration between REEF - Autonomous Vehicles lab and UNR van Breugel lab.

This project will contain code for simulations, drone setup, drone control, and data analysis.

# Setting up the px4
jump to [getting the drone set up](https://github.com/Alopez6991/2023_Drone_AVL_UNR/blob/main/getting_the_drone_set_up.md)

# First Flight Instructions (position hold mode)
After setting up the drone jump to [First flight Instructions](https://github.com/Alopez6991/2023_Drone_AVL_UNR/blob/main/First_Flight_Instructions.md)\
\
These instructions are for setting the drone into position mode via the RC controller. and is a good first check that you are propperly using mocap. 

# Second Flight Instructions (setpoint position offboard mode)
After gettin your first flight in, in positon mode a good next step is sending the offboard position commands. check out [Offboard Setpoint Position Instructions](https://github.com/Alopez6991/2023_Drone_AVL_UNR/blob/main/Offboard_Setpoint_Position_Instructions.md) 

# Thrid Flight Instructions (setpoint velocity)
After geting setpoint positions working you can jump to getting set point velocities working. This will be where most of your utility will come from. You can design a simple PID controller for position control where the output is velocity comands to get you to your positon. The pixhawk will handle the attitude control for you. Head over to [offboard setpoint velocity instructions](https://github.com/Alopez6991/2023_Drone_AVL_UNR/blob/main/offboard_setpoint_velocity_instructions.md)

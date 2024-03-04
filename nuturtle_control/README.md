# Nuturtle_Control Package

This package controls the robot in simulation and the real world.

The launchfile can be called like ```ros2 launch nuturtle_control start_robot.launch.xml cmd_src:=X robot:=Y use_rviz:=Z```

**Possible args for cmd_src:**
* circle
* teleop
* none

**Possible args for robot:**
* nusim
* localhost
* none

**Possible args for use_rviz:**
* true
* false

## Physical Testing

A video of the turtlebot3 driving forward (CCW), in reverse (CW), and stopping along a circle with radius 0.3m.

https://github.com/ME495-Navigation/slam-project-henryburon/assets/141075086/709dd219-b25d-450f-9623-35e67807d3cf

Forward: 0:06 - 1:09  
Reverse: 1:09 - 2:10  
Stop: 2:10 - 2:12  
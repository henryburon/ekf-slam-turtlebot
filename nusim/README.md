# Nusim Package

Provides a simulated environment for the robot in RViz.

**Launch File**

`nusim.launch.xml` starts the simulator
* Launches RViz and displays the walls and obstacles
* Launches a single red robot using `load_one.launch.py`
* Uses a default config file, but let's the user specify a .yaml file to configure the simulator

**Parameters** stored in `basic_world.yaml`

* x0: Initial x-coordinate of robot
* y0: Initial y-coordinate of robot
* theta0: Initial orientation of robot
* rate: frequency of main loop
* walls/arena_x_length: length of the arena in the world *x* direction
* walls/arena_y_length: length of the arena in the world *y* direction
* obstacles/x: a list of the obstacles' x coordinates (float64)
* obstacles/y: a list of the obstacles' y coordinates (float64)
* obstacles/r: the radius of the obstacles (float64)


![Screenshot from 2024-01-22 22-07-50](https://github.com/ME495-Navigation/slam-project-henryburon/assets/141075086/2a07a2e3-23d9-4d17-ac36-77ed0d9a646e)
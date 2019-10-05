## Running basic URDF

You'll be checking out the `simple_robot.urdf` file and run the following commands (open new terminals if the current command takes control of the terminal)

- Start the `roscore` and set parameters
    ```bash
    roscore
    rosparam set robot_description -t ~/ros_workspaces/prj_6r_ws1/src/robot_viz/urdf/simple_robot.urdf
    rosparam set use_gui true
    ```
- Run the nodes for `RViZ`, `robot_state_publisher` and `joint_state_publisher`
    ```bash
    rviz -d ~/ros_workspaces/prj_6r_ws1/src/robot_viz/rviz/model_viewer.rviz 
    rosrun joint_state_publisher joint_state_publisher
    rosrun robot_state_publisher robot_state_publisher
    ```

# Bugs 
## URDF
- For some reason, the STL files are not getting parsed in the correct dimensional scale. It's suggested that you scale down the mesh by a 1000.

# References
## Links
- [URDF](http://wiki.ros.org/urdf/XML) and [XACRO](http://wiki.ros.org/xacro)
- [Launch](http://wiki.ros.org/roslaunch)

[![TheProjectsGuy Shield](https://img.shields.io/badge/Developer-TheProjectsGuy-blue)](https://github.com/TheProjectsGuy)

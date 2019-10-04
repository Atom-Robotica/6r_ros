## Running basic URDF

You'll be checking out the `simple_robot.urdf` file and run the following commands (open new terminals if the current command takes control of the terminal)

- Start the `roscore` and set parameters
    ```bash
    roscore
    rosparam set robot_description -t /home/avneesh/ros_workspaces/prj_6r_ws1/src/robot_viz/urdf/simple_robot.urdf
    rosparam set use_gui true
    ```
- Run the nodes for `RViZ`, `robot_state_publisher` and `joint_state_publisher`
    ```bash
    rviz -d ~/ros_workspaces/prj_6r_ws1/src/robot_viz/rviz/model_viewer.rviz 
    rosrun joint_state_publisher joint_state_publisher
    rosrun robot_state_publisher robot_state_publisher
    ```

[![TheProjectsGuy Shield](https://img.shields.io/badge/Developer-TheProjectsGuy-blue)](https://github.com/TheProjectsGuy)
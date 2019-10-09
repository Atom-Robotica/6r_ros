## Information 
### Launching everything
Everything can be launch using the following command
```bash
roslaunch robot_viz robot_viz.launch
```
The commands and more information can be found using the following commands:
```bash
roslaunch robot_viz robot_viz.launch --nodes
roslaunch robot_viz robot_viz.launch --ros-args
```

### Robot builder API
Robot builder is an API made to create an articulated robot from scratch using minimal efforts. To use this API, do the following:
1. Open the file `robot_builder.xacro` (link [here](./urdf/robot_builder.xacro))
2. Use the XACRO macro create_base_link to create a base link
3. Use the XACRO macro attach link to create a link and a joint for links after base link

The usage for **create_base_link** macro is as shown below:
```xml
<!-- Base link creation -->
<xacro:create_base_link name="base_link">
    <inertia_visual_collision>
        <!-- Inertia properties here -->
        <inertial>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <mass value="0.0"/>
            <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
        </inertial>
        <!-- Visual properties here -->
        <visual>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <box size="0.1 0.2 0.3"/>
            </geometry>
            <material name="base_material">
                <color rgba="0.0 1.0 0.0 1.0"/>
            </material>
        </visual>
        <!-- Collision properties here -->
        <collision>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <geometry>
                <box size="0.1 0.2 0.3"/>
            </geometry>
        </collision>
    </inertia_visual_collision>
</xacro:create_base_link>
```
The following things can be noted:
- The `name` attribute is used to set the name of the base link (default value is *base_link*)
- Since this is the base link, all properties need to be defined
  - Create the tag `<inertia_visual_collision>` to hold:
    - `<inertia>` tag 
    - `<visual>` tag
    - `<collision>` tag
      - A link to more information on all this can be found [here](http://wiki.ros.org/urdf/XML).

This finishes the base link description. Now we must define the links that come next. We use the **attach_link** macro for this. The usage is as follows:
```xml
<xacro:attach_link parent="base_link" child="link1" file_path="${link1_mesh}" min_q="${-pi}" max_q="${pi}">
    <!-- Origin of child frame in parent frame -->
    <origin xyz="0.0 0.0 0.15" rpy="0.0 0.0 0.0"/>
    <!-- Collision boundary -->
    <box size="1 0.2 0.2"/>
    <!-- Inertial properties of the link -->
    <inertia_block>
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        <mass value="0.0"/>
        <inertia ixx="0.0" ixy="0.0" ixz="0.0" iyy="0.0" iyz="0.0" izz="0.0"/>
    </inertia_block>
</xacro:attach_link>
```
Notice the following:
- The following attributes are important:
  - `parent` is the name of the parent link
  - `child` is the name of the child link that is newly being created
  - `file_path` is the full path to the `.stl` file of the _child_ link. You're suggested to set a `<xacro:property .../>` and then use the property value to access the file.
  - `min_q` is the minimum actuation angle and `max_q` is the maximum.
- The `<origin>` tag is used to give the origin of the _child_ frame in the _parent_ frame. This is used to define the joint. The joint is revolute and actuated along the **Z axis**.
- The `<box>` tag is for the collision boundary.
- The `<inertia_block>` is to define the inertia properties of the link.

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
- [URDF](http://wiki.ros.org/urdf) and [XACRO](http://wiki.ros.org/xacro)
- [Launch](http://wiki.ros.org/roslaunch)

[![TheProjectsGuy Shield][dev-shield]][dev-link]

[dev-shield]:https://img.shields.io/badge/Developer-TheProjectsGuy-blue
[dev-link]:https://github.com/TheProjectsGuy

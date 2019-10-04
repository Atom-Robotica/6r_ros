# Table of contents
- [Table of contents](#table-of-contents)
- [6r_ros](#6rros)
- [Setup](#setup)
  - [Creating a workspace](#creating-a-workspace)
- [Files](#files)

# 6r_ros
Repository for project 6R (for all ROS related stuff)

# Setup
## Creating a workspace

```bash
cd ros_workspaces/
mkdir -p prj_6r_ws1/src/
cd prj_6r_ws1/src/
catkin_init_workspace
cd ../
catkin_make
```

# Files
The following folders are present here
| Folder name | Purpose |
| :------: | :---- |
| [matlab](./matlab/) | All files related to matlab |
| [packages](./packages/) | All the ROS packages created |

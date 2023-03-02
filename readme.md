# sw2urdf_ros2

The current solidworks sw2urdf plugin can only be used in ros1, which is obviously unfriendly to the current learning
environment.So I made this tool to convert files to ros2 for use. At this stage, the tool has only been successfully tested in rviz2.

### Table of contents

1. [Environment](#1-environment)
2. [Getting tool](#2-getting-tool)
3. [Getting started](#3-getting-started)
4. [Changing configuration](#4-changing-configuration)
5. [Running launch file](#5-running-launch-file)
6. [Reference](#5-reference)

### 1. Environment

- Solidworks 2019
- Ubuntu 20.04
- Ros2 foxy
- Python == 3.8

### 2. Getting tool

We download the tool from github

~~~ bash
git clone https://github.com/xiaoming-sun6/sw2urdf_ros2.git
~~~ 

### 3. Getting started

We should have an urdf folder that outputs from solidworks.Let's say the name of the folder is test_urdf(I had put this folder in the project as a demo file).

Let's create a workspace called test_urdf_tool_ws and a ROS package called test_urdf_tool

~~~ bash
mkdir test_urdf_tool_ws && mkdir test_urdf_tool_ws/src
cd test_urdf_tool_ws && colcon build
cd src && ros2 pkg create test_urdf_tool --build-type ament_python
~~~

### 4. Changing configuration

We must change the configuration variables in conversion_urdf_ros_2_ros2.py if we use this tool.

~~~python
# Configuration variable

# This variable is the path to the solidworks output folder of urdf files
source_dir = '/home/sxm/Project/ros/test_urdf/'

# This variable is the package path of ros2
target_dir = '/home/sxm/Project/ros/test_urdf_tool_ws/src/test_urdf_tool/'

# This variable is the package-name of ros2
package_name = "test_urdf_tool"

# This variable is the solidworks output folder name
output_folder_name = "test_urdf"
~~~

After changing the configuration variables, we run the python file in a new terminal.

```bash
python conversion_urdf_ros_2_ros2.py
```

Then,We go into the workspace and build the code.
~~~ bash
cd .. && colcon build && source install/setup.bash
~~~

### 5. Running launch file
Finally，We run the launch files.
~~~ bash
ros2 launch test_urdf_tool launch.py 
~~~

Don't forget to change the rviz configuration
![rviz](https://github.com/xiaoming-sun6/sw2urdf_ros2/blob/master/img/rviz.png)

### 5. Reference
Thanks for the links

[sw2urdf plugin](http://wiki.ros.org/sw_urdf_exporter)  
[lesson_urdf](https://github.com/olmerg/lesson_urdf)  
[Source of inspiration](https://zhuanlan.zhihu.com/p/465398486)



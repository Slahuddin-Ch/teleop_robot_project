# Installation Instructions
1. Install ROS by following the instruction in [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu)
2. For development install ros-noetic-desktop-full
3. For deployment install ros-noetic-desktop (excluding simulation)
4. Method 1 - Using Webrtc Source (Follow this method only if the binaries are not available)
    1. To get webrtc source, run command `git clone https://github.com/RobotWebTools/webrtc_ros` into the src folder of your workspace
    2. cd into worspace and run command `rosdep install --from-paths src --ignore-src -r -y`
    3. Run command `catkin_make_isolated` if you have webrtc source code in the workspace.
    4. If webrtc fails to build, try to solve it with the help of internet for now, soon we will have our binary and detailed instructions for installing webrtc.
    5. Run command `echo "source <path to workspace>/devel_isolated/setup.bash" >> ~/.bashrc`
5. Method 2 - Using Webrtc Binary (Preferred method)
    1. To install webrtc binary, run command `sudo dpkg -i <name of the .deb file>`
    2. cd into worspace and run command `rosdep install --from-paths src --ignore-src -r -y`
    2. Run command `catkin_make` if you have webrtc binaries installed and 
        `echo "source <path to workspace>/devel/setup.bash" >> ~/.bashrc`

# Launch Instructions
>To run this code and also all the necessary packages required for Remote Teleop using web

For Simulation run commands

1. `roslaunch mr_robot_gazebo cafehouse_world.launch`
2. `roslaunch controller teleop.launch`

For Deployment run commands
1. `roslaunch controller teleop.launch`
2. To be added for hardware

To Launch webapp
1. cd into the webapp directory
2. Run command `node index.js`

For more information look into Readme.md files of each package and Repo of the webapp
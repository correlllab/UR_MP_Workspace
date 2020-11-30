# UR_MP_Workspace
Universal Robots Motion Planning ROS1 (Melodic) Workspace

# Download
* `git clone https://github.com/correlllab/UR_MP_Workspace.git --recursive` (This repo)

# Installation Steps (Ubuntu 18)
### Install Qt
1. Login to Qt: https://www.qt.io/download-open-source
1. Download and install latest Qt
### Install Eigen
1. `cd /tmp` (Choose another directory if you do not want repo to disappear on shutdown)
1. `git clone https://gitlab.com/libeigen/eigen.git`
1. `cd eigen`
1. `mkdir build && cd $_`
1. `cmake ..`
1. `sudo make -j6 install`
### Install ROS
1. `sudo apt install ros-melodic-desktop-full python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential`
1. `echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc`
1. `source ~/.bashrc`
1. `sudo rosdep init && rosdep update && rosdep install rviz`
### Install Bullet (Optional? Try without first)
1. `./src/vcpkg/bootstrap-vcpkg.sh`
1. `./src/vcpkg/vcpkg integrate install`
1. `./src/vcpkg/vcpkg install bullet3`
### Download MoveIt! (Ver.1) and prep for install (NOTE: Must be built from source)
1. `wstool init src`
1. `wstool merge -t src https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall`
1. `wstool update -t src`
1. `rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}`
1. `catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release`
1. `touch src/moveit/moveit_planners/pilz_industrial_motion_planner/CATKIN_IGNORE`
### Build All Packages (including MoveIt!)
1. `catkin build`

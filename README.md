# AUT-841

## Installation of the contents in this repository

### Cloning the repository

	git clone https://github.com/JokinenL/AUT-841.git
	
### Installing/building the workspace
	source /opt/ros/noetic/setup.bash && \
	cd ~/AUT-841/robot_ws && \
	rosdep update && \
	rosdep install --from-paths src/ --ignore-src --rosdistro noetic && \
	catkin_make
	
### Activating motoman_ws
	source ~/AUT-841/robot_ws/devel/setup.bash
	

### Visualizing the dual arm robot

	source /opt/ros/noetic/setup.bash && \
	cd ~/AUT-841/robot_ws && \
	source devel/setup.bash
	roslaunch dual_arm_robot test_sia20d_dual.launch 

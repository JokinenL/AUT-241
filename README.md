# AUT-841

## Installation of the contents in this repository

### Cloning the repository

	git clone https://github.com/JokinenL/AUT-841.git
	
### Building motoman_ws
	source /opt/ros/noetic/setup.bash && \
	cd ~/AUT-841/motoman_ws && \
	rosdep update && \
	rosdep install --from-paths src/ --ignore-src --rosdistro noetic && \
	catkin_make
	
### Activating motoman_ws
	source ~/AUT-841/motoman_ws/devel/setup.bash

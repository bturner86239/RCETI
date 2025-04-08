If saying launch package isnt working:
sudo apt install ros-humble-ros2launch
source /opt/ros/humble/setup.bash
ros2 --help | grep launch
  launch                  Run a launch file


### Launch old visualizer
ros2 launch continuum_robot visualize.launch.py

### Launch new visualizer
ros2 launch continuum_robot visualize_interface.launch.py



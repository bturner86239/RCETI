
https://www.youtube.com/watch?v=sLpNf3Wudt8

To run the demo visualization on youtube:

1- download the package from github inot your src folder of yor ROS workspace.

2- colcon build ROS workspace.

3- Run the rceti_continuum core_node

ros2 run rceti_continuum core_node

4- launch rviz

ros2 launch rceti_continuum visulize.launch.py

5- Input user char into core_node

6- Change fixed frame to base_link

7- Add interactive marker topic cable0

8- User input in core_node again to display


Cite as:

Seleem, Ibrahim A., Samy FM Assal, Hiroyuki Ishii, and Haitham El-Hussieny. "Guided pose planning and tracking for multi-section continuum robots considering robot dynamics." IEEE Access 7 (2019): 166690-166703.

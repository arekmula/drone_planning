# drone_planning
Project for Motion Planning Methods and Algorithms course.

The goal of the project is to compare 3 diffrent methods of motion planning for drone. The environment in which drone moves is represented by Octomap.

Tasks of project:
- prepare octomap environment
- prepare drone (mesh)
- launch chosen methods of motion planning for defined problem
- visualize planned path and motion of the drone

Dependencies:
- libccd https://github.com/danfis/libccd
- fcl https://github.com/flexible-collision-library/fcl
- octomap https://github.com/OctoMap/octomap
- MoveIt! https://moveit.ros.org/install/


Bag file with octomap environment used in project:
- bag file: https://drive.google.com/open?id=11DyT1_8go2_tEr34aZ9rSi7Hpfpf1Mm4

Launch with:
- play bag file with octomap in loop mode
- $ roslaunch drone_planning drone_planning.launch

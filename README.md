# AutonomousRobotNavigation

In this project, we successfully simulated a four-wheel differential-drive rover following a pre-planned cubic trajectory on a flat plane, using ROS and Gazebo. The primary objectives were to design a rover model, implement a cubic path-following algorithm, and test the rover's movement in a virtual environment. Key accomplishments include:

•	Rover Model Design: Developed a detailed URDF model for a four-wheel rover, accurately representing its physical structure and dynamics.

•	Path-Following Algorithm: Created a custom control node to compute the velocity commands needed to follow the cubic trajectory, utilizing differential-drive kinematics to convert the path into precise movements.

•	Potential Fields Algorithm: Successfully implemented the potential fields algorithm for obstacle avoidance, allowing the rover to dynamically navigate around obstacles by balancing attractive forces from the goal and repulsive forces from obstacles.

•	Gazebo Simulation Environment: Successfully simulated the rover's movement, enabling testing of path-following accuracy and turn-handling capabilities.

•	Real-Time Visualization: Used Rviz to monitor the rover's progress along the path, observing its movement in real time.

•	Control Value Plotting: Leveraged RQT to plot steering commands for the front wheels and angular speed commands for the rear wheels, providing insights into the control dynamics and enabling fine-tuning of the control parameters.

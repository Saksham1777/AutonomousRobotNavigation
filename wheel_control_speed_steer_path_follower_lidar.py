#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math, time
import tf
from gazebo_msgs.msg import ModelStates, LinkState
from std_msgs.msg import Header
from sensor_msgs.msg import JointState, LaserScan
import numpy as np
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class WaypointFollower:
    def __init__(self):
        # Initialize node
        rospy.init_node('wheel_control_speed_path_follower', anonymous=True)

        # Publisher for wheel control commands
        self.rear_left_speed_command_pub = rospy.Publisher('/rear_left_wheel_controller/command', Float64, queue_size=10)
        self.rear_right_speed_command_pub = rospy.Publisher('/rear_right_wheel_controller/command', Float64, queue_size=10)

        # Publisher for front steering commands
        self.front_left_steering_pub = rospy.Publisher('/front_left_wheel_steering_controller/command', Float64, queue_size=10)
        self.front_right_steering_pub = rospy.Publisher('/front_right_wheel_steering_controller/command', Float64, queue_size=10)

        # Publisher for bot's pose for visulaization
        self.pose_pub = rospy.Publisher('/bot_pose', PoseStamped, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # Subscribe to bot state.
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        #control params
        self.STEERING_GAIN = 1.0   # Adjusts how aggressively the robot steers
        self.MAX_STEERING_ANGEL = 0.5 #in both direciton. In radian. Defined in URDF.
        self.DISTANCE_THRESHOLD = 0.5  # In meter. How close is "close enough" to the goal
        self.WHEELBASE = 0.44  # In meter. Distance between front and rear axles (meters). Defined in URDF.
        self.TRACKWIDTH = 0.5 # In meter. Distance between rear wheels (meters) Defined in URDF.
        self.WHEELRADIUS = 0.1 #radius of each wheel. In meter. Defined in URDF.
        
        # In rad/sec - Maximum wheel speed. [v = omega * r]. We want 1m/s linear speed.
        self.MAX_WHEEL_SPEED = 1.0 / self.WHEELRADIUS

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_q = None
        self.model_state_available = False
        self.marker = Marker()

        #for lodar object avoidance
        self.goal = None
        self.lidar_data = [float('inf')] * 10
        self.k_attraction = 1.0
        self.k_repulsion = 2.0
        self.repulsion_range = 2.0
        self.lidar_theta = 180.0
        self.potential_data = []

        time.sleep(10) #wait till gazebo and rviz launch.

        ## Below is to move bot in cubic path
        # # way points calculations
        # self.generate_cubic_waypoints(-0.1, -0.1, 2.8, 2.8)
        # rospy.loginfo(f"Waypoints: {self.waypoints}")

        # #publish desired path for rviz
        # self.generate_markers_for_rviz()
        # self.marker_pub.publish(self.marker)

        # #publish pose every 0.1 sec.
        # rospy.Timer(rospy.Duration(0.1), self.async_timer_callback_publish_pose)

        # #real work here.
        # self.follow_waypoints(with_back_and_forth = False)

        ## THis is for going to goal with obstacle avoidance
        # set goal coordinates
        self.go_to_goal(5.0, 10.0, False)
        return

    def spin(self):
        # Keep the node running
        rospy.spin()

    def scan_callback(self, msg):
        self.lidar_data = msg

    def model_states_callback(self, msg):
        # Update the current position and yaw
        try:
            self.current_x = msg.pose[1].position.x
            self.current_y = msg.pose[1].position.y

            # # Get orientation from quaternion
            orientation_q = msg.pose[1].orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

            # # Convert quaternion to roll, pitch, yaw using tf transformations
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)
            # Normalize yaw between -pi and +pi
            self.current_yaw = self.normalize_angle(yaw)

            self.current_q = msg.pose[1].orientation

            self.model_state_available = True
        except:
            pass

    def async_timer_callback_publish_pose(self, event):
        #wait for model state to be available
        if self.model_state_available:
            pass
        else: #not yet available.
            return

        #Now, publish pose for rviz:
        pose = PoseStamped()
        pose.header.frame_id = 'map'  # or 'odom', depending on your setup
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = self.current_x  
        pose.pose.position.y = self.current_y 
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = self.current_q.x
        pose.pose.orientation.y = self.current_q.y
        pose.pose.orientation.z = self.current_q.z
        pose.pose.orientation.w = self.current_q.w

        self.pose_pub.publish(pose)

    def normalize_angle(self, angle):
        #Normalize the angle to the range [-pi, pi].
        return math.atan2(math.sin(angle), math.cos(angle))

    def shortest_angular_distance(self, current_angle, target_angle):
        #Compute the shortest angular distance, considering wrap-around.
        #this will tell us angular direction to move

        difference = target_angle - current_angle
        # Normalize it to be within -180 and +180 degrees
        difference =  math.pi - (difference + math.pi) % (2*math.pi)

        return difference

    def generate_cubic_waypoints(self, a, b, c, d):
        # Generate 20 evenly spaced x-values within the range [-10, 10]
        waypoints_count = 20
        x_values = np.linspace(-6, 6, waypoints_count)
        
        # Compute the corresponding y-values using the cubic equation: y = ax^3 + bx^2 + cx + d
        y_values = a * x_values**3 + b * x_values**2 + c * x_values + d
        
        # Combine x and y values into waypoints (as tuples)
        waypoints = [(round(x, 1), round(y, 1)) for x, y in zip(x_values, y_values)]
        
        self.waypoints = waypoints
    
    def generate_current_location_markers_for_rviz(self):
        #now, we create markers for rviz visualization
        self.marker.header.frame_id = "map"  # Coordinate frame
        self.marker.header.stamp = rospy.Time.now()
        
        # Marker type as a LINE_STRIP (continuous line)
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD

        # Define the marker scale (line width)
        self.marker.scale.x = 0.05  # Line width

        # Define marker color (e.g., red)
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0  # Transparency

        # Add points to the path (x, y, z)
        self.marker.points.append(Point(self.current_x, self.current_y, 0.0))

    def generate_markers_for_rviz(self):
        #now, we create markers for rviz visualization
        self.marker = Marker()
        self.marker.header.frame_id = "map"  # Coordinate frame
        self.marker.header.stamp = rospy.Time.now()
        
        # Marker type as a LINE_STRIP (continuous line)
        self.marker.type = Marker.LINE_STRIP
        self.marker.action = Marker.ADD

        # Define the marker scale (line width)
        self.marker.scale.x = 0.05  # Line width

        # Define marker color (e.g., red)
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 1.0  # Transparency

        # Add points to the path (x, y, z)
        self.marker.points = []

        for point in self.waypoints:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = 0.0
            self.marker.points.append(p)

    def get_rear_differential_angular_speeds(self, steering_angle, vehicle_speed_x, front_movement_flag=True):
        # Calculates and returns angular speeds of two rear wheels (differential speeds).

        # Inputs:
        # steering angle - angle with witch vehicle is turning.
        # vehicle_speed_x - desired linear speed of vehicle in x direction.
        # front_movement_flag = True for bot moving front while turning, False for bot moving back while turning.

        if abs(steering_angle) < 0.05: #radian
            #special case of too small steering angle.
            inner_wheel_speed = abs(vehicle_speed_x) / self.WHEELRADIUS
            outer_wheel_speed = abs(vehicle_speed_x) / self.WHEELRADIUS
        else:
            # turning radius R
            turning_radius = abs(self.WHEELBASE / math.tan(steering_angle))
            
            # Calculate the radii for the inner and outer wheels
            r_inner = turning_radius - (self.TRACKWIDTH / 2)
            r_outer = turning_radius + (self.TRACKWIDTH / 2)
            
            # Calculate the speed of inner and outer wheels based on turning radii
            inner_wheel_speed = abs(vehicle_speed_x) * (r_inner / turning_radius)
            outer_wheel_speed = abs(vehicle_speed_x) * (r_outer / turning_radius)

            # Convert to angular speed
            inner_wheel_speed = inner_wheel_speed / self.WHEELRADIUS
            outer_wheel_speed = outer_wheel_speed / self.WHEELRADIUS

        #Basic logic for below code:
        """
        Front movement – front wheels turned left
            Steering angle: +ve
            Rear right wheel: High, +ve
            Rear left wheel: low, +ve
        Front movement – front wheels turn right
            Steering angle: -ve
            Rear right wheel: low, +ve
            Rear left wheel: high, +ve
        Backward movement – front wheels turned left
            Steering angle: +ve
            Rear right wheel: High, -ve
            Rear left wheel: low, -ve
        Backward movement – front wheels turned left right
            Steering angle: -ve
            Rear right wheel: low, -ve
            Rear left wheel: high, -ve
        """
        if front_movement_flag:
            # Assign to left / right wheels.
            if steering_angle >= 0: #turning left
                right_wheel_speed = outer_wheel_speed
                left_wheel_speed = inner_wheel_speed
            else: #turning right
                right_wheel_speed = inner_wheel_speed
                left_wheel_speed = outer_wheel_speed
        else: #moving backward
            if steering_angle >= 0: #wheel turning left
                right_wheel_speed = -outer_wheel_speed
                left_wheel_speed = -inner_wheel_speed
            else: #turning right
                right_wheel_speed = -inner_wheel_speed
                left_wheel_speed = -outer_wheel_speed

        return left_wheel_speed, right_wheel_speed

    def move_to_waypoint(self, goal_x, goal_y, with_back_and_forth):
        rate = rospy.Rate(10)  # 10 Hz update rate
        
        while not rospy.is_shutdown():
            #wait for model state to be available
            if self.model_state_available:
                pass
            else: #not yet available.
                continue
            
            # Calculate the distance and angle to the goal
            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            distance_to_goal = math.sqrt(dx**2 + dy**2)

            # Check if the robot is close to the goal
            if distance_to_goal < self.DISTANCE_THRESHOLD:
                # Stop the robot
                #self.rear_left_speed_command_pub.publish(0.0)
                #self.rear_right_speed_command_pub.publish(0.0)
                break  # Goal reached

            # Calculate the desired heading to the goal
            desired_angle = math.atan2(dy, dx)

            # Calculate the steering angle to turn towards the goal (shorttest angular dist.)
            angle_diff = -self.shortest_angular_distance(self.current_yaw, desired_angle)
            
            # Apply steering command based on angle difference
            steering_command = self.STEERING_GAIN * angle_diff

            # apply turning limits
            if steering_command > self.MAX_STEERING_ANGEL: steering_command = self.MAX_STEERING_ANGEL
            if steering_command < -self.MAX_STEERING_ANGEL: steering_command = -self.MAX_STEERING_ANGEL

            # Get the speed at which vehicle should travel.
            # The speed should be low when turning.           

            #wheel_speed = self.MAX_WHEEL_SPEED * max(0, 1 - abs(steering_command)) # rad/sec
            wheel_speed = self.MAX_WHEEL_SPEED * math.exp(-abs(3.5*steering_command)) # rad/sec
            vehicle_speed_x = wheel_speed * self.WHEELRADIUS

            # Get the differental speeds for the two rear wheels. Moving frontward
            left_wheel_speed, right_wheel_speed = self.get_rear_differential_angular_speeds(steering_command, vehicle_speed_x, True)

            # Publish the steering commands to the front wheels
            self.front_left_steering_pub.publish(steering_command)
            self.front_right_steering_pub.publish(steering_command)
            
            # Publish the speed commands to the rear wheels
            self.rear_left_speed_command_pub.publish(left_wheel_speed)
            self.rear_right_speed_command_pub.publish(right_wheel_speed)
            
            # handling sharp turn, go back a bit to turn sharply
            if (abs(steering_command) > 15.0 * math.pi/180.0) and (with_back_and_forth == True):
                left_wheel_speed_backward, right_wheel_speed_backward = self.get_rear_differential_angular_speeds(-steering_command, vehicle_speed_x, False)
                rospy.sleep(2)
                self.front_left_steering_pub.publish(-steering_command)
                self.front_right_steering_pub.publish(-steering_command)
                rospy.sleep(1)
                self.rear_left_speed_command_pub.publish(left_wheel_speed_backward * 2) #factor of 2 to go back a bit fast
                self.rear_right_speed_command_pub.publish(right_wheel_speed_backward * 2) #factor of 2 to go back a bit fast
                rospy.sleep(2)

            rate.sleep()

    def follow_waypoints(self, with_back_and_forth):
        for waypoint in self.waypoints:
            goal_x, goal_y = waypoint
            rospy.loginfo(f"Moving to waypoint: ({goal_x}, {goal_y})")
            self.move_to_waypoint(goal_x, goal_y, with_back_and_forth)
            rospy.loginfo(f"Reached waypoint: ({goal_x}, {goal_y})")

    ######################################################################
    def compute_attractive_force(self, goal_x, goal_y):
        #forces are calculated w.r.t world, not bot.
        #attractive force magnitude is kept = self.k_attraction.
        #And, x component and y component is calculated.

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = math.sqrt(dx ** 2 + dy ** 2)
        force_x = self.k_attraction * dx / distance # equal to cos(theta) * self.k_attraction
        force_y = self.k_attraction * dy / distance # equal to sin(theta) * self.k_attraction
        return force_x, force_y

    def compute_repulsive_force(self):
        force_x, force_y = 0.0, 0.0
        
        angle_increment = math.radians(self.lidar_theta / (len(self.lidar_data.ranges) - 1))
        angle = -math.radians(self.lidar_theta/2.0) #start from -135 and go to +135
        
        for distance in self.lidar_data.ranges:
            if distance < self.repulsion_range:
                repulsion_strength = self.k_repulsion * (
                    (1 / distance) - (1 / self.repulsion_range)
                    )
                force_x = force_x + repulsion_strength * math.cos(angle + self.current_yaw)
                force_y = force_y + repulsion_strength * math.sin(angle + self.current_yaw)
            angle += angle_increment
        
        #as it is repulsive, return negative of x and y components.
        return -force_x, -force_y
    
    def calculate_potential_forces(self, goal_x, goal_y):
        attractive_force_x, attractive_force_y = self.compute_attractive_force(goal_x, goal_y)
        repulsive_force_x, repulsive_force_y = self.compute_repulsive_force()
        
        # Combining forces
        net_force_x = attractive_force_x + repulsive_force_x
        net_force_y = attractive_force_y + repulsive_force_y
        angle = math.degrees(math.atan2(net_force_y, net_force_x))

        #print(f"x: {self.current_x} y: {self.current_y} attractive_force_x: {attractive_force_x} attractive_force_y: {attractive_force_y} repulsive_force_x: {repulsive_force_x} repulsive_force_y: {repulsive_force_y} net_force_x: {net_force_x} net_force_y: {net_force_y}")
        data = {
            'x': self.current_x,
            'y': self.current_y,
            'attractive_force_x': attractive_force_x,
            'attractive_force_y': attractive_force_y,
            'repulsive_force_x': repulsive_force_x,
            'repulsive_force_y': repulsive_force_y,
            'net_force_x': net_force_x,
            'net_force_y': net_force_y,
            'angle_dgrees': angle,
        }
        self.potential_data.append(data)

        return net_force_x, net_force_y

    def go_to_goal(self, goal_x, goal_y, with_back_and_forth):
        rate = rospy.Rate(10)  # 10 Hz update rate
        
        while not rospy.is_shutdown():
            #wait for model state to be available
            if self.model_state_available:
                pass
            else: #not yet available.
                continue
            
            # Calculate the distance and angle to the goal
            dx = goal_x - self.current_x
            dy = goal_y - self.current_y
            distance_to_goal = math.sqrt(dx**2 + dy**2)

            # Check if the robot is close to the goal
            if distance_to_goal < self.DISTANCE_THRESHOLD:
                # Stop the robot
                self.rear_left_speed_command_pub.publish(0.0)
                self.rear_right_speed_command_pub.publish(0.0)

                #save data for analysis
                df = pd.DataFrame(self.potential_data, columns=['x', 'y', 'attractive_force_x', 'attractive_force_y', 'repulsive_force_x', 'repulsive_force_y', 'net_force_x', 'net_force_y', 'angle_dgrees'])
                df.to_excel('~/catkin_ws/src/mobile_manipulator_body/output.xlsx', index=False)
                
                x = df['x']
                y = df['y']
                attractive_force_x = df['attractive_force_x']
                attractive_force_y = df['attractive_force_y']
                repulsive_force_x = df['repulsive_force_x']
                repulsive_force_y = df['repulsive_force_x']
                
                fig = plt.figure()
                ax = fig.add_subplot(111, projection='3d')

                # Plot x, y as base coordinates, and x_force as the Z-axis
                ax.scatter(x, y, attractive_force_x, c=attractive_force_x, cmap='viridis', marker='o')

                # Labels and title
                ax.set_xlabel('X Coordinate')
                ax.set_ylabel('Y Coordinate')
                ax.set_zlabel('X Force')
                ax.set_title('3D Plot of X Force')

                plt.show()
                     
                break  # Goal reached

            # Publish marker on rviz
            self.generate_current_location_markers_for_rviz()
            self.marker_pub.publish(self.marker)

            # Calculate the desired heading to the goal
            net_force_x, net_force_y = self.calculate_potential_forces(goal_x, goal_y)
            desired_angle = math.atan2(net_force_y , net_force_x)

            # Calculate the steering angle to turn towards the goal (shorttest angular dist.)
            angle_diff = -self.shortest_angular_distance(self.current_yaw, desired_angle)
            
            # Apply steering command based on angle difference
            steering_command = self.STEERING_GAIN * angle_diff

            # apply turning limits
            if steering_command > self.MAX_STEERING_ANGEL: steering_command = self.MAX_STEERING_ANGEL
            if steering_command < -self.MAX_STEERING_ANGEL: steering_command = -self.MAX_STEERING_ANGEL

            # Get the speed at which vehicle should travel.
            # The speed should be low when turning.           

            #wheel_speed = self.MAX_WHEEL_SPEED * max(0, 1 - abs(steering_command)) # rad/sec
            wheel_speed = self.MAX_WHEEL_SPEED * math.exp(-abs(3.5*steering_command)) # rad/sec
            vehicle_speed_x = wheel_speed * self.WHEELRADIUS

            # Get the differental speeds for the two rear wheels. Moving frontward
            left_wheel_speed, right_wheel_speed = self.get_rear_differential_angular_speeds(steering_command, vehicle_speed_x, True)

            # Publish the steering commands to the front wheels
            self.front_left_steering_pub.publish(steering_command)
            self.front_right_steering_pub.publish(steering_command)
            
            # Publish the speed commands to the rear wheels
            self.rear_left_speed_command_pub.publish(left_wheel_speed)
            self.rear_right_speed_command_pub.publish(right_wheel_speed)
            
            # handling sharp turn, go back a bit to turn sharply
            if (abs(steering_command) > 15.0 * math.pi/180.0) and (with_back_and_forth == True):
                left_wheel_speed_backward, right_wheel_speed_backward = self.get_rear_differential_angular_speeds(-steering_command, vehicle_speed_x, False)
                rospy.sleep(2)
                self.front_left_steering_pub.publish(-steering_command)
                self.front_right_steering_pub.publish(-steering_command)
                rospy.sleep(1)
                self.rear_left_speed_command_pub.publish(left_wheel_speed_backward * 2) #factor of 2 to go back a bit fast
                self.rear_right_speed_command_pub.publish(right_wheel_speed_backward * 2) #factor of 2 to go back a bit fast
                rospy.sleep(2)

            rate.sleep()

if __name__ == '__main__':
    try:
        follower = WaypointFollower()
        follower.spin()
        
    except rospy.ROSInterruptException:
        pass

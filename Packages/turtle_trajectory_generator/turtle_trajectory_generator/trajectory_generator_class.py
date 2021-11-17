# ROS module
import rclpy
from rclpy.node import Node

# ROS geometry message module
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

# Custom message module
from turtle_msgs.msg import TurtleStateStamped
from turtle_msgs.srv import SetDesiredPose

# Numpy!
import numpy as np

# Mutex
from threading import Lock

# This class provides methods to transform turtle velocities (Twist) to commanded turtle pose
class Twists2CMD(Node):
    def __init__(self):
        super().__init__('twist_2_cmd_converter')

        # Subscriber to receive the turtle velocity
        self.subscription = self.create_subscription(
            Twist,
            'turtle_twist',
            self.listener_callback,
            10)
        # Publisher to publish the commanded turtle pose
        self.publisher_ = self.create_publisher(
            TurtleStateStamped, 'turtle_cmd', 10)
        # TurtleState message
        self.init_pose = TurtleStateStamped()
        self.init_pose.pose.x = 0.0
        self.init_pose.pose.y = 0.0
        self.init_pose.pose.theta = 0.0
        
        # Flag to control when we have received a velocity message
        self.first_message = False

        # Velocity message
        self.twist_data = Twist()

        # Mutex to protect the shared variables
        self.mutex = Lock()

        # Sample period for the main thread
        self.timer_period = 0.01

        # Main thread to continuously publish the commanded turtle pose
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # Callback function for the subscriber. To receive the desired turtle velocity 
    def listener_callback(self, msg):
        # self.get_logger().info('I heard vx:' + str(msg.linear.x) +
        #                        ", vy: " + str(msg.linear.y) + ", wz: " + str(msg.angular.z))

        # the twist message will be shared between the topic thread and the main thread
        # We need to protect the reading/writing
        self.mutex.acquire()
        self.twist_data = msg
        self.mutex.release()

        # Get current time
        # c_time = self.get_clock().now().nanoseconds
        if not self.first_message:
            self.first_message = True
            # self.c_ini = c_time

        # Compute elapsed time (not used)
        # delta_time = (c_time - self.c_ini)*1e-9
        # self.c_ini = c_time

        # print("----------------------------------")
        # self.get_logger().info("Slept for " + str(delta_time) + " secs")

    # Main thread callback function (Timer Callback)
    def timer_callback(self):
        if self.first_message:
            # We protect the shared variable
            self.mutex.acquire()
            self.init_pose.vel = self.twist_data
            self.mutex.release()

            # Transform the desired Turtle velocity into commanded turtle pose
            # The controller receives commanded positions and not velocities
            self.init_pose.pose.x += self.init_pose.vel.linear.x * self.timer_period
            self.init_pose.pose.y += self.init_pose.vel.linear.y * self.timer_period
            self.init_pose.pose.theta += self.init_pose.vel.angular.z * self.timer_period

            # self.get_logger().info("Data Vel: " + str(self.init_pose.vel.linear.x) + ", " +
            #                        str(self.init_pose.vel.linear.y) + ", " + str(self.init_pose.vel.angular.z))
            # self.get_logger().info("Data Pose: " + str(self.init_pose.pose.x) + ", " +
            #                        str(self.init_pose.pose.y) + ", " + str(self.init_pose.pose.theta))

            # Populate the message to publish the commanded turtle state
            msg = TurtleStateStamped()
            msg.header._frame_id = "/world"
            msg.header._stamp = self.get_clock().now().to_msg()
            msg.pose.x = self.init_pose.pose.x
            msg.pose.y = self.init_pose.pose.y
            msg.pose.theta = self.init_pose.pose.theta
            msg.vel = self.init_pose.vel

            # Publish the commanded turtle pose message
            self.publisher_.publish(msg)

# This class provides methods to generate a smooth trajectory from the current turtle pose to a desired turtle pose
# This smooth trajectory will generate the commanded turtle state for the controller
class PolyTrajectory(Node):
    def __init__(self):
        super().__init__('poly_trajectory')

        # Service to define the desired turtle pose
        self.srv = super().create_service(
            SetDesiredPose, 'set_desired_pose', self.set_desired_pose_callback)

        #  2D pose message
        self.turtle_d = Pose2D()

        # Publisher to publish the commanded turtle pose
        self.publisher_ = self.create_publisher(
            TurtleStateStamped, 'turtle_cmd', 10)
        
        # Convergence time (by default set to 10 sec). How much time will take the turtle to go from the current pose
        # to the desired pose.
        # This time will be changed in the service request
        self.t_time = 10.0

        self.got_request = False

        # period for the main thread
        self.timer_period = 0.01

        # Main thread. Computes the smooth trajectory and publish the result as a commanded turtle pose
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Flag to reset the internal clock for the trajectory generator
        self.reset_clock = True
        self.elapsed_time = 0.0

        # TurtleState message
        self.init_pose = TurtleStateStamped()

        # Initial turtle pose. It will be updated with the current turtle pose
        self.init_pose.pose.x = 0.0
        self.init_pose.pose.y = 0.0
        self.init_pose.pose.theta = 0.0

    # Callback function for the service. The user requests the desired turtle pose
    def set_desired_pose_callback(self, request, response):
        self.turtle_d.x = request.turtle_d.x
        self.turtle_d.y = request.turtle_d.y
        self.turtle_d.theta = request.turtle_d.theta
        self.t_time = request.time

        # Flag to trigger the trajectory generator
        self.got_request = True

        # print out the requested data
        self.get_logger().info('Processing request: %f, %f, %f, in %f' % (
                               self.turtle_d.x, self.turtle_d.y, self.turtle_d.theta, self.t_time))

        # Response for the client
        response.confirmation = True

        return response

    # Main thread callback function
    def timer_callback(self):

        # Turtle Sate message to publish the commanded turtle state
        msg = TurtleStateStamped()

        # Define the time stamp for the message
        msg.header._frame_id = "/world"
        msg.header._stamp = self.get_clock().now().to_msg()

        # Only when the service has been triggered, we start to generate a new trajectory
        if self.got_request:
            
            # The trajectory generator uses as initial time t0=0, and final time the requested time from the service
            if self.reset_clock:
                self.elapsed_time = 0.0
                self.reset_clock = False
                self.get_logger().info('Begin turtle_ini: %f, %f, %f, in %f' % (
                    self.init_pose.pose.x, self.init_pose.pose.y, self.init_pose.pose.theta, self.elapsed_time))

            # Calculate a spline trajectory between the initial position to the desired position
            # This spline has to be calculated for each DOF (x,y, theta)
            msg.pose.x = self.get_spline_5(self.init_pose.pose.x,
                                           self.turtle_d.x, self.elapsed_time, self.t_time)
            msg.pose.y = self.get_spline_5(self.init_pose.pose.y,
                                           self.turtle_d.y, self.elapsed_time, self.t_time)
            msg.pose.theta = self.get_spline_5(self.init_pose.pose.theta,
                                               self.turtle_d.theta, self.elapsed_time, self.t_time)

            # Plot the generated commanded turtle pose
            self.get_logger().info('turtle_cmd: %f, %f, %f, in %f' % (
                msg.pose.x, msg.pose.y, msg.pose.theta, self.elapsed_time))

            # self.publisher_.publish(msg)

            # Calculate the elapsed time
            self.elapsed_time += self.timer_period

            # if the elapsed time is bigger than the requested time, then we stop the spline in the last position
            if self.elapsed_time > self.t_time:
                self.got_request = False
                self.reset_clock = True
                self.init_pose.pose = msg.pose
                self.get_logger().info('END turtle_ini: %f, %f, %f, in %f' % (
                    self.init_pose.pose.x, self.init_pose.pose.y, self.init_pose.pose.theta, self.elapsed_time))
        else:
            # We need to continuosly publish a commanded pose, then if there's no requested pose, we just publish
            # the current turtle pose
            msg.pose = self.init_pose.pose

        # Publish the commanded pose for the controller
        self.publisher_.publish(msg)

    # Simple 5th grade polynomial function for point to point motion
    def get_spline_5(self, start, goal, elapsed_time, total_time):

        if elapsed_time > total_time:
            elapsed_time = total_time
        elif elapsed_time < 0.0:
            elapsed_time = 0.0

        r = elapsed_time/total_time
        r2 = r*r
        r3 = r2*r
        r4 = r3*r
        r5 = r4*r
        return (10*r3 - 15*r4 + 6*r5)*(goal-start) + start

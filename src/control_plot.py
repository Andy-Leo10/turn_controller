#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

# Global variable to store the x position
yaw_ang = []

# This class is a ROS2 node that subscribes to the '/odometry/filtered' topic
class OdomSubscriber(Node):
    def __init__(self):
        # Initialize the node with the name 'odom_subscriber'
        super().__init__('odom_subscriber')
        # Create a subscription to the '/odometry/filtered' topic
        # The callback function is called whenever a new message is published on this topic
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.callback,
            10)

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    # This function is called whenever a new Odometry message is published on the '/odometry/filtered' topic
    def callback(self, msg):
        global yaw_ang
        # Convert the quaternion to euler angles
        roll, pitch, yaw = self.euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        # Append the yaw angle to the list
        yaw_ang.append(yaw*180/3.14159)
        # Keep only the last 100 x positions
        yaw_ang = yaw_ang[-200:]
        # Log the yaw angle
        # self.get_logger().info('x: %f' % yaw)

# This function is called every 1000ms to update the plot
def animate(i):
    # Clear the previous plot
    ax1.clear()
    # Plot the x positions
    ax1.plot(yaw_ang)
    
    # Set the limits for Y-axis
    ax1.set_ylim(-181.0, 180.0)
    # add a grid to the plot
    ax1.grid(True)
    # Add a title to the plot
    ax1.set_title('Odometry visualizer')

def main(args=None):
    # Initialize the ROS2 library
    rclpy.init(args=args)
    # Create an instance of the OdomSubscriber node
    odom_subscriber = OdomSubscriber()
    # Spin the node so it can process callbacks
    rclpy.spin(odom_subscriber)
    # Destroy the node when it's done
    odom_subscriber.destroy_node()
    # Shutdown the ROS2 library
    rclpy.shutdown()

if __name__ == '__main__':
    # Create a figure for the plot
    fig = plt.figure()
    # Add a subplot to the figure
    ax1 = fig.add_subplot(1,1,1)
    
    # Create an animation that calls the 'animate' function every 1000ms
    ani = animation.FuncAnimation(fig, animate, interval=1000)

    # Start the ROS2 node in a separate thread
    thread = threading.Thread(target=main)
    thread.start()

    # Show the plot
    plt.show()
# import math

# import rclpy
# from geometry_msgs.msg import Pose, TransformStamped
# from rclpy.node import Node
# from messages.msg import Encoder  # noqa: F401
# from nav_msgs.msg import Odometry as Odom
# from rclpy.publisher import Publisher
# from rclpy.subscription import Subscription
# from .dataclasses.robot import Robot
# from tf2_ros import TransformBroadcaster
# from tf_transformations import quaternion_from_euler


# class Odometry(Node):
#     def __init__(self):
#         super().__init__('odometry')

#         self.get_logger().info("Initializing Odometry")

#         # Pose
#         self.x: float = 0.0
#         self.y: float = 0.0
#         self.yaw: float = 0.0

#         # Save time of last update for delta time calculation
#         self.last_time = self.get_clock().now()

#         # /encoder subscription
#         self.get_logger().info("Creating subscription on /encoder topic")
#         self.subscription_sub: Subscription = self.create_subscription(Encoder, '/encoder', self.calculate_odom, 10)

#         # /odom publisher
#         self.get_logger().info("Creating publisher on /odom topic")
#         self.odom_pub: Publisher = self.create_publisher(Odom, 'odom', 10)

#         # /tf publisher
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Robot details
#         self.get_logger().info("Getting robot details")
#         self.robot = Robot()

#     def calculate_odom(self, encoder: Encoder) -> None:
#         """
#         Calculate the odometry of the robot based on the new encoder data.
#         :param encoder: new encoder data.
#         """

#         # Compute delta time since last update
#         now = self.get_clock().now()
#         dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
#         if dt <= 0:
#             return
#         self.last_time = now

#         # Update pulses per wheel
#         self.robot.front_left.pulses = encoder.front_left
#         self.robot.front_right.pulses = encoder.front_right
#         self.robot.rear_left.pulses = encoder.rear_left
#         self.robot.rear_right.pulses = encoder.rear_right

#         # Calculate velocities
#         vel_x = self.robot.get_forward_velocity()
#         vel_y = self.robot.get_right_velocity()
#         vel_yaw = self.robot.get_angular_velocity()

#         # Update X, Y and Yaw
#         self.x += vel_x * math.cos(self.yaw) * dt - vel_y * math.sin(self.yaw) * dt
#         self.y += vel_x * math.sin(self.yaw) * dt + vel_y * math.cos(self.yaw) * dt
#         self.yaw += vel_yaw * dt

#         # Normalize yaw to [-pi, pi]
#         while self.yaw > math.pi:
#             self.yaw -= 2 * math.pi
#         while self.yaw < -math.pi:
#             self.yaw += 2 * math.pi

#         # Publish results
#         self.publish_odom(vel_x, vel_y, vel_yaw)
#         self.publish_tf()


#     def publish_odom(self, vel_x, vel_y, vel_yaw) -> None:
#         """
#         Publishes the odometry of the robot to /odom
#         :param vel_x: Linear velocity on the X axis
#         :param vel_y: Linear velocity on the Y axis
#         :param vel_yaw: Angular velocity on the Z axis
#         """
#         q = quaternion_from_euler(0.0, 0.0, self.yaw)
#         odom = Odom()

#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = 'odom'
#         odom.child_frame_id = 'base_link'

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation.x = q[0]
#         odom.pose.pose.orientation.y = q[1]
#         odom.pose.pose.orientation.z = q[2]
#         odom.pose.pose.orientation.w = q[3]

#         odom.twist.twist.linear.x = vel_x
#         odom.twist.twist.linear.y = vel_y
#         odom.twist.twist.angular.z = vel_yaw

#         self.odom_pub.publish(odom)

#     def publish_tf(self) -> None:
#         """
#         Publishes the transformation matrix to /tf
#         """

#         q = quaternion_from_euler(0.0, 0.0, self.yaw)
#         tf = TransformStamped()

#         tf.header.stamp = self.get_clock().now().to_msg()
#         tf.header.frame_id = 'odom'
#         tf.child_frame_id = 'base_link'

#         tf.transform.translation.x = self.x
#         tf.transform.translation.y = self.y
#         tf.transform.translation.z = 0.0

#         tf.transform.rotation.x = q[0]
#         tf.transform.rotation.y = q[1]
#         tf.transform.rotation.z = q[2]
#         tf.transform.rotation.w = q[3]

#         self.tf_broadcaster.sendTransform(tf)


# def main():
#     rclpy.init()
#     node = Odometry()

#     rclpy.spin(node)

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
#from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from geometry_msgs.msg import Twist
from math import sin, cos, pi
import numpy as np

# This is the node that calculate the position of the robot (odometry) by integrating the linear velocities.
# The TF2 is broadcasted using TransformBroadcaster from tf2_ros

# DO WE NEED JOINTSTATE PUBLISHER ????????????????????????????????????????



class OdomNode(Node):
    def __init__(self):
        super().__init__('odometry')
        # Pub, sub, bc
        self.speedSub = self.create_subscription(Twist, '/fb_speed', self.fb_speed_callback, 50)
        self.odomPub = self.create_publisher(Odometry, '/odom', 50)
        self.tf_broadcast = TransformBroadcaster(self)
        self.timer1 = self.create_timer(0.1, self.odom_update)

        self.last_time = self.get_clock().now()

        # vars
        self.x = 0.0 #dx
        self.y = 0.0 # dy
        self.theta = 0.0 
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.omega_z = 0.0
        self.get_logger().info("Odometry node running (and broadcast TF)")
        #CORRECTION PARAMETERS!

    def fb_speed_callback(self, msg: Twist):
        self.vel_x = msg.linear.x 
        self.vel_y = msg.linear.y 
        self.omega_z = msg.angular.z 


    def odom_update(self):
        # dt = 0.1 #Time dt

        # compute dt based on actual time since last update
        now = self.get_clock().now()
        dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
        # If dt is zero or too large, skip
        if dt <= 0.0 or dt > 1.0:
            self.last_time = now
            return
        self.last_time = now

        delta_x = (self.vel_x * cos(self.theta) - self.vel_y * sin(self.theta)) * dt
        delta_y = (self.vel_y * cos(self.theta) + self.vel_x * sin(self.theta)) * dt
        delta_theta = self.omega_z * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
        # wrap angle (?)
        self.theta = (self.theta + pi) % (2*pi) - pi
        # self.theta = np.mod((self.theta+delta_theta), 2*pi)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        odom_msg.pose.pose.orientation.z = sin(self.theta/2.0)
        odom_msg.pose.pose.orientation.w = cos(self.theta/2.0)

        odom_msg.twist.twist.linear.x = self.vel_x
        odom_msg.twist.twist.linear.y = self.vel_y

       # print("Publishing odom:", self.x, self.y, self.theta)
       #Message Filter dropping message: frame 'laser' at time 1760095760.711 for reason 'discarding message because the queue is full'
        # Broadcast TF2:
        self.odomPub.publish(odom_msg)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        # Check this !!!
        transform.transform.rotation.z = sin(self.theta / 2.0)
        transform.transform.rotation.w = cos(self.theta / 2.0)

        self.tf_broadcast.sendTransform(transform)

""" 
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)
"""



def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdomNode()
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()







# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped, Twist
# from tf2_ros import TransformBroadcaster
# import math
# import numpy as np

# def yaw_to_quaternion(yaw: float):
#     """Return quaternion (x,y,z,w) for rotation about Z only."""
#     return (0.0, 0.0, math.sin(yaw/2.0), math.cos(yaw/2.0))


# class MecanumOdometry(Node):
#     def __init__(self):
#         super().__init__('mecanum_odometry')

#         self.speedSub = self.create_subscription(Twist, '/fb_speed', self.fb_speed_callback, 50)
#         self.odomPub = self.create_publisher(Odometry, '/odom', 50)
#         self.tf_broadcast = TransformBroadcaster(self)
#         self.timer1 = self.create_timer(0.1, self.odom_update)

#         self.declare_parameter('wheel_radius', 0.033)
#         self.declare_parameter('wheel_separation_x', 0.375)
#         self.declare_parameter('wheel_separation_y', 0.290)

#         self.r = float(self.get_parameter('wheel_radius').value)
#         self.Lx = float(self.get_parameter('wheel_separation_x').value) / 2.0
#         self.Ly = float(self.get_parameter('wheel_separation_y').value) / 2.0
#         self.L = self.Lx + self.Ly

#         # State
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.last_time = self.get_clock().now()

#         # Publisher / TF
#         self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Subscribe: expect serial_driver to publish joint_states with names for the wheels
#         self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

#         self.get_logger().info("Mecanum Odometry node started")

#     def joint_callback(self, msg: JointState):
#         # Map wheel names -> indices so we can accept any order
#         name_to_idx = {name: i for i, name in enumerate(msg.name)}

#         # expected keys (change these to the names you publish in serial_driver)
#         expected = ['front_left', 'front_right', 'rear_left', 'rear_right']

#         try:
#             idx_fl = name_to_idx['front_left']
#             idx_fr = name_to_idx['front_right']
#             idx_rl = name_to_idx['rear_left']
#             idx_rr = name_to_idx['rear_right']
#         except KeyError:
#             # If exact names are different, try common alternatives
#             self.get_logger().warn("JointState does not contain expected wheel names. Received names: " + ", ".join(msg.name))
#             # Fallback: if velocities length == 4, assume order matches [fl, fr, rl, rr]
#             if len(msg.velocity) == 4:
#                 w_fl, w_fr, w_rl, w_rr = msg.velocity
#             else:
#                 return
#         else:
#             # get velocities safely (length guard)
#             vel = msg.velocity
#             if max(idx_fl, idx_fr, idx_rl, idx_rr) >= len(vel):
#                 self.get_logger().warn("JointState velocities array too short")
#                 return
#             w_fl = vel[idx_fl]
#             w_fr = vel[idx_fr]
#             w_rl = vel[idx_rl]
#             w_rr = vel[idx_rr]

#         now = self.get_clock().now()
#         dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
#         if dt <= 0.0:
#             return
#         self.last_time = now

#         # forward kinematics -> body velocities
#         vx = (self.r / 4.0) * (w_fl + w_fr + w_rl + w_rr)
#         vy = (self.r / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
#         wz = (self.r / (4.0 * self.L)) * (-w_fl + w_fr - w_rl + w_rr)

#         # integrate to global frame
#         self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
#         self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
#         self.yaw = math.atan2(math.sin(self.yaw + wz * dt), math.cos(self.yaw + wz * dt))

#         self.publish_odom(vx, vy, wz)

#     def publish_odom(self, vx, vy, wz):
#         qx, qy, qz, qw = yaw_to_quaternion(self.yaw)

#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation.x = qx
#         odom.pose.pose.orientation.y = qy
#         odom.pose.pose.orientation.z = qz
#         odom.pose.pose.orientation.w = qw

#         odom.twist.twist.linear.x = vx
#         odom.twist.twist.linear.y = vy
#         odom.twist.twist.angular.z = wz

#         self.odom_pub.publish(odom)

#         tf = TransformStamped()
#         tf.header.stamp = odom.header.stamp
#         tf.header.frame_id = "odom"
#         tf.child_frame_id = "base_link"
#         tf.transform.translation.x = self.x
#         tf.transform.translation.y = self.y
#         tf.transform.translation.z = 0.0
#         tf.transform.rotation.x = qx
#         tf.transform.rotation.y = qy
#         tf.transform.rotation.z = qz
#         tf.transform.rotation.w = qw
#         self.tf_broadcaster.sendTransform(tf)


# def main(args=None):
#     rclpy.init(args=args)
#     node = MecanumOdometry()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import JointState
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import TransformStamped
# from tf_transformations import quaternion_from_euler
# from tf2_ros import TransformBroadcaster
# import math


# class MecanumOdometry(Node):
#     def __init__(self):
#         super().__init__('mecanum_odometry')

#         self.declare_parameter('wheel_radius', 0.033)
#         self.declare_parameter('wheel_separation_x', 0.375)
#         self.declare_parameter('wheel_separation_y', 0.290)

#         self.r = self.get_parameter('wheel_radius').value
#         self.Lx = self.get_parameter('wheel_separation_x').value / 2.0
#         self.Ly = self.get_parameter('wheel_separation_y').value / 2.0
#         self.L = self.Lx + self.Ly

#         # State
#         self.x = 0.0
#         self.y = 0.0
#         self.yaw = 0.0
#         self.last_time = self.get_clock().now()
#         # self.last_vels = [0.0, 0.0, 0.0, 0.0]

#         # Publishers
#         self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
#         self.tf_broadcaster = TransformBroadcaster(self)

#         # Subscriber
#         self.create_subscription(JointState, 'joint_states', self.joint_callback, 10)

#         self.get_logger().info("Mecanum Odometry node started")

#     def joint_callback(self, msg: JointState):
#         if len(msg.velocity) != 4:
#             return

#         now = self.get_clock().now()
#         dt = (now.nanoseconds - self.last_time.nanoseconds) / 1e9
#         if dt <= 0.0:
#             return
#         self.last_time = now

#         w_fl, w_fr, w_rl, w_rr = msg.velocity

#         # Kinematics: compute robot motion in body frame
#         vx = (self.r / 4.0) * (w_fl + w_fr + w_rl + w_rr)
#         vy = (self.r / 4.0) * (-w_fl + w_fr + w_rl - w_rr)
#         wz = (self.r / (4.0 * self.L)) * (-w_fl + w_fr - w_rl + w_rr)

#         # Integrate motion to global frame
#         self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
#         self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
#         self.yaw += wz * dt

#         # Normalize yaw
#         self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

#         self.publish_odom(vx, vy, wz)

#     def publish_odom(self, vx, vy, wz):
#         q = quaternion_from_euler(0.0, 0.0, self.yaw)

#         odom = Odometry()
#         odom.header.stamp = self.get_clock().now().to_msg()
#         odom.header.frame_id = "odom"
#         odom.child_frame_id = "base_link"

#         odom.pose.pose.position.x = self.x
#         odom.pose.pose.position.y = self.y
#         odom.pose.pose.orientation.x = q[0]
#         odom.pose.pose.orientation.y = q[1]
#         odom.pose.pose.orientation.z = q[2]
#         odom.pose.pose.orientation.w = q[3]

#         odom.twist.twist.linear.x = vx
#         odom.twist.twist.linear.y = vy
#         odom.twist.twist.angular.z = wz

#         self.odom_pub.publish(odom)

#         tf = TransformStamped()
#         tf.header.stamp = odom.header.stamp
#         tf.header.frame_id = "odom"
#         tf.child_frame_id = "base_link"
#         tf.transform.translation.x = self.x
#         tf.transform.translation.y = self.y
#         tf.transform.rotation.x = q[0]
#         tf.transform.rotation.y = q[1]
#         tf.transform.rotation.z = q[2]
#         tf.transform.rotation.w = q[3]
#         self.tf_broadcaster.sendTransform(tf)


# def main(args=None):
#     rclpy.init(args=args)
#     node = MecanumOdometry()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

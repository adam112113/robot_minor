# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy
# from std_msgs.msg import Float32MultiArray
# import numpy as np
# import serial
# import time


# # ----------------------------
# # CONSTANTS
# # ----------------------------
# SERIAL_PORT = '/dev/ttyACM0'
# BAUD_RATE = 115200

# # Geometry (meters)
# RADIUS = (60 / 2) / 1000       # wheel radius = 30 mm = 0.03 m
# LX = (330 / 2) / 1000          # robot half length (x)
# LY = (260 / 2) / 1000          # robot half width (y)

# SEND_RATE_HZ = 20.0            # steady command rate to Arduino (Hz)
# SMOOTHING_ALPHA = 0.2          # smoothing of commanded velocity (0 = none, 1 = instant)


# class MotionController(Node):
#     def __init__(self):
#         super().__init__("motion_controller_node")

#         # ----------------------------
#         # SERIAL CONNECTION
#         # ----------------------------
#         try:
#             self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
#             self.get_logger().info(f"Opened serial port {SERIAL_PORT}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial port: {e}")
#             raise e

#         # ----------------------------
#         # ROS INTERFACES
#         # ----------------------------
#         self.motorPublisher = self.create_publisher(Float32MultiArray, '/controlspeed', 10)
#         self.joySubscriber = self.create_subscription(Joy, '/joy', self.joycallback, 10)
#         self.cmdSubscriber = self.create_subscription(Twist, '/cmd_vel', self.cmdcallback, 10)
#         self.feedbackPub = self.create_publisher(Twist, '/fb_speed', 10)

#         # ----------------------------
#         # TIMERS
#         # ----------------------------
#         self.send_timer = self.create_timer(1.0 / SEND_RATE_HZ, self.send_to_arduino)
#         self.read_timer = self.create_timer(0.05, self.read_serial_feedback)

#         # ----------------------------
#         # STATE VARIABLES
#         # ----------------------------
#         self.last_w_speeds = np.zeros((4, 1))
#         self.smooth_vel = np.zeros((3, 1))
#         self.invKinMatrix = np.array([
#             [1, -1, -(LX + LY)],
#             [1,  1,  (LX + LY)],
#             [1,  1, -(LX + LY)],
#             [1, -1,  (LX + LY)]
#         ])

#         self.get_logger().info("Motion controller node started and running.")


#     # ---------------------------------------------------
#     # CALLBACKS
#     # ---------------------------------------------------

#     def joycallback(self, msg: Joy):
#         """Called when joystick input arrives."""
#         x_vel = 0.8 * msg.axes[1]  # forward/backward
#         y_vel = 0.8 * msg.axes[0]  # strafe
#         w_z   = 2.0 * msg.axes[3]  # rotation

#         self.update_target_velocity(x_vel, y_vel, w_z)

#     def cmdcallback(self, msg: Twist):
#         """Called when Twist command arrives."""
#         self.update_target_velocity(msg.linear.x, msg.linear.y, msg.angular.z)

#     def update_target_velocity(self, x_vel, y_vel, w_z):
#         """Smooth the velocity command and compute wheel speeds."""
#         alpha = SMOOTHING_ALPHA
#         target_vel = np.array([[x_vel], [y_vel], [w_z]])
#         self.smooth_vel = alpha * target_vel + (1 - alpha) * self.smooth_vel

#         # Inverse kinematics
#         w_speeds = np.dot((self.invKinMatrix / RADIUS), self.smooth_vel)

#         # Limit to max 30 rad/s
#         maxOmega = np.max(np.abs(w_speeds))
#         if maxOmega > 30.0:
#             w_speeds *= 30.0 / maxOmega

#         self.last_w_speeds = w_speeds

#         # publish to ROS for monitoring (optional)
#         msg = Float32MultiArray()
#         msg.data = w_speeds.flatten().tolist()
#         self.motorPublisher.publish(msg)


#     # ---------------------------------------------------
#     # SERIAL COMMUNICATION
#     # ---------------------------------------------------

#     def send_to_arduino(self):
#         """Send wheel speeds at a constant 20 Hz."""
#         w = self.last_w_speeds
#         command = f"[{w[0,0]:.3f},{w[1,0]:.3f},{w[2,0]:.3f},{w[3,0]:.3f}]\n"
#         try:
#             self.ser.write(command.encode('utf-8'))
#             # Avoid spamming console – keep this debug only
#             self.get_logger().debug(f"Sent: {command.strip()}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to send command: {e}")


#     def read_serial_feedback(self):
#         """Read feedback from Arduino (rad/s from encoders)."""
#         try:
#             line = self.ser.readline().decode('utf-8').strip()
#             if not line or not line.startswith('['):
#                 return

#             parts = line.strip('[]').split(',')
#             if len(parts) != 4:
#                 return

#             raw_vals = [float(x) for x in parts]
#             fl, fr, rl, rr = raw_vals

#             # Convert wheel speeds → robot twist
#             msg = Twist()
#             msg.linear.x = (fl + fr + rl + rr) * (RADIUS / 4)
#             msg.linear.y = (-fl + fr + rl - rr) * (RADIUS / 4)
#             msg.angular.z = (-fl + fr - rl + rr) * (RADIUS / (4 * (LX + LY)))

#             self.feedbackPub.publish(msg)

#         except Exception as e:
#             self.get_logger().warn(f"Feedback read error: {e}")


#     # ---------------------------------------------------
#     # CLEANUP
#     # ---------------------------------------------------
#     def destroy_node(self):
#         try:
#             if self.ser.is_open:
#                 self.ser.close()
#         except Exception:
#             pass
#         super().destroy_node()



# def main(args=None):
#     rclpy.init(args=args)
#     node = MotionController()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
import numpy as np
import serial
import time

# Serial parameters
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 115200

# IN MM
RADIUS = (60/2)/1000
LX = (330/2)/1000
LY = (260/2)/1000

class MotionController(Node):
    def __init__(self):
        super().__init__("motion_controller_node")
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            self.get_logger().info(f"Opened serial port {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e
        self.motorPublisher = self.create_publisher(Float32MultiArray, '/controlspeed', 10)
        self.joySubscriber = self.create_subscription(Joy, '/joy', self.joycallback, 10)
        self.cmdSubscriber = self.create_subscription(Twist, '/cmd_vel', self.cmdcallback, 10)
        self.feedbackSub = self.create_subscription(Float32MultiArray, '/fb_rot', self.fbCallback, 10)
        self.feedbackPub = self.create_publisher(Twist, '/fb_speed', 10)
        # self.last_feedback_time = 0.0
        # self.feedback_interval = 0.01  # seconds (i.e. 10 Hz)
        # self.timer = self.create_timer(0.05, self.read_serial_feedback)
        self.serialRead = self.create_timer(0.1, self.read_serial_feedback)
        self.get_logger().info("Motion controller node has started!")

        # self.last_w_speeds = np.zeros((4,1))
        # self.send_timer = self.create_timer(0.05, self.send_to_arduino)  # 20 Hz steady

        self.feedbackMsg = Twist()
        self.invKinMatrix = np.array([
            [1,-1, -(LX+LY)],
            [1, 1,  (LX+LY)],
            [1, 1, -(LX+LY)],
            [1,-1,  (LX+LY)]
        ])
        
    def joycallback(self, msg: Joy):
        x_vel = 0.8 * msg.axes[1]  # m/s
        y_vel = 0.8 * msg.axes[0]
        w_z = 2 * msg.axes[3]      # rad/s
        smoothOpVel = np.array([[x_vel],[y_vel],[w_z]])
        w_speeds = self.inv_kin(smoothOpVel)
        self.publish_speeds(w_speeds)

    def cmdcallback(self, msg: Twist):
        x_vel = msg.linear.x
        y_vel = msg.linear.y
        w_z = msg.angular.z
        smoothOpVel = np.array([[x_vel],[y_vel],[w_z]])
        w_speeds = self.inv_kin(smoothOpVel)
        self.publish_speeds(w_speeds)

    def fbCallback(self, msg: Float32MultiArray):
        
        fl, fr, rl, rr = msg.data
        self.feedbackMsg.linear.x = (fl + fr + rl + rr) * (RADIUS / 4)
        self.feedbackMsg.linear.y = (-fl + fr + rl - rr) * (RADIUS / 4)
        self.feedbackMsg.angular.z = (-fl + fr - rl + rr) * (RADIUS / (4*(LX+LY)))
        self.feedbackPub.publish(self.feedbackMsg)

    def inv_kin(self, smoothOpVel):
        w_speeds = np.dot((self.invKinMatrix / RADIUS), smoothOpVel)
        maxOmega = np.max(np.abs(w_speeds))
        if maxOmega > 30:
            speedFactor = 30 / maxOmega
            w_speeds = w_speeds * speedFactor
        return w_speeds

    # def publish_speeds(self, w_speeds):
    #     # motor_msg = Float32MultiArray()
    #     # motor_msg.data = w_speeds.flatten().tolist()
    #     # self.motorPublisher.publish(motor_msg)

    #     self.last_w_speeds = w_speeds

    # def send_to_arduino(self):
    #     # Always send at 20 Hz from timer thread, independent of joystick rate
    #     command = f"[{self.last_w_speeds[0,0]:.2f},{self.last_w_speeds[1,0]:.2f},{self.last_w_speeds[2,0]:.2f},{self.last_w_speeds[3,0]:.2f}]\n"
    #     try:
    #         self.ser.write(command.encode('utf-8'))
    #     except Exception as e:
    #         self.get_logger().error(f"Serial write failed: {e}")

    def publish_speeds(self, w_speeds):
        # Publish to ROS topic
        motor_msg = Float32MultiArray()
        motor_msg.data = w_speeds.flatten().tolist()
        self.motorPublisher.publish(motor_msg)

        command = f"[{w_speeds[0,0]:.3f},{w_speeds[1,0]:.3f},{w_speeds[2,0]:.3f},{w_speeds[3,0]:.3f}]\n"

        # now = time.time()
        # if not hasattr(self, 'last_send_time'):
        #     self.last_send_time = 0.0

        # Only send every 50 ms (20 Hz)
        # if now - self.last_send_time >= 0.02:
        try:
            self.ser.write(command.encode('utf-8'))
            # self.ser.flush()
            # self.get_logger().info(f"Sent to Arduino: {command.strip()}")
        except Exception as e:
            self.get_logger().error(f"Failed to send command: {e}")
            # self.last_send_time = now
            
    def read_serial_feedback(self):
       # self.get_logger().info("Timer fired: checking serial buffer...")
        # current_time = time.time()
        # if current_time - self.last_feedback_time < self.feedback_interval:
        #     return  # Skip until interval elapsed
        # try:
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            # if not line.startswith('[') or line.endswith(']'):
            #     return
          #  self.get_logger().info(f"Sent to Arduino: {command.strip()}")
        
        #  self.get_logger().info(line)

        parts = line.strip('[]').split(',')
        if len(parts) == 4:
            #self.get_logger().info(f"Received from Arduino: {parts}")
            #self.get_logger().info(f"Parts length: {len(parts)}")
            # print(line)
            #float(line)
            # print(line)
            #if len(parts):
            # raw_vals = [float(x) for x in parts]
            # fl, fr, rl, rr = raw_vals
            fl, fr, rl, rr = [float(x) for x in parts]
            # self.get_logger().debug(f"RAW wheels: {raw_vals}, interpreted (rad/s): {[fl,fr,rl,rr]}")
            # inside read_serial_feedback after parsing fl,fr,rl,rr
            # self.get_logger().info(f"RAW wheels (from Arduino): fl={fl:.3f} fr={fr:.3f} rl={rl:.3f} rr={rr:.3f}")

            msg = Twist()
            msg.linear.x = (fl + fr + rl + rr) * (RADIUS / 4)
            msg.linear.y = (-fl + fr + rl - rr) * (RADIUS / 4)   
            msg.angular.z = (-fl + fr - rl + rr) * (RADIUS / (4*(LX+LY)))

            self.feedbackPub.publish(msg)
        # except Exception as e:
        #     self.get_logger().error(f"Failed to read command: {e}")
        #    # self.get_logger().info(f"Published /fb_speed: {msg.linear.x:.2f}, {msg.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    motion_controller = MotionController()
    try:
        rclpy.spin(motion_controller)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            motion_controller.ser.close()
        except Exception:
            pass
        motion_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import Joy
# from sensor_msgs.msg import JointState
# from std_msgs.msg import Float32MultiArray
# import serial
# import math
# import threading
# import time

# class MecanumSerialDriver(Node):
#     """
#     Subscribes to /cmd_vel (geometry_msgs/Twist), computes mecanum wheel speeds,
#     sends encoded command to Arduino via serial, reads responses and publishes
#     wheel velocities on /joint_states (sensor_msgs/JointState).

#     Encoding format used (by default):
#       Sent:  <FL,FR,RL,RR>\n   (integers, RPM)
#       Received (Arduino -> Pi): <FL,FR,RL,RR>\n  (integers, RPM)
#     Framing with '<' and '>' helps keep parsing robust.
#     """

#     def __init__(self):
#         super().__init__('mecanum_serial_driver')

#         # parameters (can be set from launch)
#         self.declare_parameter('port', '/dev/ttyACM0')
#         self.declare_parameter('baudrate', 115200)
#         self.declare_parameter('wheel_radius', 0.033)  # meters
#         self.declare_parameter('wheel_separation_x', 0.375)  # front-back distance (m)
#         self.declare_parameter('wheel_separation_y', 0.290)  # left-right distance (m)
#         self.declare_parameter('max_rpm', 10)  # optional limiting
#         self.declare_parameter('send_rate_hz', 20.0)  # how many cmd messages/sec max

#         self.port = self.get_parameter('port').get_parameter_value().string_value
#         self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value
#         self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
#         # We will interpret wheel_separation_x,y as half-distances or full? Use half-distances for typical formulas:
#         self.Lx = self.get_parameter('wheel_separation_x').get_parameter_value().double_value / 2.0
#         self.Ly = self.get_parameter('wheel_separation_y').get_parameter_value().double_value / 2.0
#         self.max_rpm = self.get_parameter('max_rpm').get_parameter_value().integer_value
#         self.send_rate_hz = self.get_parameter('send_rate_hz').get_parameter_value().double_value

#         # Combined term used in inverse kinematics: (Lx + Ly)
#         self.L = self.Lx + self.Ly

#         self.get_logger().info(f"Params: port={self.port} baud={self.baud} r={self.r:.3f} Lx={self.Lx:.3f} Ly={self.Ly:.3f}")

#         # Serial port
#         try:
#             self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
#             self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
#         except Exception as e:
#             self.get_logger().error(f"Failed to open serial {self.port}: {e}")
#             self.ser = None

#         self.motorPublisher = self.create_publisher(Float32MultiArray, '/controlspeed', 10)
#         self.joySubscriber = self.create_subscription(Joy, '/joy', self.joycallback, 10)
#         self.cmdSubscriber = self.create_subscription(Twist, '/cmd_vel', self.cmdcallback, 10)
#         self.feedbackSub = self.create_subscription(Float32MultiArray, '/fb_rot', self.fbCallback, 10)
#         self.feedbackPub = self.create_publisher(Twist, '/fb_speed', 10)
#         self.serialRead = self.create_timer(0.1, self.read_serial_feedback)
#         # Subscriber to /cmd_vel
#         self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

#         # Publisher for wheel states
#         self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

#         # Keep most recent cmd and rate-limit sends
#         self._last_cmd = None
#         self._last_sent = 0.0
#         self._lock = threading.Lock()

#         # Buffer for serial reading
#         self._rx_buffer = ""

#         # Start a thread to continuously read serial and publish feedback

#         self.timer = self.create_timer(0.1, self._serial_reader_thread)

#         # self._reader_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
#         # self._reader_thread.start()

#         self.get_logger().info("Mecanum serial driver ready")

#     def cmd_vel_callback(self, msg: Twist):
#         # get velocities from twist
#         vx = float(msg.linear.x)
#         vy = float(msg.linear.y)
#         wz = float(msg.angular.z)

#         # save last command
#         with self._lock:
#             self._last_cmd = (vx, vy, wz)

#         # call send (rate-limited)
#         now = time.time()
#         if now - self._last_sent >= (1.0 / self.send_rate_hz):
#             self._send_last_cmd()
#             self._last_sent = now

#     def _send_last_cmd(self):
#         with self._lock:
#             cmd = self._last_cmd
#         if cmd is None:
#             return
#         vx, vy, wz = cmd

#         # inverse kinematics for mecanum:
#         # wheel angular velocity w = (1/r) * (vx +/- vy +/- L*wz)
#         w_fl = (1.0 / self.r) * (vx - vy - self.L * wz)  # M1 (Front Left)
#         w_fr = (1.0 / self.r) * (vx + vy + self.L * wz)  # M2 (Front Right)
#         w_rl = (1.0 / self.r) * (vx + vy - self.L * wz)  # M3 (Rear Left)
#         w_rr = (1.0 / self.r) * (vx - vy + self.L * wz)  # M4 (Rear Right)

#         # Convert rad/s -> RPM (integer)
#         rpm_fl = int(self._clamp_rpm(w_fl))
#         rpm_fr = int(self._clamp_rpm(w_fr))
#         rpm_rl = int(self._clamp_rpm(w_rl))
#         rpm_rr = int(self._clamp_rpm(w_rr))

#         # Build framed message: <FL,FR,RL,RR>\n
#         # msg = f"<{rpm_fl},{rpm_fr},{rpm_rl},{rpm_rr}>\n"
#         msg = f"<{w_fl},{w_fr},{w_rl},{w_rr}>\n"

#         motor_msg = Float32MultiArray()
#         motor_msg.data = w_speeds.flatten().tolist()

#         self.motorPublisher.publish(motor_msg)

#         if self.ser:
#             try:
#                 self.ser.write(msg.encode('utf-8'))
#                 self.get_logger().debug(f"TX: {msg.strip()}")
#             except Exception as e:
#                 self.get_logger().error(f"Serial write error: {e}")

#     def _clamp_rpm(self, w_rad_s):
#         """Convert rad/s to RPM and clamp to +/- max_rpm"""
#         rpm = w_rad_s * 60.0 / (2.0 * math.pi)
#         if rpm > self.max_rpm:
#             return self.max_rpm
#         if rpm < -self.max_rpm:
#             return -self.max_rpm
#         return rpm

#     def _serial_reader_thread(self):
#         """
#         Continuously read from serial, accumulate until newline, parse framed messages.
#         Expected reply format from Arduino: <FL,FR,RL,RR>\n   (integers RPM)
#         On parse, publish sensor_msgs/JointState
#         """
#         # joint names consistent with your URDF
#         joint_names = ['front_left_wheel_joint', 'front_right_wheel_joint',
#                        'rear_left_wheel_joint', 'rear_right_wheel_joint']
        

#         if not self.ser:
#             # no serial connection
#             return

#         try:
#             # read whatever is available, non-blocking
#             data = self.ser.read(256)  
#             if not data:
#                 return  # nothing this cycle

#             try:
#                 chunk = data.decode('utf-8', errors='ignore')
#             except Exception:
#                 chunk = ''
#             if not chunk:
#                 return

#             # append to RX buffer
#             self._rx_buffer += chunk

#             # process complete framed messages: they look like <...>\n
#             while True:
#                 start = self._rx_buffer.find('<')
#                 end = self._rx_buffer.find('>', start + 1)
#                 if start == -1 or end == -1:
#                     break  # wait for complete frame

#                 raw = self._rx_buffer[start+1:end]
#                 self._rx_buffer = self._rx_buffer[end+1:]  # drop processed part

#                 # parse CSV of four ints
#                 parts = [p.strip() for p in raw.split(',') if p.strip()]
#                 if len(parts) != 4:
#                     self.get_logger().warning(f"Bad feedback format: '{raw}'")
#                     continue

#                 try:
#                     rpm_vals = [int(p) for p in parts]
#                 except ValueError:
#                     self.get_logger().warning(f"Non-int feedback: '{raw}'")
#                     continue

#                 # convert RPM -> rad/s for JointState velocities
#                 vel_rad_s = [(rpm * 2.0 * math.pi) / 60.0 for rpm in rpm_vals]

#                 # publish JointState
#                 js = JointState()
#                 js.header.stamp = self.get_clock().now().to_msg()
#                 js.name = joint_names
#                 js.velocity = vel_rad_s
#                 self.joint_pub.publish(js)

#                 self.get_logger().debug(f"RX parsed rpm={rpm_vals}")

#         except Exception as e:
#             self.get_logger().error(f"Serial read error: {e}")
            

# def main(args=None):
#     rclpy.init(args=args)
#     node = MecanumSerialDriver()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         try:
#             if node.ser and node.ser.is_open:
#                 node.ser.close()
#         except Exception:
#             pass
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()
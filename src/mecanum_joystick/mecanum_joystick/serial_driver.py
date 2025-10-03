#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import serial
import math
import threading
import time

class MecanumSerialDriver(Node):
    """
    Subscribes to /cmd_vel (geometry_msgs/Twist), computes mecanum wheel speeds,
    sends encoded command to Arduino via serial, reads responses and publishes
    wheel velocities on /joint_states (sensor_msgs/JointState).

    Encoding format used (by default):
      Sent:  <FL,FR,RL,RR>\n   (integers, RPM)
      Received (Arduino -> Pi): <FL,FR,RL,RR>\n  (integers, RPM)
    Framing with '<' and '>' helps keep parsing robust.
    """

    def __init__(self):
        super().__init__('mecanum_serial_driver')

        # parameters (can be set from launch)
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.033)  # meters
        self.declare_parameter('wheel_separation_x', 0.375)  # front-back distance (m)
        self.declare_parameter('wheel_separation_y', 0.290)  # left-right distance (m)
        self.declare_parameter('max_rpm', 10)  # optional limiting
        self.declare_parameter('send_rate_hz', 20.0)  # how many cmd messages/sec max

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        # We will interpret wheel_separation_x,y as half-distances or full? Use half-distances for typical formulas:
        self.Lx = self.get_parameter('wheel_separation_x').get_parameter_value().double_value / 2.0
        self.Ly = self.get_parameter('wheel_separation_y').get_parameter_value().double_value / 2.0
        self.max_rpm = self.get_parameter('max_rpm').get_parameter_value().integer_value
        self.send_rate_hz = self.get_parameter('send_rate_hz').get_parameter_value().double_value

        # Combined term used in inverse kinematics: (Lx + Ly)
        self.L = self.Lx + self.Ly

        self.get_logger().info(f"Params: port={self.port} baud={self.baud} r={self.r:.3f} Lx={self.Lx:.3f} Ly={self.Ly:.3f}")

        # Serial port
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Opened serial {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial {self.port}: {e}")
            self.ser = None

        # Subscriber to /cmd_vel
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Publisher for wheel states
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # Keep most recent cmd and rate-limit sends
        self._last_cmd = None
        self._last_sent = 0.0
        self._lock = threading.Lock()

        # Buffer for serial reading
        self._rx_buffer = ""

        # Start a thread to continuously read serial and publish feedback
        self._reader_thread = threading.Thread(target=self._serial_reader_thread, daemon=True)
        self._reader_thread.start()

        self.get_logger().info("Mecanum serial driver ready")

    def cmd_vel_callback(self, msg: Twist):
        # get velocities from twist
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        # save last command
        with self._lock:
            self._last_cmd = (vx, vy, wz)

        # call send (rate-limited)
        now = time.time()
        if now - self._last_sent >= (1.0 / self.send_rate_hz):
            self._send_last_cmd()
            self._last_sent = now

    def _send_last_cmd(self):
        with self._lock:
            cmd = self._last_cmd
        if cmd is None:
            return
        vx, vy, wz = cmd

        # inverse kinematics for mecanum:
        # wheel angular velocity w = (1/r) * (vx +/- vy +/- L*wz)
        # order: FL, FR, RL, RR (you can change order but matches our URDF/joint naming)
        # w_fl = (1.0 / self.r) * (vx - vy - self.L * wz)
        # w_fr = (1.0 / self.r) * (vx + vy + self.L * wz)
        # w_rl = (1.0 / self.r) * (vx + vy - self.L * wz)
        # w_rr = (1.0 / self.r) * (vx - vy + self.L * wz)

        # wheel angular velocity w = (1/r) * (vx +/- vy +/- L*wz)
        # order: FL, FR, RL, RR
        w_fl = (1.0 / self.r) * (vx - vy + self.L * wz)  # M1 (Front Left)
        w_fr = (1.0 / self.r) * (vx + vy + self.L * wz)  # M2 (Front Right)
        w_rl = (1.0 / self.r) * (vx + vy + self.L * wz)  # M3 (Rear Left)
        w_rr = (1.0 / self.r) * (vx - vy - self.L * wz)  # M4 (Rear Right)

        # Convert rad/s -> RPM (integer)
        rpm_fl = int(self._clamp_rpm(w_fl))
        rpm_fr = int(self._clamp_rpm(w_fr))
        rpm_rl = int(self._clamp_rpm(w_rl))
        rpm_rr = int(self._clamp_rpm(w_rr))

        # Build framed message: <FL,FR,RL,RR>\n
        # msg = f"<{rpm_fl},{rpm_fr},{rpm_rl},{rpm_rr}>\n"
        msg = f"<{w_fl},{w_fr},{w_rl},{w_rr}>\n"

        if self.ser:
            try:
                self.ser.write(msg.encode('utf-8'))
                self.get_logger().debug(f"TX: {msg.strip()}")
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")

    def _clamp_rpm(self, w_rad_s):
        """Convert rad/s to RPM and clamp to +/- max_rpm"""
        rpm = w_rad_s * 60.0 / (2.0 * math.pi)
        if rpm > self.max_rpm:
            return self.max_rpm
        if rpm < -self.max_rpm:
            return -self.max_rpm
        return rpm

    def _serial_reader_thread(self):
        """
        Continuously read from serial, accumulate until newline, parse framed messages.
        Expected reply format from Arduino: <FL,FR,RL,RR>\n   (integers RPM)
        On parse, publish sensor_msgs/JointState
        """
        # joint names consistent with your URDF
        joint_names = ['front_left_wheel_joint', 'front_right_wheel_joint',
                       'rear_left_wheel_joint', 'rear_right_wheel_joint']

        while rclpy.ok():
            if not self.ser:
                time.sleep(0.5)
                continue
            try:
                data = self.ser.read(256)  # read up to 256 bytes
                if not data:
                    time.sleep(0.01)
                    continue
                try:
                    chunk = data.decode('utf-8', errors='ignore')
                except Exception:
                    chunk = ''
                if not chunk:
                    continue
                self._rx_buffer += chunk

                # process complete framed messages: they look like <...>\n
                while True:
                    start = self._rx_buffer.find('<')
                    end = self._rx_buffer.find('>', start+1)
                    if start == -1 or end == -1:
                        break
                    raw = self._rx_buffer[start+1:end]
                    # remove processed prefix
                    self._rx_buffer = self._rx_buffer[end+1:]

                    # parse CSV of four ints
                    parts = [p.strip() for p in raw.split(',') if p.strip() != '']
                    if len(parts) != 4:
                        self.get_logger().warning(f"Bad feedback format: '{raw}'")
                        continue
                    try:
                        rpm_vals = [int(p) for p in parts]
                    except ValueError:
                        self.get_logger().warning(f"Non-int feedback: '{raw}'")
                        continue

                    # convert RPM -> rad/s for JointState velocities
                    vel_rad_s = [ (rpm * 2.0 * math.pi) / 60.0 for rpm in rpm_vals ]

                    # publish JointState
                    js = JointState()
                    js.header.stamp = self.get_clock().now().to_msg()
                    js.name = joint_names
                    js.velocity = vel_rad_s
                    # positions and efforts left empty
                    self.joint_pub.publish(js)
                    self.get_logger().debug(f"RX parsed rpm={rpm_vals}")

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = MecanumSerialDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.ser and node.ser.is_open:
                node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

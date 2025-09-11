import rclpy
import rclpy.time
import rclpy.logging
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from .. import Create2, Leds

INPUT_TIMEOUT = 1


class LowPassFilter(object):

  def __init__(self, rc):
    self.last_value = 0
    self.last_time = 0
    self.rc = rc

  def filter(self, value):
    if self.last_time > 0:
      current_time = time.time()
      a = current_time - self.last_time
      a /= (a + self.rc)
      self.last_time = current_time
      self.last_value = a * value + (1 - a) * self.last_value
    else:
      self.last_time = time.time()

    if abs(self.last_value) <= 0.0001:
      return 0.0

    return self.last_value


class Create2RemixNode(Node):

  def __init__(self):
    super().__init__("create2remix_node")
    self.declare_parameter("serial_path", "/dev/roomba")
    self.declare_parameter("low_pass_filter_rc", 0.05)
    self.declare_parameter("left_wheel_joint_name", "left_wheel_joint")
    self.declare_parameter("right_wheel_joint_name", "right_wheel_joint")
    self.declare_parameter("odom_frame_id", "odom")
    self.declare_parameter("base_footprint_frame_id", "base_footprint")

    self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
    self.odom_pub = self.create_publisher(Odometry, "odom", 10)
    self.battery_pub = self.create_publisher(BatteryState, "battery", 10)
    self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)

    self.left_wheel_joint_name = self.get_parameter("left_wheel_joint_name").get_parameter_value().string_value
    self.right_wheel_joint_name = self.get_parameter("right_wheel_joint_name").get_parameter_value().string_value
    self.odom_frame_id = self.get_parameter("odom_frame_id").get_parameter_value().string_value
    self.base_footprint_frame_id = self.get_parameter("base_footprint_frame_id").get_parameter_value().string_value

    self.vel_timestamp = time.time()
    self.forward_velocity = 0.0
    self.angular_velocity = 0.0

    self.last_left_encoder_counts = None
    self.last_right_encoder_counts = None
    self.left_encoder_total = 0.0
    self.right_encoder_total = 0.0

    # TODO: make tf broadcasting configurable so we can use robot_localization.
    self.tf_broadcaster = TransformBroadcaster(self)
    self.logger = rclpy.logging.get_logger('create2remix')

    self.timer = self.create_timer(1 / 30.0, self.timer_callback) # TODO: parameterize

    rc = self.get_parameter("low_pass_filter_rc").get_parameter_value().double_value
    self.linear_lpf = LowPassFilter(rc)
    self.angular_lpf = LowPassFilter(rc)

    serial_path = self.get_parameter("serial_path").get_parameter_value().string_value
    self.bot = Create2(serial_path)
    self.bot.safe()
    self.bot.add_sensor_callback(self.on_sensor_message)

    self.bot.digit_leds_ascii(*"ARGH")
    self.bot.leds(Leds.DEBRIS, 0, 255)
    self.logger.info(f"Started create2remix on serial path: {serial_path}")

  def shutdown(self):
    self.bot.drive_direct(0, 0)
    self.bot.digit_leds_ascii(*"YARG")
    self.bot.leds(Leds.DEBRIS, 0, 255)

  def timer_callback(self):
    if time.time() - self.vel_timestamp > INPUT_TIMEOUT: # TODO: input_timeout should be configurable
      self.bot.drive_direct(0, 0) # TODO: low pass filter it to 0.
      return

    forward_velocity = self.linear_lpf.filter(self.forward_velocity)
    angular_velocity = self.angular_lpf.filter(self.angular_velocity)

    right = int((forward_velocity + (0.235 / 2 * angular_velocity)) * 1000)
    left = int((forward_velocity - (0.235 / 2 * angular_velocity)) * 1000)

    self.bot.drive_direct(right, left)

  def cmd_vel_callback(self, data):
    self.forward_velocity = data.linear.x
    self.angular_velocity = data.angular.z
    self.vel_timestamp = time.time()

  def on_sensor_message(self, packets):
    # Valid data:
    # packets.bumps_wheel_drops
    # packets.cliff_left
    # packets.cliff_front_left
    # packets.cliff_front_right
    # packets.cliff_right
    # packets.distance
    # packets.angle
    # packets.left_encoder_counts
    # packets.right_encoder_counts
    # packets.light_bump_left
    # packets.light_bump_front_left
    # packets.light_bump_center_left
    # packets.light_bump_center_right
    # packets.light_bump_front_right
    # packets.light_bump_right
    # packets.stasis

    seconds = math.floor(packets.timestamp)
    nanoseconds = int((packets.timestamp - seconds) * 1000000000)
    stamp_msg = rclpy.time.Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()

    x, y, yaw = packets.pose
    quaternion = quaternion_from_euler(0, 0, yaw)

    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg() # Need to use the wrong time because otherwise rviz won't be happy
    t.header.frame_id = self.odom_frame_id
    t.child_frame_id = self.base_footprint_frame_id

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    t.transform.rotation.x = quaternion[0]
    t.transform.rotation.y = quaternion[1]
    t.transform.rotation.z = quaternion[2]
    t.transform.rotation.w = quaternion[3]

    self.tf_broadcaster.sendTransform(t)

    odom = Odometry()

    odom.header.stamp = stamp_msg
    odom.header.frame_id = self.odom_frame_id
    odom.child_frame_id = self.base_footprint_frame_id

    odom.pose.pose.position.x = x
    odom.pose.pose.position.y = y
    odom.pose.pose.position.z = 0.0

    odom.pose.pose.orientation.x = quaternion[0]
    odom.pose.pose.orientation.y = quaternion[1]
    odom.pose.pose.orientation.z = quaternion[2]
    odom.pose.pose.orientation.w = quaternion[3]

    # TODO: odom.twist MUST be set!
    odom.twist.twist.linear.x = packets.velocity[0]
    odom.twist.twist.angular.z = packets.velocity[1]

    self.odom_pub.publish(odom)

    battery_state = BatteryState()
    battery_state.charge = packets.battery_charge / 1000.0
    battery_state.capacity = packets.battery_capacity / 1000.0
    battery_state.percentage = packets.battery_charge / float(packets.battery_capacity)
    self.battery_pub.publish(battery_state)

    joint_states = JointState()
    joint_states.header.stamp = stamp_msg
    joint_states.name = [self.left_wheel_joint_name, self.right_wheel_joint_name]
    if self.last_left_encoder_counts is None:
      self.last_left_encoder_counts = packets.left_encoder_counts

    if self.last_right_encoder_counts is None:
      self.last_right_encoder_counts = packets.right_encoder_counts

    # Left encoder
    left_delta = packets.left_encoder_counts - self.last_left_encoder_counts
    if left_delta > 32767:
      left_delta -= 65536
    elif left_delta < -32768:
      left_delta += 65536
    self.left_encoder_total += left_delta
    self.last_left_encoder_counts = packets.left_encoder_counts

    # Right encoder
    right_delta = packets.right_encoder_counts - self.last_right_encoder_counts
    if right_delta > 32767:
      right_delta -= 65536
    elif right_delta < -32768:
      right_delta += 65536
    self.right_encoder_total += right_delta
    self.last_right_encoder_counts = packets.right_encoder_counts

    joint_states.position = [
      (self.left_encoder_total / 508.8) * 2 * math.pi,
      (self.right_encoder_total / 508.8) * 2 * math.pi,
    ]

    self.joint_state_pub.publish(joint_states)

    # print("l = {} r = {} x = {:.2f} y = {:.2f} yaw = {:.2f} ({:.2f}% {}/{})".format(
    #   packets.left_encoder_counts,
    #   packets.right_encoder_counts,
    #   packets.pose[0],
    #   packets.pose[1],
    #   packets.pose[2] * 180 / math.pi,
    #   packets.battery_charge / float(packets.battery_capacity) * 100,
    #   packets.battery_charge,
    #   packets.battery_capacity,
    # ))


def main(args=None):
  rclpy.init(args=args)
  node = Create2RemixNode()
  try:
    rclpy.spin(node)
    rclpy.shutdown()
  finally:
    node.shutdown()

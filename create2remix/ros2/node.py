import rclpy
import rclpy.time
import rclpy.logging
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

from .. import Create2, Leds

INPUT_TIMEOUT = 1


class Create2RemixNode(Node):

  def __init__(self):
    super().__init__("create2remix_node")
    # TODO: figure out the QoS parameters better to replace the 10.
    self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)
    self.odom_pub = self.create_publisher(Odometry, "odom", 10)
    # TODO: make tf broadcasting configurable so we can use robot_localization.
    self.tf_broadcaster = TransformBroadcaster(self)
    self.logger = rclpy.logging.get_logger('create2remix')

    self.timer = self.create_timer(1 / 30.0, self.timer_callback)

    serial_path = "/dev/roomba" # TODO: parameter
    self.bot = Create2(serial_path)
    self.bot.safe()
    self.bot.add_sensor_callback(self.on_sensor_message)

    self.bot.digit_leds_ascii(*"ARGH")
    self.bot.leds(Leds.DEBRIS, 0, 255)
    self.logger.info(f"Started create2remix on serial path: {serial_path}")

    self.vel_timestamp = time.time()
    self.left_speed = 0
    self.right_speed = 0

  def shutdown(self):
    self.bot.drive_direct(0, 0)
    self.bot.digit_leds_ascii(*"YARG")
    self.bot.leds(Leds.DEBRIS, 0, 255)

  def timer_callback(self):
    if time.time() - self.vel_timestamp > INPUT_TIMEOUT: # TODO: input_timeout should be configurable
      self.bot.drive_direct(0, 0)
      return

    self.bot.drive_direct(self.right_speed, self.left_speed)

  def cmd_vel_callback(self, data):
    fwd_vel = data.linear.x
    rot_vel = data.angular.z

    right = fwd_vel + (0.235 / 2 * rot_vel)
    left = fwd_vel - (0.235 / 2 * rot_vel)

    self.left_speed = int(left * 1000)
    self.right_speed = int(right * 1000)
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

    x, y, yaw = packets.pose
    quaternion = quaternion_from_euler(0, 0, yaw)

    t = TransformStamped()
    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"

    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0

    t.transform.rotation.x = quaternion[0]
    t.transform.rotation.y = quaternion[1]
    t.transform.rotation.z = quaternion[2]
    t.transform.rotation.w = quaternion[3]

    self.tf_broadcaster.sendTransform(t)

    odom = Odometry()
    seconds = math.floor(packets.timestamp)
    nanoseconds = int((packets.timestamp - seconds) * 1000000000)
    odom.header.stamp = rclpy.time.Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
    odom.header.frame_id = "odom" # TODO: configurable
    odom.child_frame_id = "base_link"

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

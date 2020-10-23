from __future__ import absolute_import, print_function, unicode_literals

import logging
import math
import struct
import time

from .constants import Opcodes, Packets
from .serial_interface import SerialInterface


def limit(n, minn, maxn):
  return max(min(maxn, n), minn)


class Create2(object):
  def __init__(self, path, baud=115200):
    self.x, self.y, self.yaw = 0.0, 0.0, 0.0
    self._first_data_processed = False
    self._prev_left_encoder = 0
    self._prev_right_encoder = 0

    self.si = SerialInterface(path, baud)
    self.add_sensor_callback(self._compute_pose)  # lol this code is bad

    self.logger = logging.getLogger("create2")
    self.start()
    self.start_sensors()

  def __del__(self):
    if self.si.connected():
      try:
        self.start()
        self.stop()
      except Exception:
        self.logger.warn("couldn't send START and STOP to robot upon shutdown")

    # turnoff leds and screen
    # TODO: LEDs
    # TODO: screen
    self.si.disconnect()

  def _write(self, opcode, *data):
    self.si.write(opcode, *data)

  def reset(self):
    raise NotImplementedError

  def add_sensor_callback(self, f):
    self.si.add_sensor_callback(f)

  def get_sensor_data(self):
    if self.si is not None:
      return self.si.sensor_data
    else:
      return None

  def start(self):
    self._write(Opcodes.START)

  def stop(self):
    self._write(Opcodes.STOP)

  def safe(self):
    self._write(Opcodes.SAFE)
    time.sleep(0.3)  # Without this sleep, things like LEDs won't work

  def full(self):
    self._write(Opcodes.FULL)
    time.sleep(0.3)  # Without this sleep, things like LEDs won't work

  def power(self):
    self._write(Opcodes.POWER)

  def drive_direct(self, right_vel, left_vel):
    right_vel = limit(right_vel, -500, 500)
    left_vel = limit(left_vel, -500, 500)
    data = struct.unpack('4B', struct.pack('>2h', right_vel, left_vel))
    self._write(Opcodes.DRIVE_DIRECT, *data)

  def drive_stop(self):
    self.drive_direct(0, 0)

  def leds(self, leds_bit=0, power_color=0, power_intensity=0):
    self._write(Opcodes.LEDS, leds_bit, power_color, power_intensity)

  def digit_leds_ascii(self, l1=" ", l2=" ", l3=" ", l4=" "):
    data = [l1, l2, l3, l4]
    values = [32, 32, 32, 32]
    for i, letter in enumerate(data):
      value = ord(letter.upper())
      if value < 32 or value > 126:
        self.logger.warning("{} (idx = {}) is not a valid character for display, converting to space".format(value, i))
        value = 32

      values[i] = value

    self._write(Opcodes.DIGIT_LEDS_ASCII, *values)

  def start_sensors(self):
    self.si.start_sensors()

  def _compute_pose(self, packets):
    if not self._first_data_processed:
      self._prev_left_encoder = packets.left_encoder_counts
      self._prev_right_encoder = packets.right_encoder_counts
      packets[Packets.POSE] = (self.x, self.y, self.yaw)
      self._first_data_processed = True
      return

    delta_left = packets.left_encoder_counts - self._prev_left_encoder
    delta_right = packets.right_encoder_counts - self._prev_right_encoder

    if delta_left > 60000:  # some sufficiently large num
      delta_left = -((65535 % delta_left) + 1)
    elif delta_left < -60000:
      delta_left = (delta_left % 65535) + 1

    if delta_right > 60000:
      delta_right = -((65535 % delta_right) + 1)
    elif delta_right < -60000:
      delta_right = (delta_right % 65535) + 1

    left_dist = delta_left * (math.pi * 72.0 / 508.8) / 1000.0
    right_dist = delta_right * (math.pi * 72.0 / 508.8) / 1000.0

    right_left_diff_dist = right_dist - left_dist
    delta_yaw = right_left_diff_dist / 0.235

    if abs(right_left_diff_dist) < 0.0001:  # 0.1mm
      # Straight, without this condition, there would be a division by zero err
      avg_dist = (left_dist + right_dist) / 2.0
      delta_x = avg_dist * math.cos(self.yaw)
      delta_y = avg_dist * math.sin(self.yaw)
    else:
      turn_radius = (0.235 / 2) * (left_dist + right_dist) / right_left_diff_dist
      delta_x = turn_radius * (math.sin(self.yaw + delta_yaw) - math.sin(self.yaw))
      delta_y = -turn_radius * (math.cos(self.yaw + delta_yaw) - math.cos(self.yaw))

    self.x += delta_x
    self.y += delta_y
    self.yaw += delta_yaw
    self.yaw = self.yaw % (2 * math.pi)

    packets[Packets.POSE] = (self.x, self.y, self.yaw)

    self._prev_left_encoder = packets.left_encoder_counts
    self._prev_right_encoder = packets.right_encoder_counts

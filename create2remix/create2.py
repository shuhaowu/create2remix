from __future__ import absolute_import, print_function, unicode_literals

import logging
import struct
import time

from .constants import Opcodes
from .serial_interface import SerialInterface


def limit(n, minn, maxn):
  return max(min(maxn, n), minn)


class Create2(object):
  def __init__(self, path, baud=115200):
    self.si = SerialInterface(path, baud)
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
    pass

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

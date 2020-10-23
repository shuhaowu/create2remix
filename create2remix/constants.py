from collections import namedtuple
import struct

from six.moves import range


class Leds(object):
  CHECK_ROBOT = 0b1000
  DOCK        = 0b0100
  SPOT        = 0b0010
  DEBRIS      = 0b0001


class Opcodes(object):
  """Define all the applicable opcodes as constants"""
  START               = 128
  RESET               = 7
  STOP                = 173
  SAFE                = 131
  FULL                = 132
  POWER               = 133
  DRIVE_DIRECT        = 145
  LEDS                = 139
  DIGIT_LEDS_ASCII    = 164
  STREAM              = 148
  PAUSE_RESUME_STREAM = 150


class Packets(dict):
  HEADER_BYTE = 19

  @classmethod
  def get(cls, packet_id):
    return packet_decoders.get(packet_id, (None, None, None))

  # Can access any attribute in packet_ids via their lowercased name
  def __getattr__(self, name):
    name = name.upper()
    if name in packet_ids:
      return self[packet_ids[name]]

    raise AttributeError("{} is not a valid data packet (valid ones are: {})".format(name, packet_ids.keys()))

  def __repr__(self):
    d = {}
    for k, v in self.items():
      d[packet_names[k]] = v

    return repr(d)


BumpsAndWheelDrops = namedtuple("BumpsAndWheelDrops", [
  "wheel_drop_left",
  "wheel_drop_right",
  "bump_left",
  "bump_right",
])


LightBumper = namedtuple("LightBumper", [
  "light_bumper_right",
  "light_bumper_front_right",
  "light_bumper_center_right",
  "light_bumper_center_left",
  "light_bumper_front_left",
  "light_bumper_left",
])


def _signed_single_byte(data):
  return struct.unpack("b", data[0])[0]


def _unsigned_single_byte(data):
  return struct.unpack("B", data[0])[0]


def _signed_two_bytes(data):
  return struct.unpack(">h", data[0] + data[1])[0]


def _unsigned_two_bytes(data):
  return struct.unpack(">H", data[0] + data[1])[0]


def _unsigned_bit_to_bool(data):
  return bool(_unsigned_single_byte(data))


def _byte_to_int_array(data):
  return [1 if data & (1 << (7 - n)) else 0 for n in range(8)]


def _decode_bumps_and_wheel_drops(data):
  # Bit 7, 6, 5, 4: Reserved
  # Bit 3: Wheel drop left
  # Bit 2: Wheel drop right
  # Bit 1: Bump left
  # Bit 0: Bump right
  data = _byte_to_int_array(_unsigned_single_byte(data))
  return BumpsAndWheelDrops(data[4], data[5], data[6], data[7])


def _decode_light_bumper(data):
  # Byte 7, 6: Reserved
  # Byte 5: Light bumper right
  # Byte 4: Light bumper front right
  # Byte 3: Light bumper center right
  # Byte 2: Light bumper center left
  # Byte 1: Light bumper front left
  # Byte 0: Light bumper left
  data = _byte_to_int_array(_unsigned_single_byte(data))
  return LightBumper(data[2], data[3], data[4], data[5], data[6], data[7])


def _decode_wheel_overcurrents(data):
  # Bit 7, 6, 5, 1: Reserved
  # Bit 4: Left wheel
  # Bit 3: Right wheel
  # Bit 2: Main brush
  # Bit 0: Side brush
  data = _byte_to_int_array(_unsigned_single_byte(data))
  return data


packet_ids = dict(
  BUMPS_WHEEL_DROPS        = 7,
  WALL                     = 8,
  CLIFF_LEFT               = 9,
  CLIFF_FRONT_LEFT         = 10,
  CLIFF_FRONT_RIGHT        = 11,
  CLIFF_RIGHT              = 12,
  VIRTUAL_WALL             = 13,
  WHEEL_OVERCURRENTS       = 14,
  DIRT_DETECT              = 15,
  UNUSED_BYTE              = 16,
  INFRARED_CHARACTER_OMNI  = 17,
  INFRARED_CHARACTER_LEFT  = 52,
  INFRARED_CHARACTER_RIGHT = 53,
  BUTTONS                  = 18,
  DISTANCE                 = 19,
  ANGLE                    = 20,
  CHARGING_STATE           = 21,
  VOLTAGE                  = 22,
  CURRENT                  = 23,
  TEMPERATURE              = 24,
  BATTERY_CHARGE           = 25,
  BATTERY_CAPACITY         = 26,
  WALL_SIGNAL              = 27,
  CLIFF_LEFT_SIGNAL        = 28,
  CLIFF_FRONT_LEFT_SIGNAL  = 29,
  CLIFF_FRONT_RIGHT_SIGNAL = 30,
  CLIFF_RIGHT_SIGNAL       = 31,
  UNUSED_BYTE2             = 32,
  UNUSED_BYTE3             = 33,
  CHARGING_SOURCES         = 34,
  OI_MODE                  = 35,
  SONG_NUMBER              = 36,
  SONG_PLAYING             = 37,
  NUMBER_OF_STREAM_PACKETS = 38,
  REQUESTED_VELOCITY       = 39,
  REQUESTED_RADIUS         = 40,
  REQUESTED_RIGHT_VELOCITY = 41,
  REQUESTED_LEFT_VELOCITY  = 42,
  LEFT_ENCODER_COUNTS      = 43,
  RIGHT_ENCODER_COUNTS     = 44,
  LIGHT_BUMPER             = 45,
  LIGHT_BUMP_LEFT          = 46,
  LIGHT_BUMP_FRONT_LEFT    = 47,
  LIGHT_BUMP_CENTER_LEFT   = 48,
  LIGHT_BUMP_CENTER_RIGHT  = 49,
  LIGHT_BUMP_FRONT_RIGHT   = 50,
  LIGHT_BUMP_RIGHT         = 51,
  LEFT_MOTOR_CURRENT       = 54,
  RIGHT_MOTOR_CURRENT      = 55,
  MAIN_BRUSH_MOTOR_CURRENT = 56,
  SIDE_BRUSH_MOTOR_CURRENT = 57,
  STASIS                   = 58,

  TIMESTAMP                = -1,  # Fake packet, constructed in serial_interface.py
  POSE                     = -2,
)

# Length, decoder function, validation
# Not all of these may be correct yet...
packet_decoders = {
  packet_ids["BUMPS_WHEEL_DROPS"]:        (1, _decode_bumps_and_wheel_drops, None),
  packet_ids["WALL"]:                     (1, _unsigned_bit_to_bool, None),
  packet_ids["CLIFF_LEFT"]:               (1, _unsigned_bit_to_bool, None),
  packet_ids["CLIFF_FRONT_LEFT"]:         (1, _unsigned_bit_to_bool, None),
  packet_ids["CLIFF_FRONT_RIGHT"]:        (1, _unsigned_bit_to_bool, None),
  packet_ids["CLIFF_RIGHT"]:              (1, _unsigned_bit_to_bool, None),
  packet_ids["VIRTUAL_WALL"]:             (1, _unsigned_bit_to_bool, None),
  packet_ids["WHEEL_OVERCURRENTS"]:       (1, _decode_wheel_overcurrents, None),
  packet_ids["DIRT_DETECT"]:              (1, _signed_single_byte, None),
  packet_ids["UNUSED_BYTE"]:              (1, _signed_single_byte, lambda v: v == 0),
  packet_ids["INFRARED_CHARACTER_OMNI"]:  (1, _unsigned_single_byte, None),
  packet_ids["INFRARED_CHARACTER_LEFT"]:  (1, _unsigned_single_byte, None),
  packet_ids["INFRARED_CHARACTER_RIGHT"]: (1, _unsigned_single_byte, None),
  packet_ids["BUTTONS"]:                  (1, _unsigned_single_byte, None),
  packet_ids["DISTANCE"]:                 (2, _signed_two_bytes, None),
  packet_ids["ANGLE"]:                    (2, _signed_two_bytes, None),
  packet_ids["CHARGING_STATE"]:           (1, _unsigned_single_byte, lambda v: v >= 0 and v <= 5),
  packet_ids["VOLTAGE"]:                  (2, _unsigned_two_bytes, None),
  packet_ids["CURRENT"]:                  (2, _signed_two_bytes, None),
  packet_ids["TEMPERATURE"]:              (2, _signed_single_byte, lambda v: v >= -128 and v <= 127),
  packet_ids["BATTERY_CHARGE"]:           (2, _unsigned_two_bytes, None),
  packet_ids["BATTERY_CAPACITY"]:         (2, _unsigned_two_bytes, None),
  packet_ids["WALL_SIGNAL"]:              (2, _unsigned_two_bytes, lambda v: v >= 0 and v <= 1023),
  packet_ids["CLIFF_LEFT_SIGNAL"]:        (2, _unsigned_two_bytes, lambda v: v >= 0 and v <= 4095),
  packet_ids["CLIFF_FRONT_LEFT_SIGNAL"]:  (2, _unsigned_two_bytes, lambda v: v >= 0 and v <= 4095),
  packet_ids["CLIFF_FRONT_RIGHT_SIGNAL"]: (2, _unsigned_two_bytes, lambda v: v >= 0 and v <= 4095),
  packet_ids["CLIFF_RIGHT_SIGNAL"]:       (2, _unsigned_two_bytes, lambda v: v >= 0 and v <= 4095),
  packet_ids["CHARGING_SOURCES"]:         (1, _unsigned_single_byte, lambda v: v >= 0 and v <= 3),
  packet_ids["OI_MODE"]:                  (1, _unsigned_single_byte, lambda v: v >= 0 and v <= 3),
  packet_ids["SONG_NUMBER"]:              (1, _unsigned_single_byte, lambda v: v >= 0 and v <= 15),
  packet_ids["SONG_PLAYING"]:             (1, _unsigned_bit_to_bool, None),
  packet_ids["NUMBER_OF_STREAM_PACKETS"]: (1, _unsigned_single_byte, None),
  packet_ids["REQUESTED_VELOCITY"]:       (2, _signed_two_bytes, lambda v: v >= -500 and v <= 500),
  packet_ids["REQUESTED_RADIUS"]:         (2, _signed_two_bytes, None),
  packet_ids["REQUESTED_RIGHT_VELOCITY"]: (2, _signed_two_bytes, None),
  packet_ids["REQUESTED_LEFT_VELOCITY"]:  (2, _signed_two_bytes, None),
  packet_ids["LEFT_ENCODER_COUNTS"]:      (2, _signed_two_bytes, None),
  packet_ids["RIGHT_ENCODER_COUNTS"]:     (2, _signed_two_bytes, None),
  packet_ids["LIGHT_BUMPER"]:             (1, _decode_light_bumper, None),
  packet_ids["LIGHT_BUMP_LEFT"]:          (2, _unsigned_two_bytes, None),
  packet_ids["LIGHT_BUMP_FRONT_LEFT"]:    (2, _unsigned_two_bytes, None),
  packet_ids["LIGHT_BUMP_CENTER_LEFT"]:   (2, _unsigned_two_bytes, None),
  packet_ids["LIGHT_BUMP_CENTER_RIGHT"]:  (2, _unsigned_two_bytes, None),
  packet_ids["LIGHT_BUMP_FRONT_RIGHT"]:   (2, _unsigned_two_bytes, None),
  packet_ids["LIGHT_BUMP_RIGHT"]:         (2, _unsigned_two_bytes, None),
  packet_ids["LEFT_MOTOR_CURRENT"]:       (2, _signed_two_bytes, None),
  packet_ids["RIGHT_MOTOR_CURRENT"]:      (2, _signed_two_bytes, None),
  packet_ids["MAIN_BRUSH_MOTOR_CURRENT"]: (2, _signed_two_bytes, None),
  packet_ids["SIDE_BRUSH_MOTOR_CURRENT"]: (2, _signed_two_bytes, None),
  packet_ids["STASIS"]:                   (1, _signed_single_byte, None),
}


packet_names = {}
for k, v in packet_ids.items():
  packet_names[v] = k
  setattr(Packets, k, v)  # convenience or confusion?

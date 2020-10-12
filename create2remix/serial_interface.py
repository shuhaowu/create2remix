import os
import logging
import serial
import serial.threaded
import struct
import time
import traceback

from .constants import Opcodes, Packets, _unsigned_single_byte


class SerialInterface(serial.threaded.Protocol):
  READ_HEADER      = 0
  READ_NBYTES      = 1
  READ_PACKET_ID   = 2
  READ_PACKET_DATA = 3
  READ_CHECKSUM    = 4

  MAX_ATTEMPT = 10

  def __init__(self, path, baudrate):
    self._s = serial.Serial(path, baudrate, timeout=1, write_timeout=1)
    self.logger = logging.getLogger("si")
    self.sensor_data = None

    self.log_data = not bool(os.environ.get("DO_NOT_LOG_DATA", ""))

    self.num_packets_per_stream = 0
    self.packet_ids = []
    self.add_packet(Packets.BUMPS_WHEEL_DROPS)
    self.add_packet(Packets.CLIFF_LEFT)
    self.add_packet(Packets.CLIFF_FRONT_LEFT)
    self.add_packet(Packets.CLIFF_FRONT_RIGHT)
    self.add_packet(Packets.CLIFF_RIGHT)
    self.add_packet(Packets.DISTANCE)
    self.add_packet(Packets.ANGLE)
    self.add_packet(Packets.LEFT_ENCODER_COUNTS)
    self.add_packet(Packets.RIGHT_ENCODER_COUNTS)
    self.add_packet(Packets.LIGHT_BUMP_LEFT)
    self.add_packet(Packets.LIGHT_BUMP_FRONT_LEFT)
    self.add_packet(Packets.LIGHT_BUMP_CENTER_LEFT)
    self.add_packet(Packets.LIGHT_BUMP_CENTER_RIGHT)
    self.add_packet(Packets.LIGHT_BUMP_FRONT_RIGHT)
    self.add_packet(Packets.LIGHT_BUMP_RIGHT)
    self.add_packet(Packets.STASIS)

    self.corrupted_packets = 0

    self._thr = None

    self._read_state = self.READ_HEADER
    self._checksum = 0
    self._expected_bytes_read = 0
    self._bytes_read = 0
    self._current_packet_id = 0
    self._current_packet_decoder = None
    self._current_packet_expected_byte_lengths = None
    self._current_packet_validator = None
    self._current_packet_data = []

    self._sensor_data_next = Packets()

  def add_packet(self, packet_id):
    self.num_packets_per_stream += 1
    self.packet_ids.append(packet_id)

  def connected(self):
    if self._thr is not None:
      return self._thr.serial.is_open
    else:
      return self._s.is_ok  # ._s is transformed the to transport

  def disconnect(self):
    if self._s is not None:
      self._s.close()  # should close the thread if it is there

  def write(self, opcode, *data):
    message = [opcode]
    if data:
      message = message + list(data)

    self.logger.debug("W: {}".format(message))
    message = struct.pack("B" * len(message), *message)

    self._s.write(message)  # only during the beginning

  def start_sensors(self):
    # read the ReaderThread source to know this works
    self._thr = serial.threaded.ReaderThread(self._s, lambda: self)
    self.write(Opcodes.STREAM, self.num_packets_per_stream, *self.packet_ids)
    self._thr.start()

    # wait for data
    attempt = 0
    while self.sensor_data is None:
      if attempt >= self.MAX_ATTEMPT:
        self.logger.error("cannot connect to serial, no data received!")
        self.disconnect()
        raise RuntimeError("cannot connect to serial")

      attempt += 1
      self.logger.debug("no data received, retrying connection {}/{}".format(attempt, self.MAX_ATTEMPT))
      time.sleep(0.5)
      self.write(Opcodes.START)
      self.write(Opcodes.STREAM, self.num_packets_per_stream, *self.packet_ids)

    self.logger.info("successfully connected to create2 serial with first data: {}".format(self.sensor_data))

  def connection_made(self, transport):
    self.logger.debug("connection_made")
    self._s = transport  # This will change _s to ReaderThread, which wraps serial

  def data_received(self, data):
    for byte in serial.iterbytes(data):
      unsigned_byte = _unsigned_single_byte(byte)
      self._checksum += unsigned_byte
      self._bytes_read += 1

      if self._read_state == self.READ_HEADER:
        if unsigned_byte == Packets.HEADER_BYTE:
          self._read_state = self.READ_NBYTES
          self._checksum = unsigned_byte
          self.logger.trace("received header")
        else:
          self.logger.warning("received byte {} while expecting header byte".format(unsigned_byte))

      elif self._read_state == self.READ_NBYTES:
        self._read_state = self.READ_PACKET_ID
        self._expected_bytes_read = unsigned_byte
        self._bytes_read = 0
        self.logger.trace("received nbytes = {}".format(unsigned_byte))
        # TODO: if byte != 80

      elif self._read_state == self.READ_PACKET_ID:
        self._current_packet_id = unsigned_byte
        self._current_packet_expected_byte_lengths, self._current_packet_decoder, self._current_packet_validator = Packets.get(unsigned_byte)
        if self._current_packet_decoder is None:
          self.logger.warning("cannot find packet type {}".format(unsigned_byte))
          self._read_state = self.READ_HEADER
        else:
          self._current_packet_data = []
          self._read_state = self.READ_PACKET_DATA
          self.logger.trace("found packet {}, expecting {} bytes".format(self._current_packet_id, self._current_packet_expected_byte_lengths))

      elif self._read_state == self.READ_PACKET_DATA:
        self._current_packet_data.append(byte)
        self.logger.trace("received data {}".format(unsigned_byte))
        if len(self._current_packet_data) >= self._current_packet_expected_byte_lengths:
          data = self._current_packet_decoder(self._current_packet_data)
          self._sensor_data_next[self._current_packet_id] = data
          self.logger.trace("decoded data {} for packet {}".format(data, self._current_packet_id))
          # TODO: validate

          if self._bytes_read >= self._expected_bytes_read:
            self._read_state = self.READ_CHECKSUM
          else:
            self._read_state = self.READ_PACKET_ID

      elif self._read_state == self.READ_CHECKSUM:
        checks_out = self._checksum & 0xFF
        self.logger.trace("read checksum {}, {} & 0xFF = {}".format(unsigned_byte, self._checksum, checks_out))
        if checks_out == 0:
          if self.log_data:
            self.logger.debug("got data: {}".format(self.sensor_data))

          self.sensor_data = self._sensor_data_next
          self.sensor_data[Packets.TIMESTAMP] = time.time()
          self._sensor_data_next = Packets()
        else:
          self.corrupted_packets += 1
          self.logger.warn("failed checksum: {} & 0xFF = {}".format(self._checksum, checks_out))

        self._read_state = self.READ_HEADER

  def connection_lost(self, exception):
    if exception:
      traceback.print_exc()
    self.logger.debug("connection_lost: {}".format(exception))
    self._s = None
    super(SerialInterface, self).connection_lost(exception)

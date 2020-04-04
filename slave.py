#!/usr/bin/python
#
# Copyright (C) 2020 packom.net
#
# A sample MBus Slave implemented in Python
# 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# 
# 

import serial, signal, logging

# Edit these values as appropriate
DEVICE = '/dev/ttyUSB0' # Serial device controlling the slave is connected to
BAUDRATE = 2400 # Baudrate supported by this slave
ADDR = 3 # MBus Slave address, 0=250 are valid
ID_NO = "12345678" # MBus Slave serial number, can be up to 8 characters, 0-9, A-F

ACCESS_NO = 0
PARITY=serial.PARITY_EVEN
STOPBITS=serial.STOPBITS_ONE
BYTESIZE=serial.EIGHTBITS
TEST_ADDR = 254
STOP_BYTE = 0x16
SINGLE_CHAR = 0xE5
SHORT_FRAME_START_BYTE = 0x10
LONG_FRAME_START_BYTE = 0x68
CI_FIXED_DATA_RSP = 0x73

ser = None
logger = None

def debug(msg, *args, **kwargs):
  logger.debug(msg, *args, **kwargs)

def info(msg, *args, **kwargs):
  logger.info(msg, *args, **kwargs)

def warning(msg, *args, **kwargs):
  logger.warning(msg, *args, **kwargs)

def error(msg, *args, **kwargs):
  logger.error(msg, *args, **kwargs)

def critical(msg, *args, **kwargs):
  logger.critical(msg, *args, **kwargs)

bcd_encode_table = {'0':0, '1':1, '2':2, '3':3, '4':4, '5':5, '6':6, '7':7, '8':8, '9':9, 'A':10, 'B':11, 'C':12, 'D':13, 'E':14, 'F':15}
def bcd_encode(text, bytes):
  in_bytes = []
  length = 0
  for char in text[::-1]: # Go backwards through the string
    in_bytes.append(bcd_encode_table[char])
    length += 1
  encoded = []
  while (length > 0):
    a = 0
    if length > 1:
      a = in_bytes.pop(1)
    b = in_bytes.pop(0)
    length -= 2
    byte = (a << 4) | b
    encoded.insert(0, byte)
  while len(encoded) < bytes:
    encoded.insert(0, 0)
  if len(encoded) > bytes:
    raise Exception
  return encoded[::-1] # Flip it around again

class FrameException(Exception):
  pass

class Frame:
  SHORT = 1
  LONG = 2
  CONTROL = 3
  types = [SHORT, LONG, CONTROL]

  SND_NKE = 0x40
  SND_UD = 0x53
  REQ_UD2 = 0x5B
  REQ_UD1 = 0x5A
  RSP_UD = 0x08
  _c_fields = [SND_NKE, SND_UD, REQ_UD2, REQ_UD1] # RSP_UD is Slave to Master so not included here
  _short_c_fields = [SND_NKE, REQ_UD2, REQ_UD1]
  _long_c_fields = [SND_UD] # RSP_UD is Slave to Master so not included here 

  def __init__(self, type):
    # Can't create a CONTROL - a LONG may turn into a CONTROL
    if (type in [self.SHORT, self.LONG]):
      self._type = type
      self._bytes = 0
      self._c_field = None
      self._a_field = None
      self._addressed = False
      self._csum = 0
      self._csum_passed = False
      self._data = None
    else:
      raise FrameException

  def _handle_c_field(self, byte):
    self._csum += byte
    if (self._type == self.SHORT):
      if (byte in self._short_c_fields):
        debug("C Field: 0x%2.x" % byte)
        self._c_field = byte
      else:
        debug("Unexpected Short Frame C Field: 0x%2.2x" %byte)
    elif (self._type == self.self.SHORT):
      if (byte in self._long_c_fields):
        debug("C Field: 0x%2.x" % byte)
        self._c_field = byte
      else:
        debug("Unexpected Long Frame C Field: 0x%2.2x" %byte)
    else:
      raise FrameException

  def _handle_a_field(self, byte):
    self._a_field = byte
    self._csum += byte
    if self._a_field == ADDR:
      debug("My Address: %d" % self._a_field)
      self._addressed = True
    elif self._a_field == TEST_ADDR:
      debug("Test Address: %d" % self._a_field)
      self._addressed = True
    else:
      debug("Not My Address: %d vs my address: %d" % (self._a_field, ADDR))

  def _handle_checksum(self, byte):
    debug("Checksum received: %2.2x vs stored: %2.2x" % (byte, self._csum))
    self._csum_passed = (self._csum % 256) == byte

  def _handle_stop(self, byte):
    data = None
    if byte == STOP_BYTE:
      if self._addressed:
        if self._c_field == self.SND_NKE:
          info("Slave Initialization")
          data = [SINGLE_CHAR,]
        elif self._c_field == self.REQ_UD2:
          info("Request for User Data 2")
          user_data = []
          global ID_NO
          user_data += bcd_encode(ID_NO, 4)
          global ACCESS_NO
          user_data.append(ACCESS_NO % 256)
          ACCESS_NO += 1
          user_data.append(0) # status field, indicates counter 1 and 2 are BCD
          user_data += [0xE9, 0x7E] # medium water, unit1 = 1l, unit2 = 1l (same, but historic)
          user_data += bcd_encode("123", 4)
          user_data += bcd_encode("456", 4)
          user_data = [self.RSP_UD, ADDR, CI_FIXED_DATA_RSP] + user_data
          checksum = 0
          ud_len = 0
          for byte in user_data:
            checksum += byte
            ud_len += 1
          checksum %= 256
          if ud_len > 255:
            raise Exception
          data = [LONG_FRAME_START_BYTE, ud_len, ud_len, LONG_FRAME_START_BYTE]
          data += user_data
          data += [checksum, STOP_BYTE]
        elif self._c_field == self.REQ_UD1:
          data = [SINGLE_CHAR,]
        else:
          raise FrameException
      else:
        debug("Frame not for us")
    else:
      debug("Unexpected byte: 0x%2.2x", byte)
    return data

  def handle_byte(self, byte):
    data = None
    frame = self
    if self._type == self.SHORT:
      if (self._bytes == 0):
        self._handle_c_field(byte)
      elif (self._bytes == 1):
        self._handle_a_field(byte)
      elif (self._bytes == 2):
        self._handle_checksum(byte)
      elif (self._bytes == 3):
        data = self._handle_stop(byte)
        frame = None
    elif (self._type == self.LONG):
      pass
    elif (self._type == self.CONTROL):
      pass
    else:
      raise FrameException
    self._bytes += 1
    return frame, data

def setup_serial_port():
  ser = serial.Serial(DEVICE, baudrate=BAUDRATE, parity=PARITY, stopbits=STOPBITS, bytesize=BYTESIZE)
  return ser

def handle_byte(frame, byte):
  data = None
  if (frame):
    frame, data = frame.handle_byte(byte)
  else:
    # Not in a frame already
    if byte == SINGLE_CHAR:
      debug("Single Character Frame")
      pass
    elif byte == STOP_BYTE:
      debug("Stop byte")
      pass
    elif byte == SHORT_FRAME_START_BYTE:
      debug("Short Frame")
      frame = Frame(Frame.SHORT)
    elif byte == LONG_FRAME_START_BYTE:
      debug("Control or Long Frame")
      frame = Frame(Frame.LONG)
  return frame, data

def send_data(ser, data):
  ser.write(data)
  debug("Sent data")

run = True

def signal_handler(sig, frame):
  global ser
  run = False
  ser.close()
  ser = None

def log():
  global logger
  logging.basicConfig(level=logging.INFO)
  logger = logging.getLogger('pyMbusSlave')
  info("pyMbusSlave")
  info("  Serial device:   %s", DEVICE)
  info("  Baudrate:        %d", BAUDRATE)
  info("  Slave address:   %d", ADDR)
  info("  Slave device ID: %s", ID_NO)

def main(*args, **kwargs):
  global run, ser
  log()
  signal.signal(signal.SIGINT, signal_handler)
  ser = setup_serial_port()
  info("Listening ...")
  frame = None
  while (ser):
    try:
      byte = ord(ser.read())
    except:
      break
    frame, data = handle_byte(frame, byte)
    if (data != None):
      try:
        send_data(ser, data)
      except:
        break
  info("Exiting ...")

if __name__ == "__main__":
  main()

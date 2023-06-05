#!/usr/bin/python
#
# Copyright (C) 2020 packom.net
# Copyright (C) 2022 Addiva Elektronik AB
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

import serial, signal, logging, sys, struct, argparse, os, pty

# Edit these values as appropriate
DEVICE = "/dev/ttyUSB0"  # Serial device controlling the slave is connected to
BAUDRATE = 2400  # Baudrate supported by this slave
ADDR = 3  # MBus Slave address, 0=250 are valid
ID_NO = 12345678  # MBus Slave serial number, can be up to 8 digits or upper case hex characters
MANUF = "TST"  # MBus Manufacturer identifier (registered)
VERSION = 1
MEDIUM = 0
ACCESS_NO = 0
PARITY = serial.PARITY_NONE  # PARITY_EVEN
STOPBITS = serial.STOPBITS_ONE
BYTESIZE = serial.EIGHTBITS
NETWORK_ADDR = 253
TEST_ADDR = 254
BROADCAST_ADDR = 255
STOP_BYTE = 0x16
SINGLE_CHAR = 0xE5
SHORT_FRAME_START_BYTE = 0x10
LONG_FRAME_START_BYTE = 0x68
CI_FIXED_DATA_RSP = 0x73
CI_VARIABLE_DATA_RSP = 0x72

ser = None
logger = None
is_selected = False


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


class PseudoSerial:
    def __init__(self, fd):
        self._fd = fd

    def write(self, data):
        os.write(self._fd, data)

    def close(self):
        os.close(self._fd)

    def read(self):
        return os.read(self._fd, 1)


class Frame:
    SHORT = 1
    LONG = 2
    types = [SHORT, LONG]

    SND_NKE = 0x40
    SND_UD = 0x53
    REQ_UD2 = 0x5B
    REQ_UD1 = 0x5A
    RSP_UD = 0x08
    _c_fields = [
        SND_NKE,
        SND_UD,
        REQ_UD2,
        REQ_UD1,
    ]  # RSP_UD is Slave to Master so not included here
    _short_c_fields = [SND_NKE, REQ_UD2, REQ_UD1]
    _long_c_fields = [SND_UD]  # RSP_UD is Slave to Master so not included here
    CI_SELECT = 0x52
    CI_SELECT_BE = 0x56
    FN_INST = 0b00
    FN_MIN = 0b10
    FN_MAX = 0b01
    FN_ERR = 0b11
    DF_NONE = 0b0000
    DF_INT8 = 0b0001
    DF_INT16 = 0b0010
    DF_INT24 = 0b0011
    DF_INT32 = 0b0100
    DF_INT48 = 0b0110
    DF_INT64 = 0b0111
    DF_REAL = 0b0101
    DF_SELECT = 0b1000
    DF_BCD2 = 0b1001
    DF_BCD4 = 0b1010
    DF_BCD8 = 0b1011
    DF_BCD12 = 0b1110
    DF_VARIABLE = 0b1101
    DIF_MFR = 0x0F
    DIF_MFR_EXT = 0x1F
    DIF_IDLE = 0x2F
    DIF_GLOBAL = 0x7F

    @staticmethod
    def VIF_ENERGY_Wh(e):
        return (0b0000 << 3) | (e + 3)

    @staticmethod
    def VIF_ENERGY_J(e):
        return (0b0001 << 3) | (e + 0)

    @staticmethod
    def VIF_VOLUME_l(e):
        return (0b0010 << 3) | (e + 3)

    @staticmethod
    def VIF_VOLUME_m3(e):
        return (0b0010 << 3) | (e + 6)

    @staticmethod
    def VIF_MASS_g(e):
        return (0b0011 << 3) | (e + 0)

    @staticmethod
    def VIF_MASS_kg(e):
        return (0b0011 << 3) | (e + 3)

    VIF_TIME_SECONDS = 0b00
    VIF_TIME_MINUTES = 0b01
    VIF_TIME_HOURS = 0b10
    VIF_TIME_DAYS = 0b11

    @staticmethod
    def VIF_OnTime(unit):
        return (0b01000 << 2) | unit

    @staticmethod
    def VIF_OperTime(unit):
        return (0b01001 << 2) | unit

    @staticmethod
    def VIF_POWER_W_h(e):
        return (0b0101 << 3) | (e + 3)

    @staticmethod
    def VIF_POWER_kJ_h(e):
        return (0b0110 << 3) | (e + 3)

    @staticmethod
    def VIF_FLOW_l_h(e):
        return (0b0111 << 3) | (e + 3)

    @staticmethod
    def VIF_FLOW_m3_h(e):
        return (0b0111 << 3) | (e + 6)

    @staticmethod
    def VIF_FLOW_l_m(e):
        return (0b1000 << 3) | (e + 3)

    @staticmethod
    def VIF_FLOW_m3_m(e):
        return (0b1000 << 3) | (e + 6)

    @staticmethod
    def VIF_FLOW_l_s(e):
        return (0b1001 << 3) | (e + 3)

    @staticmethod
    def VIF_FLOW_m3_s(e):
        return (0b1001 << 3) | (e + 6)

    @staticmethod
    def VIF_FLOW_g_h(e):
        return (0b1010 << 3) | (e + 0)

    @staticmethod
    def VIF_FLOW_kg_h(e):
        return (0b1010 << 3) | (e + 3)

    @staticmethod
    def VIF_FLOW_TEMP_C(e):
        return (0b10110 << 2) | (e + 3)

    @staticmethod
    def VIF_RETURN_TEMP_C(e):
        return (0b10111 << 2) | (e + 3)

    @staticmethod
    def VIF_TEMP_DIFF(e):
        return (0b11000 << 2) | (e + 3)

    @staticmethod
    def VIF_EXT_TEMP_C(e):
        return (0b11001 << 2) | (e + 3)

    @staticmethod
    def VIF_PRESSURE_bar(e):
        return (0b11010 << 2) | (e + 3)

    VIF_DATE = 0b1101100
    VIF_TIME = 0b1101101
    VIF_HCA = 0b1101110

    @staticmethod
    def VIF_AVG_DURATION(unit):
        return (0b11100 << 2) | unit

    @staticmethod
    def VIF_ACT_DURATION(unit):
        return (0b11101 << 2) | unit

    VIF_FABRICATION = 0b1111000
    VIF_ENHANCED = 0b1111001
    VIF_BUS_ADDR = 0b1111010
    VIF_EXT_b = 0b1111011
    VIF_EXT_STR = 0b1111100
    VIF_EXT_a = 0b1111101
    VIF_ANY = 0b1111110
    VIF_MFR_DEFINED = 0b1111111

    @staticmethod
    def VIF_CREDIT(e):
        return (0b1111101 << 8) | (0b00000 << 2) | (e + 3)

    @staticmethod
    def VIF_DEBIT(e):
        return (0b1111101 << 8) | (0b00001 << 2) | (e + 3)

    VIF_ACCESS_NUMBER = (0b1111101 << 8) | (0b0001000)
    VIF_MEDIUM = (0b1111101 << 8) | (0b0001001)
    VIF_MANUFACTURER = (0b1111101 << 8) | (0b0001010)
    VIF_PARAMETER_SET = (0b1111101 << 8) | (0b0001011)
    VIF_MODEL = (0b1111101 << 8) | (0b0001100)
    VIF_HW_VERSION = (0b1111101 << 8) | (0b0001101)
    VIF_FW_VERSION = (0b1111101 << 8) | (0b0001110)
    VIF_SW_VERSION = (0b1111101 << 8) | (0b0001111)
    VIF_CUSTOMER_LOCATION = (0b1111101 << 8) | (0b0010000)
    VIF_CUSTOMER = (0b1111101 << 8) | (0b0010001)
    VIF_DIGITAL_OUT = (0b1111101 << 8) | (0b0011010)
    VIF_DIGITAL_IN = (0b1111101 << 8) | (0b0011011)
    VIF_BAUD_RATE = (0b1111101 << 8) | (0b0011100)
    VIF_RESP_DELAY = (0b1111101 << 8) | (0b0011101)
    VIF_RETRY = (0b1111101 << 8) | (0b0011110)

    @staticmethod
    def VIF_VOLT(e):
        return (0b1111101 << 8) | (0b100 << 4) | (e + 9)

    @staticmethod
    def VIF_AMPERE(e):
        return (0b1111101 << 8) | (0b101 << 4) | (e + 12)

    bcd_encode_table = {
        "0": 0,
        "1": 1,
        "2": 2,
        "3": 3,
        "4": 4,
        "5": 5,
        "6": 6,
        "7": 7,
        "8": 8,
        "9": 9,
        "A": 10,
        "B": 11,
        "C": 12,
        "D": 13,
        "E": 14,
        "F": 15,
    }

    @staticmethod
    def bcd_encode(value, bytes):
        encoded = bytearray(bytes)
        offset = 0
        nibble = 0
        for char in str(value).zfill(bytes * 2)[::-1]:
            ch = Frame.bcd_encode_table[char]
            encoded[offset] += ch << nibble
            offset += 1 if nibble else 0
            nibble = 0 if nibble else 4
        return encoded

    @staticmethod
    def mfr_encode(manufacturer):
        assert len(manufacturer) == 3, "Invalid manufacturer ID"

        mfr = 0
        for chr in manufacturer[::]:
            v = ord(chr) - 64
            assert v >= 0 and v < 32, "Invalid manufacturer ID"
            mfr = mfr * 32 + v
        return struct.pack("<H", mfr)

    @staticmethod
    def fixed_data_header(
        id=ID_NO, mfr=MANUF, ver=VERSION, medium=MEDIUM, access=0, status=0, signature=0
    ):
        return (
            Frame.bcd_encode(id, 4)
            + Frame.mfr_encode(mfr)
            + struct.pack("<B", ver)
            + struct.pack("<B", medium)
            + struct.pack("<B", access % 256)
            + struct.pack("<B", status)
            + struct.pack("<H", signature)
        )

    @staticmethod
    def dib(storage, fn, df, subunit=0, tariff=0):
        data = bytearray(1)
        data[0] = ((storage & 1) << 6) | (fn << 4) | (df << 0)
        storage <<= 1
        while (storage > 0 or subunit > 0 or tariff > 0) and data.len < 10:
            data[-1] |= 1 << 7
            data += bytes(
                [
                    ((subunit & 1) << 6)
                    | ((tariff & 0b11) << 4)
                    | ((storage & 0b1111) << 0)
                ]
            )
            subunit <<= 1
            tariff <<= 2
            storage <<= 4
        assert (
            storage == 0 and subunit == 0 and tariff == 0
        ), "Invalid storage/subunit/tariff"
        return data

    @staticmethod
    def vib(vif, *evif):
        data = bytearray()
        if vif > 0xFF:
            data += bytes([(vif >> 8) | 0x80])
            vif &= 0xFF
        data += bytes([vif])
        for f in evif:
            if data.len > 0:
                data[-1] |= 1 << 7
            data += bytes([f])
        return data

    @staticmethod
    def data_block_int16(
        vif, value, *evif, df=DF_INT16, fn=FN_INST, storage=0, subunit=0, tariff=0
    ):
        return (
            Frame.dib(storage=storage, fn=fn, df=df, subunit=subunit, tariff=tariff)
            + Frame.vib(vif=vif, *evif)
            + struct.pack("<H", value)
        )

    @staticmethod
    def data_block_int8(
        vif, value, *evif, df=DF_INT16, fn=FN_INST, storage=0, subunit=0, tariff=0
    ):
        return (
            Frame.dib(storage=storage, fn=fn, df=df, subunit=subunit, tariff=tariff)
            + Frame.vib(vif=vif, *evif)
            + struct.pack("<H", value)
        )

    @staticmethod
    def nibble_match(byte, match):
        if (match & 0xF0) == 0xF0:
            byte |= 0xF0
        if (match & 0x0F) == 0x0F:
            byte |= 0x0F
        return byte == match

    def byte_match(byte, match):
        return byte == match or match == 0xFF

    def match_secondary(self):
        # 0-3 ID (nibble wildcard)
        # 4-5 Manufacturer
        # 6 Generation
        # 7 Media

        # Is the selector of expected length?
        if len(self._message) != 8:
            return False

        secondary = list(
            Frame.bcd_encode(ID_NO, 4)
            + Frame.mfr_encode(MANUF)
            + struct.pack("<B", VERSION)
            + struct.pack("<B", MEDIUM)
        )
        print("".join(["{:02X}".format(x) for x in secondary]))
        print("".join(["{:02X}".format(x) for x in self._message]))
        # ID is BCD encoded, nibble match
        for i in range(0, 4):
            if not Frame.nibble_match(secondary[i], self._message[i]):
                return False
            pass
        # Rest is binary, byte match
        for i in range(4, 8):
            if not Frame.byte_match(secondary[i], self._message[i]):
                return False
            pass

        return True

    class FrameException(Exception):
        pass

    def __init__(self, type):
        self._type = type
        self._bytes = 0
        self._c_field = None
        self._a_field = None
        self._addressed = False
        self._broadcast = False
        self._network = False
        self._csum = 0
        self._csum_passed = False
        self._data = None
        self._fcb = None
        self._fcv = None
        self._message = []

    def _handle_start(self, byte):
        if byte != LONG_FRAME_START_BYTE:
            warning("Invalid START byte %02x" % byte)

    def _handle_l_field(self, byte, offset):
        if offset == 0:
            self._l_field = byte
        elif self._l_field != byte:
            warning("Inconsistent L field, %d != %d" % (self._l_field, byte))

    def _handle_c_field(self, byte):
        self._fcv = (byte & 1 << 4) != 0
        self._fcb = (byte & 1 << 5) != 0
        byte &= ~(1 << 5)  # Strip FCB bit from the C value

        if self._type == self.SHORT:
            if byte in self._short_c_fields:
                debug("C Field: 0x%2.x" % byte)
                self._c_field = byte
            else:
                debug("Unexpected Short Frame C Field: 0x%2.2x" % byte)
        elif self._type == self.LONG:
            if byte in self._long_c_fields:
                debug("C Field: 0x%2.x" % byte)
                self._c_field = byte
            else:
                debug("Unexpected Long Frame C Field: 0x%2.2x" % byte)
        else:
            raise Frame.VIF_VOLUME_m3FrameException

    def _handle_a_field(self, byte):
        global is_selected
        self._a_field = byte
        if self._a_field == ADDR:
            debug("My Address: %d" % self._a_field)
            self._addressed = True
        elif self._a_field == TEST_ADDR:
            debug("Test Address: %d" % self._a_field)
            self._addressed = True
        elif self._a_field == NETWORK_ADDR:
            self._network = True
            if is_selected:
                debug("Network Address selected: %d" % self._a_field)
                self._addressed = True
            else:
                debug("Network Address NOT selected: %d" % self._a_field)
            pass
        elif self._a_field == BROADCAST_ADDR:
            debug("Broadcast Address selected: %d" % self._a_field)
            self._addressed = True
            self._broadcast = True
        else:
            debug("Not My Address: %d vs my address: %d" % (self._a_field, ADDR))

    def _handle_ci_field(self, byte):
        global is_selected
        self._ci_field = byte
        if self._network and self._ci_field in [self.CI_SELECT, self.CI_SELECT_BE]:
            self._addressed = True
            is_selected = False

    def _handle_checksum(self, byte):
        debug("Checksum received: %2.2x vs stored: %2.2x" % (byte, self._csum))
        self._csum_passed = self._csum == byte

    def _handle_stop(self, byte):
        global is_selected
        debug("Frame received: %2x" % self._c_field)
        if byte != STOP_BYTE:
            warning("Invalid STOP byte %02x" % byte)
            return None

        if self._addressed:
            if self._c_field is None:
                info("Ignoring unknown frame")
                return None
            if self._c_field == self.SND_NKE:
                info("Slave Initialization")
                if self._network:
                    is_selected = False
                return bytearray(
                    [
                        SINGLE_CHAR,
                    ]
                )
            elif self._c_field == self.REQ_UD2:
                info("Request for User Data 2")
                user_data = bytearray([self.RSP_UD, ADDR, CI_VARIABLE_DATA_RSP])
                global ACCESS_NO
                user_data += Frame.fixed_data_header(access=ACCESS_NO)
                ACCESS_NO += 1
                user_data += Frame.data_block_int16(Frame.VIF_VOLT(0), 1234)
                user_data += Frame.data_block_int16(Frame.VIF_VOLUME_m3(0), 456)
                checksum = 0
                ud_len = len(user_data)
                assert ud_len <= 255, "Too long response"
                for byte in user_data:
                    checksum += byte
                checksum %= 256
                return (
                    bytearray(
                        [LONG_FRAME_START_BYTE, ud_len, ud_len, LONG_FRAME_START_BYTE]
                    )
                    + user_data
                    + bytearray([checksum, STOP_BYTE])
                )
            elif self._c_field == self.REQ_UD1:
                return bytearray(
                    [
                        SINGLE_CHAR,
                    ]
                )
            elif self._c_field == self.SND_UD and self._ci_field == self.CI_SELECT:
                if self.match_secondary():
                    is_selected = True
                    info("Secondary selected")
                    return bytearray(
                        [
                            SINGLE_CHAR,
                        ]
                    )
                debug("Secondary selection do not match")
            else:
                raise Frame.FrameException
        else:
            debug("Frame not for us")
        return None

    def handle_byte(self, byte):
        data = None
        frame = self
        if self._type == self.SHORT:
            if self._bytes == 0:
                self._handle_c_field(byte)
            elif self._bytes == 1:
                self._handle_a_field(byte)
            elif self._bytes == 2:
                self._handle_checksum(byte)
            elif self._bytes == 3:
                data = self._handle_stop(byte)
                frame = None
                if self._broadcast:
                    data = None
        elif self._type == self.LONG:
            if self._bytes == 0:
                self._handle_l_field(byte, 0)
            elif self._bytes == 1:
                self._handle_l_field(byte, 1)
            elif self._bytes == 2:
                self._handle_start(byte)
            elif self._bytes == 0 + 3:
                self._handle_c_field(byte)
            elif self._bytes == 1 + 3:
                self._handle_a_field(byte)
            elif self._bytes == 2 + 3:
                self._handle_ci_field(byte)
            elif self._bytes == self._l_field + 3:
                self._handle_checksum(byte)
            elif self._bytes == self._l_field + 3 + 1:
                data = self._handle_stop(byte)
                frame = None
                if self._broadcast:
                    data = None
            else:
                self._message.append(byte)
            pass
        else:
            raise Frame.FrameException
        if self._type == self.SHORT or self._bytes >= 3:
            self._csum = (self._csum + byte) & 0xFF
        self._bytes += 1
        return frame, data


def setup_serial_port():
    if DEVICE == "/dev/ptmx":
        # Pseudo TTY
        fd, slave = pty.openpty()
        info("Point your M-Bus master at %s", os.ttyname(slave))
        debug("Listening on FD=%r", fd)
        ser = PseudoSerial(fd)
    else:
        ser = serial.serial_for_url(
            DEVICE,
            baudrate=BAUDRATE,
            parity=PARITY,
            stopbits=STOPBITS,
            bytesize=BYTESIZE,
        )
    return ser


def handle_byte(frame, byte):
    data = None
    if frame:
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


def signal_handler(sig, frame):
    global ser
    ser.close()
    ser = None


def log():
    global logger
    logging.basicConfig(level=logging.INFO)
    logger = logging.getLogger("pyMbusSlave")
    logger.setLevel(logging.DEBUG)
    info("pyMbusSlave")
    info("  Serial device:   %s", DEVICE)
    info("  Baudrate:        %d", BAUDRATE)
    info("  Slave address:   %d", ADDR)
    info("  Slave device ID: %s", ID_NO)


def main():
    global ser
    log()
    ser = setup_serial_port()
    info("Listening ...")
    frame = None
    while ser:
        try:
            byte = ord(ser.read())
        except:
            break
        frame, data = handle_byte(frame, byte)
        if data != None:
            try:
                send_data(ser, data)
            except:
                break
    info("Exiting ...")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--device", type=str, help="Serial device", default=DEVICE)
    ap.add_argument("--baud", type=int, help="Baud rate", default=BAUDRATE)
    ap.add_argument("--addr", type=int, help="Primary address", default=ADDR)
    ap.add_argument("--id", type=int, help="Serial Number", default=ID_NO)
    ap.add_argument("--manuf", type=str, help="Manufacturer", default=MANUF)

    args = ap.parse_args()

    DEVICE = args.device
    BAUDRATE = args.baud
    ADDR = args.addr
    ID_NO = args.id
    MANUF = args.manuf

    try:
        main()
    except KeyboardInterrupt:
        print("quit")

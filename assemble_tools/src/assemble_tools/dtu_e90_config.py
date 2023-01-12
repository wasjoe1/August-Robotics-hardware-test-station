#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

import time
import sys
import binascii

from serial import Serial

def eprint(*args, **kwargs):
    """
    Print to stderr.

    Parameters
    ----------
    args: list
       Arguments to print
    kwargs: dict
       Keyword arguments to print
    """
    print(*args, file=sys.stderr, **kwargs)

PARITY_DICT = {
    0: '8N1',
    1: '8O1',
    2: '8E1',
    3: '8N1',
}

PACK_DICT = {
    0: 240,
    1: 128,
    2: 64,
    3: 32,
}

BAUDRATE_DICT = {
    0: 1200,
    1: 2400,
    2: 4800,
    3: 9600,
    4: 19200,
    5: 38400,
    6: 57600,
    7: 115200,
}

for k, v in list(BAUDRATE_DICT.items()):
    BAUDRATE_DICT[v] = k

TRANSMITTING_POWER_DICT = {
    0: '1000mW',
    1: '500mW',
    2: '250mW',
    3: '125mW',
}

# unit: bps,
# This value should be the same when communicating.
AIR_SPEED_DICT = {
    0: '0.3k',
    1: '1.2k',
    2: '2.4k',
    3: '4.8k',
    4: '9.6k',
    5: '19.2k',
    6: '38.4k',
    7: '62.5k',
}


if sys.version_info[0] < 3:
    # The following two function is for converting hex to binary
    # Check this answer: https://stackoverflow.com/a/1427846
    def byte_to_binary(n):
        return ''.join(str((n & (1 << i)) and 1) for i in reversed(range(8)))

    def hex_to_binary(h):
        return ''.join(byte_to_binary(ord(b)) for b in binascii.unhexlify(h))

    def get_value_from_regs(bytes, byte_start, byte_len, bit_start, bit_len):
        '''
        for python2

        Get value from registers as bytes.

        bytes[byte_start:byte_start+byte_len][bit_start:bit_start+bit_len]

        '''
        value = bytes[byte_start:byte_start+byte_len]
        value = value.encode('hex')
        value = hex_to_binary(value)
        value = value[bit_start:bit_start+bit_len]
        value = int(value, 2)
        return value
else:
    def get_value_from_regs(bytes, byte_start, byte_len, bit_start, bit_len):
        '''
        for python3

        Get value from registers as bytes.

        bytes[byte_start:byte_start+byte_len][bit_start:bit_start+bit_len]

        '''
        value = bytes[byte_start:byte_start+byte_len]
        value = int.from_bytes(value, byteorder='big')
        bit_end = bit_start + bit_len - 1
        mask = (1 << bit_len) - 1
        right_shift = byte_len * 8 - bit_end - 1
        value = value >> right_shift
        value = value & mask
        return value


class DTUE90Config:
    # Default settings:
    #     Transmitting power: 240mW
    #     Baudrate:           115200
    #     Channel:            60
    #     Pack size:          240
    #     Air speed:          62.5k
    # __DEFAULT_SETTING = b'\xc0\x00\x06\x00\x00\x00\xe7\x00\x3c'

    # Default settings:
    #     Transmitting power: 1W
    #     Baudrate:           115200
    #     Channel:            60
    #     Pack size:          64
    #     Air speed:          62.5k
    __DEFAULT_SETTING = b'\xc0\x00\x06\x00\x00\x00\xe7\x80\x3c'
    def __init__(self, port=None, baudrate=9600, timeout=0.1):
        self._port = Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._current_setting = None
        self.refresh_params()

    def refresh_params(self):
        self._port.flushInput()
        self._port.write(b'\xc1\x00\x06')
        read_configs = self._port.read(9)
        if read_configs[0] in (b'\xc1'):
            self._current_setting = read_configs
            return True

    def set_params_default(self):
        self._port.write(self.__DEFAULT_SETTING)
        time.sleep(0.5)
        self.refresh_params()

    # placeholder function such that we have same function call for both
    # as69 and e90.
    def reset(self):
        pass

    def set_channel(self, chan):
        config_head = bytearray(b'\xc0\x05\x01')
        channel_config = config_head + bytearray([chan])
        self._port.write(channel_config)
        time.sleep(0.5)
        self.refresh_params()

    # the BAUDRATE_DICT contains not just seq number of baudrate, the values are also keys.
    def set_baudrate(self, baud):
        if baud not in BAUDRATE_DICT:
            eprint('bad input, should be one of the {}'.format(BAUDRATE_DICT.keys()))
            return False
        config_head = bytearray(b'\xc0\x03\x01')
        baud = BAUDRATE_DICT[baud] if baud > 8 else baud
        # baud_config = (int(binascii.hexlify(self._current_setting[6]), 16) & 0x1f) | (baud << 5)
        config = get_value_from_regs(self._current_setting, 6, 1, 0, 8)
        baud_config = (config & 0x1f) | (baud << 5)
        baud_config = config_head + bytearray([baud_config])
        self._port.write(baud_config)
        time.sleep(0.5)
        self.refresh_params()

    def set_transmitting_power(self, power=None):
        if power == None:
            eprint('Avialable transmitting powers are:')
            for i in TRANSMITTING_POWER_DICT:
                eprint(i, TRANSMITTING_POWER_DICT[i])
            power = int(raw_input('Input number: '))
        assert power in TRANSMITTING_POWER_DICT.keys(), 'Wrong input!'
        config_head = bytearray(b'\xc0\x04\x01')
        temp_config = get_value_from_regs(self._current_setting, 7, 1, 0, 8)
        temp_config = (temp_config & 0b11111100) | power
        power_config = config_head + bytearray([temp_config])
        self._port.write(power_config)
        time.sleep(0.5)
        self.refresh_params()

    def set_pack(self, pack=None):
        if pack == None:
            eprint('Avialable packs are:')
            for i in PACK_DICT:
                eprint(i, PACK_DICT[i])
            pack = int(raw_input('Input number: '))
        assert pack in PACK_DICT.keys(), 'Wrong input!'
        config_head = bytearray(b'\xc0\x04\x01')
        temp_config = get_value_from_regs(self._current_setting, 7, 1, 0, 8)
        temp_config = (temp_config & 0b00111111) | (pack << 6)
        pack_config = config_head + bytearray([temp_config])
        self._port.write(pack_config)
        time.sleep(0.5)
        self.refresh_params()

    def set_air_speed(self, speed=None):
        if speed == None:
            eprint('Avialable air speeds are:')
            for i in AIR_SPEED_DICT:
                eprint(i, AIR_SPEED_DICT[i])
            speed = int(raw_input('Input number: '))
        assert speed in AIR_SPEED_DICT.keys(), 'Wrong input!'
        config_head = bytearray(b'\xc0\x03\x01')
        temp_config = get_value_from_regs(self._current_setting, 6, 1, 0, 8)
        temp_config = (temp_config & 0b11111000) | speed
        speed_config = config_head + bytearray([temp_config])
        self._port.write(speed_config)
        time.sleep(0.5)
        self.refresh_params()

    def close(self):
        self._port.close()

    @property
    def temporary_config(self):
        config = get_value_from_regs(self._current_setting, 0, 1, 0, 8)
        return config == 0xc2

    @property
    def address(self):
        return get_value_from_regs(self._current_setting, 3, 2, 0, 16)

    @property
    def net_id(self):
        return get_value_from_regs(self._current_setting, 5, 1, 0, 8)

    @property
    def parity(self):
        config = get_value_from_regs(self._current_setting, 6, 1, 3, 2)
        return PARITY_DICT[config]

    @property
    def baudrate(self):
        config = get_value_from_regs(self._current_setting, 6, 1, 0, 3)
        return BAUDRATE_DICT[config]

    @property
    def channel(self):
        return get_value_from_regs(self._current_setting, 8, 1, 0, 8)

    @property
    def transmitting_power(self):
        config = get_value_from_regs(self._current_setting, 7, 1, 6, 2)
        return TRANSMITTING_POWER_DICT[config]

    @property
    def air_speed(self):
        config = get_value_from_regs(self._current_setting, 6, 1, 5, 3)
        return AIR_SPEED_DICT[config]

    @property
    def pack(self):
        config = get_value_from_regs(self._current_setting, 7, 1, 0, 2)
        return PACK_DICT[config]

if __name__ == "__main__":
    import unittest

    class TestDTU(unittest.TestCase):
        def setUp(self):
            self.dtu = DTUE90Config(port='/dev/ttyUSB0')
            self.channel = self.dtu.channel

        def tearDown(self):
            self.dtu.set_params_default()
            self.dtu.set_channel(self.channel)

        def test_default_config(self):
            self.dtu.set_params_default()
            self.assertEqual(self.dtu.temporary_config, False)
            self.assertEqual(self.dtu.transmitting_power, '1000mW')
            self.assertEqual(self.dtu.baudrate, 115200)
            self.assertEqual(self.dtu.air_speed, '62.5k')
            self.assertEqual(self.dtu.pack, 64)
            self.assertEqual(self.dtu.channel, 60)
            self.assertEqual(self.dtu.net_id, 0)
            self.assertEqual(self.dtu.address, 0)
            self.assertEqual(self.dtu.parity, '8N1')

        def test_set_channel(self):
            for i in range(84):
                self.dtu.set_channel(i)
                eprint("Test set_channel: {}, result: {}".format(i, self.dtu.channel))
                self.assertEqual(self.dtu.channel, i)

        def test_set_baudrate(self):
            for i in BAUDRATE_DICT.keys():
                self.dtu.set_baudrate(i)
                eprint('Testing baudrate: {}, result: {}'.format(i, self.dtu.baudrate))
                if i > 8:
                    self.assertEqual(self.dtu.baudrate, i)
                else:
                    self.assertEqual(self.dtu.baudrate, BAUDRATE_DICT[i])

        def test_set_air_speed(self):
            for i in AIR_SPEED_DICT.keys():
                self.dtu.set_air_speed(i)
                eprint('Testing air speed: {}, result: {}'.format(i, self.dtu.air_speed))
                self.assertEqual(self.dtu.air_speed, AIR_SPEED_DICT[i])

        def test_set_transmitting_power(self):
            for i in TRANSMITTING_POWER_DICT.keys():
                self.dtu.set_transmitting_power(i)
                eprint('Testing transmitting power: {}, result: {}'.format(i, self.dtu.transmitting_power))
                self.assertEqual(self.dtu.transmitting_power, TRANSMITTING_POWER_DICT[i])

        def test_set_pack(self):
            for i in PACK_DICT.keys():
                self.dtu.set_pack(i)
                eprint('Testing pack: {}, result: {}'.format(i, self.dtu.pack))
                self.assertEqual(self.dtu.pack, PACK_DICT[i])

    unittest.main()

import smbus
import struct


class WideBoyAStar(object):
    def __init__(self, i2c_addr = 20):
        self.bus = smbus.SMBus(1)
        self.i2c_addr = i2c_addr;

    def read_unpack(self, address, size, fmt):
        # Ideally we could do this:
        #    byte_list = self.bus.read_i2c_block_data(self.i2c_addr, address, size)
        # But the AVR's TWI module can't handle a quick write->read transition,
        # since the STOP interrupt will occasionally happen after the START
        # condition, and the TWI module is disabled until the interrupt can
        # be processed.

        while True:
            try:
                self.bus.write_byte(self.i2c_addr, address)
                byte_list = []
                for n in range(0, size):
                   byte_list.append(self.bus.read_byte(self.i2c_addr))
        #        byte_list = self.bus.read_i2c_block_data(self.i2c_addr, address, size)
                return struct.unpack(fmt, bytes(bytearray(byte_list)))
            except IOError:
                print(IOError)

    def write_pack(self, address, fmt, *data):
        data_array = map(ord, list(struct.pack(fmt, *data)))
        self.bus.write_i2c_block_data(self.i2c_addr, address, data_array)

    def set_leds(self, red, yellow, green):
        self.write_pack(25, 'BBB', red, yellow, green)

    def play_notes(self, notes):
        self.write_pack(28, 'B17s', 1, notes.encode("ascii"))

    def set_motor_speeds(self, left, right):
        self.write_pack(20, 'hh', left, right)

    def clear_motor_counts(self):
        self.write_pack(24, 'B', True)

    def get_motor_counts(self):
        return self.read_unpack(8, 8, 'LL');

    def get_motor_speeds(self):
        return self.read_unpack(16, 4, 'HH');

    def get_motor_state(self):
        return self.read_unpack(8, 16, 'LLHHHH')

    def is_button_a_pushed(self):
        value = self.read_unpack(0, 1, 'B')[0];
        if value > 0:
            self.write_pack(0, 'B', 0)
            return True

        return False

    def is_button_b_pushed(self):
        value = self.read_unpack(1, 1, 'B')[0];
        if value > 0:
            self.write_pack(1, 'B', 0)
            return True

        return False

    def is_button_c_pushed(self):
        value = self.read_unpack(2, 1, 'B')[0];
        if value > 0:
            self.write_pack(2, 'B', 0)
            return True

        return False

    def get_cell_count(self):
        return self.read_unpack(3, 1, 'B')[0];

    def get_low_voltage_cutoff(self):
        return self.read_unpack(6, 2, 'H')[0];

    def set_low_voltage_cutoff(self, millivolts):
        self.write_pack(6, 'H', millivolts)

    def get_battery_millivolts(self):
        return self.read_unpack(4, 2, "H")[0];


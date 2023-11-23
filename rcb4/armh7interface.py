from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection
import numpy as np
import serial

from rcb4.asm import four_bit_to_num
from rcb4.asm import rcb4_checksum
from rcb4.asm import rcb4_servo_ids_to_5bytes
from rcb4.asm import rcb4_servo_positions
from rcb4.asm import rcb4_servo_svector
from rcb4.asm import rcb4_velocity
from rcb4.ctype_utils import c_type_to_size
from rcb4.data import kondoh7_elf
from rcb4.rcb4interface import CommandTypes
from rcb4.rcb4interface import rcb4_dof
from rcb4.rcb4interface import ServoParams
from rcb4.struct_header import c_vector
from rcb4.struct_header import Madgwick
from rcb4.struct_header import max_sensor_num
from rcb4.struct_header import sensor_sidx
from rcb4.struct_header import SensorbaseStruct
from rcb4.struct_header import ServoStruct


armh7_variable_list = [
    "walking_command",
    "walking_mode",
    "rom_to_flash",
    "flash_to_rom",
    'Mfilter',
    "set_sidata_command",
    "set_sdata_command",
    "set_edata_command",
    "sidata_to_dataram",
    "dataflash_to_dataram",
    'servo_vector',
    "dataram_to_dataflash",
    "_sidata",
    "_sdata",
    "uwTickPrio",
    "__fdlib_version",
    "_impure_ptr",
    "_edata",
    "_sbss",
    "_ebss",
    "servo_idmode_scan",
    "buzzer_init_sound",
    "servo_idmode_scan_single",
    'imu_data_',
    'Sensor_vector',
]


def get_func_address(path, name):
    address = None
    with open(path, 'rb') as stream:
        elf = ELFFile(stream)
        for section in elf.iter_sections():
            if isinstance(section, SymbolTableSection):
                for symbol in section.iter_symbols():
                    if symbol.name == name:
                        address = symbol.entry['st_value']
                        break
            if address:
                break
        else:
            raise RuntimeError('Failed to find {}'.format(name))
    return address


armh7_elf_alist = {name: get_func_address(kondoh7_elf(), name)
                   for name in armh7_variable_list}

servo_eeprom_params64 = {
    'fix_header': (1, 2),
    'stretch_gain': (3, 4),
    'speed': (5, 6),
    'punch': (7, 8),
    'dead_band': (9, 10),
    'dumping': (11, 12),
    'safe_timer': (13, 14),
    'mode_flag_b7slave_b4rotation_b3pwm_b1free_b0reverse': (15, 16),
    'pulse_max_limit': (17, 18, 19, 20),
    'pulse_min_limit': (21, 22, 23, 24),
    'fix_dont_change_25': (25, 26),
    'ics_baud_rate_10_115200_00_1250000': (27, 28),
    'temperature_limit': (29, 30),
    'current_limit': (31, 32),
    'fix_dont_change_33': (33, 34),
    'fix_dont_change_35': (35, 36),
    'fix_dont_change_37': (37, 38),
    'fix_dont_change_39': (39, 40),
    'fix_dont_change_41': (41, 42),
    'fix_dont_change_43': (43, 44),
    'fix_dont_change_45': (45, 46),
    'fix_dont_change_47': (47, 48),
    'fix_dont_change_49': (49, 50),
    'response': (51, 52),
    'user_offset': (53, 54),
    'fix_dont_change_55': (55, 56),
    'servo_id': (57, 58),
    'stretch_1': (59, 60),
    'stretch_2': (61, 62),
    'stretch_3': (63, 64),
}


class ARMH7Interface(object):

    def __init__(self):
        self.serial = None
        self.id_vector = None

    def __del__(self):
        self.close()

    def open(self, port='/dev/ttyUSB0', baudrate=1000000, timeout=0.01):
        """Opens a serial connection to the ARMH7 device.

        Parameters
        ----------
        port : str
            The port name to connect to.
        baudrate : int
            The baud rate for the serial connection.
        timeout : float, optional
            The timeout for the serial connection in seconds (default is 0.01).

        Returns
        -------
        None

        Raises
        ------
        serial.SerialException
            If there is an error opening the serial port.
        """
        try:
            self.serial = serial.Serial(port, baudrate, timeout=timeout)
            print(f"Opened {port} at {baudrate} baud")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
        return self.check_ack()

    def close(self):
        if self.serial:
            self.serial.close()

    def serial_write(self, byte_list):
        if self.serial is None:
            raise RuntimeError('Serial is not opened.')

        data_to_send = bytes(byte_list)
        try:
            self.serial.write(data_to_send)
        except serial.SerialException as e:
            print(f"Error sending data: {e}")
        return self.serial_read()

    def serial_read(self):
        if self.serial is None:
            raise RuntimeError('Serial is not opened.')

        ret = ''
        while len(ret) == 0:
            try:
                ret = self.serial.read_until()
            except serial.SerialException as e:
                print(f"Error reading data: {e}")
                return None

        while len(ret) > 0 and ret[0] != len(ret):
            try:
                ret += self.serial.read_until()
            except serial.SerialException as e:
                print(f"Error reading data: {e}")
                return None
        return ret[1:len(ret) - 1]

    def get_ack(self):
        byte_list = [0x04, 0xFE, 0x06, 0x08]
        return self.serial_write(byte_list)

    def check_ack(self):
        ack_byte_list = self.get_ack()
        return ack_byte_list[1] == 0x06

    def memory_read_aux(self, addr, length):
        """Memory Read Aux function"""
        cnt = 1
        e_size = length
        skip_size = 0
        n = 11
        byte_list = [0] * n
        byte_list[0] = n
        byte_list[1] = 0xFB  # MREADV_OP
        byte_list[2] = addr & 0xFF
        byte_list[3] = (addr >> 8) & 0xFF
        byte_list[4] = (addr >> 16) & 0xFF
        byte_list[5] = (addr >> 24) & 0xFF
        byte_list[6] = cnt
        byte_list[7] = e_size
        byte_list[8] = skip_size & 0xFF
        byte_list[9] = (skip_size >> 8) & 0xFF
        byte_list[10] = rcb4_checksum(byte_list)
        return self.serial_write(byte_list)

    def memory_read(self, addr, length):
        limit = 250
        if length < limit:
            return self.memory_read_aux(addr, length)
        elif length < 2 * limit:
            return self.memory_read_aux(addr, limit) \
                + self.memory_read_aux(addr + limit, length - limit)
        else:
            return self.memory_read_aux(addr, limit) \
                + self.memory_read_aux(addr + limit, limit) \
                + self.memory_read_aux(addr + 2 * limit, length - 2 * limit)

    def memory_cstruct(self, cls, v_idx=0, addr=None, size=None):
        if addr is None:
            addr = armh7_elf_alist[cls.__name__]
        if size is None:
            size = cls.size
        buf = self.memory_read((size * v_idx) + addr, size)
        return cls(buf)

    def read_cstruct_slot_v(self, cls, slot_name):
        vcnt = c_vector[cls.__name__]
        addr = armh7_elf_alist[cls.__name__]
        cls_size = cls.size
        slot_offset = cls.__fields_types__[slot_name].offset
        element_size = c_type_to_size(cls.__fields_types__[slot_name].c_type)
        n = 11
        byte_list = np.zeros(n, dtype=np.uint8)
        byte_list[0] = n
        byte_list[1] = 0xFB
        byte_list[2:6] = np.array([(addr + slot_offset) >> (i * 8) & 0xff
                                   for i in range(4)], dtype=np.uint8)
        byte_list[6] = vcnt
        byte_list[7] = element_size
        byte_list[8] = cls_size
        byte_list[n - 1] = rcb4_checksum(byte_list)
        b = self.serial_write(byte_list)
        return np.frombuffer(b, dtype=np.uint16)

    def read_jointbase_sensor_ids(self):
        if self.id_vector is None:
            ret = []
            for i in range(max_sensor_num):
                sensor = self.memory_cstruct(
                    SensorbaseStruct, i)
                port = sensor.port
                id = sensor.id
                if port > 0 and id == (i + sensor_sidx) // 2:
                    ret.append(i + sensor_sidx)
            self.id_vector = list(reversed(ret))
        return self.id_vector

    def reference_angle_vector(self):
        return self.read_cstruct_slot_v(
            ServoStruct, slot_name='ref_angle')

    def angle_vector(self):
        return self.read_cstruct_slot_v(ServoStruct,
                                        slot_name='current_angle')

    def search_servo_ids(self):
        indices = []
        for idx in range(rcb4_dof):
            servo = self.memory_cstruct(ServoStruct, idx)
            if servo.flag > 0:
                indices.append(idx)
        self.servo_sorted_ids = indices
        return indices

    def hold(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [32767] * len(servo_ids)
        return self.servo_angle_vector(servo_ids,
                                       servo_vector,
                                       velocity=1000)

    def free(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [32768] * len(servo_ids)
        return self.servo_angle_vector(servo_ids,
                                       servo_vector,
                                       velocity=1000)

    def neutral(self, servo_ids=None, velocity=1000):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        servo_vector = [7500] * len(servo_ids)
        return self.servo_angle_vector(servo_ids,
                                       servo_vector,
                                       velocity=velocity)

    def servo_angle_vector(self, servo_ids, servo_vector, velocity=1000):
        byte_list = [CommandTypes.MultiServoSingleVelocity.value] \
            + rcb4_servo_ids_to_5bytes(servo_ids) \
            + [rcb4_velocity(velocity)] \
            + rcb4_servo_positions(servo_ids, servo_vector)
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))
        return self.serial_write(byte_list)

    def servo_param64(self, sid, param_names=None):
        v = self.memory_cstruct(ServoStruct, sid)
        param_values = {}
        if param_names is None:
            param_names = list(servo_eeprom_params64.keys())
        for param_name in param_names:
            indices = servo_eeprom_params64[param_name]
            param_value = four_bit_to_num(indices, v.params)
            param_values[param_name] = param_value
        return param_values

    def read_stretch(self, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        return [self.servo_param64(sid, ['stretch_gain'])['stretch_gain'] // 2
                for sid in servo_ids]

    def send_stretch(self, value=127, servo_ids=None):
        if servo_ids is None:
            servo_ids = self.servo_sorted_ids
        if not isinstance(value, list) or not isinstance(value, tuple):
            value = [value] * len(servo_ids)
        byte_list = [CommandTypes.ServoParam.value] \
            + rcb4_servo_ids_to_5bytes(servo_ids) \
            + [ServoParams.Stretch.value] \
            + rcb4_servo_svector(servo_ids, value)
        byte_list.insert(0, 2 + len(byte_list))
        byte_list.append(rcb4_checksum(byte_list))
        return self.serial_write(byte_list)

    def read_quaternion(self):
        cs = self.memory_cstruct(Madgwick, 0)
        return np.array([cs.q0, cs.q1, cs.q2, cs.q3], dtype=np.float32)

    def read_rpy(self):
        cs = self.memory_cstruct(Madgwick, 0)
        return [cs.roll, cs.pitch, cs.yaw]

    def gyro_norm_vector(self):
        g = self.memory_cstruct(Madgwick, 0).gyro
        n = np.sqrt(g[0] ** 2 + g[1] ** 2 + g[2] ** 2)
        return (n, g)


if __name__ == '__main__':
    interface = ARMH7Interface()
    print(interface.open())
    indices = interface.search_servo_ids()

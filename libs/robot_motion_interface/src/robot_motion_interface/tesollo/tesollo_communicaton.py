# Adapted from https://github.com/Tesollo-Delto/DELTO_B_ROS2/blob/devel/delto_3f_driver/delto_utility/delto_modbus_TCP.py

from robot_motion_interface.tesollo.tesollo_enum import Delto3F, Delto3FCoils, Delto3FHoldingRegisters, Delto3FInputRegisters
from pymodbus.client.tcp import ModbusTcpClient

import sys
import os
import threading
import struct

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


'''
default values
IP = '169.254.186.72'
PORT = 502
SUBNET_MASK = '255.255.255.0'
DEFAULT_GATEWAY = '192.168.1.1'
'''


class TesolloCommunication:
    '''
    Communication sends command and receives data from Delto Gripper
    '''

    def __init__(self, dummy=False):
        self.client = None
        self.device_id = 0
        self.dummy = dummy
        self.lock = threading.Lock()

    def __del__(self):
        self.disconnect()

    def write_registers(self, address, values):
        self.client.write_registers(
            address=address, values=values, device=self.device_id)

    def connect(self, ip, port, device_id=1):
        '''
        Connect to Delto Gripper
        '''

        self.device_id = device_id
        if self.dummy:
            print(sys._getframe().f_code.co_name)

            return

        self.client = ModbusTcpClient(host=ip, port=port)
        return self.client.connect()

    def disconnect(self):
        '''
        Disconnect from Delto Gripper
        '''
        if self.dummy:
            print(sys._getframe().f_code.co_name)
            return

        # self.client.close()

    def get_position(self):

        if self.dummy:
            status = [0]*12
            print(sys._getframe().f_code.co_name)

            return status

        # status = []
        status = self.client.read_input_registers(
            address=Delto3FInputRegisters.MOTOR1_CURRENT_POSITION.value,
            count=Delto3F.MOTOR_NUM.value,
            device_id=self.device_id).registers

        for i in range(Delto3F.MOTOR_NUM.value):
            # stats = self.client.read_input_registers(
            #     address=Delto3FInputRegisters.MOTOR1_CURRENT_POSITION.value + i,
            #     count=1,
            #     device=self.device_id).registers
            status[i] = (status[i] if status[i] <
                         32768 else status[i] - 65536)/10.0
            # unsigned 8bit to singed 8 bit
            # stats = (stats[0] if stats[0] < 32768 else stats[0] - 65536)/10.0
            # status.append(struct.unpack('h', struct.pack('H', stats[0]))[0]/10)
            # status.append(stats)
        return status

    def get_high_force(self):

        with self.lock:
            high_force = self.client.read_input_registers(address=Delto3FInputRegisters.HIGH_FORCE.value,
                                                          count=1,
                                                          device=self.device_id).registers
        return high_force[0]

    def get_low_force(self):

        with self.lock:
            low_force = self.client.read_input_registers(address=Delto3FInputRegisters.LOW_FORCE.value,
                                                         count=1,
                                                         device=self.device_id).registers
        return low_force[0]

    def set_position(self, position: list[float]):

        if (self.dummy):
            print(sys._getframe().f_code.co_name)

            return

        if (len(position) != 12):
            print(sys._getframe().f_code.co_name +
                                         " position size is not 12")
            return

        with self.lock:

            intPosion = list(map(lambda x: struct.unpack(
                'H', struct.pack('h', int((x*10))))[0], position))
            self.client.write_registers(
                address=72, values=intPosion, device=self.device_id)

    def get_pgain(self):
        if self.dummy:
            print(sys._getframe().f_code.co_name)

            return

        with self.lock:
            pGain = self.client.read_holding_registers(address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
                                                       count=Delto3F.MOTOR_NUM.value,
                                                       device=self.device_id).registers
        return pGain

    def set_pgain(self, pGain: list[int]):
        # print("setPGain", pGain)
        pGain = list(pGain)
        print("setPGain", pGain)
        self.client.write_registers(address=Delto3FHoldingRegisters.MOTOR1_PGAIN.value,
                                    values=pGain, device=self.device_id)

    def get_dgain(self):
        if self.dummy:
            print(sys._getframe().f_code.co_name)

            return

        with self.lock:
            dGain = self.client.read_holding_registers(address=Delto3FHoldingRegisters.MOTOR1_DGAIN.value,
                                                       count=Delto3F.MOTOR_NUM.value,
                                                       device=self.device_id).registers
        return dGain

    def set_dgain(self, dGain: list[int]):
        self.client.write_registers(address=Delto3FHoldingRegisters.MOTOR1_DGAIN.value,
                                    values=dGain,
                                    device=self.device_id)

    def set_free(self, isFree: bool):

        self.client.write_coil(
            address=Delto3FCoils.JOINT_CUSTOM_MODE.value, value=isFree, device=self.device_id)

    def grasp_mode(self, mode):
        if (mode == 0):
            self.grasp(False)
        else:
            self.client.write_register(address=Delto3FHoldingRegisters.GRASP_MODE.value,
                                       value=mode,
                                       device=self.device_id)

            self.grasp(True)

    def get_grasp_mode(self):

        with self.lock:
            mode = self.client.read_input_registers(address=Delto3FHoldingRegisters.GRASP_MODE.value,
                                                    count=1,
                                                    device=self.device_id).registers

        return mode

    def grasp(self, isGrasp: bool):
        with self.lock:
            print("Grasp", isGrasp)
            self.client.write_coil(address=Delto3FCoils.GRASP.value,
                                   value=isGrasp,
                                   device=self.device_id)

    def set_step(self, step):

        if (step < 1):
            step = 1
        elif (step > 32767):
            step = 32767

        self.client.write_register(address=Delto3FHoldingRegisters.MOTION_STEP.value,
                                   value=step, device=self.device_id)

    def rom_write(self):
        self.client.write_coil(address=Delto3FCoils.EEPROM_WRITE.value,
                               value=True,
                               device=self.device_id)

    def set_ip(self, ip: str):

        if self.dummy:
            print(sys._getframe().f_code.co_name)
            return

        ip = ip.split('.')

        if len(ip) != 4:

            print("Invalid IP address")
            return

        ip = list(map(int, ip))

        self.client.write_registers(address=Delto3FHoldingRegisters.ETHERNET_IP_CLASS_A.value,
                                    values=ip,
                                    device=self.device_id)

    def set_subnet_mask(self, subnet_mask: str):

        if self.dummy:
            print(sys._getframe().f_code.co_name)
            return

        subnet_mask = subnet_mask.split('.')

        if len(subnet_mask) != 4:

            print("Invalid subnet mask")
            return

        subnet_mask = list(map(int, subnet_mask))
        self.client.write_registers(Delto3FHoldingRegisters.ETHERNET_SUBNET_MASK_A.value,
                                    values=subnet_mask,
                                    device=self.device_id)

    def set_gate_way(self, gateway: str):

        if self.dummy:
            print(sys._getframe().f_code.co_name)
            return

        gateway = gateway.split('.')

        if len(gateway) != 4:

            print("Invalid gateway")
            return

        gateway = list(map(int, gateway))
        self.client.write_registers(Delto3FHoldingRegisters.ETHERNET_GATEWAY_A.value,
                                    values=gateway,
                                    device=self.device_id)

    def fix_position(self, position: list):
        """_summary_
        accessible from firmware version 1.5 or higher
        Args:
            position (list): [0, 1] 0: free, 1: fix
        """
        if (self.dummy):
            print(sys._getframe().f_code.co_name)
            return

        if (len(position) != 12):
            print(sys._getframe().f_code.co_name +
                                         " position size is not 12")
            return

        with self.lock:
            # print(position)
            self.client.write_registers(
                address=Delto3FHoldingRegisters.MOTOR1_FIXED_POSITION.value, values=position, device=self.device_id)

    def load_pose(self, pose_index: int):
        """_summary_
        accessible from firmware version 1.5 or higher
        """
        with self.lock:
            self.client.write_register(
                address=Delto3FHoldingRegisters.MOTION_LOAD.value, value=pose_index, device=self.device_id)

    def save_pose(self, pose_index: int):
        """_summary_
        accessible from firmware version 1.5 or higher
        """
        with self.lock:
            self.client.write_register(
                address=Delto3FHoldingRegisters.SAVE_TARGET_POSE.value, value=pose_index, device=self.device_id)


'''
change ip Example 

    comm = Communication()
    comm.connect('169.254.186.72',502)
    comm.set_ip('169.254.186.73')
    comm.rom_write()

    The changed IP is applied only after restarting the power.

'''
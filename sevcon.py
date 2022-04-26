#!/usr/bin/python
# -*- coding: utf-8 -*-
# The MIT License (MIT)
# Copyright (c) 2020 Cansin Demir
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from platform import node
from types import DynamicClassAttribute
import canopen
from can import CanError
import sys
import logging
from time import sleep
import struct
import time
from can import CanError  # TODO restudy need of this import since canopen already use it
import paho.mqtt.client as paho

broker="192.168.31.150"
port= 8080
def on_subscribe(client, userdata, mid, granted_qos):   #create function for callback
   print("subscribed with qos",granted_qos, "\n")
   pass
def on_message(client, userdata, message):
    print("message received  "  ,str(message.payload.decode("utf-8")))
def on_disconnect(client, userdata, rc):
   print("client disconnected ok")
def on_log(client, userdata, level, buf):
    print("log: ",buf)

client= paho.Client("client",transport="websockets")       #create client object
client.tls_set('/home/pi/mosquitto-certificate-authority.crt')
client.username_pw_set("Viena","fiatelettra")
#client.on_publish = on_publish        #assign function to callback
client.on_message = on_message        #assign function to callback
client.on_disconnect = on_disconnect
print("connecting to broker ",broker,"on port ",port)
client.connect(broker,port)           #establish connection
client.loop_start()
time.sleep(3)

StateDictionary = {'INIT' : 1,
                   'CHECK_ERRORS' : 2,
                   'CHECK_STATE' : 3,
                   'CHECK_TORQUE' : 4,
                   'CHECK_SPEED' : 5,
                   'HANDLE_ERRORS': 6}


class SEVCON:
    # Properties
    network = None
    node = None
    _connected = False
    objectIndex = {  # control unit objects, independent of drive:
        'Device Type': 0x1000,
        'Error Register': 0x1001,
        'Error History': 0x1003,
        'COB-ID SYNC Message': 0x1005,
        'Device Name': 0x1008,
        'Software Version': 0x100A,
        'Guard Time': 0x100C,
        'Life Time Factor': 0x100D,
        'Store Parameters': 0x1010,
        'Restore Default Parameters': 0x1011,
        'COB-ID Emergency Object': 0x1014,
        # 'Consumer Heartbeat Time': 0x1016,
        'Producer Heartbeat Time': 0x1017,
        'Identity Object': 0x1018,
        # 'Verify configuration': 0x1020,
        'Module List': 0x1027,
        'Error Behavior': 0x1029,
        'Server SDO Default Parameter': 0x1200,
        'Server SDO CU DO Parameter': 0x1201,
        # other communication objects
        'Server SDO Drive DO 1 Parameter': 0x1202,
        'Server SDO Drive DO 2 Parameter': 0x1203,
        'Server SDO Drive DO 3 Parameter': 0x1204,
        'Server SDO Drive DO 4 Parameter': 0x1205,
        'Server SDO Drive DO 5 Parameter': 0x1206,
        'Server SDO Drive DO 6 Parameter': 0x1207,
        'Server SDO Drive DO 7 Parameter': 0x1208,
        'Server SDO Drive DO 8 Parameter': 0x1209,
        # Drive dependent objects
        'Receive PDO 1 Parameter': 0x1400,
        'Receive PDO 2 Parameter': 0x1401,
        'Receive PDO 3 Parameter': 0x1402,
        'Receive PDO 4 Parameter': 0x1403,
        'Receive PDO 5 Parameter': 0x1404,
        'Receive PDO 6 Parameter': 0x1405,
        'Receive PDO 7 Parameter': 0x1406,
        'Receive PDO 8 Parameter': 0x1407,
        'Receive PDO 1 Mapping': 0x1600,
        'Receive PDO 2 Mapping': 0x1601,
        'Receive PDO 3 Mapping': 0x1602,
        'Receive PDO 4 Mapping': 0x1603,
        'Receive PDO 5 Mapping': 0x1604,
        'Receive PDO 6 Mapping': 0x1605,
        'Receive PDO 7 Mapping': 0x1606,
        'Receive PDO 8 Mapping': 0x1607,
        'Transmit PDO 1 Parameter': 0x1800,
        'Transmit PDO 2 Parameter': 0x1801,
        'Transmit PDO 3 Parameter': 0x1802,
        'Transmit PDO 4 Parameter': 0x1803,
        'Transmit PDO 5 Parameter': 0x1804,
        'Transmit PDO 6 Parameter': 0x1805,
        'Transmit PDO 7 Parameter': 0x1806,
        'Transmit PDO 8 Parameter': 0x1807,
        'Transmit PDO 1 Mapping': 0x1A00,
        'Transmit PDO 2 Mapping': 0x1A01,
        'Transmit PDO 3 Mapping': 0x1A02,
        'Transmit PDO 4 Mapping': 0x1A03,
        'Transmit PDO 5 Mapping': 0x1A04,
        'Transmit PDO 6 Mapping': 0x1A05,
        'Transmit PDO 7 Mapping': 0x1A06,
        'Transmit PDO 8 Mapping': 0x1A07,
        'Current limit': 0x2280,
        'Technology controller enable': 0x2898,
        'Technology controller filter time constant': 0x28D9,
        'Technology controller differentiation time constant': 0x28E2,
        'Technology controller proportional gain': 0x28E8,
        'Technology controller maximum limiting': 0x28F3,
        'Technology controller minimum limiting': 0x28F4,
        'Output frequency': 0x2042,
        'Speed setpoint smoothed': 0x2014,
        'Output frequency smoothed': 0x2018,
        'Output voltage smoothed': 0x2019,
        'DC link voltage smoothed': 0x201A,
        'Absolute actual current smoothed': 0x201B,
        'Actual torque smoothed': 0x201F,
        'Actual active power smoothed': 0x2020,
        'Motor temperature': 0x2023,
        'Power unit temperatures': 0x2025,
        'Energy display': 0x2027,
        'Command Data Set CDS effective': 0x2032,
        'Statusword 2': 0x2035,
        'Controlword 1': 0x2036,
        'CU digital input status': 0x22D2,
        'CU digital output status': 0x22EB,
        'CU analog inputs voltage/current': 0x22F0,
        'CU analog outputs voltage/current': 0x2306,
        'Fault number': 0x23B3,
        'Actual pulse frequency': 0x2709,
        'Alarm number': 0x283E,
        'Technology controller setpoint after ramp': 0x28D4,
        'Technology controller actual value after filter': 0x28DA,
        'Technology controller output signal': 0x28F6,
        'Technology controller ramp-up time': 0x28D1,
        'Technology controller ramp-down time': 0x28D2,
        'Technology controller integral action time': 0x28ED,
        # DS402 profile
        # device control
        'Abort connection option code': 0x6007,
        'ControlWord': 0x6040,
        'StatusWord': 0x6041,
        'Stop option code': 0x605D,
        'Modes of Operation': 0x6060,
        'Modes of Operation Display': 0x6061,
        'Supported drive modes': 0x6502,
        'Drive manufacturer': 0x6504,
        'Single device type': 0x67FF,
        # Factor group
        'Velocity encoder': 0x6094,
        # profile velocity mode
        'Max Profile Velocity': 0x607F,
        'Profile Velocity': 0x6081,
        # TODO
        'TargetVelocity': 0x60FF,
        # profile torque mode
        # TODO
        # velocity mode
        'vl target velocity': 0x6042,
        'vl velocity demand': 0x6043,
        'vl velocity actual value:': 0x6044,
        'vl velocity limits': 0x6046,
        'vl velocity acceleration': 0x6048}
    
    sevconDictionary = {
        'FaultInfo': 0x5300 ,
        # hatalar için yeni sözlük oluşturulmalı 
        'Motor Drive Power Steer Info': 0x2060}

    statuswordDict = {
        0x20: 'Not ready to switch on',
        0x0: 'Not ready to switch on',
        0x40: 'Switch on disabled',
        0x60: 'Switch on disabled',
        0x21: 'Ready to switch on',
        0x31: 'Ready to switch on',
        0x23: 'Switched on',
        0x33: 'Switched on',
        0x27: 'Operation enabled',
        0x37: 'Operation enabled',
        0x7: 'Quick stop active',
        0X17: 'Quick stop active',
        0xF: 'Fault reaction active',
        0x2F: 'Fault reaction active',
        0x8: 'Fault',
        0x28: 'Fault'}

    isDict = {
        'voltage_enabled': {0: 'disabled', 1: 'enabled'},
        'warning': {0: 'cleared', 1: 'set'},
        'remote': {0: 'disabled', 1: 'enabled'},
        'target_reached': {0: 'not reached', 1: 'reached'},
        'int_limit_active': {0: 'cleared', 1: 'active'},
        'op_mode_specific': {0: 'off', 1: 'on'}
    }

    

    # DICTIONARY

    # Methods
    def __init__(self,_network=None,debug=False):
        # check if network is passed over or create a new one
        if not _network:
            self.network = canopen.Network()
        else:
            self.network = _network

    def begin(self, nodeID, _channel='can0', _bustype='socketcan', object_dictionary=None):
        """Initialize SEVCON device

        Configure and setup SEVCON device.

        Args:
            nodeID:    Node ID of the device.
            _channel (optional):   Port used for communication. Default can0
            _bustype (optional):   Port type used. Default socketcan.
            object_dictionary (optional):   Name of EDS file, if any available.
        Return: 
            bool: A boolean if all went ok.
        """
        try:
            self.node = self.network.add_node(
                nodeID, object_dictionary=object_dictionary)
            self.network.connect(channel=_channel, bustype=_bustype)
            self._connected = True
            #val, _ = self.read_statusword()  # test if we really have response or is only connected to CAN bus
            #if val is None:
            #    self._connected = False
        except Exception as e:
            self.log_info("Exception caught:{0}".format(str(e)))
            self._connected = False
        finally:
            return self._connected

    def disconnect(self):
        self.network.disconect()
        self._connected = False
        return

    # --------------------------------------------------------------
    # Basic set of functions
    # --------------------------------------------------------------

    def read_object(self, index, subindex):
        """Read object from device

        Args:
            index: reference of dictionary object index
            subindex: reference of dictionary object subindex
        
        Return: 
            
        """
        try:
            return self.node.sdo.upload(index, subindex)
        except Exception as e:
            print("Exception caught:{0}".format(str(e)))
            return None


    def write_object(self, index, subindex, data):
        """Write object to device
        
        Args:
            data: data to be stored
            index: reference of dictionary object index
            subindex: reference of dictionary object subindex
            
        Return:

             """
        try:
            self.node.sdo.download(index, subindex, data)
            return True
        except canopen.SdoAbortedError as e:
            print("Code 0x{:08X}".format(e.code))
            return False
        except canopen.SdoCommunicationError:
            print('SdoAbortedError: Timeout or unexpected response')
            return False

    # ------------------------------------------------------------------------------
    # High level functions
    # ------------------------------------------------------------------------------
    
    def write_controlword(self, controlword):
        """Send controlword to device

        Args:
            controlword: word to be sent.

        Returns:
            bool: a boolean if all went ok.
        """
        # sending new controlword
        print('Sending controlword Hex={0:#06X} Bin={0:#018b}'.format(controlword))
        controlword = controlword.to_bytes(2, 'little')
        return self.write_object(0x6040, 0, controlword)

    def read_controlword(self):
    # def read_controlword(self, index, subindex)
        """Read controlword from device

        Returns:
            tuple: A tuple containing:

            :controlword:  the current value or None if any error.
            :Ok: A boolean if all went ok or not.
        """
        index = self.objectIndex['ControlWord']
        subindex = 0
        controlword = self.read_object(index, subindex)
        # failed to request?
        if not controlword:
            print('Error trying to read {0} controlword'.format(
                self.__class__.__name__))
            return controlword, False

    def read_statusword(self):
        """Read statusword from device

        Returns:
            tuple: A tuple containing:

            :statusword:  the current value or None if any error.
            :Ok: A boolean if all went ok or not.
        """
        index = self.objectIndex['StatusWord']
        subindex = 0x0
        statusword = self.read_object(index,subindex)
        # failed to request?
        if not statusword:
            print('Error trying to read {0} statusword'.format(
                self.__class__.__name__))
            return statusword, False
        
        statusword = int.from_bytes(statusword,'little')
        return statusword, True

    def check_state(self, statusword=None):
        """for statusword
        
            check instant state of inverter (SEVCON)

        Args:
            statusword: indicate the current situation of inverter
        
        Return:
            ID indicating status or -1 in case of error
            
            type of return-> int
        """
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            print('Failed to request StatusWord')
            return -1
        else:        
            bitmask = 0x6F
            if (bitmask & statusword == 0x20):
                ID = 0
                return ID
            if (bitmask & statusword == 0x0):
                ID = 0
                return ID
            if (bitmask & statusword == 0x40):
                ID = 1
                return ID
            if (bitmask & statusword == 0x60):
                ID = 1
                return ID
            if (bitmask & statusword == 0x21):
                ID = 2
                return ID
            if (bitmask & statusword == 0x31):
                ID = 2
                return ID
            if (bitmask & statusword == 0x23):
                ID = 3
                return ID
            if (bitmask & statusword == 0x33):
                ID = 3
                return ID
            if (bitmask & statusword == 0x27):
                ID = 4
                return ID
            if (bitmask & statusword == 0x37):
                ID = 4
                return ID
            if (bitmask & statusword == 0x7):
                ID = 5
                return ID
            if (bitmask & statusword == 0x17):
                ID = 5
                return ID
            if (bitmask & statusword == 0xF):
                ID = 6
                return ID
            if (bitmask & statusword == 0x2F):
                ID = 6
                return ID
            if (bitmask & statusword == 0x8):
                ID = 7
                return ID
            if (bitmask & statusword == 0x28):
                ID = 7
                return ID

        print('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
        return -1

    def print_state(self):
        """print the statusword meaning
        
        Return: the status of inverter

        """
        ID = self.check_state()
        if ID is -1:
            print('Error: Unkonown state')
        else:
            if ID == 0 :
                print('Not ready to switch on')
            elif ID == 1 :
                print('Switch on disabled')
            elif ID == 2 :
                print('Ready to switch on')
            elif ID == 3 :
                print('Switch on')
            elif ID == 4 :
                print('Operation enabled')
            elif ID == 5 :
                print('Quick stop active')
            elif ID == 6 :
                print('Fault reaction active')
            elif ID == 7 :
                print('Fault')
            print('Current State-ID : {}'.format(ID))
        return


    def is_voltage_enabled(self,statusword=None):
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            print('Failed to request StatusWord')
            return -1
        else:
            bitmask = 0x10
            bitshift = 0x4
            if ((bitmask & statusword)>>bitshift == 0):
                return 0
            if ((bitmask & statusword)>>bitshift == 1):
                return 1
            print('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
            return -1

    def is_warning(self,statusword=None):
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            print('Failed to request StatusWord')
            return -1
        else:
            bitmask = 0x80
            bitshift = 0x7
            if ((bitmask & statusword)>>bitshift == 0):
                return 0
            if ((bitmask & statusword)>>bitshift == 1):
                return 1
            print('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
            return -1    



    def is_remote(self,statusword=None):
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            print('Failed to request StatusWord')
            return -1
        else:
            bitmask = 0x200
            bitshift = 0x9
            if ((bitmask & statusword)>>bitshift == 0):
                return 0
            if ((bitmask & statusword)>>bitshift == 1):
                return 1
            print('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
            return -1   
    
    def is_target_reached(self,statusword=None):
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            print('Failed to request StatusWord')
            return -1
        else:
            bitmask = 0x400
            bitshift = 0xA
            if ((bitmask & statusword)>>bitshift == 0):
                return 0
            if ((bitmask & statusword)>>bitshift == 1):
                return 1
            print('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
            return -1 

    def is_int_limit_active(self,statusword=None):
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            print('Failed to request StatusWord')
            return -1
        else:
            bitmask = 0x800
            bitshift = 0xB
            if ((bitmask & statusword)>>bitshift == 0):
                return 0
            if ((bitmask & statusword)>>bitshift == 1):
                return 1
            print('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
            return -1 

    def is_op_mode_specific(self,statusword=None):
        if not statusword:
            statusword, ok = self.read_statusword()
        else:
            ok = True
        if not ok:
            print('Failed to request StatusWord')
            return -1
        else:
            bitmask = 0x3000
            bitshift = 0xC
            if ((bitmask & statusword)>>bitshift == 0):
                return 0
            if ((bitmask & statusword)>>bitshift == 1):
                return 1
            print('Error: Unknown state. Statusword is Bin={0:#018b}'.format(statusword))
            return -1 


    #for the power steer motor
    def controlword_psm(self):
        index = self.sevconDictionary['Motor Drive Power Steer Info']
        subindex = 0x1
        steer_motor_controlword = self.read_object(index, subindex)
        steer_motor_controlword = int.from_bytes(steer_motor_controlword,'little')
        return steer_motor_controlword

    def statusword_psm(self):
        index = self.sevconDictionary['Motor Drive Power Steer Info']
        subindex = 0x2
        steer_motor_statusword = self.read_object(index, subindex)
        steer_motor_statusword = int.from_bytes(steer_motor_statusword,'little')
        return steer_motor_statusword

    def target_velocity_psm(self):
        index = self.sevconDictionary['Motor Drive Power Steer Info']
        subindex = 0x3
        target_velocity = self.read_object(index,subindex)
        target_velocity = int.from_bytes(target_velocity,'little',signed=True)
        print(target_velocity)

    def actual_velocity_psm(self):
        index = self.sevconDictionary['Motor Drive Power Steer Info']
        subindex = 0x4
        actual_velocity = self.read_object(index, subindex)
        actual_velocity = int.from_bytes(actual_velocity, 'little', signed=True)
        print('Actual velocity is {}'.format(actual_velocity))
          
    def max_torque_psm(self):
        index = self.sevconDictionary['Motor Drive Power Steer Info']
        subindex = 0x5
        max_torque = self.read_object(index, subindex)
        max_torque = int.from_bytes(max_torque,'little',signed=True)*0.001
        print(max_torque)
 
    def actual_torque_psm(self):
        index = self.sevconDictionary['Motor Drive Power Steer Info']
        subindex = 0x6
        actual_torque = self.read_object(index, subindex)
        actual_torque = int.from_bytes(actual_torque,'little',signed=True)*0.001
        print('Actual torque is {}'.format(actual_torque))



    #for the errors
    node={'node0':1,'node1':2,'node2':3}
    object_dictionary={'command':601}
    
    def create_network(_network=None, debug=False):
        if not _network:
            network = canopen.Network()
        else:
            network = _network
        return network
    
    def create_node(network, nodeID):
        try:
            node = network.add_node(nodeID, object_dictionary=None)
            network.connect(channel='can0', bustype = 'socketcan')
            _connected = True
        except Exception as e:
            print("Exception caught:{0}".format(str(e)))
        finally:
            return node, _connected
    
    def read_object(node, index, subindex):
        try:
            return node.sdo.upload(index, subindex)
        except Exception as e:
            print("Exception caught:{0}".format(str(e)))
            return None
    
    
    def write_object(node, index, subindex, data):
        try:
            node.sdo.download(index, subindex, data)
            return True
        except canopen.SdoAbortedError as e:
            print("Code 0x{:08X}".format(e.code))
            return False
        except canopen.SdoCommunicationError:
            print('SdoAbortedError: Timeout or unexpected response')
            return False
    
    def get_no_fault(node):
        index=0x5300
        subindex=0x1
        no_faults=node.read_object(node,index,subindex)
        no_faults=int.from_bytes(no_faults,'little')
        return no_faults
    
    def select_fault(node, code, fault_no):
        index=0x5300
        subindex=0x2
        data=fault_no.to_bytes(2,'little')
        node.write_object(index, subindex, data)

    
    def get_fault_id(node):
        index=0x5300
        subindex=0x3
        fault_id=node.read_object(node,index, subindex)
        fault_id=int.from_bytes(fault_id,'little')
        return fault_id
    
    def get_fault(node):
        no_faults=node.get_no_fault(node)
        for fault in range (no_faults):
            node.select_fault(node, fault)
            fault_id=node.get_fault_id(node)
            print("Your fault ID is 0x{:04x}".format(fault_id))

def main():
    state = StateDictionary['INIT']
    STOP_FLAG = False
    
    while(~STOP_FLAG):
        if state == StateDictionary['INIT']:
            inverter = SEVCON()
            invNodeID = 1
            inverter.begin(invNodeID)
            state = StateDictionary['CHECK_ERRORS']

        if state == StateDictionary['CHECK_ERRORS']:
            fault_id = inverter.get_fault_id()
            if fault_id is None:
                state = StateDictionary['CHECK_STATE']

            else:
                fault_id = inverter.get_fault_id
                fault_id = str(fault_id)
                print(fault_id)
                
                #client.publish("viena/fault",fault_id,2)

                state = StateDictionary['HANDLE_ERROR']    

        if state == StateDictionary['CHECK_STATE']:
            ID = inverter.check_state()
            ID = str(ID)
            inverter.print_state()

            #client.publish("viena/state",ID,2)

            state = StateDictionary['CHECK_TORQUE']

        if state == StateDictionary['CHECK_TORQUE']:
            torque = inverter.actual_torque_psm()
            torque = str(torque)
            print(torque)

            #client.publish("viena/torque",torque,2)    
            
            state = StateDictionary['CHECK_SPEED']

        if state == StateDictionary['CHECK_SPEED']:
            # speed = inverter.
            state = StateDictionary['CHECK_ERROR']

        if state == StateDictionary['HANDLE_ERROR']:
            # 
            state = StateDictionary['CHECK_ERRORS']
            continue

    
    inverter.disconnect
    return

if __name__ == '__main__':
    main()
    client.disconnect()



#  flag = True
#     while flag:
#         test = input('function: ')
#         if test == 'check_states':
#             output1 = inverter.check_state()
#             print(SEVCON.statuswordDict[output1])
#         # if test == 'controlword_psm':
#         #     inverter.controlword_psm()
#         #     print(SEVCON.objectIndex['ControlWord'])
#         # if test == 'statusword_psm':
#         #     inverter.statusword_psm()
#         #     print(SEVCON.objectIndex['StatusWord'])
#         if test == 'actual_torque_psm':
#             inverter.actual_torque_psm()
#         if test == 'is_voltage_enabled':
#             output2 = inverter.is_voltage_enabled()
#             print(SEVCON.isDict['voltage_enabled'][output2])
#         if test == 'is_warning':
#             output3 = inverter.is_warning()
#             print(SEVCON.isDict['warning'][output3])
#         if test == 'is_remote':
#             output4 = inverter.is_warning()
#             print(SEVCON.isDict['remote'][output4])
#         if test == 'is_target_reached':
#             output5 = inverter.is_target_reached()
#             print(SEVCON.isDict['target_reached'][output5])
#         if test == 'is_int_limit_active':
#             output6 = inverter.is_int_limit_active()
#             print(SEVCON.isDict['int_limit_active'][output6])
#         if test == 'is_op_mode_specific':
#             output7 = inverter.is_op_mode_specific()
#             print(SEVCON.isDict['op_mode_specific'][output7])
#         if test == 'target_velocity_psm':
#             output8 = inverter.target_velocity_psm()
#             print(output8)
#         if test == 'actual_velocity_psm':
#             output9 = inverter.actual_velocity_psm()
#             print(output9)
#         if test == 'max_torque_psm':
#             output10 = inverter.max_torque_psm()
#             print(output10)
#         if test == 'get_fault':
#             inverter.get_fault()
        
        
#         if test == 'exit':
#             flag = False
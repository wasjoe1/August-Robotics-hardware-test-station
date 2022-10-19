#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time 
from boothbot_driver.modbus_driver import ModbusDriver
import rospy as logger
from assemble_tools.get_key import GetKey
import os
BAUDRATE = 3
INSTRUCTION = """
Key functions:
s: Start setting sonar unit  
q: Exiting...
"""

SONAR_ID = """


Input number and you can set the address
___________________________________________
|     |  0  |     |  1  |     |  2  |     |         
|     |0xe6 |     |0xe8 |     |0xd0 |     |         
|_____|_____|_____|_____|_____|_____|_____|                 
|     |                             |     |                    
|     |                             |     |            
|_____|                             |_____|                   
|  9  |                             |  3  |                            
|0xec |                             |0xfa | 
|_____|                             |_____|   
|     |                             |     |                            
|     |                             |     | 
|_____|                             |_____|
|  8  |                             |  4  |                            
|0xd2 |                             |0xfe | 
|_____|                             |_____|                     
|     |                             |     |       
|     |                             |     |                               
|_____|_____ _____ _____ _____ _____|_____|       
|     |  7  |     |  6  |     |  5  |     |                    
|     |0xe2 |     |0xe4 |     |0xea |     |                                    
|_____|_____|_____|_____|_____|_____|_____|                

"""





UNIT_DICT = {
            "0": 0xe6,
            "1": 0xe8,
            "2": 0xd0,
            "3": 0xfa,
            "4": 0xfe,
            "5": 0xea,
            "6": 0xe4,
            "7": 0xe2,
            "8": 0xd2,
            "9": 0xec,
        }

class DYP_SONAR_UNIT_WRITTER(object):
    def __init__(self, modbus_driver):
        super(DYP_SONAR_UNIT_WRITTER, self).__init__()
        self.modbus_driver = modbus_driver








    def check_unit(self):
        response=mbd.read_holding_registers(0x200, 1, unit=0xFF)  

        try:
            unit = response.registers[0] 
        except:
            unit = None
        return unit

    def set_unit(self,unit):     
        UNIT=UNIT_DICT[unit]
        mbd.write_register(0x200, UNIT, unit=0xFF)
        
    def set_baudrate(self):     
        try:
            mbd.write_register(0x201, BAUDRATE, unit=0xFF)
        except:
            modbus_config["baudrate"] = 115200
 

    def distance_pub(self,unit):
        response=mbd.read_holding_registers(0x0101, 1, unit=unit)    
        try:
            distance = response.registers[0]
        except:
            distance = None
        return distance

def run():
    logger.logwarn(INSTRUCTION)
    get_key = GetKey()
    mbd = ModbusDriver(**modbus_config).client
    DYP = DYP_SONAR_UNIT_WRITTER(mbd)
    while not logger.is_shutdown():
        key = get_key.get_key()
        if key in ("s", "S"):
            os.system('clear')
            while not logger.is_shutdown():  
                DYP.set_baudrate()
                mbd = ModbusDriver(**modbus_config).client
                DYP = DYP_SONAR_UNIT_WRITTER(mbd) 
                logger.logwarn(SONAR_ID)
                key = get_key.get_key()
                for i in range(0,10):                   
                    if key in (str(i), str(i)):
                        os.system('clear')
                        DYP.set_unit(key)
                        if DYP.check_unit() == UNIT_DICT[key]:
                            logger.logwarn("      ")
                            logger.logwarn("      ")
                            logger.logwarn("Key is {}. Sonar address has set {} successfully".format(key, hex(DYP.check_unit())))
                            logger.logwarn(SONAR_ID)
                            logger.logwarn("拔插sonar,按 a 检查是否能测距")
                        else:
                            logger.logwarn("Setting failed, please retry again")
                        while not logger.is_shutdown():
                            
                            key1 = get_key.get_key()
                            if key1 in ("a", "A"):
                                while not logger.is_shutdown():
                                    distance = DYP.distance_pub(UNIT_DICT[key])
                              
                                    unit = DYP.check_unit()
                                    if unit is not None:
                                        unit = hex(unit)

                                    os.system('clear')
                                    logger.logwarn("distance is :{}. Press q return to set address ".format(distance))
                                    logger.logwarn("unit is :{} ".format(unit))
                                    logger.logwarn("Press q to quit")
                                    key1 = get_key.get_key()
                                    if key1 in ("q", "Q"):
                                        os.system('clear')
                                        logger.logwarn("Press q")
                                        get_key.end_get_key()
                                        break
                            if key1 == ("q" or "Q"):
                                os.system('clear')
                                get_key.end_get_key()
                                break
                if key in ("q", "Q"):
                    os.system('clear')
                    logger.logwarn("Return to menu")
                    logger.logwarn(INSTRUCTION)
                    get_key.end_get_key()

                    break
        elif key in ("q", "Q"):
            logger.logwarn("Exiting...")
            get_key.end_get_key()
            modbus_config["baudrate"] = 9600
            break


if __name__ == '__main__':
    modbus_config = {
    "port": "/dev/ttyUSB0",
    "baudrate": 115200,
    "parity": "N",
    "timeout": 0.5,
}   
    mbd = ModbusDriver(**modbus_config).client
    DYP = DYP_SONAR_UNIT_WRITTER(mbd)   
    run()






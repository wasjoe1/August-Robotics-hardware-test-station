#!/usr/bin/env python

import serial
import time
import binascii 
import yaml

with open("sonar_config.yaml",'r') as fp:
    sonar_config = yaml.load(fp)


def connect_sonar(port,baudrate):
    #connect to sonar
    sonar_port = serial.Serial(port, baudrate)
    
    if sonar_port.is_open: 
        print("Sonar is Connected!")
        return sonar_port
    else:
        print("Error: port is closed")
        pass


def getSonarDisntance(port,address):
    """KS103 Measurement
    The timed sequence of write command and read is from ks103 user manual
    
    Input: (address, register,command)
    
    Return: measured distance in meters

    """
    #clear input and output buffer 
    port.reset_input_buffer()
    port.reset_output_buffer()
    time.sleep(0.001)

    # Write address + reg2 + command and Read measurement
    port.write(address)
    
    # Minimal 20 - 100us is required, use 1ms just in case
    time.sleep(0.001)
    port.write(b'\x02')        
    time.sleep(0.001)        
    port.write(b'\xb0') 
    
    #automatically return measurement results, 2 bytes 
    _raw_data = port.read(size=2)

    #covert result from hex to int, and from mm to m
    return int(binascii.b2a_hex(_raw_data),16) * 0.001
    
def write485addr(port,old_addr,new_addr):
    """Write address to a single KS103 module
    Only should be used when ONE KS103 is connected!

    Input: (old_address, new_address)

    Return: there is no return, you can only observe the flashing led to
    determing if it is successful. 
    
    Caution: This should only be used for new KS103

    Usage: 
        1. xxx._write485addr(b'\xe8',b'\xea') #0xe8 is the default addr
        2. Wait at least 100ms
        3. Disconnect the power to KS103
        4. Power up the KS103
    """
    #clear input and output buffer 
    port.reset_input_buffer()
    port.reset_output_buffer()
    time.sleep(0.01)
    for _cmd in [b'\x9a',b'\x92',b'\x9e']:
        time.sleep(0.1)
        port.write(old_addr)
        time.sleep(0.01)
        port.write(b'\x02')
        time.sleep(0.01)
        port.write(_cmd)
        time.sleep(0.02) # required 1ms, make it 2
    
    print "Changing address from {} to {}".format(binascii.b2a_hex(old_addr),binascii.b2a_hex(new_addr))
    
    port.write(old_addr)
    time.sleep(0.1)
    port.write(b'\x02')
    time.sleep(0.1)
    port.write(new_addr)
    time.sleep(0.5) # requried 100ms, make it 200
    
    return  



if __name__ == '__main__' :
    port = sonar_config.get('port')
    baud = sonar_config.get('baud')
    old_addr = chr(sonar_config.get('old_addr'))
    new_addr = chr(sonar_config.get('new_addr'))
    
    # connect to sonar
    sonar_port = connect_sonar(port,baud)

    # write sonar address
    write485addr(sonar_port,old_addr, new_addr)
    time.sleep(1)

    # test sonar with new address
    print "Testing soanr with new adress {}".format(binascii.b2a_hex(new_addr))
    print str(getSonarDisntance(sonar_port,new_addr)) + " m"
    print "Finshed! New address is {}".format(binascii.b2a_hex(new_addr)) 

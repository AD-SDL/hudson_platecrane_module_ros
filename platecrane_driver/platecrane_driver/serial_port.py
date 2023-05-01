import time
from threading import Lock

from serial import PARITY_EVEN, Serial, SerialException

# from error_codes import SerialError
# from utils import lock_threading_lock

import asyncio
from asyncio.unix_events import DefaultEventLoopPolicy
            
class SerialPort():
    '''
    Description: 
    Python interface that allows remote commands to be executed to the plate_crane. 
    '''
    
    def __init__(self, host_path= "/dev/ttyUSB2", baud_rate=9600):

        self.host_path = host_path
        self.baud_rate = baud_rate
        self.connection = None 

        self.status = 0
        self.error = ""

        self.__connect_plate_crane()
    
    def __connect_plate_crane(self):
        '''
        Connect to serial port / If wrong port entered inform user 
        '''
        try:
            self.connection = Serial(self.host_path, self.baud_rate, timeout=1)
            self.connection_status = Serial(self.host_path, self.baud_rate, timeout=1)
        except:
            raise Exception("Could not establish connection")    

    def __disconnect_robot(self):
        
        try:
            self.connection.close()
        except Exception as err:
            print(err)
        else:
            print("Robot is successfully disconnected")

    def send_command(self, command, timeout=0):
        '''
        Sends provided command to Peeler and stores data outputted by the peelr.
        Indicates when the confirmation that the Peeler received the command by displaying 'ACK TRUE.' 
        '''

        try:
            self.connection.write(command.encode('utf-8'))

        except SerialException as err:
            print(err)
            self.robot_error = err

        response_msg = ""
        initial_command_msg = ""

        time.sleep(timeout)

        while initial_command_msg == "" :
            response_msg, initial_command_msg = self.receive_command(timeout)
    
        # Print the full output message including the initial command that was sent
        print(initial_command_msg) 
        print(response_msg)

        return response_msg
    
    def receive_command(self, time_wait):                         
        '''
        Records the data outputted by the plate_crane and sets it to equal "" if no data is outputted in the provided time.
        '''
        
        # response_string = self.connection.read_until(expected=b'\r').decode('utf-8')
        response = ""
        response_string = ""
        initial_command_msg = ""

        if self.connection.in_waiting != 0:           
            response = self.connection.readlines()
            initial_command_msg = response[0].decode('utf-8').strip("\r\n")
            if len(response) > 1:
                for line_index in range(1, len(response)):
                    response_string += "\n" + response[line_index].decode('utf-8').strip("\r\n")
            else:        
                response_string = ""
        return response_string, initial_command_msg
    
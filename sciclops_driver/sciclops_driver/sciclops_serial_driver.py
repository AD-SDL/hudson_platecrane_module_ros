import asyncio
from asyncio.unix_events import DefaultEventLoopPolicy
from pickle import TRUE
import time  
import serial
import logging
import re
import sys

            

class SCICLOPS():
    '''
    Description: 
    Python interface that allows remote commands to be executed to the Sciclops. 
    '''
    
    def __init__(self, host_path= "/dev/ttyUSB2", baud_rate=960):

        self.host_path = host_path
        self.baud_rate = baud_rate
        self.connection = None 
        self.connect_sciclops()
    
        self.STATUS = 0
        self.ERROR = ""
        self.GRIPLENGTH = 0

        # self.status = self.get_status()
        # self.error = self.get_error()
        self.movement_state = "READY"



    def connect_sciclops(self):
        '''
        Connect to serial port / If wrong port entered inform user 
        '''
        try:
            self.connection = serial.Serial(self.host_path, self.baud_rate, write_timeout = 1, )
        except:
            raise Exception("Could not establish connection")
            

    def disconnect_robot(self):
        try:
            usb.util.dispose_resources(self.host_path)
        except Exception as err:
            print(err)
        else:
            print("Robot is disconnected")
        pass

    def command_response(self, time_wait):                         
        '''
        Records the data outputted by the Sciclops and sets it to equal "" if no data is outputted in the provided time.
        '''

        response_timer = time.time()
        while time.time() - response_timer < time_wait: 
            if self.connection.in_waiting != 0:           
                response = self.connection.read_all()
                response_string = response.decode('utf-8')
                break
            else:
                response_string = ""
        return response_string
    

    def send_command(self, command, timeout=1):
        '''
        Sends provided command to Peeler and stores data outputted by the peelr.
        Indicates when the confirmation that the Peeler received the command by displaying 'ACK TRUE.' 
        '''

        ready_timer = time.time()
        response_buffer = ""

        self.connection.write(command.encode('utf-8'))

        # Waits till there is "ready" in the response_buffer indicating
        # the command is done executing.
        while "ready" not in response_buffer:
            new_string = self.command_response(timeout)

            response_buffer = response_buffer + new_string
            
            if time.time() - ready_timer > 20:
                break
            
            print(response_buffer)

        return response_buffer


    def old_send_command(self, command):
        '''
        Sends provided command to Sciclops and stores data outputted by the sciclops.
        '''

        self.host_path.write(4,command)

        response_buffer = "Write: "+ command
        msg = None

        #Adds SciClops output to response_buffer
        while msg != command:
            # or "success" not in msg or "error" in msg
            try:
                response =  self.host_path.read(0x83,200, timeout = 5000)
            except:
                break
            msg = ''.join(chr(i) for i in response)
            response_buffer = response_buffer + "Read: " + msg

            
        print(response_buffer)

        self.success_count = self.success_count + response_buffer.count("0000 Success")

        self.get_error(response_buffer)
       
        return response_buffer

    def get_status(self):
        '''
        Checks status of Sciclops
        '''

        command = 'STATUS\r\n' # Command interpreted by Sciclops
        out_msg =  self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the status
            find_status= re.search(exp,out_msg)
            self.status = find_status[1]
        
            print(self.status)
        
        except:
            pass

    def home(self, axis = ""):
        '''
        Homes all of the axes. Returns to neutral position (above exchange)
        '''

        # Moves axes to home position
        command = 'HOME\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the success message
            home_msg = re.search(exp,out_msg)
            self.HOMEMSG = home_msg[1]

            print(self.HOMEMSG)
        except:
            pass
        
        # Moves axes to neutral position (above exchange)
        self.move(R=self.labware['neutral']['pos']['R'], Z=23.5188, P=self.labware['neutral']['pos']['P'], Y=self.labware['neutral']['pos']['Y'])






if __name__ == "__main__":
    '''
    Runs given function.
    '''
    s = SCICLOPS("/dev/ttyUSB1")
    # print(s.connection)
    s.get_status()
   
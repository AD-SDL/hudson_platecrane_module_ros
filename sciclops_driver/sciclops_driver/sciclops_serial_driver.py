import asyncio
from asyncio.unix_events import DefaultEventLoopPolicy
from pickle import TRUE
from time import sleep  
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
        self.TEACH_PLATE = 15.0
        self.STD_FINGER_LENGTH = 17.2
        self.COMPRESSION_DISTANCE = 3.35
        self.current_pos = [0, 0 ,0, 0]
        # self.NEST_ADJUSTMENT = 20.0
        self.STATUS = 0
        # self.VERSION = 0
        # self.CONFIG = 0
        self.ERROR = ""
        self.GRIPLENGTH = 0
        # self.COLLAPSEDDISTANCE = 0
        # self.STEPSPERUNIT = [0, 0 ,0, 0]
        # self.HOMEMSG = ""
        # self.OPENMSG = ""
        # self.CLOSEMSG = ""
        self.labware = self.load_labware()
        self.plate_info = self.load_plate_info()
        self.success_count = 0
        # self.status = self.get_status()
        # self.error = self.get_error()
        self.movement_state = "READY"

        # if not is_homed:
        #     self.home()

    def connect_sciclops(self):
        '''
        Connect to serial port / If wrong port entered inform user 
        '''
        try:
            self.connection = serial.Serial(self.host_path, self.baud_rate)
        except:
            raise Exception("Could not establish connection")
            

    def disconnect_robot(self):
        try:
            usb.util.dispose_resources(self.host_path)
        except Exception as err:
            print(err)
        else:
            print("Robot is disconnected")


    def send_command(self, command):
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
    s = SCICLOPS("/dev/ttyUSB2")
    print(s.connection)
    # s.get_error()
    # s.get_status()
    # s.reset()
    # s.home()
    # print("STATUS MSG: ", s.status)
    # s.check_closed()
    # print(s.CURRENT_POS)

    # dummy_sciclops.check_plate()

#Finished commands
# "GETPOS"
# "STATUS"
# "VERSION"
# "GETCONFIG"
# "GETGRIPPERLENGTH"
# "GETCOLLAPSEDISTANCE"
# "GETSTEPSPERUNIT"
# "HOME"
# "OPEN"
# "CLOSE"
# "GETGRIPPERISCLOSED"
# "GETGRIPPERISOPEN"
# "GETPLATEPRESENT"
# "SETSPEED "
# "MOVE "
# "JOG"
# "DELETEPOINT (ADD POINT)"
# "LISTPOINTS"
# "LOADPOINT R:0,Z:0,P:0,Y:0"
# "LIMP TRUE/FALSE"

# #Unfinished Commands
# "GETLIMITS"

# #Unknown Commands
# "LISTMOTIONS"
# "AUTOTEACH"
# "GETPOINT"
# "READINP 15"
# "GETGRIPSTRENGTH"
# "READINP"

import os.path
import socket
import time
import logging
import json

import usb.core
import usb.util
import sys


#Log Configuration
# file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/SCICLOPS_logs/robot_client_logs.log')
VENDOR_ID = 0x7513
PRODUCT_ID = 0x0002
file_path = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)

# logging.basicConfig(filename = file_path, level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')

class SCICLOPS():
    def __init__(self, data_file_path):
        
        self.VENDOR_ID = 0x7513
        self.PRODUCT_ID = 0x0002

        self.logger = logging.getLogger("Hudson Stacker")
        self.logger.addHandler(logging.StreamHandler())
        
        # self.ID = robot1["id"]
        self.host = data_file_path # robot1["host"]
        # self.port = robot1["port"]
        # self.robot_data = robot_data
       
        # self.logger.info("Robot created. Robot ID: {} ~ Host: {} ~ Port: {}".format(self.ID, self.host, self.port))
    
    #Connect the socket object to the robot
    def connect_robot(self):    
        #create an INET, Streaming socket (IPv4, TCP/IP)
        try:
            # SCICLOPS = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  
            SCICLOPS = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)


        except socket.error:
            self.logger.error('Failed to create socket')

        else:
            # SCICLOPS.connect((self.host,self.port))
            # self.logger.info('Socket Created')
            return SCICLOPS     

        #self.logger.info('Socket Connected to ' + self.host + " Port:", self.port )
        #TODO:Read the status of the robot 

    def disconnect_robot(self, SCICLOPS):
        SCICLOPS.close()
        # self.logger.info("TCP/IP client is closed")

    def send_command(self, cmd: str=None, ini_msg:str = None, err_msg:str = None, wait:int = 0.1):
        """
        Send and arbitrary command to the robot
        - command : 
        - wait : wait time after movement is complete
        """

        ##Command Checking 
        #TODO: We can check the available commands if the user enters a wrong one break

        if cmd==None:
            self.logger.info("Invalid command: " +cmd)
            return -1 ## make it return the last valid state
        
        ##logging messages
        if ini_msg=='':
            ini_msg = 'send command'
        if err_msg=='':
            err_msg = 'Failed to send command: '

        ##TODO check cmd against cmd list 
        ##return invalid CMD before trying to connect

        SCICLOPS_sock = self.connect_robot()

        while msg != cmd:
            response =  SCICLOPS_sock.read(0x83,200)
            msg = ''.join(chr(i) for i in response)
            print("Read:", msg)        
        # try:
        #     SCICLOPS_sock.send(bytes(cmd.encode('ascii')))
        #     robot_output = SCICLOPS_sock.recv(4096).decode("utf-8")
        #     if ini_msg:
        #         self.logger.info(ini_msg)
        #     self.logger.info(robot_output)
        #     # Wait after executing the command. Default wait time 0.1 sc
        #     time.sleep(wait)
        # except socket.error as err:
        #     self.logger.error(err_msg +' {}'.format(err))

        #     return('failed')## what is a failed state or it is the last state
        # else:
        #     self.disconnect_robot(SCICLOPS_sock)
        #     # Returning the output message as a list             
        #     return(robot_output)

    def home(self, wait:int = 0.1):

        cmd = 'HOME'
        input_msg = 'Checking robot state:'
        err_msg = 'Failed to check robot state:'

        out_msg = self.send_command(cmd, input_msg, err_msg)
        return out_msg
    
    # def GETPOS(self, wait:int = 0.1)

        TEACH_RISE = 15.0
TEACH_PLATE = 15.0
STD_FINGER_LENGTH = 17.2
COMPRESSION_DISTANCE = 3.35
CURRENT_POS = 0, 0 ,0, 0
NEST_ADJUSTMENT = 20.0

#Finished commands
"GETPOS"
"STATUS"
"VERSION"
"GETCONFIG"
"GETGRIPPERLENGTH"
"GETCOLLAPSEDISTANCE"
"GETSTEPSPERUNIT"
"HOME"
"OPEN"
"CLOSE"
"GETGRIPPERISCLOSED"
"GETGRIPPERISOPEN"
"GETPLATEPRESENT"
"SETSPEED "


#Unfinished Commands
"MOVE TEMP"

"GETGRIPSTRENGTH"
"READINP 15"
"READINP"
"LISTPOINTS"
"LOADPOINT R:0,Z:0,P:0,Y:0"
"GETLIMITS"

#Unknown Commands
"MOVE "
"JOG"
"LISTMOTIONS"
"AUTOTEACH"
"GETPOINT"



if __name__ == "__main__":
    '''
    Runs given function.
    '''
    
    sciclops = SCICLOPS(file_path)

    sciclops.home()
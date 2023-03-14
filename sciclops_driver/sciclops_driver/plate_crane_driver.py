import asyncio
from asyncio.unix_events import DefaultEventLoopPolicy
from pickle import TRUE
import time  
import serial
import logging
import re
import sys

            

class PLATE_CRANE():
    '''
    Description: 
    Python interface that allows remote commands to be executed to the plate_crane. 
    '''
    
    def __init__(self, host_path= "/dev/ttyUSB2", baud_rate=9600):

        self.host_path = host_path
        self.baud_rate = baud_rate
        self.connection = None 
        self.connect_plate_crane()
    
        self.STATUS = 0
        self.ERROR = ""
        self.GRIPLENGTH = 0

        # self.status = self.get_status()
        # self.error = self.get_error()
        self.movement_state = "READY"



    def connect_plate_crane(self):
        '''
        Connect to serial port / If wrong port entered inform user 
        '''
        try:
            self.connection = serial.Serial(self.host_path, self.baud_rate, timeout=2)
            print(self.connection.name)
        except:
            raise Exception("Could not establish connection")
            

    def disconnect_robot(self):
        
        pass

    def command_response(self, time_wait):                         
        '''
        Records the data outputted by the plate_crane and sets it to equal "" if no data is outputted in the provided time.
        '''

        response_timer = time.time()
        if self.connection.in_waiting != 0:           
            response = self.connection.read_until(expected=b'\r')
            response_string = response.decode('utf-8')
            # print(response_string)
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
            
            if time.time() - ready_timer > 1:
                break
            
        print(response_buffer)

        return response_buffer



    def get_status(self):
        '''
        Checks status of plate_crane
        '''

        command = 'STATUS\r\n' # Command interpreted by plate_crane
        out_msg =  self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the status
            find_status= re.search(exp,out_msg)
            self.status = find_status[1]
        
            print(self.status)
        
        except:
            pass

    def get_location_list(self):
        '''
        Checks status of plate_crane
        '''

        command = 'LISTPOINTS\r\n' # Command interpreted by plate_crane
        out_msg =  self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the status
            find_status= re.search(exp,out_msg)
            self.status = find_status[1]
        
            print(self.status)
        
        except:
            pass

    def get_position(self):
            '''
            Requests and stores plate_crane position.
            Coordinates:
            Z: Vertical axis
            R: Base turning axis
            Y: Extension axis
            P: Gripper turning axis
            '''

            command = 'GETPOS\r\n' # Command interpreted by plate_crane
            out_msg = self.send_command(command)
            
            try:
                # Checks if specified format is found in feedback
                exp = r"Z:([-.\d]+), R:([-.\d]+), Y:([-.\d]+), P:([-.\d]+)" # Format of coordinates provided in feedback
                find_current_pos = re.search(exp,out_msg)
                self.current_pos = [float(find_current_pos[1]), float(find_current_pos[2]), float(find_current_pos[3]), float(find_current_pos[4])]
                
                print(self.current_pos)
            except:
                pass
    def jog(self, axis, distance):
        '''
        Moves the specified axis the specified distance.
        '''

        command = 'JOG %s,%d\r\n' %(axis,distance) # Command interpreted by plate_crane
        out_msg = self.send_command(command)


        try:
            # Checks if specified format is found in feedback
            jog_msg_index = out_msg.find("0000") # Format of feedback that indicates success message
            self.JOGMSG = out_msg[jog_msg_index+4:]
            print(self.JOGMSG)
        except:
            pass
    
    def move(self, R, Z, P, Y):
        '''
        Moves to specified coordinates
        '''

        # self.loadpoint(R, Z, P, Y)

        command = "MOVE + R +" " + Z + " " + " +"\r\n" 
        out_msg_move = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            move_msg_index = out_msg_move.find("0000") # Format of feedback that indicates success message
            self.MOVEMSG = out_msg_move[move_msg_index+4:]
        except:
            pass

        # self.deletepoint(R, Z, P, Y)
    
    def move_loc(self, loc):
        '''
        Move to preset locations located in load_labware function
        '''

        #check if loc exists (later)
        self.move(self.labware[loc]['pos']['R'],self.labware[loc]['pos']['Z'],self.labware[loc]['pos']['P'],self.labware[loc]['pos']['Y'])

    def home(self, axis = ""):
        '''
        Homes all of the axes. Returns to neutral position (above exchange)
        '''

        # Moves axes to home position
        command = 'HOME\r\n' # Command interpreted by plate_crane
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
        # self.move(R=self.labware['neutral']['pos']['R'], Z=23.5188, P=self.labware['neutral']['pos']['P'], Y=self.labware['neutral']['pos']['Y'])






if __name__ == "__main__":
    '''
    Runs given function.
    '''
    s = PLATE_CRANE("/dev/ttyUSB2")
    # print(s.connection)
    # s.get_status()
    # s.get_position()
    # s.home()
    # s.get_location_list()
    # s.send_command("MOVE PeelerNest\r\n")
    # s.jog("Z", 60000)
    # s.send_command("Move 166756, -32015, -5882, 5460\r\n")
    # s.send_command("move_abs Z")
    # s.send_command("MOVE TEMP 117902 2349 -5882 0\r\n")  
    s.send_command("MOVE Y 5000\r\n")  

    # s.send_command("Move_Z Safe\r\n")  
    # s.send_command("Move_Y Safe\r\n")    
  
    # s.send_command("MOVE Safe\r\n")
    

    # s.send_command("Move_Z Safe\r\n")    
    # s.send_command("Move_Y Safe\r\n")    

    # s.send_command("MOVE PeelerNest\r\n")
    

    # s.send_command("Move_Z Safe\r\n")    
    # s.send_command("Move_Y Safe\r\n")    

    # s.send_command("MOVE Stack1\r\n")
    


    # s.send_command("Move_R PeelerNest\r\n")
    # s.send_command("Move_Z PeelerNest\r\n")
    # s.send_command("Move_P PeelerNest\r\n")

    # s.get_position()

    # s.home()

#    Crash error outputs 21(R axis),14(z axis) 
# 1:Safe, 117902, 2349, -5882, 0
# 2:Stack1, 166756, -32015, -5882, 5460
# 3:Stack2, 149127, -31887, -5882, 5460
# 4:TEST1, -11600, -34445, 0, 5735
# 5:TEST2, 100158, -34445, 0, -460
# 6:TEST3, 212416, -34445, 0, 5735
# 7:TEST4, 324175, -34445, 0, -460
# 8:PeelerNest, 298872, -30589, -8299, 5285
# 9:TEMP, 166756, -32579, -5882, 5460
# 10:Stack3, 131580, -31925, -5889, 5520
# 11:Stack4, 113890, -31923, -5866, 5462
# 12:Stack5, 96239, -31911, -5866, 5462
# 13:LidNest1, 163105, -31001, -5866, -308
# 14:LidNest2, 99817, -31001, -5890, -315
# 15:RapidPick.Source, 257776, -31006, -6627, 1854
# 16:RapidPick.Destination, 275183, -30960, -5995, 931
# 17:RapidPick.Destination2, 293181, -30927, -5355, 1383
# 18:Solo.Position2, -8570, -25921, -9865, 1224
# 19:Solo.Position3, 20085, -26481, -8850, 5236
# 20:Solo.Position6, 48425, -27211, -7818, 2793
# 21:MotorolaScanner.Reader, 224262, -30041, -7281, 2793
# 22:TorreyPinesRIC20.Nest, -8570, -25921, -9865, 1224
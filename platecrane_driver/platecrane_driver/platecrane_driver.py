import asyncio
from asyncio.unix_events import DefaultEventLoopPolicy
from pickle import TRUE
import time  
import serial
import logging
import re
import sys
   

class PlateCrane():
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
        self.gripper_length = 0
        self.robot_status = ""
        self.movement_state = "READY"
        self.connect_plate_crane()
        self.initialize()


    def connect_plate_crane(self):
        '''
        Connect to serial port / If wrong port entered inform user 
        '''
        try:
            self.connection = serial.Serial(self.host_path, self.baud_rate, timeout=1)
            self.connection_status = serial.Serial(self.host_path, self.baud_rate, timeout=1)
        except:
            raise Exception("Could not establish connection")    

    def disconnect_robot(self):
        
        try:
            self.connection.close()
        except Exception as err:
            print(err)
        else:
            print("Robot is successfully disconnected")

    def initialize(self):
        self.get_status()
        if self.robot_status == 0:
            self.home()

    def home(self, timeout = 28):
        '''
        Homes all of the axes. Returns to neutral position (above exchange)
        '''

        # Moves axes to home position
        command = 'HOME\r\n' # Command interpreted by plate_crane
        out_msg = self.send_command(command ,timeout)


        
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
    

    def send_command(self, command, timeout=0):
        '''
        Sends provided command to Peeler and stores data outputted by the peelr.
        Indicates when the confirmation that the Peeler received the command by displaying 'ACK TRUE.' 
        '''

        try:
            self.connection.write(command.encode('utf-8'))

        except serial.SerialException as err:
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

    def get_robot_movement_state(self):
        self.get_status()
        # print("Here"+ self.robot_status)
        if self.robot_status == "":
            self.movement_state = "BUSY"
        else:
            self.movement_state = "READY"
        print(self.movement_state)

    def wait_robot_movement(self):
        self.get_robot_movement_state()
        if self.movement_state != "READY":
            self.wait_robot_movement()

    def get_status(self):
        '''
        Checks status of plate_crane
        '''

        command = 'STATUS\r\n' # Command interpreted by plate_crane
        self.robot_status =  self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the status
            find_status= re.search(exp,self.robot_status)
            self.status = find_status[1]
        
            print(self.status)
        
        except:
            pass

        return self.robot_status

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
    def get_location_joint_values(self, location:str = None):
        '''
        Checks status of plate_crane
        '''

        command = "GETPOINT " + location + "\r\n" # Command interpreted by plate_crane
        out_msg =  self.send_command(command)

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
            

    def set_location(self, location_name:str = "TEMP_0", R:int = 0, Z:int = 0, P:int = 0, Y:int = 0):
        '''
        Saves a new location onto robot
        '''
        
        command = "LOADPOINT %s, %s, %s, %s, %s\r\n" % (location_name, str(Z), str(P), str(Y), str(R)) # Command interpreted by Sciclops
        out_msg = self.send_command(command)
    
    def delete_location(self,location_name:str = None):
        '''
        Deletes a location from the robot's database
        '''
        if not location_name:
            raise Exception("No location name provided")
        
        command = "DELETEPOINT %s\r\n" % (location_name) # Command interpreted by Sciclops
        out_msg = self.send_command(command)

    def gripper_open(self):
        '''
        Opens gripper
        '''

        command = 'OPEN\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)


    def gripper_close(self):
        '''
        Closes gripper
        '''

        command = 'CLOSE\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        
    def check_open(self):
        '''
        Checks if gripper is open
        '''

        command = 'GETGRIPPERISOPEN\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)


    def check_closed(self):
        '''
        Checks if gripper is closed
        '''

        command = 'GETGRIPPERISCLOSED\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

    def jog(self, axis, distance):
        '''
        Moves the specified axis the specified distance.
        '''

        command = 'JOG %s,%d\r\n' %(axis,distance) # Command interpreted by plate_crane
        out_msg = self.send_command(command)

    def move_joint_angles(self, R:int, Z:int, P:int, Y:int):
        '''
        Moves on a single axis, using an existing location on robot's database
        '''

        self.set_location("TEMP", R, Z, P, Y)

        command = "MOVE TEMP\r\n" 
        out_msg_move = self.send_command(command)

        try:
            out_msg_move = self.send_command(command)

            # Checks if specified format is found in feedback
            # move_msg_index = out_msg_move.find("0000") # Format of feedback that indicates success message
            # self.MOVEMSG = out_msg_move[move_msg_index+4:]
        except Exception as err:
            print(err)
            self.robot_error = err
        else:
            self.move_status = "COMPLETED"
            pass

        # self.deletepoint(TEMP, R, Z, P, Y) 

    def move_single(self, axis:str, location:str):
        '''
        Moves on a single axis using an existing location on robot's database
        '''

        # self.loadpoint(R, Z, P, Y)

        command = "MOVE_"+ axis.upper() + " " + location + "\r\n" 

        try:
            out_msg_move = self.send_command(command)

            # Checks if specified format is found in feedback
            # move_msg_index = out_msg_move.find("0000") # Format of feedback that indicates success message
            # self.MOVEMSG = out_msg_move[move_msg_index+4:]
        except Exception as err:
            print(err)
            self.robot_error = err
        else:
            self.move_status = "COMPLETED"
            pass

        # self.deletepoint(R, Z, P, Y)
    
    def move_location(self, loc, move_time = 4.5):
        '''
        Move to preset locations located in load_labware function
        '''

        cmd = "MOVE "+ loc +"\r\n"
        self.send_command(cmd, timeout = move_time)

    def move_tower_neutral(self):

        self.move_single("Z", "Safe")

    def move_arm_neutral(self):

        self.move_single("Y", "Safe")

    def move_gripper_neutral(self):

        self.move_single("P", "Safe")

    def move_joints_neutral(self):
        self.move_tower_neutral()
        self.move_arm_neutral()
        self.move_gripper_neutral()

    def pick_plate(self, source:str, joint_values:bool = False):
        if joint_values:
            # Create a new location data 
            # Move to this location
            pass
        self.gripper_open()
        self.move_joints_neutral()
        self.move_location(source)
        self.gripper_close()
        self.move_joints_neutral()


    def place_plate(self, target:str, joint_values:bool = False):
        if joint_values:
            # Create a new location data 
            # Move to this location
            pass

        self.move_joints_neutral()
        self.move_location(target)
        self.gripper_open()
        self.move_joints_neutral()

    def stack_transfer(self):
        '''
        Transfer a plate plate from plate stacker to exchange location
        '''
    
    def transfer(self, source:str, target:str):
        '''
        Transfer a plate plate in between two locations
        '''
        # if len(source)
        #Add an extra step to check if the locations were sent as names or joint angles. Then handle the transfer in two different ways 
        self.pick_plate(source)
        self.place_plate(target)

        #BUG: Output messages of multiple commands mix up with eachother. Fix the wait times in between the command executions"

        

if __name__ == "__main__":
    '''
    Runs given function.
    '''
    s = PlateCrane("/dev/ttyUSB2")
    source_loc = "Stack1"
    target_loc = "Stack2"
    # s.transfer(source_loc, target_loc)
    # s.send_command("LOADPOINT TEMP2 210256, -1050, 491, 5730", timeout= 5)

    # s.get_status()
    # s.get_position()
    # s.home()
    # s.wait_robot_movement()
    # s.get_status()
    # s.get_position()
    s.send_command("GETPOINT Safe\r\n")  
    # s.set_location()
    # s.get_location_list()
    # s.delete_location("TEMP_0")
    # s.get_location_list()

    # s.send_command("MOVE PeelerNest\r\n")
    # s.jog("Z", 60000)
    # s.send_command("Move 166756, -32015, -5882, 5460\r\n")
    # s.send_command("move_abs Z")
    # s.send_command("MOVE TEMP 117902 2349 -5882 0\r\n")  
    # s.send_command("MOVE Y 5000\r\n")  

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

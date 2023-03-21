import asyncio
from asyncio.unix_events import DefaultEventLoopPolicy
from pickle import TRUE
import time  
import serial
import logging
import re
import sys
   

class PlateCrane():
    """
    Description: 
    Python interface that allows remote commands to be executed to the plate_crane. 
    """
    
    def __init__(self, host_path= "/dev/ttyUSB2", baud_rate=9600):
        """[Summary]

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        self.host_path = host_path
        self.baud_rate = baud_rate
        self.connection = None 

        self.status = 0
        self.error = ""
        self.gripper_length = 0
        self.plate_above_height = 1000
        self.stack_exchange_Z_height = -31887
        self.stack_exchange_Y_axis_steps = 200 #TODO: Find the correct number of steps to move Y axis from the stack to the exchange location
        
        self.robot_status = ""
        self.movement_state = "READY"
        self.connect_plate_crane()
        self.initialize()


    def connect_plate_crane(self):
        """
        Connect to serial port / If wrong port entered inform user 

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        try:
            self.connection = serial.Serial(self.host_path, self.baud_rate, timeout=1)
            self.connection_status = serial.Serial(self.host_path, self.baud_rate, timeout=1)
        except:
            raise Exception("Could not establish connection")    

    def disconnect_robot(self):
        """[Summary]

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        
        try:
            self.connection.close()
        except Exception as err:
            print(err)
        else:
            print("Robot is successfully disconnected")

    def initialize(self):
        """[Summary]

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        self.get_status()
        if self.robot_status == 0:
            self.home()

    def home(self, timeout = 28):

        """Homes all of the axes. Returns to neutral position (above exchange)


        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """


        # Moves axes to home position
        command = 'HOME\r\n' 
        out_msg = self.send_command(command ,timeout)


        
    def receive_command(self, time_wait):                         
        """Records the data outputted by the plate_crane and sets it to equal "" if no data is outputted in the provided time.
        

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """


        # response_string = self.connection.read_until(expected=b'\r').decode('utf-8')
        response = ""
        response_string = ""
        initial_command_msg = ""

        if self.connection.in_waiting != 0:           
            response = self.connection.readlines()
            initial_command_msg = response[0].decode('utf-8').strip("\r\n")
            if len(response) > 1:
                for line_index in range(1, len(response)):
                    response_string +=  response[line_index].decode('utf-8').strip("\r\n")
            else:        
                response_string = ""
        return response_string, initial_command_msg
    

    def send_command(self, command, timeout=0.):
        """Sends provided command over the serial port and stores data outputted. 

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """



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
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """


        self.get_status()

        if self.robot_status == "":
            self.movement_state = "BUSY"
        else:
            self.movement_state = "READY"
        print(self.movement_state)

    def wait_robot_movement(self):
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        self.get_robot_movement_state()
        if self.movement_state != "READY":
            self.wait_robot_movement()

    def get_status(self):
        """Checks status of plate_crane

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = 'STATUS\r\n' 
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
        """Checks status of plate_crane

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = 'LISTPOINTS\r\n' 
        out_msg =  self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the status
            find_status= re.search(exp,out_msg)
            self.status = find_status[1]
        
            print(self.status)
        
        except:
            pass

    def __update_locations(self, robot_onboard:list, known_locations:list) -> None:
        """Checks the location database on the robot and saves the missing locations to robot onboard

        :param robot_onboard: List of string locations that are saved on the robot
        :type robot_onboard: list

        :param known_locations: List of known locations that should exist on robot database
        :type known_locations: list

        :return: None
        
        """

        for loc in known_locations:
            if loc not in robot_onboard:
                loc_values = loc.replace(",","").split(" ") # Removing the ',' caracter 
                loc_values[0] =  loc_values[0][loc_values[0].find(":")+1:] # Removing the index number from the location name
                self.set_location(loc_values[0], int(loc_values[1], int(loc_values[2], int(loc_values[3]), int(loc_values[4]))))


    def get_location_joint_values(self, location:str = None) -> list:
        """Checks status of plate_crane

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = "GETPOINT " + location + "\r\n" 

        joint_values =  list(self.send_command(command).split(" "))
        joint_values = [eval(x.strip(",")) for x in joint_values]

        return joint_values

    def get_position(self) -> list:
        """
        Requests and stores plate_crane position.
        Coordinates:
        Z: Vertical axis
        R: Base turning axis
        Y: Extension axis
        P: Gripper turning axis

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = 'GETPOS\r\n' 
        current_position = list(self.send_command(command).split(" "))
        current_position = [eval(x.strip(",")) for x in current_position]

        return current_position
    
    def get_safe_height_jog_steps(self, location:list) -> int:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        
        joint_values = self.get_location_joint_values(location)
        current_pos = self.get_position()

        module_safe_height = joint_values[1] +  self.plate_above_height

        height_jog_steps = current_pos[1] - module_safe_height

        return height_jog_steps
    
    def set_location(self, location_name:str = "TEMP_0", R:int = 0, Z:int = 0, P:int = 0, Y:int = 0):
        """Saves a new location onto robot

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        
        command = "LOADPOINT %s, %s, %s, %s, %s\r\n" % (location_name, str(Z), str(P), str(Y), str(R)) # Command interpreted by Sciclops
        out_msg = self.send_command(command)
    
    def delete_location(self,location_name:str = None):
        """ Deletes a location from the robot's database

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        if not location_name:
            raise Exception("No location name provided")
        
        command = "DELETEPOINT %s\r\n" % (location_name) # Command interpreted by Sciclops
        out_msg = self.send_command(command)

    def gripper_open(self):
        """Opens gripper

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = 'OPEN\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

    def gripper_close(self):
        """Closes gripper
        
        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """


        command = 'CLOSE\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)
    
    def check_open(self):
        """ Checks if gripper is open

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = 'GETGRIPPERISOPEN\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

    def check_closed(self):
        """Checks if gripper is closed

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = 'GETGRIPPERISCLOSED\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

    def jog(self, axis, distance) -> None:
        """Moves the specified axis the specified distance.

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        command = 'JOG %s,%d\r\n' %(axis,distance) 
        out_msg = self.send_command(command)

    def move_joint_angles(self, R:int, Z:int, P:int, Y:int) -> None:
        """Moves on a single axis, using an existing location on robot's database

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

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

        self.deletepoint("TEMP", R, Z, P, Y) 

    def move_single_axis(self, axis:str, location:str) -> None:
        """Moves on a single axis using an existing location on robot's database

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

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
    
    def move_location(self, loc, move_time = 4.7) -> None:
        """Move to preset locations located in load_labware function

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        cmd = "MOVE "+ loc +"\r\n"
        self.send_command(cmd, timeout = move_time)

    def move_tower_neutral(self) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """


        self.move_single_axis("Z", "Safe")

    def move_arm_neutral(self) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        self.move_single_axis("Y", "Safe")

    def move_gripper_neutral(self) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        self.move_single_axis("P", "Safe")

    def move_joints_neutral(self) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        self.move_tower_neutral()
        self.move_arm_neutral()
        # self.move_gripper_neutral()

    def get_module_plate(self, source:list = None, height_jog_steps:int = None) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        # TODO:Handle the error raising within error_codes.py
        if not source:
            raise Exception("Source location was not given")
        
        if not height_jog_steps:
            height_jog_steps = self.get_safe_height_jog_steps(source)

        # TODO: Decide if plate location height will be reconfigured to be the correct grabbing height or the current Z axis will be kept. 
        #       Reconfiguring the Z axis values of the locations will remove two extra movement steps from this function.
        
        self.move_single_axis("Y", source)
        self.move_single_axis("Z", source)
        self.jog("Z", - self.plate_above_height)
        self.gripper_close()
        self.move_single_axis("Z", source)
        # self.move_single_axis("Y", "Safe")


    def put_module_plate(self, target:list = None, height_jog_steps:int = None) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        # TODO:Handle the error raising within error_codes.py
        if not target:
            raise Exception("Source location was not given")
        
        if not height_jog_steps:
            height_jog_steps = self.get_safe_height_jog_steps(target)    

        # TODO: Decide if plate location height will be reconfigured to be the correct grabbing height or the current Z axis will be kept. 
        #       Reconfiguring the Z axis values of the locations will remove two extra movement steps from this function.

        self.move_single_axis("Y", target)
        self.move_single_axis("Z", target)
        self.jog("Z", - self.plate_above_height)
        self.gripper_open()
        self.move_single_axis("Z", target)
        # self.move_single_axis("Y", "Safe")

    def move_module_entry(self, source:list = None, height_jog_steps:int = None) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        # TODO:Handle the error raising within error_codes.py
        if not source:
            raise Exception("Source location was not given")
        
        if not height_jog_steps:
            height_jog_steps = self.get_safe_height_jog_steps(source)

        self.move_single_axis("R",source)
        self.move_single_axis("P", source)
        self.jog("Z", -height_jog_steps)
    
    def pick_module_plate(self, source:str, height_jog_steps: int) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
     
        self.move_joints_neutral()
        self.gripper_open()

        self.move_module_entry(source, height_jog_steps)
        self.get_module_plate(source, height_jog_steps)

        self.move_arm_neutral()
        # self.move_joints_neutral()

    def place_module_plate(self, target:str, height_jog_steps:int) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

     
        self.move_joints_neutral()

        self.move_module_entry(target, height_jog_steps)
        self.put_module_plate(target, height_jog_steps)

        self.move_arm_neutral()
        # self.move_joints_neutral()

    def pick_stack_plate(self, source:str) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        self.gripper_close()
        self.move_joints_neutral()
        self.move_location(source)
        self.jog("Z", 10)
        self.gripper_open()
        self.jog("Z", - self.plate_above_height)
        self.gripper_close() 
        self.move_joints_neutral()

    def place_plate_exchange(self) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """
        self.move_joints_neutral()
        self.move_location("Exchange")
        self.gripper_open()
        self.move_joints_neutral()

    def place_plate_stack_entry(self, target:str) -> None:
        """Summary

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """

        self.move_joints_neutral()
        #TODO: Find the exhange location distance on Y axis from the given Stack number"
        stack_joint_angles = self.get_location_joint_values(target)
        stack_exchange_joint_angles = stack_joint_angles[0], self.stack_exchange_Z_height, stack_joint_angles[2], stack_joint_angles[3] - self.stack_exchange_Y_axis_steps
        self.move_joint_angles(R = stack_exchange_joint_angles[0], Z = stack_exchange_joint_angles[1], P = stack_exchange_joint_angles[2], Y = stack_exchange_joint_angles[3])
        self.gripper_open()
        self.move_joints_neutral()

    def pick_plate_exchange(self) -> None:
        pass
    def place_plate_stack() -> None:
        pass

    def stack_transfer(self, source:str = None, target:str = None) -> None:
        """
        Transfer a plate plate from plate stacker to exchange location

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """    
        if source and target:
            raise Exception("Pass either source or target location. Transfer can either be done to the stack or from the stack")    
        elif not source and not target:
            raise Exception("No destination was provided. Pass either source or target location.")
        
        if source:
            source = eval(source)

            if isinstance(source, list): 
                #Add an extra step to check if the locations were sent as names or joint angles. Then handle the transfer in two different ways 
                # Create a new location data 
                # Move to this location
                self.set_location("source_loc", source[0], source[1], source[2], source[3])
                source = "source_loc"

            self.pick_stack_plate(source)
            self.place_plate_exchange()

        elif target:
            target = eval(target)
            if isinstance(target, list): 
                self.set_location("target_loc", target[0], target[1], target[2], target[3])
                source = "target_loc"

            self.pick_plate_exchange() #TODO: Not completed yet
            self.place_plate_stack() #TODO: Not completed yet

        
        #BUG: Output messages of multiple commands mix up with eachother. Fix the wait times in between the command executions"

    
    def module_transfer(self, source:str, target:str) -> None:
        """
        Transfer a plate in between two modules

        :param [ParamName]: [ParamDescription], defaults to [DefaultParamVal]
        :type [ParamName]: [ParamType](, optional)
        ...
        :raises [ErrorType]: [ErrorDescription]
        ...
        :return: [ReturnDescription]
        :rtype: [ReturnType]
        """ 
        #Add an extra step to check if the locations were sent as names or joint angles. Then handle the transfer in two different ways 
        # Create a new location data 
        # Move to this location

        try:       
            source = eval(source)
        except NameError as source_type_err:
            # Location was given as a location name
            print("Source: " + source)   
        else:
            # Location was given as a joint values
            self.set_location("source_loc", source[0], source[1], source[2], source[3])
            source = "source_loc"
            print("Source: " + source)   

        try:
            target = eval(target)
        except NameError as target_type_err:
            # Location was given as a location name
            print("Target: " + target)   
        else:
            # Location was given as a joint values
            self.set_location("target_loc", target[0], target[1], target[2], target[3])
            target = "target_loc"
            print("Target: " + target)   

        source_height_jog_steps = self.get_safe_height_jog_steps(source)
        target_height_jog_steps = self.get_safe_height_jog_steps(target)


        self.pick_module_plate(source, source_height_jog_steps)
        self.place_module_plate(target, target_height_jog_steps)


if __name__ == "__main__":
    """
    Runs given function.
    """
    s = PlateCrane("/dev/ttyUSB2")
    source_loc = "SealerNest"
    target_loc = "PeelerNest"
    # s.get_location_joint_values(target_loc)
    # s.module_transfer(target_loc, source_loc)
    # s.move_joints_neutral()
    # s.send_command("LOADPOINT TEMP2 210256, -1050, 491, 5730", timeout= 5)
    # s.pick_module_plate("SealerNest")
    # s.get_status()
    # s.get_position()
    # s.home()
    # s.wait_robot_movement()
    # s.get_status()
    # s.get_position()
    # s.send_command("GETPOINT Safe\r\n")  
    # s.get_location_joint_values("Safe")
    # s.send_command("limp TRUE\r\n")
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

#    Crash error outputs 21(R axis),14(z axis), 0002 Wrong location name

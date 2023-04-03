import asyncio
from asyncio.unix_events import DefaultEventLoopPolicy
from pickle import TRUE
import time  
import serial
from serial import SerialException
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
        self.secondory_connection = None
        self.robot_error = "NO ERROR"

        self.status = 0
        self.error = ""
        self.gripper_length = 0
        self.plate_above_height = 800
        self.stack_exchange_Z_height = -31887
        self.stack_exchange_Y_axis_steps = 200 #TODO: Find the correct number of steps to move Y axis from the stack to the exchange location
        self.plate_detect_z_jog_steps = 500
        self.exchange_location = "LidNest2"

        self.robot_status = ""
        self.movement_state = "READY"
        self.platecrane_current_position = None

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
            # self.secondory_connection = serial.Serial(self.host_path, self.baud_rate, timeout=1)
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
        if self.robot_status == "0":
            self.home()
        self.platecrane_current_position = self.get_position()

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
    
    def get_error_output(self, output:str):
        #Find the errors from error codes.
        self.robot_error = "err"
        pass

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

        # current_postion = self.get_position()
        # print(current_postion)
        # print(self.platecrane_current_position)
        # if self.platecrane_current_position != current_postion:
        #     self.movement_state = "BUSY"
        #     self.platecrane_current_position = current_postion
        # else:
        #     self.movement_state = "READY"
        # print(self.movement_state)
        self.movement_state = "READY"

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
        
    
    def free_joints(self):

        command = 'limp TRUE\r\n' 
        out_msg =  self.send_command(command)
    
    def lock_joints(self):

        command = 'limp FALSE\r\n' 
        out_msg =  self.send_command(command)

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
        out_msg = self.send_command(command, timeout=0.5)

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

    def move_single_axis(self, axis:str, loc:str, delay_time = 1.5) -> None:
        """Moves on a single axis using an existing location on robot's database

        :param axis: Axis name (R,Z,P,Y)
        :type axis: str
        :param loc: Name of the location. 
        :type loc: str

        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """

        # TODO:Handle the error raising within error_codes.py
        if not loc:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 

        # self.loadpoint(R, Z, P, Y)

        command = "MOVE_"+ axis.upper() + " " + loc + "\r\n" 

        out_msg_move = self.send_command(command, timeout=delay_time)

        self.move_status = "COMPLETED"
            

        # self.deletepoint(R, Z, P, Y)
    
    def move_location(self, loc: str = None, move_time:float = 4.7) -> None:
        """Moves all joint to the given location.

        :param loc: Name of the location. 
        :type loc: str
        :param move_time: Number of seconds that will take to complete this movement. Defults to 4.7 seconds which is the longest possible movement time.
        :type move_time: float
        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """

        # TODO:Handle the error raising within error_codes.py
        if not loc:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 
        
        cmd = "MOVE "+ loc +"\r\n"
        self.send_command(cmd, timeout = move_time)

    def move_tower_neutral(self) -> None:
        """Moves the tower to neutral position

        :return: None
        """

        self.move_single_axis("Z", "Safe", delay_time = 1.5)

    def move_arm_neutral(self) -> None:
        """Moves the arm to neutral position

        :return: None
        """
        self.jog("Z",200)
        self.move_single_axis("Y", "Safe", delay_time = 1)

    def move_gripper_neutral(self) -> None:
        """Moves the gripper to neutral position

        :return: None
        """
        self.move_single_axis("P", "Safe", delay_time = 0.3)

    def move_joints_neutral(self) -> None:
        """Moves all joints neutral posiiton

        :return: None
        """
        self.move_arm_neutral()
        self.move_tower_neutral()
        # self.move_gripper_neutral()

    def get_module_plate(self, source:list = None, height_jog_steps:int = 0, height_offset:int = 0) -> None:
        """picks up the plate from a module location by moving each joint step by step

        :param source: Name of the source location. 
        :type source: str
        :param height_jog_steps: Number of jogging steps that will be used to move the Z axis to the plate location 
        :type height_jog_steps: int
        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """

        # TODO:Handle the error raising within error_codes.py
        if not source:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 
        
        if not height_jog_steps:
            height_jog_steps = self.get_safe_height_jog_steps(source)

        # TODO: Decide if plate location height will be reconfigured to be the correct grabbing height or the current Z axis will be kept. 
        #       Reconfiguring the Z axis values of the locations will remove two extra movement steps from this function.

        self.move_single_axis("Y", source)
        # self.move_single_axis("Z", source)
        self.jog("Z", - 2*(self.plate_above_height+100 - height_offset))
        self.gripper_close()
        self.jog("Z", 2*self.plate_above_height)


    def put_module_plate(self, target:list = None, height_jog_steps:int = 0, height_offset:int = 0) -> None:
        """Places the plate onto a module location by moving each joint step by step

        :param target: Name of the target location. 
        :type target: str
        :param height_jog_steps: Number of jogging steps that will be used to move the Z axis to the plate location 
        :type height_jog_steps: int
        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """

        # TODO:Handle the error raising within error_codes.py
        if not target:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 
        
        
        if not height_jog_steps:
            height_jog_steps = self.get_safe_height_jog_steps(target)    

        # TODO: Decide if plate location height will be reconfigured to be the correct grabbing height or the current Z axis will be kept. 
        #       Reconfiguring the Z axis values of the locations will remove two extra movement steps from this function.

        self.move_single_axis("Y", target)
        # self.move_single_axis("Z", target)
        self.jog("Z", - 2*(self.plate_above_height+100 - height_offset))
        self.gripper_open()
        self.jog("Z", 2*self.plate_above_height)

    def move_module_entry(self, source:list = None, height_jog_steps:int = 0) -> None:
        """Moves to the entry location of the location that is given. It moves the R,P and Z joints step by step to aviod collisions. 

        :param source: Name of the source location. 
        :type source: str
        :param height_jog_steps: Number of jogging steps that will be used to move the Z axis to the plate location 
        :type height_jog_steps: int
        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """
        # TODO:Handle the error raising within error_codes.py

        if not source:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 
        
        if not height_jog_steps:
            height_jog_steps = self.get_safe_height_jog_steps(source)

        self.move_single_axis("R",source)
        self.move_single_axis("P", source)
        self.jog("Z", -height_jog_steps+200)
    
    def pick_module_plate(self, source:str = None, height_jog_steps: int = 0, height_offset:int = 0) -> None:
        """Pick a module plate from a module location.

        :param source: Name of the source location. 
        :type source: str
        :param height_jog_steps: Number of jogging steps that will be used to move the Z axis to the plate location 
        :type height_jog_steps: int
        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """
        if not source:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 
        
        self.move_joints_neutral()
        self.gripper_open()

        self.move_module_entry(source, height_jog_steps)
        self.get_module_plate(source, height_jog_steps, height_offset)

        self.move_arm_neutral()

    def place_module_plate(self, target:str = None, height_jog_steps:int = 0, height_offset:int = 0) -> None:
        """Place a module plate onto a module location.

        :param target: Name of the target location. 
        :type target: str
        :param height_jog_steps: Number of jogging steps that will be used to move the Z axis to the plate location 
        :type height_jog_steps: int
        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """
        if not target:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 
     
        self.move_joints_neutral()

        self.move_module_entry(target, height_jog_steps)
        self.put_module_plate(target, height_jog_steps, height_offset)

        self.move_arm_neutral()

    def pick_stack_plate(self, source:str = None, height_offset:int = 0) -> None:
        """Pick a stack plate from stack location.

        :param source: Name of the source location. 
        :type source: str
        :raises [PlateCraneLocationException]: [Error for None type locations]
        :return: None
        """
        #TODO: Create error exceptions for below case
        if not source:
            raise Exception("PlateCraneLocationException: 'None' type variable is not compatible as a location") 

        self.gripper_close()
        self.move_joints_neutral()
        self.move_single_axis("R",source)
        self.move_location(source)
        self.jog("Z", self.plate_detect_z_jog_steps)
        self.gripper_open()
        self.jog("Z", - self.plate_above_height + height_offset - self.plate_detect_z_jog_steps)
        self.gripper_close() 
        self.move_tower_neutral()
        self.move_arm_neutral()

    def place_stack_plate(self, target:str = None, height_offset:int = 0) -> None:
        """Place a stack plate either onto the exhange location or into a stack

        :param target: Name of the target location. Defults to None if target is None, it will be set to exchange location.
        :type target: str
        :return: None
        """
        if not target:
            target = self.exchange_location

        self.move_joints_neutral()
        self.move_single_axis("R",target)
        self.move_location(target)
        self.jog("Z", - self.plate_above_height + height_offset)
        self.gripper_open()
        self.move_tower_neutral()
        self.move_arm_neutral()


    def _is_location_joint_values(self, location:str, name:str="temp") -> str:
        """
        If the location was provided as joint values, transfer joint values into a saved location on the robot and return the location name. 
        If location parameter is a name of an already saved location, do nothing.

        :param location: Location to be checked if this is an already saved location on the robot database or a new location with 4 joint values 
        :type location: string
        :param name: Location name to be used to save a new location if the location parameter was provided as 4 joint values 
        :type name: string
        :raises [ErrorType]: [ErrorDescription]
        :return: location_name = Returns the location name that is saved on robot database with location joint values
        :rtype: str
        """    
        try:       
            location = eval(location)
        except NameError as name_err:
            # Location was given as a location name
            print(name + ": " + location)   
            location_name = location
        else:
            # Location was given as a joint values
            location_name = name + "_loc"
            self.set_location(location_name, location[0], location[1], location[2], location[3])
            print(name + ": " + location_name)

        return location_name
    
    def stack_transfer(self, source:str = None, target:str = None, source_type:str = "stack", target_type:str = "module", height_offset:int = 0) -> None:
        """
        Transfer a plate plate from a plate stack to the exchange location or make a transfer in between stacks and stack entry locations

        :param source: Source location, provided as either a location name or 4 joint values.
        :type source: str
        :param target: Target location, provided as either a location name or 4 joint values.
        :type target: str
        :raises [ErrorType]: [ErrorDescription]
        :return: None
        """    

        if not source:
            print("Please provide a source location")
            # TODO: Raise an exception here
            return
        
        source = self._is_location_joint_values(location = source, name = "source")
        
        if target:
            target = self._is_location_joint_values(location = target, name = "target") 

            # If target was provided, stack transfer can be used to pick and place plate in between stacks or stack entry locations

        elif not target:
            target = self.exchange_location # Assumes getting a new plate from the plate stack and placing onto the exchange spot
        
        if source_type.lower() == "stack":
            self.pick_stack_plate(source, height_offset)
        elif source_type.lower() == "module":
            self.pick_module_plate(source, height_offset)

        # time.sleep(2)
        target_height_jog_steps = self.get_safe_height_jog_steps(target)
        if target_type == "stack":
            self.place_stack_plate(target,height_offset)
        elif target_type == "module":
            self.place_module_plate(target, target_height_jog_steps, height_offset)
        
        #BUG: Output messages of multiple commands mix up with eachother. Fix the wait times in between the command executions"

    
    def module_transfer(self, source:str, target:str, height_offset:int = 0) -> None:
        """
        Transfer a plate in between two modules using source and target locations

        :param source: Source location, provided as either a location name or 4 joint values.
        :type source: str
        :param target: Target location, provided as either a location name or 4 joint values.
        :type target: str
        :raises [ErrorType]: [ErrorDescription]
        :return: None
        """ 
        self.move_joints_neutral()
        source = self._is_location_joint_values(location = source, name = "source")
        target = self._is_location_joint_values(location = target, name = "target")

        source_height_jog_steps = self.get_safe_height_jog_steps(source)
        target_height_jog_steps = self.get_safe_height_jog_steps(target)

        self.pick_module_plate(source, source_height_jog_steps, height_offset)
        self.place_module_plate(target, target_height_jog_steps, height_offset)

    def transfer(self, source:str = None, target:str = None, source_type:str = "stack", target_type:str = "module", height_offset:int = 0):
        """
        Handles the transfer request 

        :param source: Source location, provided as either a location name or 4 joint values.
        :type source: str
        :param target: Target location, provided as either a location name or 4 joint values.
        :type target: str
        :raises [ErrorType]: [ErrorDescription]
        :return: None
        """ 

        # if (not stack_transfer and not module_transfer) or (stack_transfer and module_transfer):
        #     raise Exception("Transfer type needs to be specified! Use either stack transfer or module transfer.")

        if source_type == "stack" or target_type == "stack":
            self.stack_transfer(source, target, source_type, target_type, height_offset)
        elif source_type == "module" and target_type == "module":
            self.module_transfer(source, target, height_offset)
        self.move_joints_neutral()
        time.sleep(2)
        self.move_location("Safe")
        time.sleep(10)

if __name__ == "__main__":
    """
    Runs given function.
    """
    s = PlateCrane("/dev/ttyUSB2")
    stack = "Stack1"
    source_loc = "SealerNest"
    target_loc = "PeelerNest"
    # s.place_stack_plate("Liconic.Nest")
    s.transfer(source_loc, "Liconic.Nest", source_type = "stack", target_type = "stack", height_offset=0)
    # s.transfer(source_loc, target_loc, stack_transfer = False, module_transfer = True)

    # s.get_location_joint_values(target_loc)
    # s.module_transfer(target_loc, source_loc)
    # s.move_joints_neutral()
    # s.pick_module_plate("SealerNest")
    # s.get_status()
    # s.get_position()
    # s.home()
    # s.wait_robot_movement()
    # s.get_status()
    # s.get_position()
    # s.get_location_joint_values("Safe")
    # s.set_location()
    # s.get_location_list()
    # s.delete_location("TEMP_0")
    # s.get_location_list()

    # s.jog("Z", 60000)
    # s.send_command("Move 166756, -32015, -5882, 5460\r\n")
    # s.send_command("move_abs Z")
    # s.send_command("MOVE TEMP 117902 2349 -5882 0\r\n")  
    # s.send_command("MOVE Y 5000\r\n")  

#    Crash error outputs 21(R axis),14(z axis), 0002 Wrong location name. 1400 (Z axis hits the plate)

from asyncio.unix_events import DefaultEventLoopPolicy
from pickle import TRUE
from time import sleep  
import usb.core
import usb.util
import sys
import re

class SCICLOPS():
    '''
    Description: 
    Python interface that allows remote commands to be executed to the Sciclops. 
    '''
    
    def __init__(self, host_path):
        self.host_path = host_path
        self.VENDOR_ID = 0x7513
        self.PRODUCT_ID = 0x0002
        self.TEACH_PLATE = 15.0
        self.STD_FINGER_LENGTH = 17.2
        self.COMPRESSION_DISTANCE = 3.35
        # self.CURRENT_POS = [0, 0 ,0, 0]
        # self.NEST_ADJUSTMENT = 20.0
        # self.STATUS = 0
        # self.VERSION = 0
        # self.CONFIG = 0
        # self.ERROR = ""
        # self.GRIPLENGTH = 0
        # self.COLLAPSEDDISTANCE = 0
        # self.STEPSPERUNIT = [0, 0 ,0, 0]
        # self.HOMEMSG = ""
        # self.OPENMSG = ""
        # self.CLOSEMSG = ""
        self.labware = self.load_labware()
        self.success_count = 0
    

    def connect_sciclops(self):
        '''
        Connect to serial port / If wrong port entered inform user 
        '''
        
        dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)

        if dev is None:
            sys.exit("Could not find Id System Barcode Reader.")
        else:
            print('Device detected')


    def load_labware(self, labware_file=None):
        '''
        Loads plate information which affects get_plate function.
        '''
        if labware_file:
            pass #will load file in the future
        
        # Dictionary for plate information
        labware = {
            'tower1':{
            'pos':{
                'Z': 23.5188,
                'R': 133.5,
                'Y': 171.9895,
                'P': 8.6648
                },
            'type': '96plates',
            'howmany': 3,
            'grab_height': 8,
            'cap_heigh': 20,
            'size': [10,11,12]
            },
            'tower2':{
            'pos': {
                'Z': 23.5188,
                'R': 151.5,
                'Y': 171.4872,
                'P': 8.4943
                },
            'type': '96plates',
            'howmany': 3,
            'grab_height': 12,
            'cap_heigh': 20,
            'size': [10,11,12]
            },
            'tower3':{
            'pos': {
                'Z': 23.5188,
                'R': 169.5,
                'Y': 171.4810,
                'P': 12.4716
                },
            'type': '96plates',
            'howmany': 3,
            'grab_height': 12,
            'cap_heigh': 20,
            'size': [10,11,12]
            },
            'tower4':{
            'pos': {
                'Z': 23.5188,
                'R': 187.5,
                'Y': 169.4470,
                'P': 5.9091
                },
            'type': '96plates',
            'howmany': 3,
            'size': [10,11,12]
            },
            'tower5':{
            'pos': {
                'Z': 23.5188,
                'R': 206.0,
                'Y': 171.2082,
                'P': 10.8807
                },
            'type': '96plates',
            'howmany': 3,
            'size': [10,11,12]
            },
            'exchange':{
            'pos': {
                'Z': 23.5188,
                'R': 109.2741,
                'Y': 32.7484,
                'P': 98.2955
                },
            'type': '96plates',
            'howmany': 3,
            'size': [10,11,2]
            },
            'neutral':{
                'pos': {
                    'Z': 23.5188,
                    'R': 109.2741,
                    'Y': 32.7484,
                    'P': 98.2955
                    }
            }
        }

        return labware
            

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

    def get_error(self, response_buffer):
        '''
        Gets error message from the feedback.
        '''

        output_line = response_buffer[response_buffer[:-1].rfind("\n"):]
        exp = r"(\d)(\d)(\d)(\d)(.*\w)" # Format of feedback that indicates an error message
        output_line = re.search(exp, response_buffer)
       
        try:
        # Checks if specified format is found in the last line of feedback
            if output_line[5][5:9] != "0000": 
                self.ERROR = "ERROR: %s" % output_line[5]
                
        except: 
            pass

################################
# Individual Command Functions
    
    def get_position(self):
        '''
        Requests and stores sciclops position.
        Coordinates:
        Z: Vertical axis
        R: Base turning axis
        Y: Extension axis
        P: Gripper turning axis
        '''

        command = 'GETPOS\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"Z:([-.\d]+), R:([-.\d]+), Y:([-.\d]+), P:([-.\d]+)" # Format of coordinates provided in feedback
            find_current_pos = re.search(exp,out_msg)
            self.CURRENT_POS = [float(find_current_pos[1]), float(find_current_pos[2]), float(find_current_pos[3]), float(find_current_pos[4])]
            
            print(self.CURRENT_POS)
        except:
            pass

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
            self.STATUS = find_status[1]
        
            print(self.STATUS)
        
        except:
            pass

    def get_version(self):
        '''
        Checks version of Sciclops
        '''

        command = 'VERSION\r\n' # Command interpreted by Sciclops
        out_msg =  self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the version
            find_version = re.search(exp,out_msg)
            self.VERSION = find_version[1]

            print(self.VERSION)
        
        except:
            pass

    def get_config(self):
        '''
        Checks configuration of Sciclops
        '''

        command = 'GETCONFIG\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the configuration
            find_config = re.search(exp,out_msg)
            self.CONFIG = find_config[1]

            print(self.CONFIG)
        
        except:
            pass

    def get_grip_length(self):
        '''
        Checks current length of the gripper (units unknown) of Sciclops
        '''

        command = 'GETGRIPPERLENGTH\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the gripper length
            find_grip_length = re.search(exp,out_msg)
            self.GRIPLENGTH = find_grip_length[1]

            print(self.GRIPLENGTH)

        except:
            pass

    def get_collapsed_distance(self):
        '''
        ???
        '''

        command = 'GETCOLLAPSEDISTANCE\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the collapsed distance
            find_collapsed_distance = re.search(exp,out_msg)
            self.COLLAPSEDDISTANCE = find_collapsed_distance[1]

            print(self.COLLAPSEDDISTANCE)

        except:
            pass

    def get_steps_per_unit(self):
        '''
        ???
        '''

        command = 'GETSTEPSPERUNIT\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)
        
        try:
            # Checks if specified format is found in feedback
            exp = r"Z:([-.\d]+),R:([-.\d]+),Y:([-.\d]+),P:([-.\d]+)" # Format of the coordinates provided in feedback
            find_steps_per_unit = re.search(exp,out_msg)        
            self.STEPSPERUNIT = [float(find_steps_per_unit[1]), float(find_steps_per_unit[2]), float(find_steps_per_unit[3]), float(find_steps_per_unit[4])]

            print(self.STEPSPERUNIT)

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


    def open(self):
        '''
        Opens gripper
        '''

        command = 'OPEN\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the success message
            open_msg = re.search(exp,out_msg)
            self.OPENMSG = open_msg[1]
            print(self.OPENMSG)

        except:
            pass

    def close(self):
        '''
        Closes gripper
        '''

        command = 'CLOSE\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line is the success message
            close_msg = re.search(exp,out_msg)
            self.CLOSEMSG = close_msg[1]

            print(self.CLOSEMSG)
        except:
            pass
        
    def check_open(self):
        '''
        Checks if gripper is open
        '''

        command = 'GETGRIPPERISOPEN\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line answers if the gripper is open
            check_open_msg = re.search(exp,out_msg)
            self.CHECKOPENMSG = check_open_msg[1]

            print(self.CHECKOPENMSG)
        except:
            pass

    def check_closed(self):
        '''
        Checks if gripper is closed
        '''

        command = 'GETGRIPPERISCLOSED\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates that the rest of the line answers if the gripper is closed
            check_closed_msg = re.search(exp,out_msg)
            self.CHECKCLOSEDMSG = check_closed_msg[1]

            print(self.CHECKCLOSEDMSG)

        except:
            pass

    def check_plate(self):
        '''
        ???
        '''

        command = 'GETPLATEPRESENT\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates ???
            check_plate_msg = re.search(exp,out_msg)
            self.CHECKPLATEMSG = check_plate_msg[1]

            print(self.CHECKPLATEMSG)
        
        except:
            pass


    def set_speed(self, speed):
        '''
        Changes speed of Sciclops
        '''

        command = 'SETSPEED %d\r\n' % speed # Command interpreted by Sciclops
        out_msg = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            exp = r"0000 (.*\w)" # Format of feedback that indicates success message
            set_speed_msg = re.search(exp,out_msg)
            self.SETSPEEDMSG = set_speed_msg[1]
            print(self.SETSPEEDMSG)
        except:
            pass
        

    def list_points(self):
        '''
        Lists all of the preset points
        '''

        command = 'LISTPOINTS\r\n' # Command interpreted by Sciclops
        out_msg = self.send_command(command)


        try:
            # Checks if specified format is found in feedback
            list_point_msg_index = out_msg.find("0000") # Format of feedback that indicates success message
            self.LISTPOINTS = out_msg[list_point_msg_index+4:]
            print(self.LISTPOINTS)
        except:
            pass


    def jog(self, axis, distance):
        '''
        Moves the specified axis the specified distance.
        '''

        command = 'JOG %s,%d\r\n' %(axis,distance) # Command interpreted by Sciclops
        out_msg = self.send_command(command)


        try:
            # Checks if specified format is found in feedback
            jog_msg_index = out_msg.find("0000") # Format of feedback that indicates success message
            self.JOGMSG = out_msg[jog_msg_index+4:]
            print(self.JOGMSG)
        except:
            pass
    
    def loadpoint(self, R, Z, P, Y):
        '''
        Adds point to listpoints function
        '''
        
        command = "LOADPOINT R:%s, Z:%s, P:%s, Y:%s, R:%s\r\n" % (R, Z, P, Y, R) # Command interpreted by Sciclops
        out_msg = self.send_command(command)
        try:
            # Checks if specified format is found in feedback
            loadpoint_msg_index = out_msg.find("0000") # Format of feedback that indicates success message
            self.LOADPOINTMSG = out_msg[loadpoint_msg_index+5:]
        except:
            pass
    
    def deletepoint(self, R, Z, P, Y):
        '''
        Deletes point from listpoints function
        '''

        command = "DELETEPOINT R:%s\r\n" % R # Command interpreted by Sciclops
        out_msg = self.send_command(command)
        try:
            # Checks if specified format is found in feedback
            deletepoint_msg_index = out_msg.find("0000") # Format of feedback that indicates success message
            self.DELETEPOINTMSG = out_msg[deletepoint_msg_index+5:]
        except:
            pass

    def move(self, R, Z, P, Y):
        '''
        Moves to specified coordinates
        '''

        self.loadpoint(R, Z, P, Y)

        command = "MOVE R:%s\r\n" % R
        out_msg_move = self.send_command(command)

        try:
            # Checks if specified format is found in feedback
            move_msg_index = out_msg_move.find("0000") # Format of feedback that indicates success message
            self.MOVEMSG = out_msg_move[move_msg_index+4:]
        except:
            pass

        self.deletepoint(R, Z, P, Y)
    
    def move_loc(self, loc):
        '''
        Move to preset locations located in load_labware function
        '''

        #check if loc exists (later)
        self.move(self.labware[loc]['pos']['R'],self.labware[loc]['pos']['Z'],self.labware[loc]['pos']['P'],self.labware[loc]['pos']['Y'])

    def get_plate(self, location):
        '''
        Grabs plate and places on exchange. Paramater is the stack that the Sciclops is requested to remove the plate from.
        Format: "Stack<num>"
        '''

        tower_info = self.labware[location]

        # Move arm up and to neutral position to avoid hitting any objects
        self.open() 
        self.set_speed(3) # 
        self.jog('Y', -1000)
        self.jog('Z', 1000) 
        self.set_speed(12) 
        self.move(R=self.labware['neutral']['pos']['R'], Z=23.5188, P=self.labware['neutral']['pos']['P'], Y=self.labware['neutral']['pos']['Y'])

        # Move above desired tower
        self.set_speed(12)
        self.move(R=tower_info['pos']['R'], Z=23.5188, P=tower_info['pos']['P'], Y=tower_info['pos']['Y'])
        
        # Remove plate from tower
        self.set_speed(7)
        self.jog('Z', -1000)
        grab_height = tower_info.get('grab_height', 12.5)
        self.jog('Z', grab_height)
        self.close()
        self.set_speed(12)
        self.jog('Z', 1000)
        
        # Place in exchange
        self.move(R=self.labware['exchange']['pos']['R'], Z=23.5188, P=self.labware['exchange']['pos']['P'], Y=self.labware['exchange']['pos']['Y'])
        self.jog('Z', -395)
        self.set_speed(1)
        self.jog('Z', -20)
        self.open()
        sleep(0.1)
        self.set_speed(12)
        self.jog('Z', 1000)

        # Move back to neutral
        self.move(R=self.labware['neutral']['pos']['R'], Z=23.5188, P=self.labware['neutral']['pos']['P'], Y=self.labware['neutral']['pos']['Y'])
        

    def limp(self, limp_bool):
        '''
        Turns on/off limp mode (allows someone to manually move joints)
        '''
        if limp_bool == True:
            limp_string = "FALSE"
        if limp_bool == False:
            limp_string = "TRUE"
        command = "LIMP %s" % limp_string # Command interpreted by Sciclops
        self.send_command(command)




if __name__ == "__main__":
    '''
    Runs given function.
    '''
    dummy_sciclops = SCICLOPS(usb.core.find(idVendor= 0x7513, idProduct=0x0002))
    dummy_sciclops.check_plate()

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
"MOVE "
"JOG"
"DELETEPOINT (ADD POINT)"
"LISTPOINTS"
"LOADPOINT R:0,Z:0,P:0,Y:0"
"LIMP TRUE/FALSE"

#Unfinished Commands
"GETLIMITS"

#Unknown Commands
"LISTMOTIONS"
"AUTOTEACH"
"GETPOINT"
"READINP 15"
"GETGRIPSTRENGTH"
"READINP"
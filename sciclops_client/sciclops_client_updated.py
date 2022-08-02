import usb.core
import usb.util
import sys
import re

class SCICLOPS():
    
    
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
    
    def connect_sciclops(self):
        dev = usb.core.find(idVendor=self.VENDOR_ID, idProduct=self.PRODUCT_ID)

        if dev is None:
            sys.exit("Could not find Id System Barcode Reader.")
        else:
            print('Device detected')


    def send_command(self, command):
        self.host_path.write(4,command)

        response_buffer = "Write: "+ command
        msg = None

        #Adds SciClops output to response_buffer
        while msg != command:
            # or "success" not in msg or "error" in msg
            try:
                response =  self.host_path.read(0x83,200)
            except:
                break
            msg = ''.join(chr(i) for i in response)
            response_buffer = response_buffer + "Read:" + msg
        print(response_buffer)

        self.get_error(response_buffer)
       

        return response_buffer

    def get_error(self, response_buffer):
        output_line = response_buffer[response_buffer[:-1].rfind("\n"):]
        exp = r"(\d)(\d)(\d)(\d)(.*\w)"
        output_line = re.search(exp, response_buffer)
       
        try:
            if output_line[5][5:9] != "0000":
                self.ERROR = "ERROR: %s" % output_line[5]
            print(self.ERROR)
        except: 
            pass

################################
# Individual Command Functions
    
    def get_position(self):
        command = 'GETPOS\r\n'
        out_msg = self.send_command(command)
        
        exp = r"Z:([-.\d]+), R:([-.\d]+), Y:([-.\d]+), P:([-.\d]+)"
        find_current_pos = re.search(exp,out_msg)
        self.CURRENT_POS = [float(find_current_pos[1]), float(find_current_pos[2]), float(find_current_pos[3]), float(find_current_pos[4])]
        
        print(self.CURRENT_POS)


    def get_status(self):
        command = 'STATUS\r\n'
        out_msg =  self.send_command(command)
        
        exp = r"0000 (.*\w)"
        find_status= re.search(exp,out_msg)
        self.STATUS = find_status[1]
        
        print(self.STATUS)
        

    def get_version(self):
        command = 'VERSION\r\n'
        out_msg =  self.send_command(command)
        
        exp = r"0000 (.*\w)"
        find_version = re.search(exp,out_msg)
        self.VERSION = find_version[1]

        print(self.VERSION)
        

    def get_config(self):
        command = 'GETCONFIG\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        find_config = re.search(exp,out_msg)
        self.CONFIG = find_config[1]

        print(self.CONFIG)
        

    def get_grip_length(self):
        command = 'GETGRIPPERLENGTH\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        find_grip_length = re.search(exp,out_msg)
        self.GRIPLENGTH = find_grip_length[1]

        print(self.GRIPLENGTH)


    def get_collapsed_distance(self):
        command = 'GETCOLLAPSEDISTANCE\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        find_collapsed_distance = re.search(exp,out_msg)
        self.COLLAPSEDDISTANCE = find_collapsed_distance[1]

        print(self.COLLAPSEDDISTANCE)
    

    def get_steps_per_unit(self):
        command = 'GETSTEPSPERUNIT\r\n'
        out_msg = self.send_command(command)
        
        exp = r"Z:([-.\d]+),R:([-.\d]+),Y:([-.\d]+),P:([-.\d]+)"
        find_steps_per_unit = re.search(exp,out_msg)        
        self.STEPSPERUNIT = [float(find_steps_per_unit[1]), float(find_steps_per_unit[2]), float(find_steps_per_unit[3]), float(find_steps_per_unit[4])]

        print(self.STEPSPERUNIT)


    def home(self, axis = ""):
        command = 'HOME\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        home_msg = re.search(exp,out_msg)
        self.HOMEMSG = home_msg[1]

        print(self.HOMEMSG)
    

    def open(self):
        command = 'OPEN\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        open_msg = re.search(exp,out_msg)
        self.OPENMSG = open_msg[1]

        print(self.OPENMSG)
    

    def close(self):
        command = 'CLOSE\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        close_msg = re.search(exp,out_msg)
        self.CLOSEMSG = close_msg[1]

        print(self.CLOSEMSG)
        
        
    def check_open(self):
        command = 'GETGRIPPERISOPEN\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        check_open_msg = re.search(exp,out_msg)
        self.CHECKOPENMSG = check_open_msg[1]

        print(self.CHECKOPENMSG)


    def check_closed(self):
        command = 'GETGRIPPERISCLOSED\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        check_closed_msg = re.search(exp,out_msg)
        self.CHECKCLOSEDMSG = check_closed_msg[1]

        print(self.CHECKCLOSEDMSG)

    def check_plate(self):
        command = 'GETPLATEPRESENT\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        check_plate_msg = re.search(exp,out_msg)
        self.CHECKPLATEMSG = check_plate_msg[1]

        print(self.CHECKPLATEMSG)


    def set_speed(self, speed):
        command = 'SETSPEED %d\r\n' % speed
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"
        set_speed_msg = re.search(exp,out_msg)
        self.SETSPEEDMSG = set_speed_msg[1]

        print(self.SETSPEEDMSG)


    def move_temp(self):
        command = 'MOVE TEMP\r\n'
        out_msg = self.send_command(command)

        exp = r"0000 (.*\w)"

        try:
            move_temp_msg = re.search(exp,out_msg)
            self.MOVETEMPMSG = move_temp_msg[1]

            print(self.MOVETEMPMSG)
        except:
            pass


    def list_points(self):
        command = 'LISTPOINTS\r\n'
        out_msg = self.send_command(command)


        try:
            list_point_msg_index = out_msg.find("0000")
            self.LISTPOINTS = out_msg[list_point_msg_index+4:]
            print(self.LISTPOINTS)
        except:
            pass




if __name__ == "__main__":
    '''
    Runs given function.
    '''

    dummy_sciclops = SCICLOPS(usb.core.find(idVendor= 0x7513, idProduct=0x0002))
    dummy_sciclops.list_points()

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



"LISTPOINTS"
"LOADPOINT R:0,Z:0,P:0,Y:0"
"GETLIMITS"

#Unknown Commands
"MOVE "
"JOG"
"LISTMOTIONS"
"AUTOTEACH"
"GETPOINT"
"READINP 15"
"GETGRIPSTRENGTH"
"READINP"
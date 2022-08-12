#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from std_msgs.msg import String
import usb.core
import usb.util

from sciclops_module_services.srv import SciclopsDescription 
from sciclops_module_services.srv import SciclopsActions   


from sciclops_driver.sciclops_driver import SCICLOPS # import sciclops driver



class sciclopsNode(Node):
    '''
    The sciclopsNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the sciclops and a description of the sciclops to the respective topics.
    '''
    def __init__(self, PORT = usb.core.find(idVendor= 0x7513, idProduct=0x0002), NODE_NAME = "Sciclops_Node"):
        '''
        The init function is neccesary for the sciclopsNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''

        super().__init__(NODE_NAME)
        
        print("Wakey wakey eggs & bakey") 

        self.sciclops = SCICLOPS(PORT)

        self.state = "UNKNOWN"

        # Format:
        # [
        # [command, [subcommand 1, subcommand 2], [[paramater 1( subcommand 1), paramater 2( subcommand 1)], [""],[""]]
        # repeat
        # ]
        
        self.peelerDescription = [
            ["Get Plate 1",["get_plate"], [["tower1"]]],
            ["Get Plate 2",["get_plate"], [["tower2"]]],
            ["Get Plate 3",["get_plate"], [["tower3"]]],
            ["Get Plate 4",["get_plate"], [["tower4"]]],
            ["Get Plate 5",["get_plate"], [["tower5"]]],
            ["Home",["home"], [[""]]],
            ]



        timer_period = 0.5  # seconds


        self.statePub = self.create_publisher(String, 'sciclops_state', 10)

        self.stateTimer = self.create_timer(timer_period, self.stateCallback)


        self.actionSrv = self.create_service(SciclopsActions, "sciclops_actions", self.actionCallback)

        self.descriptionSrv = self.create_service(SciclopsDescription, "sciclops_description", self.descriptionCallback)


    def descriptionCallback(self, request, response):

        '''
        The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can perform as well as deliver essential information required by the master node.
        '''

        if request.description_request == 'Sciclops': 

            response.description_response = self.sciclopsDescription

            self.get_logger().info('Incoming  Good')
        
        else:

            response.description_response = 'Sciclops Description Failed'

        return response


    def actionCallback(self, request, response):

        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        '''
        
        self.manager_command = request.action_request # Run commands if manager sends corresponding command
    
        self.state = "BUSY"

        self.stateCallback()

        match self.manager_command:
            
            case "Get Plate 1":
                self.sciclops.get_plate('tower1')

                response.action_response = True
            
            case "Get Plate 2":
                    self.sciclops.get_plate('tower2')

                    response.action_response = True
                        
            case "Get Plate 3":
                    self.sciclops.get_plate('tower3')

                    response.action_response = True
                        
            case "Get Plate 4":
                    self.sciclops.get_plate('tower4')

                    response.action_response = True
                        
            case "Get Plate 5":
                    self.sciclops.get_plate('tower5')

                    response.action_response = True

            case "Home":
                    self.sciclops.home()

                    response.action_response = True
            
            case "Status":
                self.sciclops.get_status()
                
                response.action_response = True

            case other:
                response.action_response= False
        
        self.state = "COMPLETED"

        # if "Error:" in self.peeler.peeler_output:
        #     self.state = self.peeler.error_msg
        

        return response


    def stateCallback(self):

        '''
        Publishes the sciclops state to the 'state' topic. 
        '''

        msg = String()

        msg.data = 'State: %s' % self.state

        self.statePub.publish(msg)

        self.get_logger().info('Publishing: "%s"' % msg.data)
        
        self.state = "READY"



def main(args = None):

    PORT = usb.core.find(idVendor= 0x7513, idProduct=0x0002)           # port name for peeler
    NAME = "Sciclops_Node"

    rclpy.init(args=args)  # initialize Ros2 communication

    node = sciclopsNode(PORT=PORT, NODE_NAME=NAME)

    rclpy.spin(node)     # keep Ros2 communication open for action node

    rclpy.shutdown()     # kill Ros2 communication


if __name__ == '__main__':

    main()

#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from std_msgs.msg import String

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions   

from time import sleep

from sciclops_driver.sciclops_driver import SCICLOPS # import sciclops driver

class sciclopsNode(Node):
    '''
    The sciclopsNode inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the sciclops and a description of the sciclops to the respective topics.
    '''
    def __init__(self, NODE_NAME = "sciclopsNode"):
        '''
        The init function is neccesary for the sciclopsNode class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''
        super().__init__(NODE_NAME)
        self.sciclops = SCICLOPS()
        self.state = "UNKNOWN"
        
        self.description = {
            'name': NODE_NAME,
            'type': 'sciclops_plate_stacker',
            'actions':
            {
                'status':'',
                'home':'',
                'get_plate':'pos lid trash'
            }
        }

        timer_period = 1  # seconds
        self.statePub = self.create_publisher(String, 'sciclops_state', 10)
        self.stateTimer = self.create_timer(timer_period, self.stateCallback)

        self.actionSrv = self.create_service(WeiActions, NODE_NAME + "/action_handler", self.actionCallback)

        self.descriptionSrv = self.create_service(WeiDescription, NODE_NAME + "/description_handler", self.descriptionCallback)

    def descriptionCallback(self, request, response):
        """The descriptionCallback function is a service that can be called to showcase the available actions a robot
        can preform as well as deliver essential information required by the master node.

        Parameters:
        -----------
        request: str
            Request to the robot to deliver actions
        response: str
            The actions a robot can do, will be populated during execution

        Returns
        -------
        str
            The robot steps it can do
        """
        response.description_response = str(self.description)

        return response

    def actionCallback(self, request, response):
        '''
        The actionCallback function is a service that can be called to execute the available actions the robot
        can preform.
        '''
        
        if request.action_handle=='status':
            self.sciclops.get_status()
            response.action_response = True
        if request.action_handle=='home':            
            self.state = "BUSY"
            self.stateCallback()
            self.sciclops.home()    
            response.action_response = True
        if request.action_handle=='get_plate':
            self.state = "BUSY"
            self.stateCallback()
            vars = eval(request.vars)
            print(vars)

            pos = vars.get('pos')
            lid = vars.get('lid',False)
            trash = vars.get('trash',False)

            err = self.sciclops.get_plate(pos, lid, trash)

            response.action_response = 0
            response.action_msg= "all good sciclops"
            self.get_logger().info('Finished Action: ' + request.action_handle)
            return response
            
        self.state = "COMPLETED"

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
    NAME = "sciclopsNode"
    rclpy.init(args=args)  # initialize Ros2 communication
    node = sciclopsNode(NODE_NAME=NAME)
    rclpy.spin(node)     # keep Ros2 communication open for action node
    rclpy.shutdown()     # kill Ros2 communication

if __name__ == '__main__':
    main()

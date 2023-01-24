#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import String

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions   

from time import sleep

from sciclops_driver.sciclops_driver import SCICLOPS # import sciclops driver

class ScilopsClient(Node):
    '''
    The ScilopsClient inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the sciclops and a description of the sciclops to the respective topics.
    '''
    def __init__(self, TEMP_NODE_NAME = "ScilopsClientNode"):
        '''
        The init function is neccesary for the ScilopsClient class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''
        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()
        self.state = "UNKNOWN"

        # Setting temporary default parameter values        
        self.declare_parameter("vendor_id",0x7513)
        self.declare_parameter("product_id",0x0002)

        # Receiving the real IP and PORT from the launch parameters
        
        self.vendor_id = self.get_parameter("vendor_id").get_parameter_value().integer_value
        self.product_id = self.get_parameter("product_id").get_parameter_value().integer_value
        self.get_logger().info("Received Vendor ID: " + str(self.vendor_id) + " Product ID: " + str(self.product_id))

        self.connect_robot()
        
        self.description = {
            'name': TEMP_NODE_NAME,
            'type': 'sciclops_plate_stacker',
            'actions':
            {
                'status':'',
                'home':'',
                'get_plate':'pos lid trash'
            }
        }
        action_cb_group = ReentrantCallbackGroup()
        description_cb_group = ReentrantCallbackGroup()
        state_cb_group = ReentrantCallbackGroup()

        timer_period = 1  # seconds
        self.statePub = self.create_publisher(String, node_name + '/state', 10)
        self.stateTimer = self.create_timer(timer_period, self.stateCallback, callback_group = state_cb_group)

        self.actionSrv = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group)

        self.descriptionSrv = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group)
   
    def connect_robot(self):
        try:
            print("HERE")
            self.sciclops = SCICLOPS(VENDOR_ID = self.vendor_id, PRODUCT_ID = self.product_id)

        except Exception as error_msg:
            self.state = "SCICLOPS CONNECTION ERROR"
            self.get_logger().error("------- SCICLOPS Error message: " + str(error_msg) +  (" -------"))

        else:
            self.get_logger().info("SCICLOPS online")

    def stateCallback(self):
        '''
        Publishes the sciclops state to the 'state' topic. 
        '''
        msg = String()

        try:
            self.sciclops.get_status() 
            state = self.sciclops.status

        except Exception as err:
            self.get_logger().error("SCICLOPS IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "SCICLOPS CONNECTION ERROR"


        if self.state != "SCICLOPS CONNECTION ERROR":
            #TODO: EDIT THE DRIVER TO RECEIVE ACTUAL ROBOT STATUS
            if state == "Ready":
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif state == "RUNNING":
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif state == "ERROR":
                self.state = "ERROR"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
        else:
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Trying to connect again! Vendor ID: " + str(self.vendor_id) + " Product ID: " + str(self.product_id))
            self.connect_robot()

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
            response.action_msg= "All good sciclops"
            self.get_logger().info('Finished Action: ' + request.action_handle)
            return response
            
        self.state = "COMPLETED"

        return response


def main(args = None):

    rclpy.init(args=args)  # initialize Ros2 communication

    try:
        sciclops_client = ScilopsClient()
        executor = MultiThreadedExecutor()
        executor.add_node(sciclops_client)

        try:
            sciclops_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            sciclops_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            executor.shutdown()
            sciclops_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

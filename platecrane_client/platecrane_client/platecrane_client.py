#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import String

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions   

from time import sleep
import threading
import asyncio

from platecrane_driver.platecrane_driver import PlateCrane # import PlateCrane driver

class PlatecraneClient(Node):
    '''
    The PlatecraneClient inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback, 
    based on the executed command and publishes the state of the sciclops and a description of the sciclops to the respective topics.
    '''
    def __init__(self, TEMP_NODE_NAME = "PlatecraneClientNode"):
        '''
        The init function is neccesary for the PlatecraneClient class to initialize all variables, parameters, and other functions.
        Inside the function the parameters exist, and calls to other functions and services are made so they can be executed in main.
        '''
        super().__init__(TEMP_NODE_NAME)
        node_name = self.get_name()
        self.state = "UNKNOWN"
        self.action_flag = "READY"

        # Setting temporary default parameter values        
        self.declare_parameter("port","/dev/ttyUSB2")

        # Receiving the real vendor_id and product_id from the launch parameters
        self.port = self.get_parameter("port").get_parameter_value().string_value
        self.get_logger().info("Received Port name: " + str(self.port))

        self.connect_robot()

        self.platecrane.get_status() 
        self.robot_status = self.platecrane.status
        asyncio.run(self.platecrane.check_complete())
        self.robot_movement_state = self.platecrane.movement_state
        self.past_movement_state = "-1"
        self.state_refresher_timer = 0
        self.robot_home_iter = 0

        self.description = {
            'name': node_name,
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
        state_refresher_cb_group = ReentrantCallbackGroup()


        state_pub_timer_period = 1 # seconds
        state_refresher_timer_period = 5.3 # seconds

        self.statePub = self.create_publisher(String, node_name + '/state', 10)
        self.stateTimer = self.create_timer(state_pub_timer_period, self.stateCallback, callback_group = state_cb_group)
        
        self.StateRefresherTimer = self.create_timer(state_refresher_timer_period, callback = self.robot_state_refresher_callback, callback_group = state_refresher_cb_group)

        self.actionSrv = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group)

        self.descriptionSrv = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group)
        
        # self.lock = threading.Lock()

    def connect_robot(self):
        try:
            self.get_logger().info("Trying robot connection")
            self.platecrane = PlateCrane()

        except Exception as error_msg:
            self.state = "PLATECRANE CONNECTION ERROR"
            self.get_logger().error("------- PlateCrane Error message: " + str(error_msg) +  (" -------"))

        else:
            self.get_logger().info("PlateCrane online")

    def robot_state_refresher_callback(self):
        "Refreshes the robot states if robot cannot update the state parameters automatically because it is not running any jobs"
        try:

            if self.action_flag.upper() == "READY": #Only refresh the state manualy if robot is not running a job.
                asyncio.run(self.platecrane.check_complete())
                self.state_refresher_timer = 0 
            
            if self.past_movement_state == self.robot_movement_state:
                self.state_refresher_timer += 1
            elif self.past_movement_state != self.robot_movement_state:
                self.past_movement_state = self.robot_movement_state
                self.state_refresher_timer = 0 

            if self.state_refresher_timer > 30: # Refresh the state if robot has been stuck at a status for more than 25 refresh times.
                # self.get_logger().info("Refresh state, robot state is frozen...")
                self.action_flag = "READY"

        except Exception as err:
            self.get_logger().error(str(err))

    def stateCallback(self):
        '''
        Publishes the platecrane state to the 'state' topic. 
        '''
        msg = String()

        try:
            self.robot_status = self.platecrane.status
            self.robot_movement_state = self.platecrane.movement_state


        except Exception as err:
            self.get_logger().error("PLATECRANE IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "PLATECRANE CONNECTION ERROR"


        if self.state != "PLATECRANE CONNECTION ERROR":

            if self.robot_status == "1" and self.robot_movement_state == "READY" and self.action_flag == "READY":
                self.state = "READY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.state == "COMPLETED":
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                self.action_flag = "READY"

            elif self.robot_movement_state == "BUSY" or self.action_flag == "BUSY":
                self.state = "BUSY"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.robot_status == "0":
                self.state = "ERROR"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.get_logger().error("ROBOT IS NOT HOMED")
                
                if self.robot_home_iter == 0 :
                    self.robot_home_iter = 1
                    self.get_logger().warn("Resetting the robot")
                    self.platecrane.reset()
                    sleep(25)
                    self.get_logger().warn("Homing the robot")
                    sleep(25)
                    self.get_logger().warn("Homing completed")
                    self.robot_home_iter = 0    

            elif self.robot_status == "ERROR":
                self.state = "ERROR"
                msg.data = 'State: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.action_flag = "READY"


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
        if self.state == "SCICLOPS CONNECTION ERROR":
            message = "Connection error, cannot accept a job!"
            self.get_logger().error(message)
            response.action_response = -1
            response.action_msg = message
            return response

        while self.state != "READY":
            self.get_logger().warn("Waiting for SCICLOPS to switch READY state...")
            sleep(0.5)
            
        self.action_flag = "BUSY"

        if request.action_handle == 'status':
            try:
                self.platecrane.get_status()
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Get status failed. Error:" + err
            else:    
                response.action_response = 0
                response.action_msg= "Get status successfully completed"  

            self.state = "COMPLETED"
            return response 


        elif request.action_handle == 'stack_stransfer':            

            try:
                self.platecrane.home()  
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Homing failed. Error:" + err
            else:    
                response.action_response = 0
                response.action_msg= "Homing successfully completed"  
            
            self.state = "COMPLETED"
            return response


        elif request.action_handle=='module_trasnfer':
            # self.state = "BUSY"
            self.get_logger().info("Starting get plate")
            vars = eval(request.vars)
            print(vars)

            pos = vars.get('pos')
            lid = vars.get('lid',False)
            trash = vars.get('trash',False)

            try:
                self.platecrane.get_plate(pos, lid, trash)
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Get plate failed. Error:" + err
            else:    
                response.action_response = 0
                response.action_msg= "Get plate successfully completed"

            self.get_logger().info('Finished Action: ' + request.action_handle)
            self.state = "COMPLETED"

            return response

        else: 
            msg = "UNKOWN ACTION REQUEST! Available actions: stack_transfer, module_transfer"
            response.action_response = -1
            response.action_msg= msg
            self.get_logger().error('Error: ' + msg)
            self.state = "ERROR"
            return response



def main(args = None):

    rclpy.init(args=args)  # initialize Ros2 communication

    try:
        platecrane_client = PlatecraneClient()
        executor = MultiThreadedExecutor()
        executor.add_node(platecrane_client)

        try:
            platecrane_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            platecrane_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            platecrane_client.platecrane.disconnect_robot()
            platecrane_client.get_logger().warn("Robot connection is closed")
            executor.shutdown()
            platecrane_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

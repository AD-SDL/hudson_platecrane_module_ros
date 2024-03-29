#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import String

from wei_services.srv import WeiDescription
from wei_services.srv import WeiActions

import ast
from time import sleep
import threading
import asyncio
import json

from platecrane_driver.platecrane_driver import PlateCrane

class PlatecraneClient(Node):
    '''
    The PlatecraneClient inputs data from the 'action' topic, providing a set of commands for the driver to execute. It then receives feedback,
    based on the executed command and publishes the state of the platecrane and a description of the platecrane to the respective topics.
    '''
    def __init__(self, TEMP_NODE_NAME = "PlatecraneNode"):
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

        self.platecrane = None
        self.connect_robot()

        self.robot_status = None
        self.robot_movement_state = None
        self.robot_error_status = None
        self.past_movement_state = "-1"
        self.state_refresher_timer = 0
        self.robot_home_iter = 0

        self.description = {
            'name': node_name,
            'type': 'platecrane_plate_stacker',
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
        state_refresher_timer_period = 2 # seconds

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
                self.platecrane.get_robot_movement_state()
                self.platecrane.get_status()
                self.state_refresher_timer = 0

            if self.past_movement_state == self.robot_movement_state:
                self.state_refresher_timer += 1
            elif self.past_movement_state != self.robot_movement_state:
                self.past_movement_state = self.robot_movement_state
                self.state_refresher_timer = 0

            # if self.state_refresher_timer > 30: # Refresh the state if robot has been stuck at a status for more than 25 refresh times.
            #     # self.get_logger().info("Refresh state, robot state is frozen...")
            #     self.action_flag = "READY"

        except Exception as err:
            self.get_logger().error(str(err))

    def stateCallback(self):
        '''
        Publishes the platecrane state to the 'state' topic.
        '''
        msg = String()

        try:
            self.robot_status = self.platecrane.robot_status.strip()
            self.robot_error_status = self.platecrane.robot_error
            self.robot_movement_state = self.platecrane.movement_state

        except Exception as err:
            self.get_logger().error("PLATECRANE IS NOT RESPONDING! ERROR: " + str(err))
            self.state = "PLATECRANE CONNECTION ERROR"


        if self.state != "PLATECRANE CONNECTION ERROR":

            if self.robot_status == "0":
                self.state = "ERROR"
                msg.data = 'Statesss: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.action_flag = "READY"
                self.get_logger().warn("Robot is not homed! Homing now!")
                self.platecrane.home()

            elif self.robot_error_status == "ERROR" or (self.state == "ERROR" and self.action_flag == "BUSY"):
                self.state = "ERROR"
                msg.data = 'Statesss: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().error(msg.data)
                self.action_flag = "READY"

            elif self.state == "COMPLETED" and self.action_flag == "BUSY":
                msg.data = 'Statessss: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)
                self.action_flag = "READY"

            elif self.robot_movement_state == "BUSY" or self.action_flag == "BUSY":
                self.state = "BUSY"
                msg.data = 'Statesss: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

            elif self.robot_status == "1" and self.robot_movement_state == "READY" and self.action_flag == "READY":
                self.state = "READY"
                msg.data = 'Statesssss: %s' % self.state
                self.statePub.publish(msg)
                self.get_logger().info(msg.data)

        else:
            msg.data = 'State: %s' % self.state
            self.statePub.publish(msg)
            self.get_logger().error(msg.data)
            self.get_logger().warn("Trying to connect again! Port: " + str(self.port))
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
        if self.state == "PLATECRANE CONNECTION ERROR":
            message = "Connection error, cannot accept a job!"
            self.get_logger().error(message)
            response.action_response = -1
            response.action_msg = message
            return response

        while self.state != "READY":
            self.get_logger().warn("Waiting for PLATECRANE to switch READY state...")
            sleep(0.5)

        self.action_flag = "BUSY"
        sleep(1)

        self.get_logger().info(str(request.vars))
        vars = ast.literal_eval(request.vars) #json.loads(str(request.vars))
        self.get_logger().info(str(vars))

        source = vars.get('source')
        self.get_logger().info("Source location: " + str(source))
        target = vars.get('target')
        self.get_logger().info("Target location: "+ str(target))
        plate_type = vars.get('plate_type', "96_well")
        self.get_logger().info("Plate type: "+ str(target))

        if request.action_handle == 'transfer':
            self.get_logger().info("Starting the transfer request")

            source_type = vars.get('source_type', None)
            self.get_logger().info("Source Type: " + str(source_type))

            target_type = vars.get('target_type', None)
            self.get_logger().info("Target Type: " + str(target_type))

            if not source_type or not target_type:
                self.get_logger().error("Please provide source and target transfer types!")
                self.state = "ERROR"

            height_offset = vars.get('height_offset', 0)
            self.get_logger().info("Height Offset: " + str(height_offset))

            try:
                self.platecrane.transfer(source, target, source_type = source_type.lower(), target_type = target_type.lower(), height_offset = int(height_offset), plate_type = plate_type)
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Transfer failed. Error:" + str(err)
                self.get_logger().error(str(err))
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg= "Transfer successfully completed"
                self.state = "COMPLETED"
            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response

        elif request.action_handle == "remove_lid":

            try:
                self.platecrane.remove_lid(source = source, target = target, plate_type = plate_type)
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Remove lid failed. Error:" + str(err)
                self.get_logger().error(str(err))
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg= "Remove lid successfully completed"
                self.state = "COMPLETED"
            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response

        elif request.action_handle == "replace_lid":
            try:
                self.platecrane.replace_lid(source = source, target = target, plate_type = plate_type)
            except Exception as err:
                response.action_response = -1
                response.action_msg= "Replace lid failed. Error:" + str(err)
                self.get_logger().error(str(err))
                self.state = "ERROR"
            else:
                response.action_response = 0
                response.action_msg= "Replace lid  successfully completed"
                self.state = "COMPLETED"
            finally:
                self.get_logger().info('Finished Action: ' + request.action_handle.upper())
                return response
        else:
            msg = "UNKOWN ACTION REQUEST! Available actions:transfer, remove_lid, replace_lid"
            response.action_response = -1
            response.action_msg= msg
            self.get_logger().error('Error: ' + msg)
            self.state = "ERROR"
            return response

def main(args = None):

    rclpy.init(args=args)  # initialize Ros2 communication

    try:
        platecrane_ros_client = PlatecraneClient()
        executor = MultiThreadedExecutor()
        executor.add_node(platecrane_ros_client)

        try:
            platecrane_ros_client.get_logger().info('Beginning client, shut down with CTRL-C')
            executor.spin()
        except KeyboardInterrupt:
            platecrane_ros_client.get_logger().info('Keyboard interrupt, shutting down.\n')
        finally:
            # platecrane_ros_client.platecrane.__serial_port.__disconnect_robot()
            platecrane_ros_client.get_logger().warn("Robot connection is closed")
            executor.shutdown()
            platecrane_ros_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

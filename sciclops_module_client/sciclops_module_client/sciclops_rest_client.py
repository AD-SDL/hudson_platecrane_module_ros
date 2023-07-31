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

from platecrane_driver.sciclops_driver import SCICLOPS # import sciclops driver


#! /usr/bin/env python3

import rclpy                 # import Rospy
from rclpy.node import Node  # import Rospy Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor

from std_msgs.msg import String
from std_srvs.srv import Empty

from time import sleep
import json

from threading import Thread

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions 

from pf400_driver.errors import ConnectionException, CommandException
from pf400_driver.pf400_driver import PF400
# from pf400_driver.errors import ConnectionException, CommandException
from pf400_driver.pf400_camera_driver import PF400_CAMERA


"""The server that takes incoming WEI flow requests from the experiment application"""
import json
from argparse import ArgumentParser
from contextlib import asynccontextmanager
import time
from fastapi import FastAPI, File, Form, UploadFile
from fastapi.responses import JSONResponse

workcell = None
global sealer, state
serial_port = '/dev/ttyUSB0'
local_ip = 'parker.alcf.anl.gov'
local_port = '8000'

global sciclops
       
@asynccontextmanager
async def lifespan(app: FastAPI):
    global sciclops, state
    """Initial run function for the app, parses the worcell argument
        Parameters
        ----------
        app : FastApi
           The REST API app being initialized

        Returns
        -------
        None"""
    
    try:
            pass #self.get_logger().info("Trying robot connection")
            sciclops = SCICLOPS()

        except Exception as error_msg:
            state = "SCICLOPS CONNECTION ERROR"
            pass #self.get_logger().error("------- SCICLOPS Error message: " + str(error_msg) +  (" -------"))

        else:
            pass #self.get_logger().info("SCICLOPS online")


app = FastAPI(lifespan=lifespan, )

@app.get("/state")
async def state():
    global sealer
    return JSONResponse(content={"State": sealer.get_status() })

@app.get("/description")
async def description():
    global state
    return JSONResponse(content={"State": state})

@app.get("/resources")
async def resources():
    global sealer
    return JSONResponse(content={"State": sealer.get_status() })


@app.post("/action")
async def do_action(
    action_handle: str,
    action_vars: dict, 
):  
    global state, sciclops
    response = {"action_response": "", "action_msg": "", "action_log": ""}
    if state == "SCICLOPS CONNECTION ERROR":
            message = "Connection error, cannot accept a job!"
            pass #self.get_logger().error(message)
            response["action_response"] = -1
            response["action_msg"] = message
            return response

   # while state != "READY":
            pass #self.get_logger().warn("Waiting for SCICLOPS to switch READY state...")
            sleep(0.5)
            
    sate  = "BUSY"

    if action_handle == 'status':
                try:
                    sciclops.get_status()
                except Exception as err:
                    response["action_response"] = -1
                    response["action_msg"]= "Get status failed. Error:" + err
                else:    
                    response["action_response"] = 0
                    response["action_msg"]= "Get status successfully completed"  

                state = "COMPLETED"
                return response 


    elif action_handle == 'home':            

                try:
                    sciclops.home()  
                except Exception as err:
                    response["action_response"] = -1
                    response["action_msg"]= "Homing failed. Error:" + err
                else:    
                    response["action_response"] = 0
                    response["action_msg"]= "Homing successfully completed"  
                
                state = "COMPLETED"
                return response


    elif action_handle=='get_plate':
                # state = "BUSY"
                pass #self.get_logger().info("Starting get plate")
                vars = json.loads(action_vars)
                print(vars)

                pos = vars.get('pos')
                lid = vars.get('lid',False)
                trash = vars.get('trash',False)

                try:
                    sciclops.get_plate(pos, lid, trash)
                except Exception as err:
                    response["action_response"] = -1
                    response["action_msg"]= "Get plate failed. Error:" + err
                else:    
                    response["action_response"] = 0
                    response["action_msg"]= "Get plate successfully completed"

                pass #self.get_logger().info('Finished Action: ' + action_handle)
                state = "COMPLETED"

                return response

    else: 
                msg = "UNKOWN ACTION REQUEST! Available actions: status, home, get_plate"
                response["action_response"] = -1
                response["action_msg"]= msg
                pass #self.get_logger().error('Error: ' + msg)
                state = "ERROR"
                return response


if __name__ == "__main__":
    import uvicorn
    print("asdfsaf")
    uvicorn.run("a4s_sealer_REST:app", host=local_ip, port=local_port, reload=True, ws_max_size=100000000000000000000000000000000000000)



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
        state = "UNKNOWN"
        self.action_flag = "READY"

        # Setting temporary default parameter values        
        self.declare_parameter("vendor_id",0x7513)
        self.declare_parameter("product_id",0x0002)

        # Receiving the real vendor_id and product_id from the launch parameters
        self.vendor_id = self.get_parameter("vendor_id").get_parameter_value().integer_value
        self.product_id = self.get_parameter("product_id").get_parameter_value().integer_value
        pass #self.get_logger().info("Received Vendor ID: " + str(self.vendor_id) + " Product ID: " + str(self.product_id))

        self.connect_robot()

        sciclops.get_status() 
        self.robot_status = sciclops.status
        asyncio.run(sciclops.check_complete())
        self.robot_movement_state = sciclops.movement_state
        self.past_movement_state = "-1"
        state_refresher_timer = 0
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

        statePub = self.create_publisher(String, node_name + '/state', 10)
        stateTimer = self.create_timer(state_pub_timer_period, stateCallback, callback_group = state_cb_group)
        
        stateRefresherTimer = self.create_timer(state_refresher_timer_period, callback = self.robot_state_refresher_callback, callback_group = state_refresher_cb_group)

        self.actionSrv = self.create_service(WeiActions, node_name + "/action_handler", self.actionCallback, callback_group = action_cb_group)

        self.descriptionSrv = self.create_service(WeiDescription, node_name + "/description_handler", self.descriptionCallback, callback_group = description_cb_group)
        
        # self.lock = threading.Lock()

    def connect_robot(self):
       

    def robot_state_refresher_callback(self):
        "Refreshes the robot states if robot cannot update the state parameters automatically because it is not running any jobs"
        try:

            if self.action_flag.upper() == "READY": #Only refresh the state manualy if robot is not running a job.
                asyncio.run(sciclops.check_complete())
                state_refresher_timer = 0 
            
            if self.past_movement_state == self.robot_movement_state:
                state_refresher_timer += 1
            elif self.past_movement_state != self.robot_movement_state:
                self.past_movement_state = self.robot_movement_state
                state_refresher_timer = 0 

            if state_refresher_timer > 30: # Refresh the state if robot has been stuck at a status for more than 25 refresh times.
                # pass #self.get_logger().info("Refresh state, robot state is frozen...")
                self.action_flag = "READY"

        except Exception as err:
            pass #self.get_logger().error(str(err))

    def stateCallback(self):
        '''
        Publishes the sciclops state to the 'state' topic. 
        '''
        msg = String()

        try:
            self.robot_status = sciclops.status
            self.robot_movement_state = sciclops.movement_state


        except Exception as err:
            pass #self.get_logger().error("SCICLOPS IS NOT RESPONDING! ERROR: " + str(err))
            state = "SCICLOPS CONNECTION ERROR"


        if state != "SCICLOPS CONNECTION ERROR":

            if self.robot_status == "1" and self.robot_movement_state == "READY" and self.action_flag == "READY":
                state = "READY"
                msg.data = 'State: %s' % state
                statePub.publish(msg)
                pass #self.get_logger().info(msg.data)

            elif state == "COMPLETED":
                msg.data = 'State: %s' % state
                statePub.publish(msg)
                pass #self.get_logger().info(msg.data)
                self.action_flag = "READY"

            elif self.robot_movement_state == "BUSY" or self.action_flag == "BUSY":
                state = "BUSY"
                msg.data = 'State: %s' % state
                statePub.publish(msg)
                pass #self.get_logger().info(msg.data)

            elif self.robot_status == "0":
                state = "ERROR"
                msg.data = 'State: %s' % state
                statePub.publish(msg)
                pass #self.get_logger().error(msg.data)
                pass #self.get_logger().error("ROBOT IS NOT HOMED")
                
                if self.robot_home_iter == 0 :
                    self.robot_home_iter = 1
                    pass #self.get_logger().warn("Resetting the robot")
                    sciclops.reset()
                    sleep(25)
                    pass #self.get_logger().warn("Homing the robot")
                    sleep(25)
                    pass #self.get_logger().warn("Homing completed")
                    self.robot_home_iter = 0    

            elif self.robot_status == "ERROR":
                state = "ERROR"
                msg.data = 'State: %s' % state
                statePub.publish(msg)
                pass #self.get_logger().error(msg.data)
                self.action_flag = "READY"


        else:
            msg.data = 'State: %s' % state
            statePub.publish(msg)
            pass #self.get_logger().error(msg.data)
            pass #self.get_logger().warn("Trying to connect again! Vendor ID: " + str(self.vendor_id) + " Product ID: " + str(self.product_id))
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
        if state == "SCICLOPS CONNECTION ERROR":
            message = "Connection error, cannot accept a job!"
            pass #self.get_logger().error(message)
            response["action_response"] = -1
            response["action_msg"] = message
            return response

        while state != "READY":
            pass #self.get_logger().warn("Waiting for SCICLOPS to switch READY state...")
            sleep(0.5)
            
        self.action_flag = "BUSY"

        if action_handle == 'status':
            try:
                sciclops.get_status()
            except Exception as err:
                response["action_response"] = -1
                response["action_msg"]= "Get status failed. Error:" + err
            else:    
                response["action_response"] = 0
                response["action_msg"]= "Get status successfully completed"  

            state = "COMPLETED"
            return response 


        elif action_handle == 'home':            

            try:
                sciclops.home()  
            except Exception as err:
                response["action_response"] = -1
                response["action_msg"]= "Homing failed. Error:" + err
            else:    
                response["action_response"] = 0
                response["action_msg"]= "Homing successfully completed"  
            
            state = "COMPLETED"
            return response


        elif action_handle=='get_plate':
            # state = "BUSY"
            pass #self.get_logger().info("Starting get plate")
            vars = json.loads(action_vars)
            print(vars)

            pos = vars.get('pos')
            lid = vars.get('lid',False)
            trash = vars.get('trash',False)

            try:
                sciclops.get_plate(pos, lid, trash)
            except Exception as err:
                response["action_response"] = -1
                response["action_msg"]= "Get plate failed. Error:" + err
            else:    
                response["action_response"] = 0
                response["action_msg"]= "Get plate successfully completed"

            pass #self.get_logger().info('Finished Action: ' + action_handle)
            state = "COMPLETED"

            return response

        else: 
            msg = "UNKOWN ACTION REQUEST! Available actions: status, home, get_plate"
            response["action_response"] = -1
            response["action_msg"]= msg
            pass #self.get_logger().error('Error: ' + msg)
            state = "ERROR"
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
            sciclops_client.sciclops.disconnect_robot()
            sciclops_client.get_logger().warn("Robot connection is closed")
            executor.shutdown()
            sciclops_client.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()

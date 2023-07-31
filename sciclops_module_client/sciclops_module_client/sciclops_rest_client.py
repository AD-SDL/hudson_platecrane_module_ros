#! /usr/bin/env python3


from std_msgs.msg import String

from wei_services.srv import WeiDescription 
from wei_services.srv import WeiActions   

from time import sleep
import threading
import asyncio

from platecrane_driver.sciclops_driver import SCICLOPS # import sciclops driver

from time import sleep
import json

from threading import Thread

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


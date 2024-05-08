import logging
import json

from fastapi import FastAPI
from fastapi.responses import HTMLResponse

import asyncio # allows your program to run async functions/codes
import socket 

from fastapi import Request
from fastapi import WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates


import os # allows programmer to perform OS dependent operations i.e. reading & writing to files

import _thread
from concurrent import futures


import rospy
import rospy as logger
from ros_mi import MeterialInspection

# -------------------------------------------------------------------------------------------------
# init

logger = logging.getLogger()
logger.setLevel(logging.INFO)
ch = logging.StreamHandler() # creates a stream handler object => sends log msgs to a stream (usually for python its the console)
logger.addHandler(ch) # adds the stream handler to the logger object

logger.info(os.getcwd()) # os.getcwd() gets current working directory from which the python script is being executed

app = FastAPI()
app.data = None
app.lang = 0 # represent language settings => 0 might be default/ arbitrary choice
app.mode = None # may represent a step or stage in a process within the app

app.mount("/static", StaticFiles(directory="static"), name="static") # this line configures the API to serve static files from the "static" directory
templates = Jinja2Templates(directory="templates") # templates directory is used to store the html

h_name = socket.gethostname()
IP_addres = socket.gethostbyname(h_name)
logger.info("h_name: {}, IP_address {}".format(h_name, IP_addres))

rospy.init_node("fastapi_ros") # initialize the ros node
app.mi = MeterialInspection()

responseData = {
    "data": "{random input 1: input 1, random input 2: input 2, random input 3: input 3}",
    "state": "IDLE",
    "info": {
        "isExist": True,
        "message": "no error",
        "isError": False,
    }
}
# -------------------------------------------------------------------------------------------------
# functions

def formatKeyAndVal(k, v):
    data = {}
    data[k] = v # {"command": wtv cmd data}
    return json.dumps(data) # takes in python objects & converts to json string

def ros_serve():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        # if len(mi.send_queue) > 0:

def get_lang_data():
    try:
        with open("static/lang.txt", "r") as f:
            langdata = f.read()
    except Exception as e:
        logger.info(e)
        return 0
    return langdata

# -------------------------------------------------------------------------------------------------
# lifespan events

@app.on_event("startup") # runs before the app start; also a "shutdown event"
async def startup_event():
    try:
        _thread.start_new_thread(ros_serve, ()) # run the ros rate method
    except Exception as e:
        logger.info("Error: cannot init thread. {}".format(e))

# -------------------------------------------------------------------------------------------------
# middleware
@app.get("/", response_class=HTMLResponse)
async def get_html(request: Request): # request is the 1st arg
    app.mode = None
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/step/{mode}", response_class=HTMLResponse) # indicates its a html response
async def step(request: Request, mode: str): # mode is of string type; its from clicking the page btn
    logger.info(f"get step {mode}")

    app.mode = mode
    
    if app.mode is not None:

        logger.info("static/"+app.mode+".txt")
        with open("static/"+str(app.lang)+"_"+app.mode+".txt") as f:
            just_do = f.readlines() # read the lines of the txt file
            logger.info(just_do)

        return templates.TemplateResponse(app.mode+".html", { "request": request, "just_do": just_do, "responseData": responseData })
    else:
        return None

@app.get("/command/{cmd}", response_class=HTMLResponse)
async def command(request: Request, cmd: str):
    try:
        logger.info(f"get command {cmd}")
        srv_call_formatted_data = cmd # for now put imu only; returns json string
        # msg_dict[sub_node].service_call(cmd) => json string is passed to the node in the service call; might be the old way of making a service call
        app.mi.send_srv(srv_call_formatted_data)
    except Exception as e:
        logger.error(e)

# -------------------------------------------------------------------------------------------------
# WEBSOCKETS (for frontend to connect to)

async def listen_to_websocket(websocket, topic, component):
    logger.info(f"get websocket data from /{component}_{topic} topic.")
    #TODO
    await websocket.accept()
    while True:
        await asyncio.sleep(0.2)
        if app.mi.has_topic_msg(topic, component):
            qData = app.mi.get_topic_msg(topic, component)
            logger.info(f"queue data: {qData}")
            await websocket.send_text(f"{qData}")
            app.mi.pop_topic_msg(topic, component)

# IMU
# /imu_configs socket
@app.websocket("/imu/configs")
async def cb_imu_data(websocket: WebSocket):
    await listen_to_websocket(websocket, "configs", "imu")

# /imu_data socket
@app.websocket("/imu/data")
async def cb_imu_data(websocket: WebSocket):
    await listen_to_websocket(websocket, "data", "imu")

# /imu_info socket
@app.websocket("/imu/info")
async def cb_imu_state(websocket: WebSocket):
    await listen_to_websocket(websocket, "info", "imu")

# /imu_state socket
@app.websocket("/imu/state")
async def cb_imu_state(websocket: WebSocket):
    await listen_to_websocket(websocket, "state", "imu")


# SONAR
# /sonar_configs socket
@app.websocket("/sonar/configs")
async def cb_sonar_data(websocket: WebSocket):
    await listen_to_websocket(websocket, "configs", "sonar")

# /sonar_data socket
@app.websocket("/sonar/data")
async def cb_sonar_data(websocket: WebSocket):
    await listen_to_websocket(websocket, "data", "sonar")

# /sonar_info socket
@app.websocket("/sonar/info")
async def cb_sonar_state(websocket: WebSocket):
    await listen_to_websocket(websocket, "info", "sonar")

# /sonar_state socket
@app.websocket("/sonar/state")
async def cb_sonar_state(websocket: WebSocket):
    await listen_to_websocket(websocket, "state", "sonar")


# TODO add inclinometer, cb, sonar, depth sensor
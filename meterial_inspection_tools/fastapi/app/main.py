import logging
import json

from fastapi import FastAPI
from fastapi.responses import HTMLResponse

import asyncio # allows your program to run async functions/codes
import socket # 

from fastapi import Request
from fastapi import WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates


import os # allows programmer to perform OS dependent operations i.e. reading & writing to files

import grpc
import _thread
import time
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
    logger.info("get step {}".format(mode))

    app.mode = mode

    ret = distribute_data(request)
    if ret is not None:
        return ret
    
    # different from here
    if app.mode is not None:
        logger.info("static/"+app.mode+".txt")
        with open("static/"+str(app.lang)+"_"+app.mode+".txt") as f:
            just_do = f.readlines() # read the lines of the txt file
            logger.info(just_do)

        return templates.TemplateResponse("index.html", {"request": request, "just_do": just_do}) # directory & context are the arguments
        # responseData is returned as first readings, but websocket data should come once sockets are opened
    else:
        return None

@app.get("/command/{cmd}", response_class=HTMLResponse)
async def command(request: Request, cmd: str):
    try:
        logger.warn("get command {}".format(cmd))
        srv_call_formatted_data = cmd # for now put imu only; returns json string
        # msg_dict[sub_node].service_call(cmd) => json string is passed to the node in the service call; might be the old way of making a service call
        # TODO service call using app.mi
        app.mi.send_srv(srv_call_formatted_data)
    except Exception as e:
        print(e)

# -------------------------------------------------------------------------------------------------
# websockets
# /imu_data socket
@app.websocket("/imu_data") # web socket decorators to define websocket end points
async def cb_imu_data(websocket: WebSocket): # web socket event handler
    logger.info("get websocket data from /imu_data topic.")
    #TODO
    await websocket.accept() # accept the web socket connection
    # global data
    while True: # enter loop to continuously receive data
        await asyncio.sleep(0.2) # this is required as it pauses current coroutine to allow execution of other coroutinese
        if app.mi.has_topic_data_msg():
            qData = app.mi.topic_data_send_queue[0]
            logger.info("queue data: {}".format(qData))
            await websocket.send_text(f"{qData}") # always take the 1st message in the queue
            app.mi.pop_topic_data_msg() # then pop it off once you are done returning it

# /imu_state socket
@app.websocket("/imu_state") # web socket decorators to define websocket end points
async def cb_imu_state(websocket: WebSocket): # web socket event handler
    logger.info("get websocket data from /imu_state topic.")
    #TODO
    await websocket.accept() # accept the web socket connection
    # global data
    while True: # enter loop to continuously receive data
        await asyncio.sleep(0.2) # this is required as it pauses current coroutine to allow execution of other coroutinese
        if app.mi.has_topic_state_msg():
            qData = app.mi.topic_state_send_queue[0]
            logger.info("queue data: {}".format(qData))
            await websocket.send_text(f"{qData}") # always take the 1st message in the queue
            app.mi.pop_topic_state_msg() # then pop it off once you are done returning it

# /imu_info socket
@app.websocket("/imu_info") # web socket decorators to define websocket end points
async def cb_imu_state(websocket: WebSocket): # web socket event handler
    logger.info("get websocket data from /imu_info topic.")
    #TODO
    await websocket.accept() # accept the web socket connection
    # global data
    while True: # enter loop to continuously receive data
        await asyncio.sleep(0.2) # this is required as it pauses current coroutine to allow execution of other coroutinese
        if app.mi.has_topic_info_msg():
            qData = app.mi.topic_info_send_queue[0]
            logger.info("queue data: {}".format(qData))
            await websocket.send_text(f"{qData}") # always take the 1st message in the queue
            app.mi.pop_topic_info_msg() # then pop it off once you are done returning it


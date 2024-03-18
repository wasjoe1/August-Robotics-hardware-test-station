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

logger = logging.getLogger() # retrieves the root logger
logger.setLevel(logging.INFO) #sets logging level to INFO, only msgs with severity level of INFO or higher will be handled by this logger
# without additional config, destination of log msgs is not yet specified 
ch = logging.StreamHandler() # creates a stream handler object => sends log msgs to a stream (usually for python its the console)
logger.addHandler(ch) # adds the stream handler to the logger object


logger.info(os.getcwd()) # os.getcwd() gets current working directory from which the python script is being executed
# os.getcwd() could also be the directory set by os.chdir()
# directory string is logged to the logging system

app = FastAPI()
app.long_img = None # represents a long image => no long image yet
app.long_camera_time = None # represents timestamp associated with the long img => no specific timestamp yet
app.short_img = None # represents a short image => no short image yet
app.short_camera_time = None # represents timestamp associated with the short img => no specific timestamp yet
app.data = None # represents additional data associated to the images
app.lang = 0 # represent language settings => 0 might be default/ arbitrary choice
app.step = None # may represent a step or stage in a process within the app
# not common props, chat says might be related to requirements for the app having to handle images??

app.mount("/static", StaticFiles(directory="static"), name="static") # this line configures the API to serve static files from the "static" directory

templates = Jinja2Templates(directory="templates") # templates directory is used to store the html 

h_name = socket.gethostname()
IP_addres = socket.gethostbyname(h_name)
logger.info("h_name: {}, IP_address {}".format(h_name, IP_addres))

rospy.init_node("fastapi_ros") # initialize the ros node
app.mi = MeterialInspection()

data = {
    "readings": "{random input 1: input 1, random input 2: input 2, random input 3: input 3}",
    "status": "IDLE",
    "popup_messages": {
        "isExist": True,
        "message": "no error",
        "isError": False,
    }
}
# -------------------------------------------------------------------------------------------------
# functions

def python_to_json_string(k, v):
    data = {}
    data[k] = v
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
    app.step = None
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/step/{step}", response_class=HTMLResponse) # indicates its a html response
async def step(request: Request, step: str): # step is of string type; its from clicking the page btn
    logger.info("get step {}".format(step))

    app.step = step

    if app.step is not None:
        logger.info("static/"+app.step+".txt")
        with open("static/"+app.step+".txt") as f:
            just_do = f.readlines() # read the lines of the txt file

        logger.info(just_do)

        return templates.TemplateResponse("sub_page.html", {"request": request, "just_do": just_do, "data": data}) # directory & context are the arguments
    else:
        return None

@app.get("/command/{cmd}", response_class=HTMLResponse)
async def command(request: Request, cmd: str):
    try:
        logger.warn("get command {}".format(cmd))
        grpc_data = python_to_json_string("command", cmd) # json string
        # msg_dict[sub_node].service_call(cmd) => json string is passed to the node in the service call
        # TODO
        app.mi.send_srv(cmd)
    except Exception as e:
        print(e)

# -------------------------------------------------------------------------------------------------
# websockets

@app.websocket("/data")
async def data(websocket: WebSocket):
    logger.info("get data websocket.")
    #TODO
    await websocket.accept()
    # global data
    while True:
        await asyncio.sleep(0.2)
        if app.mi.has_msg:
            await websocket.send_text(f"{app.mi.send_queue[0]}")
            app.mi.pop_msg()

@app.websocket("/img_ws")
async def img_ws(websocket: WebSocket):
    logger.info("started.......")
    await websocket.accept()
    try:
        while True:
            encode_string = app.long_img
            await websocket.send_bytes(encode_string)
            await asyncio.sleep(0.02)

    except Exception as e:
        logger.info(e)
    finally:
        websocket.close()


# notes -------------------------------------------------------------------------------------------
# subscribing to topics provided by the ROS
# receive 
# ros nodes will make service calls upon action()
# no need for external json file, directly send it through service calls
        
# JSON.render(response.data) # take in json string & returns python objects
# data.status
# data.title # => "hello"

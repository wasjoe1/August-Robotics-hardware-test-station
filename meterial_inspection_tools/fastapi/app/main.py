import logging
import json

from fastapi import FastAPI
from fastapi.responses import HTMLResponse

import asyncio
import socket

from fastapi import Request
from fastapi import WebSocket
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates


import os

import grpc
import _thread
import time
from concurrent import futures


import rospy
import rospy as logger
from ros_mi import MeterialInspection


logger = logging.getLogger()
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
logger.addHandler(ch)


logger.info(os.getcwd())

app = FastAPI()
app.long_img = None
app.long_camera_time = None
app.short_img = None
app.short_camera_time = None
app.data = None
app.lang = 0
app.step = None

app.mount("/static", StaticFiles(directory="static"), name="static")

templates = Jinja2Templates(directory="templates")


h_name = socket.gethostname()
IP_addres = socket.gethostbyname(h_name)
logger.info("h_name: {}, IP_address {}".format(h_name, IP_addres))

rospy.init_node("fastapi_ros")
app.mi = MeterialInspection()


def json_to_serial(k, v):
    data = {}
    data[k] = v
    return json.dumps(data)


def ros_serve():
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
        # if len(mi.send_queue) > 0:


@app.on_event("startup")
async def startup_event():
    try:
        _thread.start_new_thread(ros_serve, ())
    except Exception as e:
        logger.info("Error: cannot init thread. {}".format(e))


@app.get("/", response_class=HTMLResponse)
async def get_html(request: Request):
    app.step = None
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/step/{step}", response_class=HTMLResponse)
async def step(request: Request, step: str):
    logger.info("get step {}".format(step))

    app.step = step

    if app.step is not None:
        logger.info("static/"+app.step+".txt")
        with open("static/"+app.step+".txt") as f:
            just_do = f.readlines()

        logger.info(just_do)
        #TODO

        return templates.TemplateResponse("sub_page.html", {"request": request, "just_do": just_do})
    else:
        return None


def get_lang_data():
    try:
        with open("static/lang.txt", "r") as f:
            langdata = f.read()
    except Exception as e:
        logger.info(e)
        return 0
    return langdata



@app.get("/command/{cmd}", response_class=HTMLResponse)
async def command(request: Request, cmd: str):
    try:
        logger.warn("get command {}".format(cmd))
        grpc_data = json_to_serial("command", cmd)
        # msg_dict[sub_node].service_call(cmd)
        # TODO
        app.mi.send_srv(cmd)
    except Exception as e:
        print(e)


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

# load message


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

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
# import cv2
import _thread
import time
from concurrent import futures

# import sys

from proto_msg import (
    data_pb2_grpc,
    data_pb2
)

# setup loggers
logger = logging.getLogger()
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
# fh = logging.FileHandler(filename='./server.log')
# ch.setFormatter(logger.LogFormatter())
# fh.setFormatter(LogFormatter())
logger.addHandler(ch)
# logger.addHandler(fh)


logger.info(os.getcwd())

app = FastAPI()
app.long_img = None
app.long_camera_time = None
app.short_img = None
app.short_camera_time = None
app.data = None
app.lang = 0
app.mode = None

app.mount("/static", StaticFiles(directory="static"), name="static")

templates = Jinja2Templates(directory="templates")


h_name = socket.gethostname()
IP_addres = socket.gethostbyname(h_name)
logger.info("h_name: {}, IP_address {}".format(h_name, IP_addres))


class DataService(data_pb2_grpc.data_ServiceServicer):
    def GetMsg(self, request, context):
        # logger.info("Received step: %s" % request.step)
        # logger.info("Got new msg")
        data_from = json.loads(request.request_data)
        data_from = json.loads(data_from)
        # logger.warn("type request {}".format(type(request.request_data)))
        # logger.warn("data_from {}".format(type(data_from)))
        for k, v in data_from.items():
            logger.info("got new msg {}".format(k))
            if k == "long":
                logger.info("Got long img")
                app.long_camera_time = v["time"]
                app.long_img = json.dumps(v)
            elif k == "short":
                logger.info("Got short img")
                app.short_camera_time = v["time"]
                app.short_img = json.dumps(v)
            elif k == "data":
                # logger.info(v)
                # data = v
                # app.data = json.loads(v)
                app.data = v
        return data_pb2.dataResponse()


def json_to_serial(k, v):
    data = {}
    data[k] = v
    return json.dumps(data)


def serve():
    _ONE_DAY_IN_SECONDS = 60 * 60 * 24
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    data_pb2_grpc.add_data_ServiceServicer_to_server(DataService(), server)
    server.add_insecure_port('0.0.0.0:50051')
    server.start()
    try:
        while True:
            time.sleep(_ONE_DAY_IN_SECONDS)
    except KeyboardInterrupt:
        logger.info("stop!!!")
        server.stop(0)


@app.on_event("startup")
async def startup_event():
    try:
        _thread.start_new_thread(serve, ())
    except Exception as e:
        logger.info("Error: cannot init thread. {}".format(e))


@app.get("/", response_class=HTMLResponse)
async def get_html(request: Request):
    # html_file = open("app/www/index.html", 'r').read()
    # return html_file
    app.mode = None
    return templates.TemplateResponse("index.html", {"request": request})


@app.get("/step/{mode}", response_class=HTMLResponse)
async def step(request: Request, mode: str):
    logger.info("get step {}".format(mode))

    app.mode = mode

    ret = distribute_data(request)
    if ret is not None:
        return ret
    # with open("static/lang.txt", "r") as f:
    #     langdata = f.read()

    # logger.info("lang.app : {}".format(langdata))
    # app.lang = int(langdata)
    # logger.info("lang.app : {}".format(app.lang))
    # with open("static/"+str(app.lang)+"_"+app.mode+".txt") as f:
    #     just_do = f.readlines()

    # grpc_data = json_to_serial("step", app.mode)
    # logger.info("grpc_data : {}".format(grpc_data))

    # with grpc.insecure_channel('host.docker.internal:50052') as channel:
    #     stub = data_pb2_grpc.data_ServiceStub(channel)
    #     response = stub.GetMsg(data_pb2.dataRequest(request_data=grpc_data))
    # app.long_img = ""
    # app.short_img = ""
    # logger.info("Client received: " + response.response_data)
    # logger.info("grpc_data : {}".format(grpc_data))
    # return templates.TemplateResponse("index.html", {"request": request, "just_do": just_do})


def get_lang_data():
    try:
        with open("static/lang.txt", "r") as f:
            langdata = f.read()
    except Exception as e:
        logger.info(e)
        return 0
    return langdata


def distribute_data(request):
    if app.mode is not None:
        # with open("static/lang.txt", "r") as f:
        langdata = get_lang_data()

        logger.info("lang.app : {}".format(langdata))
        app.lang = int(langdata)
        logger.info("lang.app : {}".format(app.lang))
        logger.info("static/"+str(app.lang)+"_"+app.mode+".txt")
        with open("static/"+str(app.lang)+"_"+app.mode+".txt") as f:
            just_do = f.readlines()

        logger.info(just_do)

        grpc_data = json_to_serial("step", app.mode)

        with grpc.insecure_channel('host.docker.internal:50052') as channel:
        # with grpc.insecure_channel('0.0.0.0:50052') as channel:
            stub = data_pb2_grpc.data_ServiceStub(channel)
            response = stub.GetMsg(data_pb2.dataRequest(request_data=grpc_data))
        app.long_img = ""
        app.short_img = ""
        logger.info("Client received: " + response.response_data)
        return templates.TemplateResponse("index.html", {"request": request, "just_do": just_do})
    else:
        return None


@app.get("/command/{cmd}", response_class=HTMLResponse)
async def step(request: Request, cmd: str):
    logger.info("get command {}".format(cmd))
    grpc_data = json_to_serial("command", cmd)
    # with grpc.insecure_channel('0.0.0.0:50052') as channel:
    with grpc.insecure_channel('host.docker.internal:50052') as channel:
        stub = data_pb2_grpc.data_ServiceStub(channel)
        response = stub.GetMsg(data_pb2.dataRequest(request_data=grpc_data))
    logger.info("Client received: " + response.response_data)


@app.get("/switch_lang/{lang}", response_class=HTMLResponse)
async def step(request: Request, lang: str):
    logger.info("get switch lang {}".format(lang))
    app.lang = int(lang)
    with open("static/lang.txt", "w+") as f:
        f.write(lang)

    with open("static/"+str(app.lang)+"_"+app.mode+".txt") as f:
        just_do = f.readlines()
        # just_do = f.readlines()
    return json.dumps(just_do, ensure_ascii=False)


@app.get("/get_lang", response_class=HTMLResponse)
async def step(request: Request):
    logger.info("get lang")
    # with open("static/lang.txt", "r") as f:
    langdata = get_lang_data()
    logger.info("lang.app : {}".format(langdata))
    return json.dumps(langdata, ensure_ascii=False)


@app.websocket("/data")
async def data(websocket: WebSocket):
    logger.info("get data websocket.")
    await websocket.accept()
    # global data
    while True:
        await asyncio.sleep(0.2)
        await websocket.send_text(f"{app.data}")

# load message


@app.websocket("/long_img_ws")
async def long_img_ws(websocket: WebSocket):
    logger.info("started.......")
    await websocket.accept()
    try:
        while True:
            # data = await websocket.receive_text()
            # logger.info(f"received: {int(data)}")
            # index = int(data)
            # image = get_image(volume, index)
            # logger.info("send imgs")
            # with open('static/'+str(img_id)+'.png', mode="rb") as image_file:
            #     encode_string = base64.b64encode(
            #         image_file.read()).decode("utf-8")
            # global long_img
            encode_string = app.long_img
            # encode_string = long_img.decode("utf-8")
            # encode_string = base64.b64encode(long_img).decode("utf-8")
            # logger.info("send long img")
            await websocket.send_bytes(encode_string)
            await asyncio.sleep(0.02)
            # img_id += 1
            # if img_id == 3:
            #     img_id = 0
    except Exception as e:
        logger.info(e)
    finally:
        websocket.close()

# load message


@app.websocket("/short_img_ws")
async def short_img_ws(websocket: WebSocket):
    logger.info("started.......")
    await websocket.accept()
    try:
        while True:
            encode_string = app.short_img
            # logger.info("send short img")
            await websocket.send_bytes(encode_string)
            await asyncio.sleep(0.02)

    except Exception as e:
        logger.info(e)
    finally:
        websocket.close()

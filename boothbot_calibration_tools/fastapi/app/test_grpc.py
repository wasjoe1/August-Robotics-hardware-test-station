

import grpc
import json
import socket
from proto_msg import data_pb2
from proto_msg import data_pb2_grpc

grpc_data = {}
grpc_data["command"] =  "servos"
grpc_data = json.dumps(grpc_data)

h_name = socket.gethostname()
IP_addres = socket.gethostbyname(h_name)

with grpc.insecure_channel(IP_addres + ':50052') as channel:
    stub = data_pb2_grpc.data_ServiceStub(channel)
    response = stub.GetMsg(data_pb2.dataRequest(request_data=grpc_data))
print("Client received: " + response.response_data)
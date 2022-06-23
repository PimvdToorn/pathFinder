import asyncio
#import websockets
import websockets
import websock as ws
from importlib.resources import path
import socket
import os
import threading
from _thread import *
import time
import json
ServerSocket = socket.socket()
host = '127.0.0.1'
port = 1024
ThreadCount = 0
robot_names = ["robot_1","robot_2","robot_3","robot_4"]
robot_current_positions = [[],[],[],[]]
dashboard_call_position = []
dashboard_messages = []

#asyncio.run(websocketDash.send("sdas"))
def pathvinderzooi():
    return [10,10]
try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))

print('Waitiing for a Connection..')
ServerSocket.listen(5)

async def hello(websocket, path):
    async for data in websocket:
        print(f"Received: '{data}'")
        await websocket.send(data)
def between_callback():
    while True:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        ws_server = websockets.serve(hello, 'localhost', 8000)

        loop.run_until_complete(ws_server)
        loop.run_forever() # this is missing
        loop.close()
async def send_receive_message(uri):
    async with websockets.connect(uri) as websocket:
        await websocket.send('This is some text.')
        reply = await websocket.recv()
        print(f"The reply is: '{reply}'")
        
def threaded_client(connection):
    connection.send(str.encode('Welcome to the Servern'))
    while True:
        try:
            if(len(dashboard_messages) >= 1):
                print(dashboard_messages)
                print("ok")
            print(len(dashboard_messages))
            data = connection.recv(1024)
            reply = data.decode('utf-8')
            print(reply)
            replyS = reply.split("-")
            print(replyS)
            
            if (replyS[0] == robot_names[0]):
                robot_current_positions[0] = replyS
            elif (replyS[0] == robot_names[1]):
                robot_current_positions[1] = replyS
            elif (replyS[0] == robot_names[2]):
                robot_current_positions[2] = replyS
            elif (replyS[0] == robot_names[3]):
                robot_current_positions[3] = replyS
            else:
                print("invalid robot name")        
            if not data:
                break
            #hier moet je je positie sturen naar de pathvinder ?
            #of je eindpositie op het dashbaord
            pathvinderTest = [[1.3],[1,5],[1,5],[1,5]]
            print(robot_current_positions)
            pathvinderStr = (replyS[0]+"-"+str(pathvinderTest))
            #dit is een test voor pathvinder hij zal ook aan robot id moeten verbonden zijn
            connection.sendall(str.encode(pathvinderStr))
            time.sleep(2)
        except KeyboardInterrupt:
            print("Done")
            connection.close()
##   async with websockets.connect("ws://localhost:8000") as websocket:
  #      await websocket.send("Hello world!")
server = threading.Thread(target=between_callback, daemon=True)
server.start()
while True:
    #als er dashboard message binnen komt dush dashboard ++ moet er actie uitgevoerd worden 
    #daarna kan je de messages clearen en dus dezelfde functie gebruiken
    #poepje
    print("sadsa")
    Client, address = ServerSocket.accept()
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    start_new_thread(threaded_client, (Client, ))
    ThreadCount += 1
    print('Thread Number: ' + str(ThreadCount))
ServerSocket.close()
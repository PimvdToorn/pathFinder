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
import pathVinder as pv
ServerSocket = socket.socket()
host = '127.0.0.1'
port = 1024
ThreadCount = 0
robot_hisotry = [["robot1"],["robot2"],["robot3"],["robot4"]]
robot_names = ["robot(1)","robot(2)","robot(3)","robot(4)"]
robot_current_positions = [[],[],[],[]]
dashboard_call_position = []
dashboard_messages = []

grid = pv.Grid(10,10)
grid.addObstacle((1,1))
grid.addObstacle((3,0))
grid.addObstacle((4,1))
grid.addObstacle((4,4))
grid.addObstacle((6,4))
grid.addObstacle((1,2))
grid.addObstacle((1,3))
grid.addObstacle((1,4))
grid.addObstacle((4,3))
grid.addObstacle((2,5))
grid.addObstacle((6,3))
grid.addObstacle((5,1))
grid.addObstacle((5,7))
grid.addObstacle((4,7))
grid.addObstacle((3,7))
grid.addObstacle((6,7))
grid.addObstacle((7,7))

robots : list[pv.Robot] = []

robots.append(pv.Robot("Robot(1)", 1, grid.getNode((0,0)), grid.getNode((5,0))))
robots.append(pv.Robot("Robot(2)", 1, grid.getNode((8,8)), grid.getNode((2,0))))
robots.append(pv.Robot("Robot(3)", 1, grid.getNode((9,0)), grid.getNode((3,2))))
robots.append(pv.Robot("Robot(4)", 1, grid.getNode((3,3)), grid.getNode((7,9))))

for robot in robots:
    robot.findPath(robots)
length = 0
for i, robot in enumerate(robots):
    length += len(robot.path)
    print("Robot " + str(i+1) + ": " + str(robot.path))
print(robots)

    
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
        dashboard_messages.append(data)
        t = json.loads(data)
        coordinaten = t['Coordinates']
        print(coordinaten)
        #dashboard data dit heb je straks nodig om pathvinder nieuwe coordinates te geve
        

def between_callback():
    while True:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        ws_server = websockets.serve(hello, 'localhost', 8000)

        loop.run_until_complete(ws_server)
        loop.run_forever() # this is missing
        loop.close()
# async def send_receive_message(uri):
#     async with websockets.connect(uri) as websocket:
#         await websocket.send('This is some text.')
#         reply = await websocket.recv()
#         print(f"The reply is: '{reply}'")
        
def threaded_client(connection):
    # if len(dashboard_messages) >= 1:
    #     print("ik zit hier")
        
        
    robotsS = str(robots)
    connection.send(str.encode(robotsS))
    while True:
        print(dashboard_messages)
        data = connection.recv(1024)
        reply = data.decode('utf-8')
        print(reply)
        replyS = reply.split("-")
        print(replyS)
        
        if (replyS[0] == robot_names[0]):
                robot_current_positions[0] = replyS
                robot_hisotry[0].append(replyS[1])
        elif (replyS[0] == robot_names[1]):
                robot_current_positions[1] = replyS
                robot_hisotry[1].append(replyS[1])
        elif (replyS[0] == robot_names[2]):
                robot_current_positions[2] = replyS
                robot_hisotry[2].append(replyS[1])
        elif (replyS[0] == robot_names[3]):
                robot_current_positions[3] = replyS
                robot_hisotry[3].append(replyS[1])
        else:
            print("invalid robot name")        
        if not data:
            break
        print(robot_current_positions)
        print(robot_hisotry)
            #hier moet je je positie sturen naar de pathvinder ?
            #of je eindpositie op het dashbaord
            #pathvinderTest = [[1.3],[1,5],[1,5],[1,5]]
            #print(robot_current_positions)
            #pathvinderStr = (replyS[0]+"-"+str(pathvinderTest))
            #dit is een test voor pathvinder hij zal ook aan robot id moeten verbonden zijn
            #connection.sendall(str.encode(pathvinderStr))
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

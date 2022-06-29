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
import numpy as np
import itertools
from copy import copy
ServerSocket = socket.socket()
host = '127.0.0.1'
port = 1024
ThreadCount = 0
robot_hisotry = [["robot1"],["robot2"],["robot3"],["robot4"]]
robot_names = ["robot(1)","robot(2)","robot(3)","robot(4)"]
robot_current_positions = [[],[],[],[]]
dashboard_call_position = []
dashboard_messages = []
obstacle_list = []
obstalce_list_test = [[4, 3, 2], [4, 3, 2], [4, 4, 2], [1, 1, 2], [4, 4, 2], [1, 1, 2], [1, 2, 2], [2, 5, 2], [2, 5, 2], [1, 2, 2], [1, 3, 2], [1, 3, 2], [2, 5, 2], [1, 4, 2], [1, 4, 2], [2, 5, 2], [6, 3, 2], [6, 3, 2], [5, 1, 2], [5, 1, 2]]
grid = pv.Grid(10,10)

# print(obstalce_list_test)
# dup_free = []
# for x in obstalce_list_test:
#     if x not in dup_free:
#         dup_free.append(x)
# print(dup_free)
# print()
# def checkForDupli(input):
#     for input in obstalce_list_test:
#         if x not in dup_free:
    
    
    



# grid.addObstacle((3,0))
# grid.addObstacle((4,1))
# grid.addObstacle((4,4))
# grid.addObstacle((6,4))
# grid.addObstacle((1,2))
# grid.addObstacle((1,3))
# grid.addObstacle((1,4))
# grid.addObstacle((4,3))
# grid.addObstacle((2,5))
# grid.addObstacle((6,3))
# grid.addObstacle((5,1))
# grid.addObstacle((5,7))
# grid.addObstacle((4,7))
# grid.addObstacle((3,7))
# grid.addObstacle((6,7))
# grid.addObstacle((7,7))

robots : list[pv.Robot] = []

robots.append(pv.Robot("Robot(1)", 1, grid.getNode((9,9))))
robots.append(pv.Robot("Robot(2)", 1, grid.getNode((9,0))))
robots.append(pv.Robot("Robot(3)", 1, grid.getNode((0,0))))
robots.append(pv.Robot("Robot(4)", 1, grid.getNode((0,9))))
robots[0].setLocationAndGoal(grid,(9,9),(1,2))
robots[1].setLocationAndGoal(grid,(9,0),(6,5))
robots[2].setLocationAndGoal(grid,(0,0),(6,7))
robots[3].setLocationAndGoal(grid,(0,9),(9,8))

robotsPath = pv.calculateBestPaths(robots)
print(robotsPath)

    
try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))

print('Waitiing for a Connection..')
ServerSocket.listen(5)
async def handle_WS(websocket, uri):
    print(f"Connection accepted from {uri}")
    while True:
        try:
            data = await websocket.recv()
        except:
            print(f"Connection lost to {uri}")
            return
        #print(data)
        dataJ = json.loads(data)
        if dataJ['command'] == "Robot-position":          
            sendData =  {'MessageType' : "Robot-position",
                         'Robot_1' : robot_current_positions[0],
                         'Robot_2' : robot_current_positions[1],
                         'Robot_3' : robot_current_positions[2],
                         'Robot_4' : robot_current_positions[3]} 
            sendDataJ = json.dumps(sendData)
            await websocket.send(sendDataJ)
            sendDataObj = {"MessageType" : "Object-position",
                           "ObjectList" : obstacle_list
            }
            sendDataObjJ = json.dumps(sendDataObj)
            await websocket.send(sendDataObjJ)
        if dataJ['command'] == "sendRobot": 
            dashboard_messages.append(dataJ)
            print(dashboard_messages)
            ####################
            #ashboard_robot_id = dashboard_messages[0]["robot_ID"]
            #dashboard_coordinates = dashboard_messages[0]["Coordinates"]
            #print(ashboard_robot_id,dashboard_coordinates)
            #####################
            await websocket.send("ok")
def between_callback():
    while True:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        ws_server = websockets.serve(handle_WS, 'localhost', 8000)

        loop.run_until_complete(ws_server)
        loop.run_forever() # this is missing
        loop.close()
        
# async def send_receive_message(uri):
#     async with websockets.connect(uri) as websocket:
#         await websocket.send('This is some text.')
#         reply = await websocket.recv()
#         print(f"The reply is: '{reply}'")
        
def threaded_client(connection):
    print(len(dashboard_messages))
        
    robotsS = str(robotsPath)
    connection.send(str.encode(robotsS))
    while True:
        print("dsadas")
        data = connection.recv(1024)
        reply = data.decode('utf-8')
        replyJ = json.loads(reply)
        print(replyJ)
        if(len(dashboard_messages) > 0):
            connection.send(str.encode("halo"))
            replyJ = json.loads(reply)
            robot_ID = replyJ["Robot_ID"]
            dashboard_robot_id = dashboard_messages[0]["robot_ID"]
            dashboard_coordinates = dashboard_messages[0]["Coordinates"]
            try:
                if(replyJ["Waiting"] == True):
                    connection.send(str.encode("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"))
                    for i in range(4):
                        if i == robot_ID[6]:
                            i =+ i
                            robots[0].setLocationAndGoal(grid,(1,2),(9,6))
                            robots[1].setLocationAndGoal(grid,(6,5))
                            robots[2].setLocationAndGoal(grid,(6,7))
                            robots[3].setLocationAndGoal(grid,(9,8))
                            asd = pv.calculateBestPaths(robots)
                            print(asd)
                            connection.send(str.encode("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ"))
            except:
                print("Robot ziet een dashboard message")
        else:
            #print(reply)
            #print(type(reply))
            
            #time.sleep(0.03)
            #robotsS = str(robotsPath)
            #.send(str.encode(robotsS))
            try:
                if(replyJ["Waiting"] == True):
                    connection.send(str.encode("Nog geen message in het dashboard"))
            except:
                print("Robot ziet een dashboard message")
            print(robotsS)
            if(replyJ["Message"] == "SendPosition"):
                if (replyJ["Robot_ID"] == robot_names[0]):
                        robot_current_positions[0] = replyJ["Position"]
                        robot_hisotry[0].append(replyJ["Position"])
                elif (replyJ["Robot_ID"] == robot_names[1]):
                        robot_current_positions[1] = replyJ["Position"]
                        robot_hisotry[1].append(replyJ["Position"])
                elif (replyJ["Robot_ID"] == robot_names[2]):
                        robot_current_positions[2] = replyJ["Position"]
                        robot_hisotry[2].append(replyJ["Position"])
                elif (replyJ["Robot_ID"] == robot_names[3]):
                        robot_current_positions[3] = replyJ["Position"]
                        robot_hisotry[3].append(replyJ["Position"])
                else:
                    print("invalid robot name")  
            elif(replyJ["Message"] == "ObjectDetection"):
                    #[x,x,x,x] 0 = niks, 1 = intruder, 2 = obstacle
                    #[rechts.links.boven.onder.]
                #print(replyJ)
                robot_position = replyJ["PositionObs"]
                obstacle = replyJ["ObsType"]
                robot_ID = replyJ["Robot_ID"]
                #print(obstacle)
                #print(robot_position)
                if(obstacle[0] != 0):
                    isRobot = False
                    obstacletype = obstacle[0]
                    post = copy(robot_position)
                    post[0] = post[0]+1
                    #print(post)
                    cpost = [post[0],post[1]]
                    for i in range(4):
                        if(robot_current_positions[i] == cpost):
                            isRobot = True  
                    if(post[0] == 10 and not isRobot):
                            print("muur found",robot_ID,post[0],post[1])
                    elif (isRobot):
                        print("is robot")
                    else: 
                        if [post[0],post[1],obstacletype] not in obstacle_list:
                            obstacle_list.append([post[0],post[1],obstacletype])
                if(obstacle[1] != 0):
                    isRobot = False
                    post = copy(robot_position)
                    obstacletype = obstacle[1]
                    print(robot_ID)
                    print(post[0],post[1])
                    post[0] = post[0]-1
                    print(post[0])
                    cpost = [post[0],post[1]]
                    for i in range(4):
                        if(robot_current_positions[i] == cpost):
                            isRobot = True 
                    if(post[0] == -1 and not isRobot):
                            print("muur found", robot_ID,post[0],post[1])
                            print("FOUT")
                    elif (isRobot):
                        print("is robot")
                    else:
                        if [post[0],post[1],obstacletype] not in obstacle_list:
                            obstacle_list.append([post[0],post[1],obstacletype])
                if(obstacle[2] != 0):
                    isRobot = False
                    post = copy(robot_position)
                    post[1] = post[1]+1
                    obstacletype = obstacle[2]
                    cpost = [post[0],post[1]]
                    for i in range(4):
                        if(robot_current_positions[i] == cpost):
                            isRobot = True 
                    if(post[1] == 10 and not isRobot):
                            print("muur found",robot_ID,post[0],post[1])
                    elif (isRobot):
                        print("is robot")
                            
                    else:
                        if [post[0],post[1],obstacletype] not in obstacle_list:
                            obstacle_list.append([post[0],post[1],obstacletype]) 
                if(obstacle[3] != 0):
                    isRobot = False
                    post = copy(robot_position)
                    post[1] = post[1]-1
                    obstacletype = obstacle[3]
                    cpost = [post[0],post[1]]
                    for i in range(4):
                        if(robot_current_positions[i] == cpost):
                            isRobot = True    
                    if(post[1] == -1 and not isRobot):
                            print("muur found",robot_ID,post[0],post[1])
                    elif (isRobot):
                        print("is robot")
                            
                    else:
                        if [post[0],post[1],obstacletype] not in obstacle_list:
                            obstacle_list.append([post[0],post[1],obstacletype])
        #print(robot_current_positions)
server = threading.Thread(target=between_callback, daemon=True)
server.start()
while True:
    Client, address = ServerSocket.accept()
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    start_new_thread(threaded_client, (Client, ))
    ThreadCount += 1
    print('Thread Number: ' + str(ThreadCount))


ServerSocket.close()

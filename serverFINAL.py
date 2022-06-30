import asyncio
#import websockets
import websockets
from importlib.resources import path
import socket
import threading
from _thread import *
import time
import json
import pathfinder as pv
from copy import copy
ServerSocket = socket.socket()
host = '145.24.238.52'
#host = '127.0.0.1'
port = 1024
ThreadCount = 0
robot_hisotry = [["robot1"],["robot2"],["robot3"],["robot4"]]#this is where the data is placed of where al the robots have been
robot_names = ["robot(1)","robot(2)","robot(3)","robot(4)"]#robot names
robot_current_positions = [[],[],[],[]]#current positions of the robots
dashboard_messages = []#list of where the robot messages are placed
obstacle_list = []#obdstcle list, used for dynamic obstacle detection for the dashboard
grid = pv.Grid(11,11)

obstacles = [(2,2),(2,3),(2,4),(2,5),(2,6),(2,7),(2,8),
             (4,2),(4,3),(4,4),(4,5),(4,6),(4,7),(4,8),
             (6,2),(7,2),(8,2),
             (6,4),(7,4),(8,4),
             (6,6),(7,6),(8,6),
             (6,8),(7,8),(8,8)]
for obstacle in obstacles:#this is where the obstacles ale placed
    grid.addObstacle(obstacle)
robots : list[pv.Robot] = []
#this is where robots are made
robots.append(pv.Robot("Robot(1)", 4, grid.getNode((0,0))))
robots.append(pv.Robot("Robot(2)", 3, grid.getNode((10,0))))
robots.append(pv.Robot("Robot(3)", 2, grid.getNode((10,10))))
robots.append(pv.Robot("Robot(4)", 1, grid.getNode((0,10))))
robots[0].setLocationAndGoal(grid,(0,0),(10,10))#hier zet je de locatie en goal per robot
robots[1].setLocationAndGoal(grid,(10,0),(0,10))
robots[2].setLocationAndGoal(grid,(10,10),(10,0))
robots[3].setLocationAndGoal(grid,(0,10),(0,0))

robotsPath = pv.calculateBestPaths(robots)

try:
    ServerSocket.bind((host, port))
except socket.error as e:
    print(str(e))

print('Waitiing for a Connection..')
ServerSocket.listen(5)#message handler for the dashbaord websocket
async def handle_WS(websocket, uri):
    print(f"Connection accepted from {uri}")
    while True:
        try:
            data = await websocket.recv()
        except:
            print(f"Connection lost to {uri}")
            return
        dataJ = json.loads(data)
        if dataJ['command'] == "Robot-position":          
            sendData =  {'MessageType' : "Robot-position",
                         'Robot_1' : robot_current_positions[0],
                         'Robot_2' : robot_current_positions[1],
                         'Robot_3' : robot_current_positions[2],
                         'Robot_4' : robot_current_positions[3]} 
            sendDataJ = json.dumps(sendData)#this is where the position is sent to the dashbaord
            await websocket.send(sendDataJ)
            sendDataObj = {"MessageType" : "Object-position",
                           "ObjectList" : obstacle_list
            }#this is where the list of obstacles is sent to the server 
            sendDataObjJ = json.dumps(sendDataObj)
            await websocket.send(sendDataObjJ)
        if dataJ['command'] == "command":#thid is where the command is gonna be for the task appointment 
            dashboard_messages.append(dataJ)
            print(dashboard_messages)
            await websocket.send("ok")
def between_callback():
    while True:#websocket thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        ws_server = websockets.serve(handle_WS, 'localhost', 8000)
        loop.run_until_complete(ws_server)
        loop.run_forever() # this is missing
        loop.close()     
def threaded_client(connection):
    print(len(dashboard_messages))
    robotsS = str(robotsPath)
    connection.send(str.encode(robotsS))
    while True:
        data = connection.recv(1024)
        reply = data.decode('utf-8')
        replyJ = json.loads(reply)
        print(replyJ)
        if(replyJ["Message"] == "SendPosition"):#here the position of the robot get appendet in the correct list
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
                #[right.left.top.bots.]
            robot_position = replyJ["PositionObs"]#this is the position of the robot to get the obsatcle position you need to add 1 at the correct side
            obstacle = replyJ["ObsType"]#obstacle type and the side at which its locate
            robot_ID = replyJ["Robot_ID"]#theese if statement are for the detection of different object and their positions
            if(obstacle[0] != 0):#this is to see at which side of the robot the obstacle 
                isRobot = False
                obstacletype = obstacle[0]
                post = copy(robot_position)
                post[0] = post[0]+1
                objectpos = [post[0],post[1]]
                for i in range(4):#this is a check to see if the obstacle detected is not a robot
                    if(robot_current_positions[i] == objectpos):
                        isRobot = True  
                if(post[0] == 10 and not isRobot):
                        print("muur found",robot_ID,post[0],post[1])
                elif (isRobot):
                    print("is robot")
                else: 
                    if [post[0],post[1],obstacletype] not in obstacle_list:#this is to not have double object in the list
                        obstacle_list.append([post[0],post[1],obstacletype])
            if(obstacle[1] != 0):
                isRobot = False
                post = copy(robot_position)
                obstacletype = obstacle[1]
                print(robot_ID)
                print(post[0],post[1])
                post[0] = post[0]-1
                print(post[0])
                objectpos = [post[0],post[1]]
                for i in range(4):
                    if(robot_current_positions[i] == objectpos):
                        isRobot = True 
                if(post[0] == -1 and not isRobot):
                        print("muur found", robot_ID,post[0],post[1])
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
                objectpos = [post[0],post[1]]
                for i in range(4):
                    if(robot_current_positions[i] == objectpos):
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
                objectpos = [post[0],post[1]]
                for i in range(4):
                    if(robot_current_positions[i] == objectpos):
                        isRobot = True    
                if(post[1] == -1 and not isRobot):
                        print("muur found",robot_ID,post[0],post[1])
                elif (isRobot):
                    print("is robot")
                else:
                    if [post[0],post[1],obstacletype] not in obstacle_list:
                        obstacle_list.append([post[0],post[1],obstacletype])
#websocket thread
server = threading.Thread(target=between_callback, daemon=True)
server.start()
while True:#this is the main thread where the client thread are made
    Client, address = ServerSocket.accept()
    print('Connected to: ' + address[0] + ':' + str(address[1]))
    start_new_thread(threaded_client, (Client, ))
    ThreadCount += 1
    print('Thread Number: ' + str(ThreadCount))
ServerSocket.close()

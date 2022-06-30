from controller import Robot, GPS, LED, DistanceSensor,Supervisor
from math import floor
import sys
import socket
import json
from copy import copy
import time
ClientSocket = socket.socket()
host = '127.0.0.1'
port = 1024
robot_position = []
try:
    ClientSocket.connect((host, port))
except socket.error as e:
    print(str(e))
# from time import sleep

msPerStep = 500
 
    
# robot = Robot()
robot = Supervisor()
supervisorNode = robot.getSelf()
naam = robot.getName()
print(naam)


gps = robot.getDevice("gps")
gps.enable(100) # sampling period; ms


distX = robot.getDevice("dist X")
distnX = robot.getDevice("dist -X")
distY = robot.getDevice("dist Y")
distnY = robot.getDevice("dist -Y")

distX.enable(100)
distnX.enable(100)
distY.enable(100)
distnY.enable(100)



ledX = robot.getDevice("led X")
lednX = robot.getDevice("led -X")

ledY = robot.getDevice("led Y")
lednY = robot.getDevice("led -Y")


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
duration = (msPerStep // timestep ) * timestep


tileSize = 0.1
zeroTile = 0

# tilesX = 10
# tilesY = 10 
# maxTileX = zeroTile + (tilesX-1)*tileSize
# maxTileY = zeroTile + (tilesY-1)*tileSize

trans = supervisorNode.getField("translation")

def setPosition(pos):
    x, y = pos
    
    xC = zeroTile + x * tileSize
    yC = zeroTile + y * tileSize
    zC = 0.045
    
    # if int(sys.argv[1]) == 3:
        # print(robot.getName() + ": " + str((x,y)))
        
    trans.setSFVec3f([xC,yC,zC])


def setLeds(leds):
    X, nX, Y, nY = leds
    
    ledX.set(X)
    lednX.set(nX)
    ledY.set(Y)
    lednY.set(nY)
def sendObs(positie,type):
    time.sleep(0.1)
    dummydata = {"Message":"asdsa","Robot_ID": "Test","Position" : [9,9]}
    print(robot.getName())
    print(type)
    print(positie)
    sendData = {"Message": "ObjectDetection","Robot_ID": robot.getName(),"ObsType" : type,"PositionObs" : positie}
    finData = json.dumps(sendData)
    print(finData)
    ClientSocket.send(str.encode(finData))

# def nextDest():
    # if len(path) > 0:
        # destination = path.pop(0)
            
    # else: setLeds((1,1,1,1))
#xC, yC, zC = gps.getValues()
#x = floor((1/tileSize) * xC)
#y = floor((1/tileSize) * yC)
def getPositionAsList():
    xC, yC, zC = gps.getValues()
    x = floor((1/tileSize) * xC)
    y = floor((1/tileSize) * yC) 
    robot_position = [x,y]
    return robot_position
x = 0
y = 0
robot_pos = [0,0]
#sendData = {"Message":"SendPosition","Robot_ID": robot.getName(),"Position" : robot_pos}
#finData = json.dumps(sendData)
#ClientSocket.send(str.encode(finData))
print("test1")
Response = ClientSocket.recv(1024)
ResponseList = eval(Response)

print(Response)
paths = ResponseList

destination = ()
path = paths[int(sys.argv[1])]
moveTo = path.pop(0)
didmove= True
# #print(robot.getName() + ": " + str(moveTo))

def checksur():
    #[x,x,x,x] 0 = niks, 1 = intruder, 2 = klokbiertje
    #[rechts.links.boven.onder.]
    surroundings = [0,0,0,0]
    obstacleType = 0
    oX = distX.getValue() < 1
    onX = distnX.getValue() < 1
    oY = distY.getValue() < 1
    onY = distnY.getValue() < 1
    if oX:
        if distX.getValue() <= 0.90:
            print(robot.getName() + "obstical x")
            surroundings[0] = 2            
            path = []
        elif distX.getValue() <= 0.99:
            print(robot.getName() + "intruder x")
            surroundings[0] = 1
            path = []  
    else:
        print(robot.getName() + "error ") 
        surroundings [0] = 0
    
    if onX:
        if distnX.getValue() <= 0.90:
            print(robot.getName() + "obstical negx")
            surroundings [1] = 2
            ObstacleType = 0
            path = []
        elif distnX.getValue() <= 0.99:
            print(robot.getName() + "intruder negx")
            surroundings [1] = 1
            ObstacleType = 1
            path = []  
    else:
        print(robot.getName() + "error") 
        surroundings [1] = 0
    
    
    
    if oY:
        if distY.getValue() <= 0.90:
            print(robot.getName() + "obstical y")
            surroundings [2] = 2
            ObstacleType = 0
            path = []
        elif distY.getValue() <= 0.99:
            print(robot.getName() + "intruder y")
            surroundings [2] = 1
            ObstacleType = 1
            
            path = []
    else:
        print(robot.getName() + "error") 
        surroundings [2] = 0
        
       
    if onY:
        if distnY.getValue() <= 0.90:
            print(robot.getName() + "obstical negy")
            surroundings [3] = 2
            ObstacleType = 0
            path = []
        elif distnY.getValue() <= 0.99:
            print(robot.getName() + "intruder negy")
            surroundings [3] = 1
            ObstacleType = 1
            
            path = []     
    else:
        print(robot.getName() + "error")
        surroundings [3] = 0
    sendObs(robot_pos,surroundings)
def setgetPath():
    Response = ClientSocket.recv(1024)
    return Response
while robot.step(duration) != -1:
    waiting = False
    if(len(path) == 0):
        print("waiting")
        waiting = True

    xC, yC, zC = gps.getValues()
    
    x = round((1/tileSize) * xC)
    y = round((1/tileSize) * yC) 
    robot_pos = [x,y]
    x = robot_pos[0]
    y = robot_pos[1]
    

    # destination = ()
    # path = paths[int(sys.argv[1])]
    # moveTo = path.pop(0)
    # didmove= True
    # print(robot_pos)
    #positionSTR = str(robot_pos)
    #robotdata = (robot.getName()+"-"+positionSTR)
    #robotdata = (robot.getName()+"-"+robot_pos)
    #ClientSocket.send(str.encode(robotdata))
    sendData = {"Message":"SendPosition","Robot_ID": robot.getName(),"Position" : robot_pos,"Waiting":waiting}
    finData = json.dumps(sendData)
    ClientSocket.send(str.encode(finData))
    #if(waiting == True):
        #print("yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy")
        #Response = ClientSocket.recv(1024)
        #print(Response)
    #print("test1")
    #Response = ClientSocket.recv(1024)
    # print(Response)
    # ResponseL = eval(Response)
    # time.sleep(0.5)
    # print(ResponseL)
    #kanker = setgetPath()
    #print(kanker)
    didmove= True
    checksur()  
    if didmove:
        if moveTo != -1:
            if moveTo != (x, y):
                setPosition(moveTo)
                
            moveTo = -1
            setLeds((0,0,0,0))
            
            
            if len(path) > 0:
                destination = path.pop(0)
                
            else:
                #print(path)
                didmove = True
                #print("test")
                destination = -1
                setLeds((1,1,1,1))
            
            
        elif destination != -1:
        
            # #Object detection
            oX = distX.getValue() < 1
            onX = distnX.getValue() < 1
            oY = distY.getValue() < 1
            onY = distnY.getValue() < 1
            
            #print(robot.getName() +str((oX, onX, oY, onY)))
            if x < destination[0] and not oX:
                setLeds((1,0,0,0))
                moveTo = (x+1, y)
                didmove = True
            elif x < destination[0] and oX:
                moveTo = (x, y)
                didmove = False
                
            elif x > destination[0] and not onX: 
                setLeds((0,1,0,0))
                moveTo = (x-1, y)
                didmove = True
            elif x > destination[0] and onX:
                moveTo = (x, y)
                didmove = False 
    
            
            elif y < destination[1] and not oY:
                setLeds((0,0,1,0))
                moveTo = (x, y+1)
                didmove = True
            elif y < destination[1] and oY:
                moveTo = (x, y)
                didmove = False 
                
            elif y > destination[1] and not onY:
                setLeds((0,0,0,1))
                moveTo = (x, y-1)
                didmove = True             
            elif y > destination[1] and onY:
                moveTo = (x, y)
                didmove = False 
        
            else:
                moveTo = (x, y)
                didmove = True
        else:
            print("ok")
    elif not didmove:
        #time.sleep(0.01)
        checksur()
        didmove = True
    pass

# Enter here exit cleanup code.


# Enter here exit cleanup code.


# Enter here exit cleanup code.

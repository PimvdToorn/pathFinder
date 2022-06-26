import re
from time import sleep
from timeit import default_timer as time
from itertools import permutations
from copy import copy


class Node:
    def __init__(self, pos):
        self.pos = pos
        self.links = []

    def __str__(self) -> str:
        return "(" + str(self.pos[0]) + ',' + str(self.pos[1]) + ")"
        
    def __repr__(self):
        return str(self)


    def addLink(self, linkNode):
        for node in self.links:
            if node == linkNode: return

        # If not already linked, add the node
        self.links.append(linkNode)
        linkNode.addLink(self)




class Grid:
    def __init__(self, x, y):   

        self.nodes = []

        for i in range(x):

            for j in range(y):
                self.nodes.append(Node((i, j)))
                
                if j > 0: self.nodes[-1].addLink(self.nodes[-2])    # Add previous as link
                if i > 0: self.nodes[-1].addLink(self.nodes[-1-y])  # Add the "top" one from the previous row

    def addObstacle(self, pos):
        for node in self.nodes:
            if node.pos == pos:

                for link in node.links:         # Remove the link to the obstacle in the other links
                    link.links.remove(node)

                self.nodes.remove(node)
                break    


    def getNode(self, pos):
        for node in self.nodes:
            if node.pos == pos: return node

        return None




class dijkstraNode:
    def __init__(self, node : Node, dist : int, previous):
        self.node = node
        self.dist = dist
        self.previous = previous
        self.visited = False

    def __str__(self) -> str:
        if self.dist == 0:          return f"{self.node} Start"
        elif self.previous == None: return f"{self.node} d:{self.dist} p:None v:{self.visited}" 
        else:                       return f"{self.node} d:{self.dist} p:{self.previous.node} v:{self.visited}" 

    def __repr__(self):
        return str(self.node) + " d:" + str(self.dist)




class Robot:

    def __init__(self, name : str, cost : int, location : Node, goal : Node = None):
        self.name = name
        self.cost = cost
        self.location = location
        self.goal = goal
        self.path : list[tuple] = []
        # if goal == None: self.path.append(location)
        # self.path : list[tuple] = [location]
    
    def __str__(self):
        steps = (len(self.path) - 1)
        cost = steps * self.cost

        if len(self.path) <= 0: 
            return(f"{self.name:8s} cost inf | steps{steps:-3d}: {self.path}")
        else:
            return(f"{self.name:8s} cost{cost:-4d} | steps{steps:-3d}: {self.path}")
        

    def __repr__(self):
        return str(self.path)

    def setPath(self, path : list[Node]):
        self.path = path

    def setPositions(self, positions : list[(int,int)]):
        for pos in positions:
            self.path.append(grid.getNode(pos))

    def getPositions(self):
        positions = []
        for node in self.path:
            positions.append(node.pos)
        return positions

    def findPath(self, robots : list):
        self.path = []
        self.path = findPath(robots, self.location, self.goal)

        if self.path == []: 
            # print("No path")
            return float("inf")
        return (len(self.path) - 1) * self.cost


def newPrevious(dijkNodes : list[dijkstraNode], unlinkedNode : dijkstraNode):

    for dNode in dijkNodes:
        if dNode.node in unlinkedNode.node.links:
            if dNode.dist != 0 and (unlinkedNode.previous == None or (((unlinkedNode.previous == dNode and dNode.dist > unlinkedNode.previous.dist) or (unlinkedNode.previous != dNode and dNode.dist < unlinkedNode.previous.dist)) and dNode.previous != unlinkedNode)):
                # print(f"dNode previous:{dNode.previous}\tunlinked:{unlinkedNode}\tsame:{dNode.previous == unlinkedNode}")
                unlinkedNode.previous = dNode
                nodeCollide(robots, unlinkedNode)

                # print(f"New prev {dNode}\tfor {unlinkedNode}")
    
    if unlinkedNode.previous == None or unlinkedNode.dist == float("inf"): return False
    # if unlinkedNode.dist <= unlinkedNode.previous.dist: nodeCollide(robots, unlinkedNode)
    return True



def nodeCollide(robots : list[Robot], next : dijkstraNode):
    # Check if any robot will be at the location the step before, during or after this robot would move there, if so add 1 to distance and check again
    # (wait 1 more at current location before moving to link)

    next.dist = max(next.dist, next.previous.dist + 1)

    # print(f"Checking collision with: {next}")
    intersect = True
    while intersect:
        intersect = False

        for robot in robots:

            # Option 1 -------------------------------------------------------------------------------------------------

            #    robot has steps then      and       r will be there          or     r just left                 or     r has step after             and   will be there just after 
            # if len(robot.path) > next.dist and (robot.path[next.dist] == next.node or robot.path[next.dist-1] == next.node or (len(robot.path) > next.dist + 1 and robot.path[next.dist+1] == next.node)):
            #     next.dist += 1
            #     # print("Distance upped to: " + str(next.dist))
            #     intersect = True
            #     break

            # Option 2 -------------------------------------------------------------------------------------------------

            # #    Robot has steps then   
            # if len(robot.path) > next.dist:

            #     # Robot will be there
            #     if robot.path[next.dist] == next.node:
            #         next.dist += 2
            #         intersect = True
            #         break

            #     # Robot just left
            #     elif robot.path[next.dist-1] == next.node:
            #         next.dist += 1
            #         intersect = True
            #         break

            #     # Robot has step after                and   will be there just after
            #     elif len(robot.path) > next.dist + 1 and robot.path[next.dist+1] == next.node:
            #         next.dist += 3
            #         intersect = True
            #         break

                # if intersect: 
                #     # print("Distance upped to: " + str(next.dist))
                #     break

            # Option 3 -------------------------------------------------------------------------------------------------

            if robot.path != []:
                
                # Add distance if another robot is blocking the next node
                length = len(robot.path) - 1
                endIndex = length - next.dist + 1
                # print(f"Start:{startIndex} end:{endIndex} length:{length}")

                if length + 1 > 0 and robot.path[-1] == next.node and length <= next.dist:
                    next.dist = float("inf")
                    return

                elif endIndex > 0:
                    startIndex = length - next.dist - 1
                    if startIndex < 0: startIndex = 0

                    try:
                        lastIndex = length - robot.path[::-1].index(next.node, startIndex, endIndex)
                    except:
                        lastIndex = -1
                    else:
                        next.dist = lastIndex + 2
                        intersect = True
                        break


                # Check if the robot is not in the way before it moves to the next node

                if not intersect:
                    length = len(robot.path) - 1
                    startIndex = length - next.dist
                    if startIndex < 0: startIndex = 0
                    endIndex = length - next.previous.dist + 1
                    # if endIndex > length + 1: endIndex = length + 1


                    if length + 1 > 0 and robot.path[-1] == next.previous.node and length <= next.dist:
                        # next.previous = None
                        next.dist = float("inf")
                        break

                    elif endIndex >= 0:

                        try:
                            lastIndex = length - robot.path[::-1].index(next.previous.node, startIndex, endIndex)
                        except:
                            lastIndex = -1
                        else:
                            # print(f"Can't reach in time: {next} from: {next.previous} due to {robot.name} being there at {lastIndex}")
                            # sleep(1)

                            # if not intersect or (len(dNodesToAdd) > 0 and dNodesToAdd[-1].dist != lastIndex + 2):
                            #     if current.previous != None:
                            #         dNodesToAdd.append(dijkstraNode(current.node, lastIndex + 2, current.previous))

                            # if current.previous != None:
                            #     print("in da way: " + str(current.node) + " at" + str(lastIndex) + "\tPr:" + str(current.previous.node))
                            
                            # next.previous = None
                            next.dist = float("inf")
            
                            # intersect = True
                            break

            # ----------------------------------------------------------------------------------------------------------

        # if intersect: 
        #     print(f"Intersect {next}")
        #     sleep(1)


    return next



def findPath(robots : list[Robot], start : Node, goal : Node):

    # print("--------------------------------------------------------------------------------------------")
    dijkNodes = [dijkstraNode(start, 0, None)]

    while True:

        noPrevious = True
        while noPrevious:

            current = dijkstraNode(None, float("inf"), None)

            # Get the node with the shortest distance, that hasn't been visited yet (fully checked)
            for dNode in dijkNodes:
                if not dNode.visited and dNode.dist < current.dist:
                    current = dNode

            # All visited, no options left
            if current.node == None: return []

            noPrevious = current.previous == None and current.dist != 0
            if noPrevious: noPrevious = not newPrevious(dijkNodes, current)
            if noPrevious: dijkNodes.remove(current)
       


        # print(dijkNodes)
        # for node in dijkNodes:
        #     print(node)

        # print("Visiting: " + str(current))
        for link in current.node.links:
            if current.previous == None or link != current.previous.node: 

                # print("New node at:" + str(link))
                next = dijkstraNode(link, current.dist+1, current)

                # Check if any robot will be at the location the step before, during or after this robot would move there and update the distance accordingly
                # next = intersect(robots, next)
                nodeCollide(robots, next)
                
                
                if next.dist != float("inf") and (next.dist == 0 or next.previous != None):   

                    if next.node == goal:
                        # print("Goal check")
                        for robot in robots:
                            if robot.path != []:
                                length = len(robot.path) - 1
                                # startIndex = length - next.dist - 1
                                # if startIndex < 0: startIndex = 0
                                startIndex = 0
                                endIndex = length - current.dist
                                # print(f"Length:{length} CurrentDist:{current.dist} endIndex:{endIndex}")
                                # print(current.dist)
                                # print(endIndex)
                                # endIndex = length - next.dist

                                # if length + 1 > 0 and robot.path[-1] == next.node and length <= next.dist: return []

                                if endIndex >= 0:
                                    try:
                                        lastIndex = length - robot.path[::-1].index(next.node, startIndex, endIndex)
                                    except:
                                        # print("\t\tNot in the way")
                                        lastIndex = -1

                                    else:
                                        next.dist = lastIndex + 2
                                        # print(f"Next dist: {next.dist}")

                        dijkNodes.append(next)
                        break

                    
                    # Check if any robot will end at the location before this robot would be there
                    intersect = False
                    for robot in robots:
                        if len(robot.path) > 0 and robot.path[-1] == next.node and len(robot.path) <= next.dist:
                            # print("End stop: not added")
                            intersect = True
                            break
                    
                    if not intersect:
                        exists = False
                        for dNode in dijkNodes:

                            if next.node == dNode.node:
                                if next.dist < dNode.dist:
                                    # print("Replaced: " + str(dNode) + "\tNow:" + str(next))
                                    dNode = next
                                # print("Exists: not added")
                                exists = True
                                break

                                
                        if not exists:
                            # print("Added: " + str(next))
                            dijkNodes.append(next)


        current.visited = True


        intersect = True
        while intersect:
            intersect = False

            noOfLinks = 0

            # print("C:" + str(current))
            linksToCurrent : list[dijkstraNode] = []
            # linksToRemove : list[dijkstraNode] = []
            dNodesToAdd : list[dijkstraNode] = []

            for link in dijkNodes:
              
                # print(i)
                # print(r[-1])
                # print(len(dijkNodes))
                # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                # link = dijkNodes[i]
                


                if link.previous == current:
                    # if link.node == goal: 
                    #     print("goal")
                    #     print("\t" + str(link))
                    noOfLinks += 1
                    linksToCurrent.append(link)

                    for robot in robots:
                        if robot.path != []:
                            length = len(robot.path) - 1
                            startIndex = length - link.dist
                            startIndex = max(0, startIndex)
                            endIndex = length - current.dist + 1
                            # if endIndex > length + 1: endIndex = length + 1


                            if length + 1 > 0 and robot.path[-1] == current.node and length <= link.dist:
                                link.previous = None
                                # linksToRemove.append(link)
                                linksToCurrent.remove(link)

                                # print("\t\tend")
   
                                noOfLinks -= 1
                                intersect = True
                                break

                            elif endIndex >= 0:

                                try:
                                    lastIndex = length - robot.path[::-1].index(current.node, startIndex, endIndex)
                                except:
                                    # print("\t\tNot in the way")
                                    lastIndex = -1


                                else:
                                    # print(f"StartIndex:{startIndex} EndIndex:{endIndex} RobotPathLength:{length}\tChecking distance: {link.dist-1} to {current.dist + 1}")
                                    # print(f"Can't reach in time: {link} from: {current} due to {robot.name} being there at {lastIndex}")
                                    # sleep(1)

                                    if not intersect or (len(dNodesToAdd) > 0 and dNodesToAdd[-1].dist != lastIndex + 2):
                                        if current.previous != None:
                                            dNodesToAdd.append(dijkstraNode(current.node, lastIndex + 2, current.previous))

                                    link.previous = None
                                    # linksToRemove.append(link)

                                    noOfLinks -= 1
                  
                                    intersect = True
                                    break

                        # # Don't step onto a starting point of a robot with no calculated path too quickly  
                        if link.node == robot.location and link.dist < 2:
                            link.dist = 2

                # if intersect: break
            
            if intersect:
                # for link in linksToRemove:
                #     # dijkNodes.remove(link)
                #     # print(f"Unlinked previous of: {link}")
                #     link.previous = None
                
                for dNode in dNodesToAdd:
                    # print(f"Added: {dNode}")
                    dijkNodes.append(dNode)

                previous = current.previous

                # if noOfLinks == 0:
                #     print(linksToCurrent)
                #     print("removed: " + str(current))
                #     dijkNodes.remove(current)
                    
                # if previous not in dijkNodes:
                #     dijkNodes.remove(current)
                #     previous = None

                # print("Current: " + str(current) + "\tLinks:" + str(noOfLinks))
                # print("Previous: " + str(previous))

                # if not current.visited:
                #     dijkNodes.remove(current)

                

                if previous == None: break
                current = previous

        reachedGoal = False
        goalDNode : dijkstraNode = None

        for dNode in dijkNodes[::-1]:
            if dNode.node == goal and dNode.previous != None:
                # print("Goal----------------------------")
                goalDNode = dNode
                reachedGoal = True
                break

        # once = reachedGoal
        while goalDNode != None:
            # print(f"Goalnodes: {goalDNode}")
            # sleep(0.5)
            reachedGoal = goalDNode.node == start
            goalDNode = goalDNode.previous

        # if once != reachedGoal: print("Didn't reach goal@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
        if reachedGoal: 
            # print("Reached goal---------------------------------------------")
            break


    # print("dijkNodes:")
    # for node in dijkNodes:
    #     print(node)

    # Change the dijkNodes to a list of nodes:

    current = None
    for dNode in dijkNodes[::-1]:
        if dNode.node == goal:
            current = dNode
            break


    # print(current)
    path = [current.node]

    while True:

        # print(current.previous)
        # print(current.previous.dist)

        # Stay the distance amount of steps at the previous, to wait on any other robot
        path = (current.dist - current.previous.dist) * [current.previous.node] + path
        # print(f"\t{path}")

        if path[0] == start: break

        current = current.previous

    # print(path)
    return path


def checkPathsPossibility(robots : list[Robot]):
    for robot in robots:
        for otherBot in robots:
            if otherBot != robot:
                length = len(otherBot.path)
                if length == 0: return False

                for i, node in enumerate(robot.path):
                    if  length <= i and otherBot.path[-1] == node:
                        # print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i-1} at node {node}")
                        return False
                    if length > i and otherBot.path[i] == node:
                        # print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i} at node {node}")
                        return False
                    if length > i + 1 and otherBot.path[i + 1] == node:
                        # print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i+1} at node {node}")
                        return False

    return True
    

def calculateBestPaths(robots : list[Robot], verbose : bool = False, count : bool = False):
    if verbose: start = time()


    dontMove = 0
    for robot in robots:
        if robot.goal == None: 
            robot.path = [robot.location]
            dontMove += 1
            robots.remove(robot)
            robots = [robot] + robots
        else:
            for otherBot in robots:
                if otherBot != robot and robot.goal == otherBot.goal or (otherBot.goal == None and otherBot.location == robot.goal): 
                    if verbose: print(f"End collision of {robot.name} with {otherBot.name}")
                    return -1


    bestRobotOrder : list[Robot] = []
    for i in range(dontMove):
        bestRobotOrder.append(copy(robots[i]))




    order = [*range(dontMove, len(robots))]

    allOrders = list(permutations(order))
    # allOrders = [(3, 2, 1, 4, 5, 6, 7)]
    # allOrders = [(1, 2, 3, 4, 5, 6, 7)]

    if count: numberOfOrders = len(allOrders)
    elif verbose: print(f"Number of calculations to do: {len(allOrders)}")


    lowestTotalCost = float("inf")
    lowestTotalSteps = float("inf")


    for i, order in enumerate(allOrders):

        if count: print(f"Calculating: {i+1} of {numberOfOrders}\t{order}")

        for robotNumber in order:
            robots[robotNumber].path = []

        totalCost = 0
        totalSteps = 0
        for robotNumber in order:
            # print(robots[robotNumber].name)
            cost = robots[robotNumber].findPath(robots)
            totalCost += cost
            if totalCost > lowestTotalCost: break
            totalSteps += len(robots[robotNumber].path) - 1
        

        if (totalCost < lowestTotalCost or (totalCost == lowestTotalCost and totalSteps < lowestTotalSteps)) and totalCost != float("inf") and checkPathsPossibility(robots):
            # print(f"Total cost: {totalCost} vs lowest: {lowestTotalCost}\tTotal steps: {totalSteps} vs lowest: {lowestTotalSteps}")
            lowestTotalCost = totalCost
            lowestTotalSteps = totalSteps

            bestRobotOrder = bestRobotOrder[0:dontMove]

            for robotNumber in order:
                bestRobotOrder.append(copy(robots[robotNumber]))
            
            # print(bestRobotOrder)


    bestRobotOrder

    if verbose:

        print(f"Duration: {time() - start:.3f} seconds")

        for robot in bestRobotOrder:
            print(robot)

        print(f"Total steps:\t{lowestTotalSteps}")
        print(f"Total cost: \t{lowestTotalCost}")


        print(bestRobotOrder)

    if lowestTotalCost == float("inf"): return -1

    return bestRobotOrder







grid = Grid(10, 10)

grid.addObstacle((1,1))
grid.addObstacle((1,2))
grid.addObstacle((1,3))
grid.addObstacle((1,4))
grid.addObstacle((2,5))
grid.addObstacle((3,0))
grid.addObstacle((3,7))
grid.addObstacle((4,1))
grid.addObstacle((4,3))
grid.addObstacle((4,4))
grid.addObstacle((4,7))
grid.addObstacle((5,1))
grid.addObstacle((5,6))
grid.addObstacle((6,3))
grid.addObstacle((6,4))
grid.addObstacle((6,7))
grid.addObstacle((7,7))



robots : list[Robot] = []

robots.append(Robot("Robot(1)", 1, grid.getNode((0,0)), grid.getNode((2,3))))
robots.append(Robot("Robot(2)", 4, grid.getNode((7,6)), grid.getNode((2,0))))
robots.append(Robot("Robot(3)", 7, grid.getNode((1,5)), grid.getNode((3,2))))
robots.append(Robot("Robot(4)", 3, grid.getNode((6,6)), grid.getNode((4,0))))
robots.append(Robot("Robot(5)", 2, grid.getNode((1,0)), grid.getNode((5,0))))
robots.append(Robot("Robot(6)", 8, grid.getNode((3,1)), grid.getNode((5,4))))
robots.append(Robot("Robot(7)", 7, grid.getNode((7,2))))
robots.append(Robot("Robot(8)", 7, grid.getNode((2,6)), grid.getNode((5,3))))
# robots.append(Robot("Robot(9)", 7, grid.getNode((7,3)), grid.getNode((4,5))))
# robots.append(Robot("Robot(7)", 7, grid.getNode((5,0))))


start = time()
robots = calculateBestPaths(robots, True)
print(f"Duration: {time() - start:.3f} seconds")
# # print(robots)











# # robots.append(Robot("Robot(7)", 7, grid.getNode((7,2))))
# # robots.append(Robot("Robot(3)", 7, grid.getNode((1,5)), grid.getNode((3,2))))
# robots.append(Robot("Robot(6)", 8, grid.getNode((3,1)), grid.getNode((5,4))))
# robots.append(Robot("Robot(4)", 3, grid.getNode((6,6)), grid.getNode((4,0))))
# # robots.append(Robot("Robot(2)", 4, grid.getNode((7,6)), grid.getNode((2,0))))
# # robots.append(Robot("Robot(1)", 1, grid.getNode((0,0)), grid.getNode((2,3))))
# robots.append(Robot("Robot(5)", 2, grid.getNode((1,0)), grid.getNode((5,0))))
# robots.append(Robot("Robot(8)", 7, grid.getNode((2,6)), grid.getNode((5,3))))


# for robot in robots:
#     if robot.goal == None: 
#         robot.path = [robot.location]
#         robots.remove(robot)
#         robots = [robot] + robots

# allPaths : list[list[Node]] = []
# totalSteps = 0
# totalCost = 0

# for robot in robots:
#     # print("--------------------------------------")
#     if robot.goal != None:
#         cost = robot.findPath(robots)
#         totalCost += cost

#         steps = (len(robot.path) - 1)
#         totalSteps += steps
#     else:
#         robot.path = [robot.location]
#         cost = 0
#         steps = 0

#     allPaths.append(robot.path)

#     if cost != float("inf"):
#         print(f"{robot.name:8s} cost{cost:-4d} | steps{steps:-3d}: {robot.path}")
#     else:
#         print(f"{robot.name:8s} cost{cost} | steps{steps:-3d}: {robot.path}")


# print(f"Total steps:\t{totalSteps}")
# print(f"Total cost: \t{totalCost}")
# # print(robots)




# print(allPaths)

# steps = list(zip(*allPaths))

# steps = np.array(allPaths).T

# longest = 0
# for robot in robots:
#     longest = max(longest, len(robot.path))

# error = False

# for robot in robots:
#     for otherBot in robots:
#         if otherBot != robot:
#             length = len(otherBot.path)

#             for i, node in enumerate(robot.path):
#                 if  length <= i and otherBot.path[-1] == node:
#                     print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i-1} at node {node}")
#                     error = True
#                     break
#                 if length > i and otherBot.path[i] == node:
#                     print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i} at node {node}")
#                     error = True
#                     break
#                 if length > i + 1 and otherBot.path[i + 1] == node:
#                     print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i+1} at node {node}")
#                     error = True
#                     break
#         if error: break
#     if error: break



# steps : list[list[Node]] = []
# for i in range(longest):
#     steps.append([])
#     for robot in robots:
#         if len(robot.path) > i: steps[i].append(robot.path[i])

# for i, step in enumerate(steps):
#     stepList = []
#     if i > 0: 


    # print(F"Step {i}: {step}")








#--------------------------------------------------------------
# old test code



# positions = [(1,4),(1,3),(1,3),(0,3),(0,2),(1,2)]
# robots.append(Robot(positions, grid))



# for node in grid.nodes:
#     print(node.pos)

# print("Links of (3, 1): ")

# for link in grid.getNode((3, 1)).links:
#     print(link.pos)


#----------------

# start = time()


# order = [*range(len(robots))]

# allOrders = list(permutations(order))

# lowestTotalCost = float("inf")
# lowestTotalSteps = float("inf")
# bestRobotOrder = []

# numberOfOrders = len(allOrders)

# for i, order in enumerate(allOrders):
#     # print(f"Calculating: {i+1} of {numberOfOrders}")

#     for robot in robots:
#         robot.path = []

#     totalCost = 0
#     totalSteps = 0
#     for robotNumber in order:

#         cost = robots[robotNumber].findPath(robots)
#         totalCost += cost
#         totalSteps += len(robots[robotNumber].path) - 1
    
#     # print(allcosts)
#     # print(f"Totalcost: {totalCost}\tLowestcost: {lowestTotalCost}")

#     if totalCost < lowestTotalCost or (totalCost == lowestTotalCost and totalSteps < lowestTotalSteps):
#         lowestTotalCost = totalCost
#         lowestTotalSteps = lowestTotalSteps

#         bestRobotOrder = []
#         for robotNumber in order:
#             bestRobotOrder.append(copy(robots[robotNumber]))


# end = time()

# print(f"Duration: {end - start:.2f} seconds")


# steps = 0
# totalCost = 0
# for robot in bestRobotOrder:
#     cost = (len(robot.path) - 1) * robot.cost
#     totalCost += cost
#     steps += (len(robot.path) - 1)
#     print(robot.name + " " + str(cost) + ": " + str(robot.path))

# print(f"Total steps:\t{steps}")
# print(f"Total cost: \t{totalCost}")


# print(bestRobotOrder)




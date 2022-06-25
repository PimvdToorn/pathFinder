# from re import T
from genericpath import exists
from time import time
from itertools import permutations
from copy import copy, deepcopy


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
        if self.previous == None:
            return str(self.node) + " Start"
            
        return str(self.node) + " d:" + str(self.dist) + " p:" + str(self.previous.node) + " v:" + str(self.visited)

    def __repr__(self):
        return str(self.node) + " d:" + str(self.dist)




class Robot:

    def __init__(self, name : str, cost : int, location : Node, goal : Node = None):
        self.name = name
        self.cost = cost
        self.location = location
        self.goal = goal
        self.path : list[tuple] = []
        if goal == None: self.path.append(location)
        # self.path : list[tuple] = [location]
    

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





def findPath(robots : list[Robot], start : Node, goal : Node):

    # print("--------------------------------------------------------------------------------------------")
    dijkNodes = [dijkstraNode(start, 0, None)]

    while True:

        current = dijkstraNode(None, float("inf"), None)

        # Get the node with the shortest distance, that hasn't been visited yet (fully checked)
        for dNode in dijkNodes:
            if not dNode.visited and dNode.dist < current.dist:
                current = dNode

        # All visited, no options left
        if current.node == None: return []

        # print(dijkNodes)
        # for node in dijkNodes:
        #     print(node)

        # print("Visiting: " + str(current))
        for link in current.node.links:
            # print("New node at:" + str(link))
            next = dijkstraNode(link, current.dist+1, current)


            # Check if any robot will be at the location the step before, during or after this robot would move there, if so add 1 to distance and check again
            # (wait 1 more at current location before moving to link)
            intersect = True
            # end = False

            while intersect:
                intersect = False

                for robot in robots:

                    # Option 1 -------------------------------------------------------------------------------------------------

                    #    robot has steps then      and       r will be there          or     r just left                 or     r has step after             and   will be there just after 
                    if len(robot.path) > next.dist and (robot.path[next.dist] == next.node or robot.path[next.dist-1] == next.node or (len(robot.path) > next.dist + 1 and robot.path[next.dist+1] == next.node)):
                        next.dist += 1
                        # print("Distance upped to: " + str(next.dist))
                        intersect = True
                        break

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

                    # if robot.path != []:
                    #     length = len(robot.path) - 1
                    #     startIndex = length - next.dist - 1
                    #     if startIndex < 0: startIndex = 0
                    #     endIndex = length - current.dist

                    #     if length + 1 > 0 and robot.path[-1] == next.node and length <= next.dist:
                    #         end = True
                    #         break

                    #     elif endIndex > 0:

                    #         try:
                    #             lastIndex = length - robot.path[::-1].index(current.node, startIndex, endIndex)
                    #         except:
                    #             lastIndex = -1


                    #         else:
                    #             next.dist = lastIndex + 2
                    #             intersect = True
                    #             break

                    # ----------------------------------------------------------------------------------------------------------

            # if not end:    

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

                        # print("\t\t" + str(robot))
                        # print("\t\tnext.dist:" + str(next.dist))
                        # print("\t\tcurr.dist:" + str(current.dist))
                        # print("\t\tlength:" + str(length))
                        # print("\t\tstr:" + str(startIndex))
                        # print("\t\tend:" + str(endIndex))

                        if length + 1 > 0 and robot.path[-1] == next.node and length <= next.dist: return []

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
            linksToRemove : list[dijkstraNode] = []
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
                            startIndex = length - link.dist - 1
                            if startIndex < 0: startIndex = 0
                            endIndex = length - current.dist

                            # if link.node == goal:
                            #     print("\t\t" + str(robot))
                            #     print("\t\tstr:" + str(startIndex))
                            #     print("\t\tend:" + str(endIndex))

                            if length + 1 > 0 and robot.path[-1] == current.node and length <= link.dist:
                                linksToRemove.append(link)
                                linksToCurrent.remove(link)
                                # dijkNodes.remove(link)
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
                                    
                                    # print("in da way: " + str(current.node) + str(robot.path[lastIndex]) + " at" + str(lastIndex))
                                    # sleep(1)

                                    # if not (intersect and dijkNodes[-1].node == current.node and dijkNodes[-1].dist == lastIndex + 1):
                                    if not intersect or (len(dNodesToAdd) > 0 and dNodesToAdd[-1].dist != lastIndex + 2):
                                        # dijkNodes.append(dijkstraNode(current.node, lastIndex + 1, current.previous))
                                        # dijkNodes = [dijkstraNode(current.node, lastIndex + 2, current.previous)] + dijkNodes
                                        dNodesToAdd.append(dijkstraNode(current.node, lastIndex + 2, current.previous))

                                        # print("New node:" + str(dijkNodes[0]))
                                    # else:
                                        # r.pop(-1)
                                        # print("not new" + str(dijkNodes[0]))

                                    # if current.previous != None:
                                    #     print("in da way: " + str(current.node) + " at" + str(lastIndex) + "\tPr:" + str(current.previous.node))

                                    # print("Removing: " + str(link))
                                    linksToRemove.append(link)
                                    # dijkNodes.remove(link)
                                    
                                    # i -= 1
                                    # if i < 0: i = 0

                                    # link = dijkstraNode(current.node, lastIndex + 1, current.previous)
                                    noOfLinks -= 1

                                    # previous = current.previous

                                    # if not current.visited:
                                    #     dijkNodes.remove(current)

                                    # current = previous
                                    
                                    intersect = True
                                    break

                        # Don't step onto a starting point of a robot too quickly  
                        elif link.node == robot.location and link.dist < 2:
                            link.dist = 2

                # if intersect: break
            
            if intersect:
                for link in linksToRemove:
                    dijkNodes.remove(link)
                
                for dNode in dNodesToAdd:
                    dijkNodes.append(dNode)

                previous = current.previous

                if noOfLinks == 0:
                    # print(linksToCurrent)
                    # print("removed: " + str(current))
                    dijkNodes.remove(current)
                    
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
        for dNode in dijkNodes[::-1]:
            if dNode.node == goal:
                # print("Goal----------------------------")
                reachedGoal = True
                break
        
        if reachedGoal: break


    # print("dijkNodes:")
    # for node in dijkNodes:
    #     print(node)

    # Change the dijkNodes to a list of nodes:

    current = None
    for dNode in dijkNodes[::-1]:
        if dNode.node == goal:
            current = dNode
            break

    # print(current.previous)
    # print(current)
    path = [current.node]

    while True:

        # print("Current:" + str(current.node) + str(current.dist))
        # print("Previous:" + str(current.previous.node) + str(current.previous.dist))

        # Stay the distance amount of steps at the previous, to wait on any other robot
        # print(current.previous.node)
        # print(current.previous.dist)
        path = (current.dist - current.previous.dist) * [current.previous.node] + path
        # print(path)
        if path[0] == start: break

        current = current.previous

    # print(path)
    return path



def calculateBestPaths(robots : list[Robot], verbose : bool = False):
    if verbose: start = time()


    dontMove = 0
    for robot in robots:
        if robot.goal == None: 
            robot.path = [robot.location]
            dontMove += 1
            robots.remove(robot)
            robots = [robot] + robots




    order = [*range(dontMove, len(robots))]

    allOrders = list(permutations(order))
    if verbose: numberOfOrders = len(allOrders)


    bestRobotOrder : list[Robot] = []
    for i in range(dontMove):
        bestRobotOrder.append(copy(robots[i]))

    lowestTotalCost = float("inf")
    lowestTotalSteps = float("inf")


    for i, order in enumerate(allOrders):

        # if verbose: print(f"Calculating: {i+1} of {numberOfOrders}")

        for robotNumber in order:
            robots[robotNumber].path = []

        totalCost = 0
        totalSteps = 0
        for robotNumber in order:

            cost = robots[robotNumber].findPath(robots)
            totalCost += cost
            if totalCost > lowestTotalCost: break
            totalSteps += len(robots[robotNumber].path) - 1
        

        if totalCost < lowestTotalCost or (totalCost == lowestTotalCost and totalSteps < lowestTotalSteps):

            # for robot in robots:

            lowestTotalCost = totalCost
            lowestTotalSteps = totalSteps

            bestRobotOrder = bestRobotOrder[0:dontMove]

            for robotNumber in order:
                bestRobotOrder.append(copy(robots[robotNumber]))
            
            print(bestRobotOrder)


    bestRobotOrder

    if verbose:
        end = time()
        print(f"Duration: {end - start:.2f} seconds")


        for robot in bestRobotOrder:
            cost = (len(robot.path) - 1) * robot.cost
            # totalCost += cost
            # steps += (len(robot.path) - 1)
            print(robot.name + " " + str(cost) + ": " + str(robot.path))

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
# robots.append(Robot("Robot(7)", 7, grid.getNode((5,0))))


robots = calculateBestPaths(robots, True)


# for robot in robots:
#     # print("--------------------------------------")
#     robot.findPath(robots)

# steps = 0
# totalCost = 0
# for robot in robots:
#     cost = (len(robot.path) - 1) * robot.cost
#     totalCost += cost
#     steps += (len(robot.path) - 1)
#     print(robot.name + " " + str(cost) + ": " + str(robot.path))

# print(f"Total steps:\t{steps}")
# print(f"Total cost: \t{totalCost}")
# print(robots)











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




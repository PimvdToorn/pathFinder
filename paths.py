from __future__ import annotations      # type suggestion of enclosing class
from timeit import default_timer as time
from itertools import permutations
from copy import copy

from numpy import true_divide

# The nodes for in the grid
class Node:
    def __init__(self, pos : tuple[int, int]):
        self.pos = pos
        self.links : list[Node] = []

    def __str__(self) -> str:
        return f"({self.pos[0]},{self.pos[1]})"
        
    def __repr__(self):
        return str(self)


    def addLink(self, linkNode : Node):
        for node in self.links:
            if node == linkNode: return

        # If not already linked, add the node
        self.links.append(linkNode)
        linkNode.addLink(self)




class Grid:
    def __init__(self, x : int, y : int):   

        self.nodes : list[Node] = []

        for i in range(x):

            for j in range(y):
                self.nodes.append(Node((i, j)))
                
                if j > 0: self.nodes[-1].addLink(self.nodes[-2])    # Add previous as link
                if i > 0: self.nodes[-1].addLink(self.nodes[-1-y])  # Add the "top" one from the previous row

                # ( 1 <- 2 <- 3 <- 4)
                # ( ^    ^    ^    ^)
                # ( 5 <- 6 <- 7 <- 8)
                
                # Nodes get doubly-linked:
                # When a node adds a link, the linked node also adds the node as a link (except if it already has the node as a link)


    def addObstacle(self, pos : tuple[int, int]):
        for node in self.nodes:
            if node.pos == pos:

                for link in node.links:         # Remove the link to the obstacle from the linked nodes
                    link.links.remove(node)

                self.nodes.remove(node)
                break    


    def getNode(self, pos : tuple[int, int]):
        for node in self.nodes:
            if node.pos == pos: return node

        return None




class dijkstraNode:
    def __init__(self, node : Node, dist : int, previous : dijkstraNode):
        self.node = node
        self.dist = dist
        self.previous = previous
        self.visited = False

    def __str__(self) -> str:
        if self.dist == 0:          return f"{self.node} Start"
        elif self.previous == None: return f"{self.node} d:{self.dist} p:None v:{self.visited}" 
        else:                       return f"{self.node} d:{self.dist} p:{self.previous.node} v:{self.visited}" 

    def __repr__(self):
        return f"{self.node} d:{self.dist}"




class Robot:

    def __init__(self, name : str, cost : int, location : Node, goal : Node = None):
        self.name = name
        self.cost = cost
        self.location = location
        self.goal = goal
        self.path : list[Node] = []

        self.pathCost = float("inf")
        self.steps = 0
    
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

    def setPositions(self, grid : Grid, positions : list[tuple[int,int]]):
        for pos in positions:
            self.path.append(grid.getNode(pos))

    def getPositions(self):
        positions = []
        for node in self.path:
            positions.append(node.pos)
        return positions

    def setLocationAndGoal(self, grid : Grid, location : tuple[int,int], goal : tuple[int,int] = None):
        self.path = []
        self.location = grid.getNode(location)
        self.goal = grid.getNode(goal)
        
    def setGoal(self, grid : Grid, goal : tuple[int,int]):
        self.path = []
        self.goal = grid.getNode(goal)

    def findPath(self, robots : list[Robot]):
        self.path = []
        self.path = findPath(robots, self.location, self.goal)

        if self.path == []: 
            self.steps = 0
            self.pathCost = float("inf")
        else:
            self.steps = (len(self.path) - 1)
            self.pathCost =  self.steps * self.cost

        return self.pathCost
        # Returns the total cost of the path, inf if the path is not possible


    # def getPathCost(self):
    #     if self.path == []: 
    #         return float("inf")
    #     return (len(self.path) - 1) * self.cost






# if a node can't be reached in time from its previous, it gets unlinked. When it gets visited, it will look for a new previous
def newPrevious(robots : list[Robot], dijkNodes : list[dijkstraNode], unlinkedNode : dijkstraNode):

    for dNode in dijkNodes:
        if dNode.node in unlinkedNode.node.links:

            # if it's not start and (it doesn't a new previous yet or (the node is the previous but with greater distance, to prevent it linking to the old impossible node or it's not the previous and the distance is lower) and The node's previous is not this node
            if dNode.dist != 0 and (unlinkedNode.previous == None or (((unlinkedNode.previous == dNode and dNode.dist > unlinkedNode.previous.dist) or (unlinkedNode.previous != dNode and dNode.dist < unlinkedNode.previous.dist)) and dNode.previous != unlinkedNode)):
                unlinkedNode.previous = dNode
                nodeCollide(robots, unlinkedNode)

                # print(f"New prev {dNode}\tfor {unlinkedNode}")
    
    if unlinkedNode.previous == None or unlinkedNode.dist == float("inf"): return False
    return True



def nodeCollide(robots : list[Robot], next : dijkstraNode):
    # Check if any robot will be at the location the step before, during or after this robot would move there, or a robot ends here before this robot could leave

    next.dist = max(next.dist, next.previous.dist + 1)

    # print(f"Checking collision with: {next}")
    intersect = True
    while intersect:
        intersect = False

        for robot in robots:


            if robot.path != []:
                
                # index gets cheched in reverse to get the last index at which the robot would be at this position
                
                # steps = 7 - 1 = 6
                # endIndex = steps - next.dist + 1 = 6 - 2 + 2 = 6 (excluded)
                # startIndex = steps - next.dist - 1 = 6 - 2 - 1 = 3 (included)

                #   end              start
                #    V                 V
                #   6(    5     4     3)    2     1     0
# Other robot   # (0,0) (0,0) (1,0) (2,0) (2,1) (2,2) (2,3)
# This robot    # (1,0) (2,0) (2,1)
                #   0     1     2
                #               ^
                #              next

                steps = len(robot.path) - 1
                endIndex = steps - next.dist + 2

                # if a robot ends here before this robot could leave
                if robot.path[-1] == next.node and steps <= next.dist:
                    next.dist = float("inf")
                    return

                # if endIndex is 1 or below, it would've checked the last and this move will be after the other robot has ended
                elif endIndex > 1:
                    startIndex = steps - next.dist - 1

                    # Already checked the last one at index 0
                    startIndex = max(1, startIndex)

                    try:
                        lastIndex = steps - robot.path[::-1].index(next.node, startIndex, endIndex)
                    except:
                        lastIndex = -1
                    else:
                        next.dist = lastIndex + 2
                        intersect = True
                        break


                # Check if the robot is not in the way before it moves to the next node

                if not intersect:
                    # end              start
                    #  V                 V
                    # 7(    6     5     4)    3     2     1     0
        # Other robot   # (0,0) (0,0) (1,0) (2,0) (2,1) (2,2) (2,3)
        # This robot    # (1,0) (2,0) (2,1)
                    #       0     1     2
                    #             ^
                    #           prev

                    steps = len(robot.path) - 1
                    
                    endIndex = steps - next.previous.dist + 1


                    if robot.path[-1] == next.previous.node and steps <= next.dist:
                        # The other robot ends here
                        next.dist = float("inf")
                        break

                    elif endIndex > 1:
                        startIndex = steps - next.dist
                        if startIndex < 1: startIndex = 1

                        try:
                            lastIndex = steps - robot.path[::-1].index(next.previous.node, startIndex, endIndex)
                        except:
                            lastIndex = -1
                        else:
                            next.dist = float("inf")

                            break


    return next



def findPath(robots : list[Robot], start : Node, goal : Node):

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
            if noPrevious: noPrevious = not newPrevious(robots, dijkNodes, current)
            if noPrevious: dijkNodes.remove(current)
       


        for link in current.node.links:
            # if current.previous == None or link != current.previous.node or (link == current.previous.node and current.previous.dist < current.dist - 1): 
            if current.previous == None or link != current.previous.node: 

                next = dijkstraNode(link, current.dist+1, current)

                nodeCollide(robots, next)
                
                
                if next.dist != float("inf") and (next.dist == 0 or next.previous != None):   

                    if next.node == goal:
                        # print("Goal check")
                        intersect = True
                        while intersect:
                            intersect = False
                            for robot in robots:
                                if robot.path != []:
                                    steps = len(robot.path) - 1
                                    startIndex = 0
                                    endIndex = steps - next.dist + 2

                                    if endIndex >= 0:
                                        try:
                                            lastIndex = steps - robot.path[::-1].index(next.node, startIndex, endIndex)
                                        except:
                                            # print("\t\tNot in the way")
                                            lastIndex = -1

                                        else:
                                            next.dist = lastIndex + 2
                                            intersect = True

                        if reachedGoal:
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
                                if exists: 
                                    exists = False
                                    break
                                exists = True
                     
                        if not exists:
                            # print("Added: " + str(next))
                            dijkNodes.append(next)


        current.visited = True


        intersect = True
        while intersect:
            intersect = False


            # print("C:" + str(current))
            dNodesToAdd : list[dijkstraNode] = []

            for link in dijkNodes:

                if link.previous == current:

                    for robot in robots:
                        if robot.path != []:
                            steps = len(robot.path) - 1
                            
                            endIndex = steps - current.dist + 1


                            if robot.path[-1] == current.node and steps <= link.dist:
                                link.previous = None
                                intersect = True
                                break

                            elif endIndex >= 0:
                                startIndex = steps - link.dist
                                startIndex = max(1, startIndex)

                                try:
                                    lastIndex = steps - robot.path[::-1].index(current.node, startIndex, endIndex)
                                except:
                                    lastIndex = -1


                                else:

                                    if not intersect or (len(dNodesToAdd) > 0 and dNodesToAdd[-1].dist != lastIndex + 2):
                                        if current.previous != None:
                                            dNodesToAdd.append(dijkstraNode(current.node, lastIndex + 2, current.previous))

                                    link.previous = None
                  
                                    intersect = True
                                    break

                        # Don't step onto a starting point of a robot with no calculated path too quickly  
                        if link.dist < 2 and link.node == robot.location:
                            link.dist = 2

            
            if intersect:

                for dNode in dNodesToAdd:
                    # print(f"Added: {dNode}")
                    dijkNodes.append(dNode)

                previous = current.previous


                

                if previous == None: break
                current = previous


        reachedGoal = False

        for dNode in dijkNodes[::-1]:
            if dNode.node == goal and dNode.previous != None:
                reachedGoal = True
                break

        if reachedGoal: 
            break



    # Change the dijkNodes to a list of nodes:

    current = None
    for dNode in dijkNodes[::-1]:
        if dNode.node == goal:
            current = dNode
            break


    path = [current.node]

    while True:

        # Stay the distance amount of steps at the previous, to wait on any other robot
        path = (current.dist - current.previous.dist) * [current.previous.node] + path

        if path[0] == start: break

        current = current.previous

    return path





def checkPathsPossibility(robots : list[Robot]):
    for robot in robots:
        for otherBot in robots:
            if otherBot != robot:
                length = len(otherBot.path)
                if length == 0: return False

                for i, node in enumerate(robot.path):
                    if length <= i and otherBot.path[-1] == node:
                        print(f"Collision {robot.name} step:{i} with {otherBot.name} endpoint at node {node}-------------------------------------------")
                        return False
                    if length > i - 1 and otherBot.path[i - 1] == node:
                        print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i-1} at node {node}-------------------------------------------")
                        return False
                    if length > i and otherBot.path[i] == node:
                        print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i} at node {node}-------------------------------------------")
                        return False
                    if length > i + 1 and otherBot.path[i + 1] == node:
                        print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i+1} at node {node}-------------------------------------------")
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
    orderLength = len(order)

    allOrders = list(permutations(order))

    if count: numberOfOrders = len(allOrders)
    elif verbose: print(f"Number of calculations to do: {len(allOrders)}")


    lowestTotalCost = float("inf")
    lowestTotalSteps = float("inf")

    impossibleOrders = []
    lastOrder = allOrders[0][::-1]

    for i, order in enumerate(allOrders):

        if count: print(f"Calculating: {i+1} of {numberOfOrders}\t{order}")

        impossible = False

        for impOrder in impossibleOrders:

            impossible = True
            for i, robotNumber in enumerate(impOrder):
                if order[i] > robotNumber:
                    impossibleOrders.remove(impOrder)
                    # print(f"Removed impossible order: {impOrder}")
                    impossible = False
                    break
                if order[i] != robotNumber:
                    impossible = False
                    break
            if impossible: 
                if count: print(f"Impossible order: {order} starts with impossible sublist: {impOrder}")
                break
                

        if not impossible:
            totalCost = 0
            totalSteps = 0
            currentCalculatedOrder = []

            reducedOrder = ()
            # print(f"order: {order}")
            for i, robotNumber in enumerate(order):
                if lastOrder[i] != robotNumber:
                    reducedOrder = order[i:]
                    currentCalculatedOrder = list(order[:i])
                    # print(f"Reduced to: {reducedOrder}")
                    break
                totalCost += robots[robotNumber].pathCost
                totalSteps += robots[robotNumber].steps

            lastOrder = order
            

            for robotNumber in reducedOrder:
                robots[robotNumber].path = []

            
            for robotNumber in reducedOrder:

                totalCost += robots[robotNumber].findPath(robots)
                currentCalculatedOrder.append(robotNumber)

                if totalCost > lowestTotalCost or totalCost == float("inf"): 
                    if len(currentCalculatedOrder) < orderLength:
                        # print(f"Found impossible order: {currentCalculatedOrder}")
                        impossibleOrders.append(currentCalculatedOrder)
                    break

                totalSteps += len(robots[robotNumber].path) - 1
            

            if (totalCost < lowestTotalCost or (totalCost == lowestTotalCost and totalSteps < lowestTotalSteps)) and totalCost != float("inf") and checkPathsPossibility(robots):
                lowestTotalCost = totalCost
                lowestTotalSteps = totalSteps

                bestRobotOrder = bestRobotOrder[0:dontMove]

                for robotNumber in range(dontMove, len(robots)):
                    bestRobotOrder.append(copy(robots[robotNumber]))
                
                # print(bestRobotOrder)



    if verbose:

        print(f"Duration: {time() - start:.3f} seconds")

        for robot in bestRobotOrder:
            print(robot)

        print(f"Total steps:\t{lowestTotalSteps}")
        print(f"Total cost: \t{lowestTotalCost}")


        print(bestRobotOrder)

    if lowestTotalCost == float("inf"): return -1

    return bestRobotOrder







grid = Grid(11, 11)

obstacles = [(2,2),(3,2),(4,2),(5,2),(6,2),(7,2),(8,2),
             (2,4),(3,4),(4,4),(5,4),(6,4),(7,4),(8,4),
             (2,6),(2,7),(2,8),
             (4,6),(4,7),(4,8),
             (6,6),(6,7),(6,8),
             (8,6),(8,7),(8,8)]

for obstacle in obstacles:
    grid.addObstacle(obstacle)



robots : list[Robot] = []

robots.append(Robot("Robot(1)", 1, grid.getNode((0,0)), grid.getNode((3,3))))
robots.append(Robot("Robot(2)", 1, grid.getNode((0,10)), grid.getNode((5,0))))
robots.append(Robot("Robot(3)", 6, grid.getNode((10,0)), grid.getNode((3,7))))
robots.append(Robot("Robot(4)", 18, grid.getNode((10,10)), grid.getNode((5,3))))
# robots.append(Robot("Robot(5)", 2, grid.getNode((1,0)), grid.getNode((7,7))))
# robots.append(Robot("Robot(6)", 8, grid.getNode((6,5)), grid.getNode((4,3))))
# robots.append(Robot("Robot(7)", 7, grid.getNode((7,3)), grid.getNode((9,3))))
# robots.append(Robot("Robot(8)", 7, grid.getNode((3,6)), grid.getNode((7,5))))
# robots.append(Robot("Robot(9)", 7, grid.getNode((7,3)), grid.getNode((4,5))))
# robots.append(Robot("Robot(7)", 7, grid.getNode((5,0))))


# start = time()
robots = calculateBestPaths(robots, True)
# print(f"Duration: {time() - start:.3f} seconds")
# print(robots)









# # # robots.append(Robot("Robot(7)", 7, grid.getNode((7,2))))
# # # robots.append(Robot("Robot(3)", 7, grid.getNode((1,5)), grid.getNode((3,2))))
# # robots.append(Robot("Robot(6)", 8, grid.getNode((3,1)), grid.getNode((5,4))))
# # robots.append(Robot("Robot(4)", 3, grid.getNode((6,6)), grid.getNode((4,0))))
# # # robots.append(Robot("Robot(2)", 4, grid.getNode((7,6)), grid.getNode((2,0))))
# # # robots.append(Robot("Robot(1)", 1, grid.getNode((0,0)), grid.getNode((2,3))))
# # robots.append(Robot("Robot(5)", 2, grid.getNode((1,0)), grid.getNode((5,0))))
# # robots.append(Robot("Robot(8)", 7, grid.getNode((2,6)), grid.getNode((5,3))))


# # for robot in robots:
# #     if robot.goal == None: 
# #         robot.path = [robot.location]
# #         robots.remove(robot)
# #         robots = [robot] + robots

# # allPaths : list[list[Node]] = []
# # totalSteps = 0
# # totalCost = 0

# # for robot in robots:
# #     # print("--------------------------------------")
# #     if robot.goal != None:
# #         cost = robot.findPath(robots)
# #         totalCost += cost

# #         steps = (len(robot.path) - 1)
# #         totalSteps += steps
# #     else:
# #         robot.path = [robot.location]
# #         cost = 0
# #         steps = 0

# #     allPaths.append(robot.path)

# #     if cost != float("inf"):
# #         print(f"{robot.name:8s} cost{cost:-4d} | steps{steps:-3d}: {robot.path}")
# #     else:
# #         print(f"{robot.name:8s} cost{cost} | steps{steps:-3d}: {robot.path}")


# # print(f"Total steps:\t{totalSteps}")
# # print(f"Total cost: \t{totalCost}")
# # # print(robots)




# # print(allPaths)

# # steps = list(zip(*allPaths))

# # steps = np.array(allPaths).T

# # longest = 0
# # for robot in robots:
# #     longest = max(longest, len(robot.path))

# # error = False

# # for robot in robots:
# #     for otherBot in robots:
# #         if otherBot != robot:
# #             length = len(otherBot.path)

# #             for i, node in enumerate(robot.path):
# #                 if  length <= i and otherBot.path[-1] == node:
# #                     print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i-1} at node {node}")
# #                     error = True
# #                     break
# #                 if length > i and otherBot.path[i] == node:
# #                     print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i} at node {node}")
# #                     error = True
# #                     break
# #                 if length > i + 1 and otherBot.path[i + 1] == node:
# #                     print(f"Collision {robot.name} step:{i} with {otherBot.name} step:{i+1} at node {node}")
# #                     error = True
# #                     break
# #         if error: break
# #     if error: break



# # steps : list[list[Node]] = []
# # for i in range(longest):
# #     steps.append([])
# #     for robot in robots:
# #         if len(robot.path) > i: steps[i].append(robot.path[i])

# # for i, step in enumerate(steps):
# #     stepList = []
# #     if i > 0: 


#     # print(F"Step {i}: {step}")








# #--------------------------------------------------------------
# # old test code



# # positions = [(1,4),(1,3),(1,3),(0,3),(0,2),(1,2)]
# # robots.append(Robot(positions, grid))



# # for node in grid.nodes:
# #     print(node.pos)

# # print("Links of (3, 1): ")

# # for link in grid.getNode((3, 1)).links:
# #     print(link.pos)


# #----------------

# # start = time()


# # order = [*range(len(robots))]

# # allOrders = list(permutations(order))

# # lowestTotalCost = float("inf")
# # lowestTotalSteps = float("inf")
# # bestRobotOrder = []

# # numberOfOrders = len(allOrders)

# # for i, order in enumerate(allOrders):
# #     # print(f"Calculating: {i+1} of {numberOfOrders}")

# #     for robot in robots:
# #         robot.path = []

# #     totalCost = 0
# #     totalSteps = 0
# #     for robotNumber in order:

# #         cost = robots[robotNumber].findPath(robots)
# #         totalCost += cost
# #         totalSteps += len(robots[robotNumber].path) - 1
    
# #     # print(allcosts)
# #     # print(f"Totalcost: {totalCost}\tLowestcost: {lowestTotalCost}")

# #     if totalCost < lowestTotalCost or (totalCost == lowestTotalCost and totalSteps < lowestTotalSteps):
# #         lowestTotalCost = totalCost
# #         lowestTotalSteps = lowestTotalSteps

# #         bestRobotOrder = []
# #         for robotNumber in order:
# #             bestRobotOrder.append(copy(robots[robotNumber]))


# # end = time()

# # print(f"Duration: {end - start:.2f} seconds")


# # steps = 0
# # totalCost = 0
# # for robot in bestRobotOrder:
# #     cost = (len(robot.path) - 1) * robot.cost
# #     totalCost += cost
# #     steps += (len(robot.path) - 1)
# #     print(robot.name + " " + str(cost) + ": " + str(robot.path))

# # print(f"Total steps:\t{steps}")
# # print(f"Total cost: \t{totalCost}")


# # print(bestRobotOrder)




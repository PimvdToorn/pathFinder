from re import T
from time import sleep


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

    def __init__(self, path : list, grid : Grid = None):
        if grid == None:
            self.path = path
            return

        self.path = []
        for pos in path:
            self.path.append(grid.getNode(pos))
    

    def __repr__(self):
        return str(self.path)





def findPath(robots : list[Robot],start : Node, goal : Node):

    dijkNodes = [dijkstraNode(start, 0, None)]

    while True:

        current = dijkstraNode(None, 60000, None)

        # Get the node with the shortest distance, that hasn't been visited yet (fully checked)
        for dNode in dijkNodes:
            if dNode.visited == False and dNode.dist < current.dist:
                current = dNode


        for link in current.node.links:
            next = dijkstraNode(link, current.dist+1, current)


            # Check if any robot will be at the location the step before, during or after this robot would move there, if so add 1 to distance and check again
            # (wait 1 more at current location before moving to link)
            intersect = True
            while intersect:
                intersect = False

                for robot in robots:
                    #    robot has steps then      and       r will be there          or     r just left                 or     r has step after             and   will be there just after 
                    if len(robot.path) > next.dist and (robot.path[next.dist] == link or robot.path[next.dist-1] == link or (len(robot.path) > next.dist + 1 and robot.path[next.dist+1] == link)):
                        next.dist += 1
                        intersect = True
                        break

                    

            if link == goal:
                dijkNodes.append(next)
                break

            
            # Check if any robot will end at the location before this robot would be there
            intersect = False
            for robot in robots:
                if len(robot.path) > 0 and robot.path[-1] == link and len(robot.path) <= next.dist:
                    intersect = True
                    break


            for dNode in dijkNodes:

                if next.node == dNode.node:
                    if next.dist < dNode.dist:
                        dNode = next

                    intersect = True
                    break

                    
            if not intersect:
                dijkNodes.append(next)


        current.visited = True


        intersect = True
        while intersect:
            intersect = False

            noOfLinks = 0
            for link in dijkNodes:
                

                if link.previous == current:
                    noOfLinks += 1

                    for robot in robots:
                        try:
                            length = len(robot.path) - 1
                            lastIndex = length - robot.path[::-1].index(current.node, length - link.dist - 1, length - current.dist)
                        except:
                            lastIndex = -1


                        else:
                            
                            print("in da way: " + str(current.node) + str(robot.path[lastIndex]) + " at" + str(lastIndex))
                            # sleep(1)

                            # if not (intersect and dijkNodes[-1].node == current.node and dijkNodes[-1].dist == lastIndex + 1):
                            if not (intersect and dijkNodes[-1].dist == lastIndex + 1):
                                dijkNodes.append(dijkstraNode(current.node, lastIndex + 1, current.previous))

                                print(str(dijkNodes[-1]))
                            else:
                                print("not new" + str(dijkNodes[-1]))

                            # if current.previous != None:
                            #     print("in da way: " + str(current.node) + " at" + str(lastIndex) + "\tPr:" + str(current.previous.node))

                            dijkNodes.remove(link)
                            noOfLinks -= 1

                            # previous = current.previous

                            # if not current.visited:
                            #     dijkNodes.remove(current)

                            # current = previous
                            
                            intersect = True
                            break
                # if intersect: break
            
            if intersect:
                previous = current.previous

                # print("Current: " + str(current) + "\tLinks:" + str(noOfLinks))
                # print("Previous: " + str(previous))

                if noOfLinks == 0:
                    dijkNodes.remove(current)

                current = previous



        if dijkNodes[-1].node == goal:
            break




    # Change the dijkNodes to a list of nodes:

    current = dijkNodes[-1]
    path = [current.node]

    while True:

        # Stay the distance amount of steps at the previous, to wait on any other robot
        path = (current.dist - current.previous.dist) * [current.previous.node] + path

        if path[0] == start: break

        current = current.previous


    return path









grid = Grid(10, 10)
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

robots = []

# positions = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (5,2), (6,2), (6,1), (6,0), (5,0)]
# robots.append(Robot(positions, grid))

# positions = [(9,0), (8,0), (7,0), (7,1), (7,2), (7,3), (7,4), (7,5), (6,5), (5,5), (5,4), (5,3), (5,2), (4,2), (3,2)]
# robots.append(Robot(positions, grid))

# positions = [(0,9), (0,8), (0,7), (0,6), (1,6), (2,6), (3,6), (3,5), (3,4), (3,3), (2,3)]
# robots.append(Robot(positions, grid))

# positions = [(9,9), (9,8), (9,7), (9,6), (8,6), (7,6), (6,6), (5,6), (4,6), (4,5), (3,5), (3,4), (2,4)]
# robots.append(Robot(positions, grid))


robots.append(Robot(findPath(robots, grid.getNode((0,0)), grid.getNode((5,0)))))
robots.append(Robot(findPath(robots, grid.getNode((9,0)), grid.getNode((3,2)))))
robots.append(Robot(findPath(robots, grid.getNode((0,9)), grid.getNode((2,3)))))
robots.append(Robot(findPath(robots, grid.getNode((9,9)), grid.getNode((2,4)))))
robots.append(Robot(findPath(robots, grid.getNode((5,0)), grid.getNode((3,4)))))


for i, robot in enumerate(robots):
    print("Robot " + str(i+1) + ": " + str(robot.path))

# print("Robot 2" + str(robots[1]))
# print("Robot 5" + str(robots[4]))

print(robots)








#--------------------------------------------------------------
# old test code



# positions = [(1,4),(1,3),(1,3),(0,3),(0,2),(1,2)]
# robots.append(Robot(positions, grid))

# # positions = [(8,2),(7,2),(6,2),(5,2),(4,2),(3,2),(2,2)]
# positions = [(8,2),(7,2),(6,2),(5,2),(4,2),(3,2)]
# robots.append(Robot(positions, grid))


# for node in grid.nodes:
#     print(node.pos)

# print("Links of (3, 1): ")

# for link in grid.getNode((3, 1)).links:
#     print(link.pos)






# gridFutures[i] is i steps in the future
# gridFutures = [Grid(xSize, ySize)]




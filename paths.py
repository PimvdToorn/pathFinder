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

        print("Visiting: " + str(current))
        for link in current.node.links:
            print("New node at:" + str(link))
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
                        print("Distance upped to: " + str(next.dist))
                        intersect = True
                        break

                    

            if link == goal:
                print("Goal")
                dijkNodes.append(next)
                break

            
            # Check if any robot will end at the location before this robot would be there
            intersect = False
            for robot in robots:
                if len(robot.path) > 0 and robot.path[-1] == link and len(robot.path) <= next.dist:
                    print("End stop: not added")
                    intersect = True
                    break


            for dNode in dijkNodes:

                if next.node == dNode.node:
                    if next.dist < dNode.dist:
                        print("Replaced: " + str(dNode) + "\tNow:" + str(next))
                        dNode = next
                    print("Exists: not added")
                    intersect = True
                    break

                    
            if not intersect:
                print("Added: " + str(next))
                dijkNodes.append(next)


        current.visited = True


        intersect = True
        while intersect:
            intersect = False

            noOfLinks = 0
            print("C:" + str(current))
            # if current.node.pos == (6,0):
            #     print(dijkNodes)

            # for link in dijkNodes:
            r = [*range(len(dijkNodes))]
            for i in r:
              
                # print(i)
                # print(r[-1])
                # print(len(dijkNodes))
                # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                link = dijkNodes[i]
                

                # if link.node.pos == (7,0) and current.node.pos == (6,0):
                #     print("\t\t\tlink:::" + str(link))
                #     print(dijkNodes)

                # if current.node.pos == (6,0):
                #     print("i: " + str(i) + "  len:" + str(len(dijkNodes)) + "\t\t\tlink:::" + str(link))
                    # print("len:" + str(len(dijkNodes)) + "\t\t\tlink:::" + str(link))

                if link.previous == current:
                    print("\t" + str(link))
                    noOfLinks += 1

                    for robot in robots:
                        length = len(robot.path) - 1
                        startIndex = length - link.dist - 1
                        if startIndex < 0: startIndex = 0

                        endIndex = length - current.dist
                        print("\t\t" + str(robot))
                        print("\t\tstr:" + str(startIndex))
                        print("\t\tend:" + str(endIndex))

                        if length + 1 > 0 and robot.path[-1] == current.node and length <= link.dist:
                            dijkNodes.remove(link)
                            print("\t\tend")
                            r.pop(-1)
                            i -= 1
                            if i < 0: i = 0

                            noOfLinks -= 1
                            intersect = True
                            break

                        elif endIndex >= 0:

                            try:
                                lastIndex = length - robot.path[::-1].index(current.node, startIndex, endIndex)
                            except:
                                print("\t\tNot in the way")
                                lastIndex = -1


                            else:
                                
                                # print("in da way: " + str(current.node) + str(robot.path[lastIndex]) + " at" + str(lastIndex))
                                # sleep(1)

                                # if not (intersect and dijkNodes[-1].node == current.node and dijkNodes[-1].dist == lastIndex + 1):
                                if not intersect or (intersect and dijkNodes[0].dist != lastIndex + 2):
                                    # dijkNodes.append(dijkstraNode(current.node, lastIndex + 1, current.previous))
                                    dijkNodes = [dijkstraNode(current.node, lastIndex + 2, current.previous)] + dijkNodes

                                    print("New node:" + str(dijkNodes[0]))
                                else:
                                    r.pop(-1)
                                    print("not new" + str(dijkNodes[0]))

                                if current.previous != None:
                                    print("in da way: " + str(current.node) + " at" + str(lastIndex) + "\tPr:" + str(current.previous.node))

                                print("Removing: " + str(link))
                                dijkNodes.remove(link)
                                
                                i -= 1
                                if i < 0: i = 0

                                # link = dijkstraNode(current.node, lastIndex + 1, current.previous)
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

                # if not current.visited:
                #     dijkNodes.remove(current)

                if noOfLinks == 0:
                    print("removed: " + str(current))
                    dijkNodes.remove(current)
                    r.pop(-1)

                if previous == None: break
                current = previous



        if dijkNodes[-1].node == goal:
            break


    print("dijkNodes:")
    for node in dijkNodes:
        print(node)

    # Change the dijkNodes to a list of nodes:

    current = dijkNodes[-1]
    path = [current.node]

    while True:

        # Stay the distance amount of steps at the previous, to wait on any other robot
        print("Current:" + str(current.node) + str(current.dist))
        print("Previous:" + str(current.previous.node) + str(current.previous.dist))
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
grid.addObstacle((5,7))
grid.addObstacle((4,7))
grid.addObstacle((3,7))
grid.addObstacle((6,7))
grid.addObstacle((7,7))

robots = []

# positions = [(0,0), (1,0), (2,0), (2,1), (2,2), (3,2), (4,2), (5,2), (6,2), (6,1), (6,0), (5,0)]
# robots.append(Robot(positions, grid))
# positions = [(8,8), (8,7), (8,6), (8,5), (7,5), (6,5), (5,5), (5,4), (5,3), (5,2), (4,2), (3,2), (2,1), (2,0)]
# robots.append(Robot(positions, grid))
# positions = [(9,0), (8,0), (7,0), (6,0), (6,1), (6,2), (6,2), (6,2), (6,2), (6,2), (6,2), (5,2), (4,2), (3,2)]
# robots.append(Robot(positions, grid))
# positions = [(3,3), (3,4), (3,5), (3,6), (2,6), (2,7), (2,8), (2,9), (3,9), (4,9), (5,9), (6,9), (7,9)]
# robots.append(Robot(positions, grid))
# positions = [(0,9), (0,8), (0,7), (0,6), (1,6), (1,6), (2,6), (3,6), (3,5), (3,4), (3,3), (2,3)]
# robots.append(Robot(positions, grid))
# positions = [(9,9), (9,8), (9,7), (9,6), (9,5), (8,5), (7,5), (6,5), (5,5), (4,5), (3,5), (3,4), (2,4)]
# robots.append(Robot(positions, grid))

robots.append(Robot(findPath(robots, grid.getNode((0,0)), grid.getNode((5,0)))))
robots.append(Robot(findPath(robots, grid.getNode((8,8)), grid.getNode((2,0)))))
print("-----------------------------------------------------------------------------------------------------")
robots.append(Robot(findPath(robots, grid.getNode((9,0)), grid.getNode((3,2)))))

robots.append(Robot(findPath(robots, grid.getNode((3,3)), grid.getNode((7,9)))))
robots.append(Robot(findPath(robots, grid.getNode((0,9)), grid.getNode((2,3)))))
robots.append(Robot(findPath(robots, grid.getNode((9,9)), grid.getNode((2,4)))))
robots.append(Robot(findPath(robots, grid.getNode((5,0)), grid.getNode((3,4)))))

# for i in range(4):
#     robots.append(Robot([],grid))



length = 0
for i, robot in enumerate(robots):
    length += len(robot.path)
    print("Robot " + str(i+1) + ": " + str(robot.path))

print("Total length: " + str(length))

# print("Robot 2" + str(robots[1]))
# print("Robot 5" + str(robots[4]))

print(robots)








#--------------------------------------------------------------
# old test code



# positions = [(1,4),(1,3),(1,3),(0,3),(0,2),(1,2)]
# robots.append(Robot(positions, grid))



# for node in grid.nodes:
#     print(node.pos)

# print("Links of (3, 1): ")

# for link in grid.getNode((3, 1)).links:
#     print(link.pos)






# gridFutures[i] is i steps in the future
# gridFutures = [Grid(xSize, ySize)]




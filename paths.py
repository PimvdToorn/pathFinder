from re import T
from time import sleep


class Node:
    def __init__(self, pos):
        self.pos = pos
        self.links = []

    def __str__(self) -> str:
        return str(self.pos)
        
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

        for dNode in dijkNodes:
            if dNode.visited == False and dNode.dist < current.dist:
                current = dNode


        for link in current.node.links:
            next = dijkstraNode(link, current.dist+1, current)


            intersect = True
            while intersect:
                intersect = False

                for robot in robots:
                    if len(robot.path) > next.dist and (robot.path[next.dist] == link or robot.path[next.dist-1] == link):
                        next.dist += 1
                        intersect = True
                        break

                    

            if link == goal:
                dijkNodes.append(next)
                break

            

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

            for link in dijkNodes:
                if link.previous == current:

                    for robot in robots:
                        try:
                            # print(current.node)
                            index = robot.path.index(current.node, current.dist, link.dist)
                            
                        except:
                            index = 0   # Do nothing

                        else:
                            # print(link.node.pos)
                            try:
                                length = len(robot.path)
                                lastIndex = length - robot.path[::-1].index(current.node, length - link.dist, length - current.dist)
                                # print(current.node)
                                # lastIndex = robot.path[::-1].index(current.node, current.dist, link.dist)
                            except:
                                lastIndex = -1
                            
                            for j in range(index+1, len(robot.path)):
                                if robot.path[j] != current.node:
                                    print("J:" + str(j) + " LastI:" + str(lastIndex))
                                    dijkNodes.append(dijkstraNode(current.node, j, current.previous))
                                    link.previous = dijkNodes[-1]
                                    current = current.previous
                                    intersect = True
                                    print("in da way")
                                    break
            if intersect:
                print("found da way")



        
        # print("Visited: " + str(current.node.pos))

        if dijkNodes[-1].node == goal:
            break

    # print(dijkNodes)
    # for dNode in dijkNodes:
    #     print(dNode)




    # Change the dijkNodes to a list of destinations:

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
grid.addObstacle((6,3))


robots = []

positions = [(0,4),(0,3),(0,2),(1,2),(2,2),(2,1),(3,1)]
robots.append(Robot(positions, grid))

positions = [(1,4),(1,3),(1,3),(0,3),(0,2),(1,2)]
robots.append(Robot(positions, grid))

# positions = [(8,2),(7,2),(6,2),(5,2),(4,2),(3,2),(2,2)]
# robots.append(Robot(positions, grid))
positions = [(8,2),(7,2),(6,2),(5,2),(4,2),(3,2)]
robots.append(Robot(positions, grid))


robots.append(Robot(findPath(robots, grid.getNode((0,0)), grid.getNode((5,5)))))


for robot in robots:
    print(robot.path)

print(robots)


# robots.append(Robot())
# robots[1].path = findPath(grid, (0,1), (5,4))







#--------------------------------------------------------------
# old test code

# for node in grid.nodes:
#     print(node.pos)

# print("Links of (3, 1): ")

# for link in grid.getNode((3, 1)).links:
#     print(link.pos)






# gridFutures[i] is i steps in the future
# gridFutures = [Grid(xSize, ySize)]




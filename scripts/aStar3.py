# Sample code from http://www.redblobgames.com/pathfinding/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>

import rospy, tf, numpy, math, random, Queue, heapq

from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from std_msgs.msg import Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from nav_msgs.msg import GridCells
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovariance
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

class Cell(object):
    def __init__(self, x, y, reachable):
        """Initialize new cell.
        @param reachable is cell reachable? not a wall?
        @param x cell x coordinate
        @param y cell y coordinate
        @param g cost to move from the starting cell to this cell.
        @param h estimation of the cost to move from this cell
                 to the ending cell.
        @param f f = g + h
        """
        self.reachable = reachable
        self.x = x
        self.y = y
        self.parent = None
        self.g = 0
        self.h = 0
        self.f = 0

class AStar(object):
    def __init__(self):
        # open list
        self.opened = []
        heapq.heapify(self.opened)
        # visited cells list
        self.closed = set()
        # grid cells
        self.cells = []
        self.grid_height = None
        self.grid_width = None

    def init_grid(self, width, height, walls, start, end):
        """Prepare grid cells, walls.
        @param width grid's width.
        @param height grid's height.
        @param walls list of wall x,y tuples.
        @param start grid starting point x,y tuple.
        @param end grid ending point x,y tuple.
        """
        self.grid_height = height
        self.grid_width = width
        for x in range(self.grid_width):
            for y in range(self.grid_height):
                if (x, y) in walls:
                    reachable = False
                else:
                    reachable = True
                self.cells.append(Cell(x, y, reachable))
        self.start = self.get_cell(*start)
        self.end = self.get_cell(*end)

    def get_heuristic(self, cell):
        """Compute the heuristic value H for a cell.
        Distance between this cell and the ending cell multiply by 10.
        @returns heuristic value H
        """
        return 10 * (abs(cell.x - self.end.x) + abs(cell.y - self.end.y))

    def get_cell(self, x, y):
        """Returns a cell from the cells list.
        @param x cell x coordinate
        @param y cell y coordinate
        @returns cell
        """
        return self.cells[int(x * self.grid_height + y)]

    def get_adjacent_cells(self, cell):
        """Returns adjacent cells to a cell.
        Clockwise starting from the one on the right.
        @param cell get adjacent cells for this cell
        @returns adjacent cells list.
        """
        cells = []
        if cell.x < self.grid_width-1:
            cells.append(self.get_cell(cell.x+1, cell.y))
        if cell.y > 0:
            cells.append(self.get_cell(cell.x, cell.y-1))
        if cell.x > 0:
            cells.append(self.get_cell(cell.x-1, cell.y))
        if cell.y < self.grid_height-1:
            cells.append(self.get_cell(cell.x, cell.y+1))
        return cells

    def get_path(self):
        cell = self.end
        path = [(cell.x, cell.y)]
        while cell.parent is not self.start:
            cell = cell.parent
            path.append((cell.x, cell.y))

        path.append((self.start.x, self.start.y))
        path.reverse()
        return path

    def update_cell(self, adj, cell):
        """Update adjacent cell.
        @param adj adjacent cell to current cell
        @param cell current cell being processed
        """
        adj.g = cell.g + 10
        adj.h = self.get_heuristic(adj)
        adj.parent = cell
        adj.f = adj.h + adj.g

    def solve(self):
        """Solve maze, find path to ending cell.
        @returns path or None if not found.
        """
        # add starting cell to open heap queue
        heapq.heappush(self.opened, (self.start.f, self.start))
        while len(self.opened):
            # pop cell from heap queue
            f, cell = heapq.heappop(self.opened)
            # add cell to closed list so we don't process it twice
            self.closed.add(cell)
            # if ending cell, return found path
            if cell is self.end:
                return self.get_path()
            # get adjacent cells for cell
            adj_cells = self.get_adjacent_cells(cell)
            for adj_cell in adj_cells:
                if adj_cell.reachable and adj_cell not in self.closed:
                    if (adj_cell.f, adj_cell) in self.opened:
                        # if adj cell in open list, check if current path is
                        # better than the one previously found
                        # for this adj cell.
                        if adj_cell.g > cell.g + 10:
                            self.update_cell(adj_cell, cell)
                    else:
                        self.update_cell(adj_cell, cell)
                        # add adj cell to open list
                        heapq.heappush(self.opened, (adj_cell.f, adj_cell))

class SimpleGraph:
    def __init__(self):
        self.edges = {}
    
    def neighbors(self, id):
        return self.edges[id]

example_graph = SimpleGraph()
example_graph.edges = {
    'A': ['B'],
    'B': ['A', 'C', 'D'],
    'C': ['A'],
    'D': ['E', 'A'],
    'E': ['B']
}

import collections

class Queue:
    def __init__(self):
        self.elements = collections.deque()
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, x):
        self.elements.append(x)
    
    def get(self):
        return self.elements.popleft()

# utility functions for dealing with square grids
def from_id_width(id, width):
    return (id % width, id // width)

def draw_tile(graph, id, style, width):
    r = "."
    if 'number' in style and id in style['number']: r = "%d" % style['number'][id]
    if 'point_to' in style and style['point_to'].get(id, None) is not None:
        (x1, y1) = id
        (x2, y2) = style['point_to'][id]
        if x2 == x1 + 1: r = "\u2192"
        if x2 == x1 - 1: r = "\u2190"
        if y2 == y1 + 1: r = "\u2193"
        if y2 == y1 - 1: r = "\u2191"
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width
    return r

def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            pass
            #print("%%-%ds" % width % draw_tile(graph, (x, y), style, width), end="")
        #print()

class SquareGrid:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = []
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height
    
    def passable(self, id):
        return id not in self.walls
    
    def neighbors(self, id):
        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]

def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        current = came_from[current]
        path.append(current)
    path.append(start) # optional
    path.reverse() # optional
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        
        for next in graph.neighbors(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
    
    return came_from, cost_so_far

###########################################################################################################################################

def setStart(msg):
    global initPoseX
    global initPoseY
    initPoseX = msg.pose.pose.position.x
    initPoseY = msg.pose.pose.position.y
    print "initPoseX"
    print initPoseX
    print "initPoseY"
    print initPoseY
    
def getMap(msg):#callBack for the map topic
    global wallList
    global offSetX
    global offSetY
    global resolution
    global astarObject

    global mapHeight
    global mapWidth

    print "width"
    print msg.info.width
    print "height"
    print msg.info.height
    
    height = msg.info.height
    width = msg.info.width
    
    offSetX = msg.info.origin.position.x
    offSetY = msg.info.origin.position.y
    resolution = msg.info.resolution

    astarObject = AStar()
    wallList = []

    #create index variable
    i = 0 #index for outer loop (keeps track of the row)
    j = 0 #index for inner loop (keeps track of the column)
    k = 0 #index for data (never gets reset)

    while (i < height) and (k < len(msg.data)): #go through all rows (starting at the top (0,0))
        j = 0 #reset index (start at the start of the new row)

        while (j < width) and (k < len(msg.data)): #go through a single row
            if msg.data[k] > 99: #check if its an obstacle
                wallList.append((i,j)) #add to the list of obstacles

            j+=1
            k+=1

        i+=1

    mapHeight = height
    mapWidth = width
    print "Map Received!"

def callAStar(msg): #takes a goal message
    global wallList
    global offSetX
    global offSetY
    global initPoseX
    global initPoseY
    global astarObject

    global mapHeight
    global mapWidth

    #get the position of the goal in terms of the grid
    goalX = initPoseX #int(round((msg.pose.position.x-offSetX)*resolution,0))
    goalY = initPoseY #int(round((msg.pose.position.y-offSetY)*resolution,0))

    #get the position of start in terms of the grid
    (trans,quat) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    #the transform array is fine for x and y
    startX = int(round((initPoseX-offSetX)*resolution,0)) #int(round((trans[0]-offSetX)*resolution,0)) #round to whole numberinitPoseX
    startY = int(round((initPoseY-offSetY)*resolution,0)) #int(round((trans[1]-offSetY)*resolution,0))

    astarObject.init_grid(mapWidth, mapHeight, wallList, (startX, startY), (goalY, goalY))

    print("Calling A*")
    
    #try:
    publishGridCells(astarObject.solve())
    #except:
    #    print "Some sort of error"


def publishGridCells(path):#takes a list of cells and publishes them to a given topic
    global seqNum
    global resolution
    global gridCellsPub
    
    #create header:
    head = Header()
    head.seq = seqNum
    seqNum += 1
    head.stamp = rospy.get_rostime()
    head.frame_id = "map"
    
    points = pointList(path)#get the points
    
    gridCells = GridCells()#create the message
    #fill the message with the necessary data
    gridCells.header = head
    gridCells.cell_width = resolution
    gridCells.cell_height = resolution
    gridCells.cells = points
    print "Points: "
    print points
    gridCellsPub.publish(gridCells)

def pointList(path): #creates a list of points from a list of tuples (x,y)
    points = []
    for i in path:
        newPoint = Point()

        newPoint.x = (i[0]*resolution)+offSetX
        newPoint.y = (i[1]*resolution)+offSetY
        newPoint.z = 0

        points.append(newPoint)

    return points


# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('aStar')

    global grid
    global pose
    global odom_tf
    global odom_list

    global publ
    global pathPub
    global gridCellsPub

    global initPoseX
    global initPoseY
    
    initPoseX = 0
    initPoseY = 0
    #create the sequence number for the gridcells messages
    global seqNum
    seqNum = 0

    map_sub = rospy.Subscriber('/map', OccupancyGrid, getMap, queue_size=1) #get the occupancy grid
    goal_sub = rospy.Subscriber('/goal', PoseStamped, callAStar, queue_size=1)
    start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, setStart, queue_size=1)
    pathPub = rospy.Publisher('aStar_Path', Path, queue_size=10)
    gridCellsPub = rospy.Publisher('aStar_Closed', GridCells, queue_size=10)
    #publ = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
    
    
    odom_list = tf.TransformListener() #save the bot's odometry

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting A*"
    
    while(not rospy.is_shutdown()):
        pass

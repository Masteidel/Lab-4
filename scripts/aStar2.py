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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler

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

def getMap(msg): #callBack for the map topic
    global grid
    global offSetX
    global offSetY
    global resolution
    
    print "width"
    print msg.info.width
    print "height"
    print msg.info.height
    
    height = msg.info.height
    width = msg.info.width
    
    offSetX = msg.info.origin.position.x
    offSetY = msg.info.origin.position.y
    resolution = msg.info.resolution

    grid = SquareGrid(msg.info.width,msg.info.height)

    #create index variable
    i = 0 #index for outer loop (keeps track of the row)
    j = 0 #index for inner loop (keeps track of the column)
    k = 0 #index for data (never gets reset)

    while (i < height) and (k < len(data)): #go through all rows (starting at the top (0,0))
        j = 0 #reset index (start at the start of the new row)

        while (j < width) and (k < len(data)): #go through a single row
            if msg.data[k] > 99: #check if its an obstacle
                grid.walls.append((i,j)) #add to the list of obstacles

            j+=1
            k+=1

        i+=1

    print "Map Received!"

def callAStar(msg): #takes a goal message
    global grid
    global offSetX
    global offSetY

    #get the position of the goal in terms of the grid
    goalX = int(round((msg.pose.position.x-offSetX)*resolution,0))
    goalY = int(round((msg.pose.position.y-offSetY)*resolution,0))

    #get the position of start in terms of the grid
    (trans,quat) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    #the transform array is fine for x and y
    startX = int(round((trans[0]-offSetX)*resolution,0)) #round to whole number
    startY = int(round((trans[1]-offSetY)*resolution,0))

    print("Calling A*")
    
    came_from, cost_so_far = aStar(grid, (startX, startY), (goalX, goalY))
    publishGridCells(came_from)

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

    gridCellsPub.publish(gridCells)

def pointList(path): #creates a list of points from a list of tuples (x,y)
    points = []
    for i in cells:
        newPoint = Point()

        newPoint.x = (i[0]/resolution)+offSetX
        newPoint.y = (i[1]/resolution)+offSetY
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

    #create the sequence number for the gridcells messages
    global seqNum
    seqNum = 0

    map_sub = rospy.Subscriber('/map', OccupancyGrid, getMap, queue_size=1) #get the occupancy grid
    goal_sub = rospy.Subscriber('/goal', PoseStamped, callAStar, queue_size=1)
    
    pathPub = rospy.Publisher('aStar_Path', Path, queue_size=10)
    gridCellsPub = rospy.Publisher('aStar_Closed', GridCells, queue_size=10)
    publ = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
    
    odom_list = tf.TransformListener() #save the bot's odometry

    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting A*"
    
    while(not rospy.is_shutdown()):
        pass

import rospy, tf, numpy, math, random, Queue, collections, heapq
#from implementation import *
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

class Cell:
	def __init__(self, xLoc, yLoc, reachable):
		self.reachable = reachable
		self.loc = (xLoc, yLoc)
		self.fCost = 0
		self.gCost = 0
		self.parent = None
		self.visited = False

class SimpleGraph:
	def __init__(self):
		self.edges = {}

	def neighbors(self, id):
		return self.edges[id]

class Queue:
	def __init__(self):
		self.elements = collections.deque()

	def empty(self):
		return len(self.elements) == 0

	def put(self, x):
		self.elements.append(x)

	def get(self):
		return self.elements.popleft()

class PriorityQueue:
	def __init__(self):
		self.elements = []

	def empty(self):
		return len(self.elements) == 0

	def put(self, item, priority):
		heapq.heappush(self.elements, (priority, item))

	def get(self):
		return heapq.heappop(self.elements)[1]

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
		if(x + y)%2 == 0: results.reverse()
		results = filter(self.in_bounds, results)
		results = filter(self.passable, results)
		return results

class GridWithWeights(SquareGrid):
	def __init__(self, width, height):
		#super().__init__(self, width, height)
		self.width = width
		self.height = height
		self.walls = []
		self.weights = {}

	def cost(self, from_node, to_node):
		return self.weights.get(to_node, 1)

def heuristic(a, b):
	(x1, y1) = a
	(x2, y2) = b
	return abs(x1 - x2) + abs(y1 - y2)

def a_star_search(graph, start, goal):
	global grid

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

def reconstruct_path(came_from, start, goal):
	current = goal
	path = [current]
	while current != start:
		current = came_from[current]
		path.append(current)
	path.append(start)
	path.reverse()
	return path

def getMap(msg):
	global grid
	global resolution

	height = msg.info.height
	width = msg.info.width
	grid = GridWithWeights(width, height)
	resolution = msg.info.resolution

	wallList = []

	#create index variable
	i = 0 #index for outer loop (keeps track of the row)
	j = 0 #index for inner loop (keeps track of the column)
	k = 0 #index for data (never gets reset)

	while (j < height) and (k < len(msg.data)): #go through all rows (starting at the top (0,0))
		j = 0 #reset index (start at the start of the new row)

		while (i < width) and (k < len(msg.data)): #go through a single row
			if msg.data[k] > 99: #check if its an obstacle
				wallList.append((i,j)) #add to the list of obstacles

			i+=1
			k+=1

		j+=1

	grid.walls = wallList
	publishGridCells(wallList)

def getStart(msg):
	global startX
	global startY

	startX = int(round(msg.pose.pose.position.x, 0))
	startY = int(round(msg.pose.pose.position.y, 0))

def publishGridCells(path):
	global seqNum
	global resolution
	global gridCellsPub

	head = Header()
	head.seq = seqNum
	seqNum += 1
	head.stamp = rospy.get_rostime()
	head.frame_id = "map"

	points = pointList(path)#get the points
	
	gridCells = GridCells()
	#fill the message with the necessary data
	gridCells.header = head
	gridCells.cell_width = resolution
	gridCells.cell_height = resolution
	gridCells.cells = points
	#print "Points: "
	#print points
	gridCellsPub.publish(gridCells)

def pointList(path): #creates a list of points from a list of tuples (x,y)
	points = []
	for i in path:
		newPoint = Point()

		newPoint.x = i[0]#*resolution
		newPoint.y = i[1]#*resolution
		newPoint.z = 0

		points.append(newPoint)

	return points

def callAStar(msg):
	global grid
	global startX
	global startY
	global goalX
	global goalY

	goalX = int(round(msg.pose.position.x, 0))
	goalY = int(round(msg.pose.position.y, 0))
	if(startX == None or startY == None):
		start = (1.0, 2.0)
	else:
		start = (startX, startY)
	goal = (goalX, goalY)
	print grid.walls
	came_from, cost_so_far = a_star_search(grid, start, goal)
	path = reconstruct_path(came_from, start, goal)
	#publishGridCells(path)

if __name__ == '__main__':
	rospy.init_node('aStar')
	global gridCellsPub
	global seqNum
	seqNum = 0

	map_sub = rospy.Subscriber('/map', OccupancyGrid, getMap, queue_size=1)
	start_sub = rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, getStart, queue_size=1)
	goal_sub = rospy.Subscriber('/goal', PoseStamped, callAStar, queue_size=1)
	gridCellsPub = rospy.Publisher('aStar_Closed', GridCells, queue_size=10)

	rospy.sleep(rospy.Duration(1, 0))
	while(not rospy.is_shutdown()):
		rospy.sleep(0.15)

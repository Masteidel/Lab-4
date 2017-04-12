import rospy, tf, numpy, math, random, Queue
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

class cell: #stores the probability of the cell being occupied and its f(n) cost
            #also stores the position (assuming origin is at the top left corner)
    prob = 50 #start at 50/50

    cost = -1 #unexplored (cost unknown) f(n)
    g = 0 #g(n)
    h = 0 #h(n)

    x = 0 #x position
    y = 0 #y position

    cameFrom = None #previous cell

    nextMove = 0; #keeps track of the direction to the next cell in the path, used to
                  #generate waypoints only

    def __init__(self, probability, xLoc, yLoc):
        prob = probability
        x = xLoc
        y = yLoc

def heuristic(current,goal): #returns h(n) euclidian distance to goal
    return int(math.sqrt(((goal.x-current.x)**2) + ((goal.y-current.y)**2))) #distance formula
    
def getMap(msg): #callBack for the map topic
    global grid
    global offSetX
    global offSetY
    global resolution
    
    print "width"
    print msg.info.width
    print "height"
    print msg.info.height

    offSetX = msg.info.origin.position.x
    offSetY = msg.info.origin.position.y
    resolution = msg.info.resolution
    grid = get2DArray(msg.data, msg.info.width, msg.info.height)#get a 2D array version of the grid
    print "Map Received!"

def aStar(grid, start, goal): #takes a grid (2D array of cell objects), start and goal (both cells) 
    print "Running A*"
    openSet = Queue.PriorityQueue(maxsize=0) #create a set to store all discovered nodes yet to be explored
    closedSet = [] #just a list because all we care about is whether or not something is here already
    path = []
    start.g = 0 #by definition
    start.h = heuristic(start, goal)#euclidian distance to goal
    start.cost = start.h
    openSet.put((start.cost,start)) #add start to the queue
    
    while (not openSet.empty()):#for as long as unexpanded (but discovered) nodes exist
        print "Loop"
        #update the sets:
        temp = openSet.get() #gets the lowest cost node (based on f(n))
        current = temp[1]
        closedSet.append(current) #put the current node in the set of closed nodes
        #path.append(current)
        print "curr XY"
        print current.x
        print current.y
        #are we there yet?!
        if (current.x == goal.x) and (current.y == goal.y):
            #yep!
            print "GOOOAAAAL!"
            path = cellPath(current)
            publishPath(path)
            publishGridCells(path, 'aStar_Closed')
            return #!!!figure out return data!!!

        #get all the children:
        x = current.x
        y = current.y
        print "current xy"
        print x
        print y
        #its not hugely important, but this is how I'm numbering the childern:
        #  7 0 1
        #  6 C 2
        #  5 4 3

        children = [] #create a list of the childern

        try: #the try except is needed because this method will try to call for cells that don't exist
             #this will catch the list index out of range error and ignore it (and skip that non-exitent child)
            children.append(grid[x][y-1]) #y-1 because origin is at top left
            print "Child"
        except IndexError:
            print "Pass"
            pass
        try: #child 1
            children.append(grid[x+1][y-1])
            print "Child"
        except IndexError:
            print "Pass"
            pass
        try: #child 2
            children.append(grid[x+1][y])
            print "Child"
        except IndexError:
            print "Pass"
            pass
        try: #child 3
            children.append(grid[x+1][y+1])
            print "Child"
        except IndexError:
            print "Pass"
            pass
        try: #child 4
            children.append(grid[x][y+1])
            print "Child"
        except IndexError:
            print "Pass"
            pass
        try: #child 5
            children.append(grid[x-1][y-1])
            print "Child"
        except IndexError:
            print "Pass"
            pass
        try: #child 6
            children.append(grid[x-1][y])
            print "Child"
        except IndexError:
            print "Pass"
            pass
        try: #child 7
            children.append(grid[x-1][y-1])
            print "Child"
        except IndexError:
            print "Pass"
            pass
        
        #go through all of the children/neighbors:
        for child in children: #make sure to go through everything
            if child not in closedSet: #if the cell isn't already expanded (if it is we ignore it)
                
                if (child.prob < 100): #probably not an obstacle (if its an obstacle we ignore it)
                    print "child loop"
                    child.g = current.g + 1
                    child.h = heuristic(child,goal)
                    child.cost = child.g + child.h #total cost
                    child.cameFrom = current #obviously we came from the current cell to get to its neighbor

                    openSet.put((child.cost,child)) #add to priority queue
                closedSet.append(child)

        #display on the grid
        #publishGridCells(openSet,'aStar_Open')
        #publishGridCells(closedSet, 'aStar_Closed')
        #END MAIN WHILE

    publishGridCells(closedSet, 'aStar_Closed')
    print("no solutions exist")
    #END

def callAStar(msg): #takes a goal message
    global grid
    global offSetX
    global offSetY

    print offSetX
    print offSetY
    #create a cell for the goal
    goal = cell(0,int(round(msg.pose.position.x-offSetX,0)),int(round(msg.pose.position.y-offSetY,0)))
    goal.x = int(round(msg.pose.position.x-offSetX,0))
    goal.y = int(round(msg.pose.position.y-offSetY,0))
    #create a cell for the start
    start = cell(0,0,0)
    (trans,quat) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    #the transform array is fine for x and y
    start.x = int(round(trans[0]-offSetX,0)) #round to whole number
    start.y = int(round(trans[1]-offSetY,0))

    print("Calling A*")
    
    aStar(grid, start, goal)

def cellPath(cell): #takes a cell and returns a list of all the cells leading to it
    print "cellPath"
    path = []
    
    path.append(cell) #start by adding the last cell to the list
    while cell.cameFrom is not None: #when a cell doesn't have a parent it is the start point
        path.append(cell.cameFrom) #get a list of all the parents 
        cell = cell.cameFrom
    path.reverse() #reverse the list so that the start node is first
    
    return path

def publishPath(cells): #takes a list of cells in the order that we wish to visit them and publishes a path message
     global pathPub

     print "Publish Path, calling get_Path"
     path = get_Path(cells)
     print "Returned from get_Path"
     pathPub.publish(path)
     print "End of publishPath"
    
def get_Path(cells): #takes a list of cells in the order that we wish to visit them and returns a path message
    print "Get Path"
    global seqNum

    #create header:
    pathHead = Header()
    pathHead.seq = seqNum
    seqNum += 1
    pathHead.stamp = rospy.get_rostime()
    pathHead.frame_id = "aStar_Path"
    
    poses = [] #create the list to store all of the poses (waypoints) as PoseStamped objects

    #keeps track of the next move the robot will make when it is in a given cell, using this notation:
    #  7 0 1
    #  6 C 2
    #  5 4 3
    
    i = 0
    #assigns each cell with a code to signify which direction the robot needs to move in to get to the next cell
    while i < (len(cells) - 1): #doesn't need to run on the last cell
        if (cells[i+1].x == cells[i].x) and (cells[i+1].y < cells[i].y): #move in negative y direction
            cells[i].nextMove = 0

        if (cells[i+1].x > cells[i].x) and (cells[i+1].y < cells[i].y): #move in positive x, negative y direction
            cells[i].nextMove = 1

        if (cells[i+1].x > cells[i].x) and (cells[i+1].y == cells[i].y): #move in positive x direction
            cells[i].nextMove = 2

        if (cells[i+1].x > cells[i].x) and (cells[i+1].y > cells[i].y): #move in positive x, positive y direction
            cells[i].nextMove = 3

        if (cells[i+1].x == cells[i].x) and (cells[i+1].y > cells[i].y): #move in positive y direction
            cells[i].nextMove = 4

        if (cells[i+1].x < cells[i].x) and (cells[i+1].y > cells[i].y): #move in negative x, positive y direction
            cells[i].nextMove = 5

        if (cells[i+1].x < cells[i].x) and (cells[i+1].y == cells[i].y): #move in negative x direction
            cells[i].nextMove = 6

        if (cells[i+1].x < cells[i].x) and (cells[i+1].y < cells[i].y): #move in negative x, negative y direction
            cells[i].nextMove = 7
        i+=1

        print cells[i].nextMove #Print the direction

    i = 1 #start with the second cell
    while (i < len(cells)):
        if not (cells[i].nextMove == cells[i-1].nextMove): #if they are the same, the robot is moving in the right
                                                           #direction to begin with, and nothing needs to be done
                                                           #if they aren't the same, our heading needs to change,
                                                           #so we need a waypoint at that cell
            if cells[i].nextMove == 0:
                turn = math.pi #turned to face the -y direction
            
            elif cells[i].nextMove == 1:
                turn = (math.pi)*(5/4) #turned to face the -y, +x direction
                
            elif cells[i].nextMove == 2:
                turn = (math.pi)*(3/2) #turned to face the +x direction
            
            elif cells[i].nextMove == 3:
                turn = (math.pi)*(7/4) #turned to face the +x,+y direction

            elif cells[i].nextMove == 4:
                turn = 0 #turned to face the +y direction

            elif cells[i].nextMove == 5:
                turn = (math.pi)*(1/4) #turned to face the -x, +y direction

            elif cells[i].nextMove == 6:
                turn = (math.pi)*(1/2) #turned to face the -x direction

            elif cells[i].nextMove == 7:
                turn = (math.pi)*(3/4) #turned to face the -x, -y direction

            #convert that angle to a quaternian:
            quaternion = tf.transformations.quaternion_from_euler(0, 0, turn)
            pose = Pose()
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]
            
            #set the coordinates (may need conversion later to go from grid coordinates to actual)
            pose.position.x = cells[i].x
            pose.position.y = cells[i].y
            pose.position.z = 0

            #create header:
            head = Header()
            head.seq = seqNum
            seqNum += 1
            head.stamp = rospy.get_rostime()
            head.frame_id = "waypoint"
                
            poseStamped = PoseStamped(head, pose) #create the PoseStamped object

            poses.append(poseStamped)
        i+=1
       #end if
    #end for loop
    
    return Path(pathHead,poses)


def get2DArray(data, width, height): #an absolutely thrilling function to take a 1D array and break
                                     #it into a 2D array (a grid) given a width and height

    grid = [[0 for row in range(0, height)] for col in range(0, width)] #create the 2D array
    #create index variable
    i = 0 #index for outer loop (keeps track of the row)
    j = 0 #index for inner loop (keeps track of the column)
    k = 0 #index for data (never gets reset)

    while (i < height) and (k < len(data)): #go through all rows (starting at the top (0,0))
        j = 0 #reset index (start at the start of the new row)

        while (j < width) and (k < len(data)): #go through a single row
            grid[j][i] = cell(data[k], j, i)#creates a cell object
            #grid[j][i]=cell()
            grid[j][i].x = j
            grid[j][i].y = i
            grid[j][i].prob = data[k]
            j+=1
            k+=1

        i+=1
    
    return grid

def publishGridCells(cells,topic):#takes a list of cells and publishes them to a given topic
    global seqNum
    global resolution
    global gridCellsPub
    
    #create header:
    head = Header()
    head.seq = seqNum
    seqNum += 1
    head.stamp = rospy.get_rostime()
    head.frame_id = "map"
    
    points = pointList(cells)#get the points
    
    gridCells = GridCells()#create the message
    #fill the message with the necessary data
    gridCells.header = head
    gridCells.cell_width = 1
    gridCells.cell_height = 1
    gridCells.cells = points

    gridCellsPub.publish(gridCells)
    

def pointList(cells): #creates a list of points from a list of cells
    points = []
    for i in cells:
        points.append(pointFromCell(i))

    return points


def pointFromCell(cell): #creates a point from a cell
    newPoint = Point()

    newPoint.x = cell.x+offSetX
    newPoint.y = cell.y+offSetY
    newPoint.z = 0
    
    return newPoint

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

    map_sub = rospy.Subscriber('/map', OccupancyGrid, getMap, queue_size=1) #get the occupancy grid
    #start_sub = rospy.Subscriber('', GridCells, callAStar, queue_size=1)
    goal_sub = rospy.Subscriber('/goal', PoseStamped, callAStar, queue_size=1)
    pathPub = rospy.Publisher('aStar_Path', Path, queue_size=10)
    gridCellsPub = rospy.Publisher('aStar_Closed', GridCells, queue_size=10)

    publ = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size=10)
    odom_list = tf.TransformListener() #save the bot's odometry

    #create the sequence number for the gridcells messages
    global seqNum
    seqNum = 0
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting A*"

    odom_list = tf.TransformListener()
    
    while(not rospy.is_shutdown()):
        pass

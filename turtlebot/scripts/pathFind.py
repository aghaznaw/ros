# Heuristics and base solution for static map reading by Jere Mourujärvi 2018
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
import math
class getGazeboMap:
    def loadMap(name):
        #Map info is in format: (x, y, z, r, p, y). The first two are the most important
        #z tells us the height of the model, so if it is +, there is a block

        writeMap = test.getMapSize()
        combineMap(writeMap, staticObjects)
        return loadedMap


class GazeboGraph:
    all_nodes = []
    for x in range(50):
        for y in range(50):
            all_nodes.append([x, y])
    def __init__(self):
        self.edges = {}
    def cost(self, from_node, to_node):
        return self.result.get(to_node, 1)
        #return self.weights.get(to_node, 1)

    def neighbors(self, id):

        (x, y) = id
        results = [(x+1, y), (x, y-1), (x-1, y), (x, y+1)]
        if (x + y) % 2 == 0: results.reverse() # aesthetics
        results = filter(self.in_bounds, results)
        results = filter(self.passable, results)
        return results

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
        if x2 == x1 + 1: r = ">"
        if x2 == x1 - 1: r = "<"
        if y2 == y1 + 1: r = "v"
        if y2 == y1 - 1: r = "^"
    if 'start' in style and id == style['start']: r = "A"
    if 'goal' in style and id == style['goal']: r = "Z"
    if 'path' in style and id in style['path']: r = "@"
    if id in graph.walls: r = "#" * width

    return r

def draw_grid(graph, width=2, **style):
    for y in range(graph.height):
        for x in range(graph.width):
            print("%%-%ds" % width % draw_tile(graph, (x, y), style, width), end="")
        print()

DIAGRAM1_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]

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

class GridWithWeights(SquareGrid): #Can be used with maps that introduce altered terrain on the z axis
    def __init__(self, width, height):#left unused for now
        super().__init__(width, height)
        self.weights = {}

    def cost(self, from_node, to_node):
        return self.weights.get(to_node, 1)

diagram4 = GridWithWeights(50, 50)
#diagram4.walls = [(1, 7), (1, 8), (2, 7), (2, 8), (3, 7), (3, 8)]
diagram4.weights = {loc: 5 for loc in [(3, 4), (3, 5), (4, 1), (4, 2),
                                       (4, 3), (4, 4), (4, 5), (4, 6),
                                       (4, 7), (4, 8), (5, 1), (5, 2),
                                       (5, 3), (5, 4), (5, 5), (5, 6),
                                       (5, 7), (5, 8), (6, 2), (6, 3),
                                       (6, 4), (6, 5), (6, 6), (6, 7),
                                       (7, 3), (7, 4), (7, 5)]}

import heapq

class mapLoader:
    def __init__(self):
        self.elements = collections.deque()


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
    path = []
    firstLast = goal
    secondLast = firstLast
    direction = ""
    upDown = ""
    leftRight = ""
    curDirection =""
    diagonal = False
    cornerList = []
    firstLastDirection = ""
    secondLastDirection = ""
    while current != start:
        path.append(current)
        current = came_from[current]
        #if (current[0] == firstLast[0] and current[1] != firstLast[1]):
            #if x here is the same as x in firstlast, while y is not..
            #we are going to this direction
                #1 and 1  = y isnt moving (1,1), (2,1), (3,1) etc
                #0 and 0 = x isnt moving

        if (current[1] == firstLast[1]):
            #We are going either left or right
            direction = "right"
            if (direction != secondLastDirection and diagonal == False):
                #IF we were going to the same way on the secondLast...
                print("Going downright")
                cornerList.append(secondLast)
            #else:
                #We are going "right"
                #leftRight = "right"
        if (current[0] == firstLast[0]):
            #We are going either up or down
            direction = "down"
            if (direction != secondLastDirection and diagonal == False):

                cornerList.append(secondLast)
                diagonal = True

        if (direction == firstLastDirection):
            diagonal = False

        print ("Direction is")
        print (direction)
        print(leftRight)
        secondLastDirection = firstLastDirection
        firstLastDirection = direction
        direction = ""
        firstLast = current
        secondLast = firstLast

    cornerList.reverse()
    print(cornerList)
    path.append(start) # optional
    #path.reverse() # optional
    return cornerList #returns the corners the path has
    #return path #returns the path the algorithm takes

#def obstacleSizeConverter(obstacle):
#    if obstacle.size =
def translateModel(modelList):
    test = modelList
    staticObjects = test.getStatics()
    for model in test:
        for i in model.position:
            model.position[i] = model.position[i] * 1000
            #print model.position[i]

def heuristic(a, b):

    #return abs(a[0] - b[0]) + abs(a[1] - b[1]) manhattan
    return math.sqrt(((a[0] - b[0])**2) + ((a[1] - b[1])**2)) #euclidean distance (not limited to grid, can traverse diagonally)
def pathFindJereSolution():
    #file_dir = os.path.dirname(__file__)
    #rel_path =
    file_subdir = "test.txt"
    #file_path = os.path.join(file_dir, file_subdir)
    file_path = ("test.txt")
    modelList = []
    strings = ("model name=")
    print("File path: ")
    poseList = []
    print(file_path)
    with open(file_path) as f:
        xmlLines = f.readlines()
        #lines = f.readlines()
        #i = 0
        for i in range(len(xmlLines)):

            #print (line)
            #if any(l in line for s in strings):
            #    modelList.add(line)
            #    print ("new model found")
                #if any(l in line for "static=")
                #    print("Model resides at the map in location:")
            #if "model name=" in xmlLines[i]:
            if "<static>1</static>" in xmlLines[i]:
                print(xmlLines[i])
                print(i)
                old_integer = i
                    #<pose> x y z roll pitch yaw </pose>
                while ("</model>" not in xmlLines[i]):
                    if "<size>" in xmlLines[i]:
                        print (xmlLines[i])
                    if "<pose frame=" in xmlLines[i]:
                        print(xmlLines[i])
                        poseList.append(xmlLines[i])

                    i = i + 1

                i = old_integer

                print ("xmlLines: ",xmlLines[i])
                print ("poseList: ",poseList)
                #while ("/model" not in line):
                    #print (line
            i = i + 1

    return modelList
def modelToCoords(modelList):
    #newList = list(map(int, modelList.split()))
    newList = []
    modelDimensions = list(map(float, modelList.split()))
    print (modelDimensions)
    for item in modelList:
        convertedItem = item
        newList.append(modelDimensions)
    return newList
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
            print ("broke! on")
            print (current)
            print (new_cost)
            break

        for next in graph.neighbors(current):
            new_cost = cost_so_far[current]

            if next not in cost_so_far or new_cost < cost_so_far[next]:

                print(next)
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    print("This is the path: ")
    print(reconstruct_path(came_from, start=(start), goal=(goal)))
    draw_grid(graph, width=2, path=reconstruct_path(came_from, start=(start), goal=(goal)))
    listOfGoals = reconstruct_path(came_from, start=(start), goal=(goal))
    w, h = 2, len(listOfGoals)

    goalArray = [[0 for x in range(w)] for y in range(h)]
    print(goalArray)
    #goalArray.append([])
    foo = 0
    for goalTuple in listOfGoals:


        x, y = goalTuple
        if x < 50:
            x = x * -1
        if x > 50:
            x = x - 50
            x = float(x / 10.0)
        goalArray[foo][0] = x

        if y < 50:
            y = y * -1
        if y > 50:
            y = y - 50
            y = float(y / 10.0)
        goalArray[foo][1] = y
        foo = foo + 1

    print(goalArray)
    return came_from, cost_so_far
def main():
    while True:
        print("ROS Turtlebot3 pathfinding utility\n")
        commVal = input("Please select a command\n1. Run map loader\n2. Calculate path in the current world\n3. Exit application\n")
        if commVal == '1':
            loader = mapLoader()
            print("Reading robot location from the Gazebo...")
            location = "(1,1)" #readFromGazeb()

            print("Robot location is ", location)
            #save the location to file
            saveFile = open("locFile.txt", "w")
            saveFile.write(location)
            saveFile.close()
            print(loader)
        if commVal == '2':
            locFile = open("locFile.txt","r")
            location = locFile.read()
            print(locFile.read())

            TEST_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
            TEST_WALLS.append((33,2))

            OBJECT_WALLS = [(1,2), (1,3), (55, 49), (4, 22), (5,22), (3,22), (2,22), (1,22)] #[from_id_width(id, width=5) for
            #graph = #gazebo map is default -10.0m * 10.0m

            graph = SquareGrid(100,100)
            graph.walls = OBJECT_WALLS #TEST_WALLS # long list, [(21, 0), (21, 2), ...]
            start = "(1,1)"
            goal = "(99,99)"
            test = a_star_search(graph, eval(location), eval(goal))
            #test = a_star_search(graph, (1,1), (40,25))
            aa = test
            #print (aa)

            graph.road = aa
            #print("loader:")
            #pathFindJereSolution()
            modelToCoords("1.41131 -1 0")

        elif commVal == '3':
            break;
        else:
            print("Invalid command")
main()

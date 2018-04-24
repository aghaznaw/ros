# Sample code from https://www.redblobgames.com/pathfinding/a-star/
# Copyright 2014 Red Blob Games <redblobgames@gmail.com>
#
# Feel free to use this code in your own projects, including commercial projects
# License: Apache v2.0 <http://www.apache.org/licenses/LICENSE-2.0.html>
import math
class getGazeboMap:
    def loadMap(name):
        #Map info is in format: (x, y, z, r, p, y). The first two are the most important
        #z tells us the height of the model, so if it is +, there is a block
        test = test.map
        staticObjects = test.getStatics()

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
    def pathFindJereSolution():
        file_dir = os.path.dirname(__file__)
        print("I am here")
        #rel_path =
        file_subdir = "test.txt"
        file_path = os.path.join(file_dir, file_subdir)
        modelList = []
        strings = ("model name=")
        print(file_path)
        with open(file_path) as f:
            print(f)
            for line in f:
                if any(l in line for s in strings):
                    modelList.add(line)
                    print ("new model found")
                    #if any(l in line for "static=")
                    #    print("Model resides at the map in location:")

                    print (line)
        return modelList

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

        #print ("Direction is")
        #print (direction)
        #print(leftRight)
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
def heuristic(a, b):

    #return abs(a[0] - b[0]) + abs(a[1] - b[1]) manhattan
    return math.sqrt(((a[0] - b[0])**2) + ((a[1] - b[1])**2)) #euclidean distance (not limited to grid, can traverse diagonally)

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
    griphSize = 50
    for goalTuple in listOfGoals:


        x, y = goalTuple
        if x >= griphSize:
            goalArray[foo][0] = (x - griphSize ) / 10.0
        if x <= griphSize:
            goalArray[foo][0] = (x - griphSize ) / 10.0
        if y >= griphSize:
            goalArray[foo][1] = ((y - griphSize) * (- 1)) / 10.0
        if y <= griphSize:
            goalArray[foo][1] = ((y - griphSize) * (- 1) ) / 10.0
        foo = foo + 1

    print(goalArray)
    thefile = open("PathArray.txt", "w")
    for item in goalArray:
        thefile.write(str(item[0]) + "," + str(item[1]) +",\n")
    thefile.close()
    return came_from, cost_so_far
def main():

    loader = mapLoader()
    print(loader)
    currentModels = loader.pathFindJereSolution

    TEST_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
    TEST_WALLS.append((33,2))

    '''OBJECT_WALLS =[(80, 22), (81, 22), (82, 22), (83, 22), (84, 22), (85, 22), (86, 22), (87, 22), (88, 22), (89, 22), (90, 22),
(80, 23), (81, 23), (82, 23), (83, 23), (84, 23), (85, 23), (86, 23), (87, 23), (88, 23), (89, 23), (90, 23),
(80, 24), (81, 24), (82, 24), (83, 24), (84, 24), (85, 24), (86, 24), (87, 24), (88, 24), (89, 24), (90, 24),
(80, 25), (81, 25), (82, 25), (83, 25), (84, 25), (85, 25), (86, 25), (87, 25), (88, 25), (89, 25), (90, 25),
(80, 26), (81, 26), (82, 26), (83, 26), (84, 26), (85, 26), (86, 26), (87, 26), (88, 26), (89, 26), (90, 26),
(80, 27), (81, 27), (82, 27), (83, 27), (84, 27), (85, 27), (86, 27), (87, 27), (88, 27), (89, 27), (90, 27),
(80, 28), (81, 28), (82, 28), (83, 28), (84, 28), (85, 28), (86, 28), (87, 28), (88, 28), (89, 28), (90, 28),
(80, 29), (81, 29), (82, 29), (83, 29), (84, 29), (85, 29), (86, 29), (87, 29), (88, 29), (89, 29), (90, 29),
(80, 30), (81, 30), (82, 30), (83, 30), (84, 30), (85, 30), (86, 30), (87, 30), (88, 30), (89, 30), (90, 30),
(80, 31), (81, 31), (82, 31), (83, 31), (84, 31), (85, 31), (86, 31), (87, 31), (88, 31), (89, 31), (90, 31),
(80, 32), (81, 32), (82, 32), (83, 32), (84, 32), (85, 32), (86, 32), (87, 32), (88, 32), (89, 32), (90, 32),
(80, 33), (81, 33), (82, 33), (83, 33), (84, 33), (85, 33), (86, 33), (87, 33), (88, 33), (89, 33), (90, 33),
(80, 34), (81, 34), (82, 34), (83, 34), (84, 34), (85, 34), (86, 34), (87, 34), (88, 34), (89, 34), (90, 34),
(80, 35), (81, 35), (82, 35), (83, 35), (84, 35), (85, 35), (86, 35), (87, 35), (88, 35), (89, 35), (90, 35),
(80, 36), (81, 36), (82, 36), (83, 36), (84, 36), (85, 36), (86, 36), (87, 36), (88, 36), (89, 36), (90, 36),
(80, 37), (81, 37), (82, 37), (83, 37), (84, 37), (85, 37), (86, 37), (87, 37), (88, 37), (89, 37), (90, 37),
(80, 38), (81, 38), (82, 38), (83, 38), (84, 38), (85, 38), (86, 38), (87, 38), (88, 38), (89, 38), (90, 38),
(80, 39), (81, 39), (82, 39), (83, 39), (84, 39), (85, 39), (86, 39), (87, 39), (88, 39), (89, 39), (90, 39),
(80, 40), (81, 40), (82, 40), (83, 40), (84, 40), (85, 40), (86, 40), (87, 40), (88, 40), (89, 40), (90, 40),
(80, 41), (81, 41), (82, 41), (83, 41), (84, 41), (85, 41), (86, 41), (87, 41), (88, 41), (89, 41), (90, 41),
(80, 42), (81, 42), (82, 42), (83, 42), (84, 42), (85, 42), (86, 42), (87, 42), (88, 42), (89, 42), (90, 42),
(80, 43), (81, 43), (82, 43), (83, 43), (84, 43), (85, 43), (86, 43), (87, 43), (88, 43), (89, 43), (90, 43),
(80, 44), (81, 44), (82, 44), (83, 44), (84, 44), (85, 44), (86, 44), (87, 44), (88, 44), (89, 44), (90, 44),
(80, 45), (81, 45), (82, 45), (83, 45), (84, 45), (85, 45), (86, 45), (87, 45), (88, 45), (89, 45), (90, 45),
(80, 46), (81, 46), (82, 46), (83, 46), (84, 46), (85, 46), (86, 46), (87, 46), (88, 46), (89, 46), (90, 46),
(80, 47), (81, 47), (82, 47), (83, 47), (84, 47), (85, 47), (86, 47), (87, 47), (88, 47), (89, 47), (90, 47),
(80, 48), (81, 48), (82, 48), (83, 48), (84, 48), (85, 48), (86, 48), (87, 48), (88, 48), (89, 48), (90, 48),
(80, 49), (81, 49), (82, 49), (83, 49), (84, 49), (85, 49), (86, 49), (87, 49), (88, 49), (89, 49), (90, 49),
(80, 50), (81, 50), (82, 50), (83, 50), (84, 50), (85, 50), (86, 50), (87, 50), (88, 50), (89, 50), (90, 50),
(80, 51), (81, 51), (82, 51), (83, 51), (84, 51), (85, 51), (86, 51), (87, 51), (88, 51), (89, 51), (90, 51),
(80, 52), (81, 52), (82, 52), (83, 52), (84, 52), (85, 52), (86, 52), (87, 52), (88, 52), (89, 52), (90, 52),
(80, 53), (81, 53), (82, 53), (83, 53), (84, 53), (85, 53), (86, 53), (87, 53), (88, 53), (89, 53), (90, 53),
(80, 54), (81, 54), (82, 54), (83, 54), (84, 54), (85, 54), (86, 54), (87, 54), (88, 54), (89, 54), (90, 54),
(80, 55), (81, 55), (82, 55), (83, 55), (84, 55), (85, 55), (86, 55), (87, 55), (88, 55), (89, 55), (90, 55),
(80, 56), (81, 56), (82, 56), (83, 56), (84, 56), (85, 56), (86, 56), (87, 56), (88, 56), (89, 56), (90, 56),
(80, 57), (81, 57), (82, 57), (83, 57), (84, 57), (85, 57), (86, 57), (87, 57), (88, 57), (89, 57), (90, 57),
(80, 58), (81, 58), (82, 58), (83, 58), (84, 58), (85, 58), (86, 58), (87, 58), (88, 58), (89, 58), (90, 58),
(80, 59), (81, 59), (82, 59), (83, 59), (84, 59), (85, 59), (86, 59), (87, 59), (88, 59), (89, 59), (90, 59),
(80, 60), (81, 60), (82, 60), (83, 60), (84, 60), (85, 60), (86, 60), (87, 60), (88, 60), (89, 60), (90, 60),
(80, 61), (81, 61), (82, 61), (83, 61), (84, 61), (85, 61), (86, 61), (87, 61), (88, 61), (89, 61), (90, 61),
(80, 62), (81, 62), (82, 62), (83, 62), (84, 62), (85, 62), (86, 62), (87, 62), (88, 62), (89, 62), (90, 62),
(80, 63), (81, 63), (82, 63), (83, 63), (84, 63), (85, 63), (86, 63), (87, 63), (88, 63), (89, 63), (90, 63),
(80, 64), (81, 64), (82, 64), (83, 64), (84, 64), (85, 64), (86, 64), (87, 64), (88, 64), (89, 64), (90, 64),
(80, 65), (81, 65), (82, 65), (83, 65), (84, 65), (85, 65), (86, 65), (87, 65), (88, 65), (89, 65), (90, 65),
(80, 66), (81, 66), (82, 66), (83, 66), (84, 66), (85, 66), (86, 66), (87, 66), (88, 66), (89, 66), (90, 66),
(80, 67), (81, 67), (82, 67), (83, 67), (84, 67), (85, 67), (86, 67), (87, 67), (88, 67), (89, 67), (90, 67),
(80, 68), (81, 68), (82, 68), (83, 68), (84, 68), (85, 68), (86, 68), (87, 68), (88, 68), (89, 68), (90, 68),
(80, 69), (81, 69), (82, 69), (83, 69), (84, 69), (85, 69), (86, 69), (87, 69), (88, 69), (89, 69), (90, 69),
(80, 70), (81, 70), (82, 70), (83, 70), (84, 70), (85, 70), (86, 70), (87, 70), (88, 70), (89, 70), (90, 70),
(80, 71), (81, 71), (82, 71), (83, 71), (84, 71), (85, 71), (86, 71), (87, 71), (88, 71), (89, 71), (90, 71),
(80, 72), (81, 72), (82, 72), (83, 72), (84, 72), (85, 72), (86, 72), (87, 72), (88, 72), (89, 72), (90, 72),
(80, 73), (81, 73), (82, 73), (83, 73), (84, 73), (85, 73), (86, 73), (87, 73), (88, 73), (89, 73), (90, 73),
(80, 74), (81, 74), (82, 74), (83, 74), (84, 74), (85, 74), (86, 74), (87, 74), (88, 74), (89, 74), (90, 74),
(80, 75), (81, 75), (82, 75), (83, 75), (84, 75), (85, 75), (86, 75), (87, 75), (88, 75), (89, 75), (90, 75),
(80, 76), (81, 76), (82, 76), (83, 76), (84, 76), (85, 76), (86, 76), (87, 76), (88, 76), (89, 76), (90, 76),
(80, 77), (81, 77), (82, 77), (83, 77), (84, 77), (85, 77), (86, 77), (87, 77), (88, 77), (89, 77), (90, 77),
(80, 78), (81, 78), (82, 78), (83, 78), (84, 78), (85, 78), (86, 78), (87, 78), (88, 78), (89, 78), (90, 78),
(80, 79), (81, 79), (82, 79), (83, 79), (84, 79), (85, 79), (86, 79), (87, 79), (88, 79), (89, 79), (90, 79),(20, 30), (20, 31), (20, 32), (20, 33), (20, 34), (20, 35), (20, 36), (20, 37),(20, 38),(20, 39),(20, 40),
(21, 30), (21, 31), (21, 32), (21, 33), (21, 34), (21, 35), (21, 36), (21, 37),(21, 38),(21, 39),(21, 40),
(22, 30), (22, 31), (22, 32), (22, 33), (22, 34), (22, 35), (22, 36), (22, 37),(22, 38),(22, 39),(22, 40),
(23, 30), (23, 31), (23, 32), (23, 33), (23, 34), (23, 35), (23, 36), (23, 37),(23, 38),(23, 39),(23, 40),
(24, 30), (24, 31), (24, 32), (24, 33), (24, 34), (24, 35), (24, 36), (24, 37),(24, 38),(24, 39),(24, 40),
(25, 30), (25, 31), (25, 32), (25, 33), (25, 34), (25, 35), (25, 36), (25, 37),(25, 38),(25, 39),(25, 40),
(26, 30), (26, 31), (26, 32), (26, 33), (26, 34), (26, 35), (26, 36), (26, 37),(26, 38),(26, 39),(26, 40),
(27, 30), (27, 31), (27, 32), (27, 33), (27, 34), (27, 35), (27, 36), (27, 37),(27, 38),(27, 39),(27, 40),
(28, 30), (28, 31), (28, 32), (28, 33), (28, 34), (28, 35), (28, 36), (28, 37),(28, 38),(28, 39),(28, 40),
(29, 30), (29, 31), (29, 32), (29, 33), (29, 34), (29, 35), (29, 36), (29, 37),(29, 38),(29, 39),(29, 40),
(30, 30), (30, 31), (30, 32), (30, 33), (30, 34), (30, 35), (30, 36), (30, 37),(30, 38),(30, 39),(30, 40),
(31, 30), (31, 31), (31, 32), (31, 33), (31, 34), (31, 35), (31, 36), (31, 37),(31, 38),(31, 39),(31, 40),
(32, 30), (32, 31), (32, 32), (32, 33), (32, 34), (32, 35), (32, 36), (32, 37),(32, 38),(32, 39),(32, 40),
(33, 30), (33, 31), (33, 32), (33, 33), (33, 34), (33, 35), (33, 36), (33, 37),(33, 38),(33, 39),(33, 40),
(34, 30), (34, 31), (34, 32), (34, 33), (34, 34), (34, 35), (34, 36), (34, 37),(34, 38),(34, 39),(34, 40),
(35, 30), (35, 31), (35, 32), (35, 33), (35, 34), (35, 35), (35, 36), (35, 37),(35, 38),(35, 39),(35, 40),
(36, 30), (36, 31), (36, 32), (36, 33), (36, 34), (36, 35), (36, 36), (36, 37),(36, 38),(36, 39),(36, 40),
(37, 30), (37, 31), (37, 32), (37, 33), (37, 34), (37, 35), (37, 36), (37, 37),(37, 38),(37, 39),(37, 40),
(38, 30), (38, 31), (38, 32), (38, 33), (38, 34), (38, 35), (38, 36), (38, 37),(38, 38),(38, 39),(38, 40),
(39, 30), (39, 31), (39, 32), (39, 33), (39, 34), (39, 35), (39, 36), (39, 37),(39, 38),(39, 39),(39, 40),
(40, 30), (40, 31), (40, 32), (40, 33), (40, 34), (40, 35), (40, 36), (40, 37),(40, 38),(40, 39),(40, 40),
(41, 30), (41, 31), (41, 32), (41, 33), (41, 34), (41, 35), (41, 36), (41, 37),(41, 38),(41, 39),(41, 40),
(42, 30), (42, 31), (42, 32), (42, 33), (42, 34), (42, 35), (42, 36), (42, 37),(42, 38),(42, 39),(42, 40),
(43, 30), (43, 31), (43, 32), (43, 33), (43, 34), (43, 35), (43, 36), (43, 37),(43, 38),(43, 39),(43, 40),
(44, 30), (44, 31), (44, 32), (44, 33), (44, 34), (44, 35), (44, 36), (44, 37),(44, 38),(44, 39),(44, 40),
(45, 30), (45, 31), (45, 32), (45, 33), (45, 34), (45, 35), (45, 36), (45, 37),(45, 38),(45, 39),(45, 40),
(46, 30), (46, 31), (46, 32), (46, 33), (46, 34), (46, 35), (46, 36), (46, 37),(46, 38),(46, 39),(46, 40),
(47, 30), (47, 31), (47, 32), (47, 33), (47, 34), (47, 35), (47, 36), (47, 37),(47, 38),(47, 39),(47, 40),
(48, 30), (48, 31), (48, 32), (48, 33), (48, 34), (48, 35), (48, 36), (48, 37),(48, 38),(48, 39),(48, 40),
(49, 30), (49, 31), (49, 32), (49, 33), (49, 34), (49, 35), (49, 36), (49, 37),(49, 38),(49, 39),(49, 40),
(50, 30), (50, 31), (50, 32), (50, 33), (50, 34), (50, 35), (50, 36), (50, 37),(50, 38),(50, 39),(50, 40),
(51, 30), (51, 31), (51, 32), (51, 33), (51, 34), (51, 35), (51, 36), (51, 37),(51, 38),(51, 39),(51, 40),
(52, 30), (52, 31), (52, 32), (52, 33), (52, 34), (52, 35), (52, 36), (52, 37),(52, 38),(52, 39),(52, 40),
(53, 30), (53, 31), (53, 32), (53, 33), (53, 34), (53, 35), (53, 36), (53, 37),(53, 38),(53, 39),(53, 40),
(54, 30), (54, 31), (54, 32), (54, 33), (54, 34), (54, 35), (54, 36), (54, 37),(54, 38),(54, 39),(54, 40),
(55, 30), (55, 31), (55, 32), (55, 33), (55, 34), (55, 35), (55, 36), (55, 37),(55, 38),(55, 39),(55, 40),
(56, 30), (56, 31), (56, 32), (56, 33), (56, 34), (56, 35), (56, 36), (56, 37),(56, 38),(56, 39),(56, 40),
(57, 30), (57, 31), (57, 32), (57, 33), (57, 34), (57, 35), (57, 36), (57, 37),(57, 38),(57, 39),(57, 40),
(58, 30), (58, 31), (58, 32), (58, 33), (58, 34), (58, 35), (58, 36), (58, 37),(58, 38),(58, 39),(58, 40),
(59, 30), (59, 31), (59, 32), (59, 33), (59, 34), (59, 35), (59, 36), (59, 37),(59, 38),(59, 39),(59, 40),
(60, 30), (60, 31), (60, 32), (60, 33), (60, 34), (60, 35), (60, 36), (60, 37),(60, 38),(60, 39),(60, 40),
(61, 30), (61, 31), (61, 32), (61, 33), (61, 34), (61, 35), (61, 36), (61, 37),(61, 38),(61, 39),(61, 40),
(62, 30), (62, 31), (62, 32), (62, 33), (62, 34), (62, 35), (62, 36), (62, 37),(62, 38),(62, 39),(62, 40),
(63, 30), (63, 31), (63, 32), (63, 33), (63, 34), (63, 35), (63, 36), (63, 37),(63, 38),(63, 39),(63, 40),
(64, 30), (64, 31), (64, 32), (64, 33), (64, 34), (64, 35), (64, 36), (64, 37),(64, 38),(64, 39),(64, 40),
(65, 30), (65, 31), (65, 32), (65, 33), (65, 34), (65, 35), (65, 36), (65, 37),(65, 38),(65, 39),(65, 40),
(66, 30), (66, 31), (66, 32), (66, 33), (66, 34), (66, 35), (66, 36), (66, 37),(66, 38),(66, 39),(66, 40),
(67, 30), (67, 31), (67, 32), (67, 33), (67, 34), (67, 35), (67, 36), (67, 37),(67, 38),(67, 39),(67, 40),
(68, 30), (68, 31), (68, 32), (68, 33), (68, 34), (68, 35), (68, 36), (68, 37),(68, 38),(68, 39),(68, 40),
(69, 30), (69, 31), (69, 32), (69, 33), (69, 34), (69, 35), (69, 36), (69, 37),(69, 38),(69, 39),(69, 40),
(70, 30), (70, 31), (70, 32), (70, 33), (70, 34), (70, 35), (70, 36), (70, 37),(70, 38),(70, 39),(70, 40),
(71, 30), (71, 31), (71, 32), (71, 33), (71, 34), (71, 35), (71, 36), (71, 37),(71, 38),(71, 39),(71, 40),
(72, 30), (72, 31), (72, 32), (72, 33), (72, 34), (72, 35), (72, 36), (72, 37),(72, 38),(72, 39),(72, 40),
(73, 30), (73, 31), (73, 32), (73, 33), (73, 34), (73, 35), (73, 36), (73, 37),(73, 38),(73, 39),(73, 40),
(74, 30), (74, 31), (74, 32), (74, 33), (74, 34), (74, 35), (74, 36), (74, 37),(74, 38),(74, 39),(74, 40),
(75, 30), (75, 31), (75, 32), (75, 33), (75, 34), (75, 35), (75, 36), (75, 37),(75, 38),(75, 39),(75, 40),
(76, 30), (76, 31), (76, 32), (76, 33), (76, 34), (76, 35), (76, 36), (76, 37),(76, 38),(76, 39),(76, 40),
(77, 30), (77, 31), (77, 32), (77, 33), (77, 34), (77, 35), (77, 36), (77, 37),(77, 38),(77, 39),(77, 40),
(78, 30), (78, 31), (78, 32), (78, 33), (78, 34), (78, 35), (78, 36), (78, 37),(78, 38),(78, 39),(78, 40),
(79, 30), (79, 31), (79, 32), (79, 33), (79, 34), (79, 35), (79, 36), (79, 37),(79, 38),(79, 39),(79, 40),

 ]'''
    OBJECT_WALLS = [(53, 50), (50, 53), (53, 53),
(53, 50), (50, 53), (53, 47),
(53, 50), (50, 47), (47, 53),
(47, 50), (50, 47), (47, 47),
(52, 50), (50, 52), (52, 52),
(52, 50), (50, 52), (52, 48),
(52, 50), (50, 48), (48, 52),
(48, 50), (50, 48), (48, 48),
(51, 50), (50, 51), (51, 51),
(51, 50), (50, 51), (51, 49),
(51, 50), (50, 49), (49, 51),
(49, 50), (50, 49), (49, 49),(43, 40), (40, 43), (43, 43),
(43, 40), (40, 43), (43, 37),
(43, 40), (40, 37), (37, 43),
(37, 40), (40, 37), (37, 37),
(42, 40), (40, 42), (42, 42),
(42, 40), (40, 42), (42, 38),
(42, 40), (40, 38), (38, 42),
(38, 40), (40, 38), (38, 38),
(41, 40), (40, 41), (41, 41),
(41, 40), (40, 41), (41, 39),
(41, 40), (40, 39), (39, 41),
(39, 40), (40, 39), (39, 39),(43, 50), (40, 53), (43, 53),
(43, 50), (40, 53), (43, 47),
(43, 50), (40, 47), (37, 53),
(37, 50), (40, 47), (37, 47),
(42, 50), (40, 52), (42, 52),
(42, 50), (40, 52), (42, 48),
(42, 50), (40, 48), (38, 52),
(38, 50), (40, 48), (38, 48),
(41, 50), (40, 51), (41, 51),
(41, 50), (40, 51), (41, 49),
(41, 50), (40, 49), (39, 51),
(39, 50), (40, 49), (39, 49),
(43, 40), (40, 43), (43, 43),
(43, 40), (40, 43), (43, 37),
(43, 40), (40, 37), (37, 43),
(37, 40), (40, 37), (37, 37),
(42, 40), (40, 42), (42, 42),
(42, 40), (40, 42), (42, 38),
(42, 40), (40, 38), (38, 42),
(38, 40), (40, 38), (38, 38),
(41, 40), (40, 41), (41, 41),
(41, 40), (40, 41), (41, 39),
(41, 40), (40, 39), (39, 41),
(39, 40), (40, 39), (39, 39),(63, 40), (60, 43), (63, 43),
(63, 40), (60, 43), (63, 37),
(63, 40), (60, 37), (57, 43),
(57, 40), (60, 37), (57, 37),
(62, 40), (60, 42), (62, 42),
(62, 40), (60, 42), (62, 38),
(62, 40), (60, 38), (58, 42),
(58, 40), (60, 38), (58, 38),
(61, 40), (60, 41), (61, 41),
(61, 40), (60, 41), (61, 39),
(61, 40), (60, 39), (59, 41),
(59, 40), (60, 39), (59, 39),(63, 60), (60, 63), (63, 63),
(63, 60), (60, 63), (63, 57),
(63, 60), (60, 57), (57, 63),
(57, 60), (60, 57), (57, 57),
(62, 60), (60, 62), (62, 62),
(62, 60), (60, 62), (62, 58),
(62, 60), (60, 58), (58, 62),
(58, 60), (60, 58), (58, 58),
(61, 60), (60, 61), (61, 61),
(61, 60), (60, 61), (61, 59),
(61, 60), (60, 59), (59, 61),
(59, 60), (60, 59), (59, 59),(63, 50), (60, 53), (63, 53),
(63, 50), (60, 53), (63, 47),
(63, 50), (60, 47), (57, 53),
(57, 50), (60, 47), (57, 47),
(62, 50), (60, 52), (62, 52),
(62, 50), (60, 52), (62, 48),
(62, 50), (60, 48), (58, 52),
(58, 50), (60, 48), (58, 48),
(61, 50), (60, 51), (61, 51),
(61, 50), (60, 51), (61, 49),
(61, 50), (60, 49), (59, 51),
(59, 50), (60, 49), (59, 49),
(53, 60), (50, 63), (53, 63),
(53, 60), (50, 63), (53, 57),
(53, 60), (50, 57), (47, 63),
(47, 60), (50, 57), (47, 57),
(52, 60), (50, 62), (52, 62),
(52, 60), (50, 62), (52, 58),
(52, 60), (50, 58), (48, 62),
(48, 60), (50, 58), (48, 58),
(51, 60), (50, 61), (51, 61),
(51, 60), (50, 61), (51, 59),
(51, 60), (50, 59), (49, 61),
(49, 60), (50, 59), (49, 59),(53, 40), (50, 43), (53, 43),
(53, 40), (50, 43), (53, 37),
(53, 40), (50, 37), (47, 43),
(47, 40), (50, 37), (47, 37),
(52, 40), (50, 42), (52, 42),
(52, 40), (50, 42), (52, 38),
(52, 40), (50, 38), (48, 42),
(48, 40), (50, 38), (48, 38),
(51, 40), (50, 41), (51, 41),
(51, 40), (50, 41), (51, 39),
(51, 40), (50, 39), (49, 41),
(49, 40), (50, 39), (49, 39),(43, 60), (40, 63), (43, 63),
(43, 60), (40, 63), (43, 57),
(43, 60), (40, 57), (37, 63),
(37, 60), (40, 57), (37, 57),
(42, 60), (40, 62), (42, 62),
(42, 60), (40, 62), (42, 58),
(42, 60), (40, 58), (38, 62),
(38, 60), (40, 58), (38, 58),
(41, 60), (40, 61), (41, 61),
(41, 60), (40, 61), (41, 59),
(41, 60), (40, 59), (39, 61),
(39, 60), (40, 59), (39, 59),]
    '''[(47, 47), (53, 47), (47, 53), (53, 53), (50, 49), (50, 51), (49, 50), (51, 50), (49, 49), (51, 51), (51, 49), (49, 51), (49, 48), (50, 48), (51, 48), (48, 48), (48, 49), (48,50), (48, 51), (48, 52),(49, 52),(50, 52),(51, 52), (52, 48), (52, 49), (52, 50), (52, 51), (52, 52),
                     (53, 37), (47, 37), (47, 43), (53, 43), (50, 40),(51, 39), (50, 41), (49, 40), (51, 40), (49, 39), (51, 41), (51, 39), (49, 41), (49, 38), (50, 38), (51, 38), (48, 38), (48, 39), (48,40), (48, 41), (48, 42),(49, 42),(50, 42),(51, 42), (52, 38), (52, 39), (52, 40), (52, 41), (52, 42),
                     (60, 40), (60, 39), (60, 41), (59, 40), (61, 40), (59, 39), (61, 41), (61, 39), (59, 41), (59, 38), (60, 38), (61, 38), (58, 38), (58, 39), (58,40), (58, 41), (58, 42),(59, 42),(60, 42),(61, 42), (62, 38), (62, 39), (62, 40), (62, 41), (62, 42),
                     (60, 50), (60, 49), (60, 51), (59, 50), (61, 50), (59, 49), (61, 51), (61, 49), (59, 51), (59, 48), (60, 48), (61, 48), (58, 48), (58, 49), (58,50), (58, 51), (58, 52),(59, 52),(60, 52),(61, 52), (62, 48), (62, 49), (62, 50), (62, 51), (62, 52),
                     (40, 50), (40, 49), (40, 51), (39, 50), (41, 50), (39, 49), (41, 51), (41, 49), (39, 51), (39, 48), (40, 48), (41, 48), (38, 48), (38, 49), (38,50), (38, 51), (38, 52),(39, 52),(40, 52),(41, 52), (42, 48), (42, 49), (42, 50), (42, 51), (42, 52),
                     (40, 60), (40, 59), (40, 61), (39, 60), (41, 60), (39, 59), (41, 61), (41, 59), (39, 61), (39, 58), (40, 58), (41, 58), (38, 58), (38, 59), (38,60), (38, 61), (38, 62),(39, 62),(40, 62),(41, 62), (42, 58), (42, 59), (42, 60), (42, 61), (42, 62),
                     (50, 60), (50, 59), (50, 61), (49, 60), (51, 60), (49, 59), (51, 61), (51, 59), (49, 61), (49, 58), (50, 58), (51, 58), (48, 58), (48, 59), (48,60), (48, 61), (48, 62),(49, 62),(50, 62),(51, 62), (52, 58), (52, 59), (52, 60), (52, 61), (52, 62),
                     (60, 60), (60, 59), (60, 61), (59, 60), (61, 60), (59, 59), (61, 61), (61, 59), (59, 61), (59, 58), (60, 58), (61, 58), (58, 58), (58, 59), (58,60), (58, 61), (58, 62),(59, 62),(60, 62),(61, 62), (62, 58), (62, 59), (62, 60), (62, 61), (62, 62),
                     (40, 40), (40, 39), (40, 41), (39, 40), (41, 40), (39, 39), (41, 41), (41, 39), (39, 41), (39, 38), (40, 38), (41, 38), (38, 38), (38, 39), (38,40), (38, 41), (38, 42),(39, 42),(40, 42),(41, 42), (42, 38), (42, 39), (42, 40), (42, 41), (42, 42)]
                     '''
    #graph = #gazebo map is default -10.0m * 10.0m
    #if (xLoc < 0)
    #    xLoc = xLoc * -1
    #if (yLoc < 0)
    #    yLoc = yLoc * -1
    graph = SquareGrid(96,96)
    graph.walls = OBJECT_WALLS #TEST_WALLS # long list, [(21, 0), (21, 2), ...]
    def cordinateToGridx(x):

        c = 0.0
        if(x == 0.0):
            c = 50.0
        if(x < 0.0):
            c = abs(x) * 10.0
            c =  50 - c

        if(x > 0.0):
            c = abs(x) * 10.0
            c = c + 50.0
            print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"+str(c))

        return c

    def cordinateToGridy(y):
        c = 0.0
        if(y == 0.0):
            c = 50.0
        if(y > 0.0):
            c = abs(y) * 10.0
            c =  50.0 - c

        if(y < 0.0):
            c = abs(y) * 10.0
            c = c + 50.0

        return c

    F = open("Location.txt","r")
    testFile = F.read()
    mylist = testFile.split(',')
    start = "("+str(int(cordinateToGridx(float(mylist[0][:4]))))+", "+str(int(cordinateToGridy(float(mylist[1][:4]))))+")"
    print("________________________________________________________________________"+ mylist[0][:4] + str(start))
    xg, yg  = input("| x | y | z |\n").split()
    print(start)
    start = "("+start+")"
    goal = "("+str(int(cordinateToGridx(float(xg))))+", "+str(int(cordinateToGridy( float(yg))))+")"

    #start = cordinateToGrid(2.0, 2.0)
    #goal = cordinateToGrid(7.0, 7.0)

    print("Start===========" + str(start))
    print("Goal============" + str(goal))

    #tupleGoal = literal
    #print(eval(goal))
    test = a_star_search(graph, eval(start), eval(goal))
    #test = a_star_search(graph, (1,1), (40,25))
    aa = test
    #print (aa)
    #print (currentModels)
    graph.road = aa

main()

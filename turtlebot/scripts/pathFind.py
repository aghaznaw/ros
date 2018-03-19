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
    #gazeboMap =
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

    #def empty(self):
    #    return len(self.elements) == 0

    #def put(self, x):
    #    self.elements.append(x)

    #def get(self):
        #return self.elements.popleft()

    def pathFindJereSolution():
        file_dir = os.path.dirname(__file__)
        print("I am here")
        #rel_path =
        file_subdir = "test.txt"
        file_path = os.path.join(file_dir, file_subdir)
        modelList = []
        strings = ("model name=")
        print(file_path)
    	#path = ""

    	#priorityNodes = [[], []]

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
    while current != start:
        path.append(current)
        current = came_from[current]

        if (current[0] == firstLast[0] and current[1] != firstLast[1]):
            #if x here is the same as x in firstlast, while y is not..
            #we are going to this direction
            if (current[0] != secondLast[0]):
                direction = "downRight"
                print("Going diagonally down at ")
                print(current)
            else:
                print("Going down at ")
                print(current)
                direction = "down"
        if (current[1] == firstLast[1] and current[0] != firstLast[0]):
            if (current[1] != secondLast[1] and current[0] == secondLast[0]):
                direction = "rightUp"
                print("Going diagonally right at ")
                print(current)
            else:
                print("Going right at ")
                print(current)
                direction = "right"
        firstLast = current
        secondLast = firstLast
        #if ((current[0] != secondLast[0] and current[1] != )  or current[1] != secondLast[1])):
        #    print("Found a corner at")
        #    print(current)
    path.append(start) # optional
    path.reverse() # optional
    return path

def heuristic(a, b):

    #return abs(a[0] - b[0]) + abs(a[1] - b[1]) manhattan
    return math.sqrt(((a[0] - b[0])**2) + ((a[1] - b[1])**2)) #euclidean

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
            new_cost = cost_so_far[current]# + graph.cost(current, next)

            if next not in cost_so_far or new_cost < cost_so_far[next]:

                print(next)
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current

    print("This is the path: ")
    print(reconstruct_path(came_from, start=(1,1), goal=(goal)))
    draw_grid(graph, width=2, path=reconstruct_path(came_from, start=(1,1), goal=(goal)))
    return came_from, cost_so_far
def main():

    loader = mapLoader()
    print(loader)
    currentModels = loader.pathFindJereSolution
    print (currentModels)
    TEST_WALLS = [from_id_width(id, width=30) for id in [21,22,51,52,81,82,93,94,111,112,123,124,133,134,141,142,153,154,163,164,171,172,173,174,175,183,184,193,194,201,202,203,204,205,213,214,223,224,243,244,253,254,273,274,283,284,303,304,313,314,333,334,343,344,373,374,403,404,433,434]]
    TEST_WALLS.append((33,2))
    #graph = GazeboGraph
    graph = SquareGrid(50,50)
    graph.walls = TEST_WALLS # long list, [(21, 0), (21, 2), ...]

    #graph = diagram4
    goal = input("Please enter the ending goal: ")
    #tupleGoal = literal
    print(eval(goal))
    test = a_star_search(graph, (1,1), eval(goal))
    #test = a_star_search(graph, (1,1), (40,25))
    aa = test
    print (aa)
    graph.road = aa

main()

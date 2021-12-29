# Brian Yang 101140298 COMP3106 Assignment 1

# input_filepath contains the full path to a CSV file with the input grid

# optimal_path is a list of tuples indicated the optimal path from start to goal
# explored_list is the list of nodes explored during search
# optimal_path_cost is the cost of the optimal path from the start state to the goal state
def pathfinding(input_filepath):

    # creates grid and all the nodes within it as a 2D list
    grid = createGrid(input_filepath)

    # coordinates of all explored nodes
    explored_list = graphSearch(grid)

    # gets the goal node
    goalNode = findNode(grid, "G")
    if goalNode == None:
        return None

    # gets the optimal path from start to the node specified (which is the goal)
    optimal_path = getOptimalPath(goalNode)
    optimal_path_cost = goalNode.pathCost

    return optimal_path, explored_list, optimal_path_cost

# represents a node on the grid, with correponding coordinates and heuristics
class Node:
    heuristic = 0
    # the type (G, O, H, X, S)
    type = "N/A"
    # where row is first element, col is second
    coordinates = (0, 0)
    validMove = True;

    # each explored node has a parent on the route (will be defined during search)
    parent = None
    pathCost = 0

    def __init__(self, t, c):
        self.type = t
        self.coordinates = c
    # calcualtes the heuristc for the node
    def calculateHeuristic(self, goalCoordinates):
        ROW = 0
        COL = 1
        selfRow = self.coordinates[ROW]
        selfCol = self.coordinates[COL]
        goalRow = goalCoordinates[ROW]
        goalCol = goalCoordinates[COL]

        # uses Manhatten distance to calculate heuristic
        self.heuristic = abs(selfRow - goalRow) + abs(selfCol-goalCol)
        self.func = self.heuristic

# does the actual A* search
def graphSearch(grid):
    # the frontier initally only consists of start-states
    # hardcoded since the edgelength between 2 adj nodes are always 1
    EDGE_WEIGHT = 1
    frontier = []
    explored = []
    # assumes that we have one start and one goal state*****
    startNode = findNode(grid, "S")
    if startNode == None:
        return None
    frontier.append(startNode)
    # create searchSTate for each node to add to frontier

    while True:
        # lead is the currently explored node
        leaf = frontier.pop(0)
        # if the highest priority node is goal node, then we're good
        explored.append(leaf)
        if leaf.type == "G":
            break

        # checks each neighbor's adj nodes
        for neighbor in findAdjacent(grid, leaf):

            # only add neighbors to frontier if theyre able for agent to move into spot
            if not neighbor.validMove:
                continue

            # Notice that the edge weight between any 2 given nodes are the same (which is 1)
            curr_path_cost = leaf.pathCost + EDGE_WEIGHT
            if (neighbor not in frontier) and (neighbor not in explored):
                updateNode(neighbor, leaf, curr_path_cost)
                frontier = insertInFrontier(frontier, neighbor)
            elif (neighbor in frontier) and (curr_path_cost < neighbor.pathCost):
                updateNode(neighbor, leaf, curr_path_cost)
                # remove the neighbor from frontier to update its position on the prior queue
                # by re-adding it
                frontier.remove(neighbor)
                frontier = insertInFrontier(frontier, neighbor)


    # return the coordinates for each of the explored nodes
    exploredCoordinates = []
    for node in explored:
        exploredCoordinates.append(node.coordinates)
    return exploredCoordinates

# updates the nodes during search, helper func
def updateNode(node, newParent, newPathCost):
    node.parent = newParent
    node.pathCost = newPathCost

# inserts the node into the frontier at correct location
# if tie, then the node on the frontier longest without being updated gets priority
def insertInFrontier(frontier, newNode):
    newNodePriorFunc = newNode.pathCost + newNode.heuristic
    for i in range(len(frontier)):
        # f(n) = n.pathcost + heuristic
        nodePriorityFunc = frontier[i].pathCost + frontier[i].heuristic
        if newNodePriorFunc < nodePriorityFunc:
            frontier.insert(i, newNode)
            return frontier
        # if tied, then check heuristic
        elif (newNodePriorFunc == nodePriorityFunc) and (newNode.heuristic < frontier[i].heuristic):
            frontier.insert(i, newNode)
            return frontier
        # this means that those that stayed in froniter longer have more priority
    frontier.append(newNode)
    return frontier

# gets the optimal path from the start to the node passed in
# returns a list of coordinates. Each coordinate is a step in the path
def getOptimalPath(goalNode):
    pathCoordinates = []
    currNode = goalNode
    while currNode != None:
        # adds the parent to front of list (since parent comes first)
        pathCoordinates.insert(0, currNode.coordinates)
        currNode = currNode.parent
    return pathCoordinates

# finds all adjacent nodes
def findAdjacent(grid, node):
    ROW = 0
    COL = 1

    colMaxIndex = len(grid[ROW])-1
    rowMaxIndex = len(grid)-1
    nodeRow = node.coordinates[ROW]
    nodeCol = node.coordinates[COL]

    # we can use this assumption because we know that the grid is a rectangle
    adjNodes = []
    if (nodeRow + 1) <= rowMaxIndex:
        adjNodes.append(grid[nodeRow+1][nodeCol])
    if (nodeCol - 1) >= 0:
        adjNodes.append(grid[nodeRow][nodeCol-1])
    if (nodeRow - 1) >= 0:
        adjNodes.append(grid[nodeRow-1][nodeCol])
    if (nodeCol + 1) <= colMaxIndex:
        adjNodes.append(grid[nodeRow][nodeCol+1])

    return adjNodes

# designates a node as an invalid move if its beside an H or is an X
def findInvalidSpots(grid):
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j].type == "X":
                grid[i][j].validMove = False
            # for hazards, all adj nodes are not valid moves either
            elif grid[i][j].type == "H":
                grid[i][j].validMove = False
                adjNodes = findAdjacent(grid, grid[i][j])
                for adjNode in adjNodes:
                    adjNode.validMove = False

# calculates the heuristic for each node in relation to goal node
def calculateHeuristics(grid):
    goalCoord = findNode(grid, "G").coordinates
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            grid[i][j].calculateHeuristic(goalCoord)

# finds the first node found that matches the type
# this works fine for find start and goal node since we assume theres only 1
def findNode(grid, type):
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j].type == type:
                return grid[i][j]
    return None

# creates all the nodes (also defines their attributes) and places them into a 2D list
def createGrid(input_filepath):
    grid = []
    f = open(input_filepath, 'r')
    lines = f.readlines()
    row = 0
    for line in lines:
        # appends an empty list to hold new nodes in that new row
        grid.append([])
        column = 0
        line = line.strip()
        for character in line:
            # makes sure to ignore all the commas
            if character == ',':
                continue
            # creates a new state with the type and coordinates
            node = Node(character, (row, column))
            grid[row].append(node)
            column += 1
        row += 1
    f.close()

    # defines whether each node is a valid path for route checking
    findInvalidSpots(grid)
    # calculates the heuristic for each node
    calculateHeuristics(grid)

    return grid

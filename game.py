class Grid():
    def __init__(self, width, height, initialValue = False):
        if initialValue not in [False, True]: raise Exception('Grids can only contain booleans')
        self.width = width
        self.height = height
        self.data = [[initialValue for y in range(height)] for x in range(width)]

    def __getitem__(self, i):
        return self.data[i]

    def __setitem__(self, key, item):
        self.data[key] = item

    def shallowCopy(self):
        g = Grid(self.width, self.height)
        g.data = self.data
        return g

    def toString(self, grid):
        print "\n".join("\t".join(map(str,l)) for l in grid)


class GameState():
    def __init__(self, prevState = None):
        self.lose = False
        self.win = False
        self.scoreChange = 0
        self.TIME_PENALTY = 1

        self.wallLocations = [(1, 1), (1, 3), (3, 1), (3, 3)]
        self.walls = Grid(5, 5, False)
        for wall in self.wallLocations:
            self.walls[wall[0]][wall[1]] = True

        if prevState != None:
            self.food = prevState.food.shallowCopy()
            self.pills = prevState.pills[:]
            self.score = prevState.score
            self.numFood = prevState.numFood
            self.positions = prevState.positions
            self.directions = prevState.directions
            self.boostTimer = prevState.boostTimer
        else: # beginning game state
            self.score = 0
            self.numFood = 16
            self.boostTimer = 0
            self.pills = [(0, 4), (4, 0)]
            self.positions = [(2, 2), (0, 0), (4, 4)]
            self.directions = ["NORTH", "NORTH", "NORTH"]

            self.food = Grid(5, 5, True)
            # Locations where pacman, ghosts, and speed boosts start so there is no food there
            self.food[0][0] = False
#            self.food[0][4] = False
#            self.food[4][0] = False
            self.food[4][4] = False
            self.food[2][2] = False
            for wall in self.wallLocations:
                self.food[wall[0]][wall[1]] = False
                    

    def getLegalMoves(self, agentIndex):
        if self.isWin() or self.isLose(): return []

        currentPos = self.positions[agentIndex]
        legalMoves = []
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                newPos = (currentPos[0] + i, currentPos[1] + j)
                if newPos[0] < 0 or newPos[0] > 4 or newPos[1] < 0 or newPos[1] > 4: # out of index
                    continue
                if self.walls[newPos[0]][newPos[1]]:
                    continue

                if agentIndex != 0: # running into another ghost is not a valid move
                    for otherAgent, otherPos in enumerate(self.positions[1:]): 
                        if agentIndex != otherAgent:
                            if newPos == otherPos:
                                continue

                if i == -1 and j == 0:
                    legalMoves.append("WEST")
                if i == 1 and j == 0:
                    legalMoves.append("EAST")
                if i == 0 and j == -1:
                    legalMoves.append("SOUTH")
                if i == 0 and j == 1:
                    legalMoves.append("NORTH")
        return legalMoves

    def generateSuccessor(self, agentIndex, action):
        # Check that successors exist
        if self.isWin() or self.isLose(): raise Exception('Can\'t generate a successor of a terminal state.')
    
        # Copy current state
        state = GameState(self)

        # Let agent's logic deal with its action's effects on the board
        self.applyAction(agentIndex, state, action)

        if state.positions[1] == state.positions[0] or state.positions[2] == state.positions[0]: # if ghost and pacman are in the same spot
            state.lose = True
            state.scoreChange += -100

        # Time passes
        if agentIndex == 0:
            state.scoreChange += -self.TIME_PENALTY # Penalty for waiting around

        # Book keeping
        # state.agentMoved = agentIndex
        state.score += state.scoreChange
        return state

    def applyAction(self, agentIndex, state, action):
        if action == "NORTH":
            i = 0
            j = 1
        elif action == "EAST":
            i = 1
            j = 0
        elif action == "WEST":
            i = -1
            j = 0
        elif action == "SOUTH":
            i = 0
            j = -1

        oldx, oldy = state.positions[agentIndex]
        state.positions[agentIndex] = (oldx + i, oldy + j)

        state.directions[agentIndex] = action
        
        if agentIndex == 0:
            self.consume(state.positions[agentIndex], state)

    def consume(self, position, state):
        x, y = position

        numFood = state.getNumFood()

        # Eat food
        if state.food[x][y]:
            state.scoreChange += 10
            state.food[x][y] = False
            numFood -= 1
            state.setNumFood(numFood)
            
        if numFood == 0 and not state.isLose():
            state.scoreChange += 500
            state.win = True

        # Eat capsule
        if position in state.getPills():
            # speed boost
            state.boostTimer = 3
            state.pills.remove(position)
          # remove from virtual map

    def get_all_coordinates(self):
        coordinates = []
        for agentIndex in range(3):
            coordinates.append(self.get_coordinates(agentIndex))
        return coordinates

    def get_coordinates(self, agentIndex):
        x, y = self.positions[agentIndex]
        x_position = -120 + (60*x)
        y_position = -120 + (60*y)
        return (x_position, y_position)
    
    def get_grid_coordinates(self, x, y):
        x_position  = -120 + (60*x)
        y_position = -120 + (60*y)
        return (x_position, y_position)
    
    def getPills(self):
        return self.pills

    def getNumFood(self):
        return self.numFood

    def setNumFood(self, newFood):
        self.numFood = newFood

    def isWin(self):
        return self.win

    def isLose(self):
        return self.lose


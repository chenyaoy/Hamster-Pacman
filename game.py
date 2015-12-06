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
        else: # beginning game state
            self.score = 0
            self.numFood = 16
            self.pills = [(0, 4), (4, 0)]
            self.positions = [(2, 2), (0, 0), (4, 4)]

            self.food = Grid(5, 5, True)
            # Locations where pacman, ghosts, and speed boosts start so there is no food there
            self.food[0][0] = False
            self.food[0][4] = False
            self.food[4][0] = False
            self.food[4][4] = False
            self.food[2][2] = False
            for wall in self.wallLocations:
                self.food[wall[0]][wall[1]] = False
                    

    def getLegalMoves(agentIndex):
        if self.isWin() or self.isLose(): return []

        currentPos = self.positions[agent]
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
                    legalMoves.append("West")
                if i == 1 and j == 0:
                    legalMoves.append("East")
                if i == 0 and j == -1:
                    legalMoves.append("South")
                if i == 0 and j == 1:
                    legalMoves.append("North")
        return legalMoves

    def generateSuccessor(agentIndex, action):
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
            state.scoreChange += -TIME_PENALTY # Penalty for waiting around

        # Book keeping
        # state.agentMoved = agentIndex
        state.score += state.scoreChange
        return state

    def applyAction(agentIndex, state, action):
        if action == "North":
            i = 0
            j = 1
        elif action == "East":
            i = 1
            j = 0
        elif action == "West":
            i = -1
            j = 0
        elif action == "South":
            i = 0
            j = -1

        state.positions[agentIndex][0] += i
        state.positions[agentIndex][1] += j

        if agentIndex == 0:
            consume(state.positions[agentIndex], state)

    def consume( position, state ):
        x, y = position
        # Eat food
        if state.food[x][y]:
            state.scoreChange += 10
            state.food[x][y] = False
            numFood = state.getNumFood()
        if numFood == 0 and not state.data._lose:
            state.scoreChange += 500
            state.win = True

        # Eat capsule
        if position in state.getPills():
            # speed boost
            state.pills.remove(position)
          # remove from virtual map

    def getPills(self):
        return self.pills

    def getNumFood(self):
        return self.numFood

    def isWin(self):
        return self.win

    def isLose(self):
        return self.lose


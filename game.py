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

    def toString(self, grid):
        print "\n".join("\t".join(map(str,l)) for l in grid)


class GameStateData():
    def __init__(self, prevState = None):
        self.lose = False
        self.win = False
        self.score = 0
        self.pacmanPos = (2, 2)
        ghostPos = [(0, 0), (4, 4)]

        self.wallLocations = [(1, 1), (1, 3), (3, 1), (3, 3)]
        self.food = Grid(5, 5, True)
        # Locations where pacman, ghosts, and speed boosts start so there is no food there
        self.food[0][0] = False
        self.food[0][4] = False
        self.food[4][0] = False
        self.food[4][4] = False
        self.food[2][2] = False
        for wall in self.wallLocations:
            self.food[wall[0]][wall[1]] = False
                
        self.walls = Grid(5, 5, False)
        for wall in self.wallLocations:
            self.walls[wall[0]][wall[1]] = True




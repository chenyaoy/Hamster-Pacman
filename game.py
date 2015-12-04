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

def printGrid(grid):
    print "\n".join("\t".join(map(str,l)) for l in grid)

    
walls = []
walls.append((1, 1))
walls.append((1, 3))
walls.append((3, 1))
walls.append((3, 3))


foodGrid = Grid(5, 5, True)
# Locations where pacman, ghosts, and speed boosts start so there is no food there
foodGrid[0][0] = False
foodGrid[0][4] = False
foodGrid[4][0] = False
foodGrid[4][4] = False
foodGrid[2][2] = False
for wall in walls:
    foodGrid[wall[0]][wall[1]] = False

wallGrid = Grid(5, 5, False)
for wall in walls:
    wallGrid[wall[0]][wall[1]] = True



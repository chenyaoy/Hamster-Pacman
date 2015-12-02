class Grid():
    def __init__(self, width, height, initialValue = False):
        if initialValue not in [False, True]: raise Exception('Grids can only contain booleans')
        self.width = width
        self.height = height
        self.data = [[initialValue for y in range(height)] for x in range(width)]

    # def __getitem__(self, i, j):
    #     return self.data[i][j]
    def __getitem__(self, i):
        return self.data[i]

    def __setitem__(self, key, item):
        self.data[key] = item

walls = []
walls.append((1, 1))
walls.append((1, 3))
walls.append((3, 1))
walls.append((3, 3))


food = Grid(5, 5, True)
food[0][0] = False
food[0][4] = False
food[4][0] = False
food[4][4] = False
food[2][2] = False
for wall in walls:
    food[wall[0]][wall[1]] = False
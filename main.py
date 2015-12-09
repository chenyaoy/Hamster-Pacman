import Tkinter as tk 
import time 
import sys
from HamsterAPI.comm_ble import RobotComm
import math
import threading
from tk_hamster_GUI import *
import numpy as np
import game
import random, copy
import util

UPDATE_INTERVAL = 30
BOARD_SIZE = 5
gMaxRobotNum = 3 # max number of robots to control
gQuit = False
m = None

BLACK_CUTOFF = 75

class VirtualWorldGui:
    def __init__(self, vWorld, m):
        self.vworld = vWorld

        self.button0 = tk.Button(m,text="Grid")
        self.button0.pack(side='left')
        self.button0.bind('<Button-1>', self.drawGrid)

        self.button1 = tk.Button(m,text="Clear")
        self.button1.pack(side='left')
        self.button1.bind('<Button-1>', self.clearCanvas)

        self.button2 = tk.Button(m,text="Reset")
        self.button2.pack(side='left')
        self.button2.bind('<Button-1>', self.resetvRobot)

        self.button3 = tk.Button(m,text="Map")
        self.button3.pack(side='left')
        self.button3.bind('<Button-1>', self.drawMap)
    
        self.button4 = tk.Button(m, text="AI Mode")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.AI_mode)
        
        self.button5 = tk.Button(m, text="Human Mode")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.Human_mode)

        self.button9 = tk.Button(m,text="Exit")
        self.button9.pack(side='left')
        self.button9.bind('<Button-1>', stopProg)


    def AI_mode(self, event=None):
        # AI_thread = threading.Thread(target= AI_game)
        # AI_thread.daemon = True
        # AI_thread.start()
        run_game_thread = threading.Thread(target=run_game, args=("AI",))
        run_game_thread.start()
    
    def Human_mode(self, event=None):
        # Human_thread= threading.Thread(target=human_game)
        # Human_thread.daemon = True
        # Human_thread.start()
        run_game_thread = threading.Thread(target=run_game, args=("Human",))
        run_game_thread.start()


    def resetvRobot(self, event=None):
        for agentIndex in range(3):
            self.vworld.vrobot.reset_robots(agentIndex)

    def drawMap(self, event=None):
        self.vworld.draw_map()

    def drawGrid(self, event=None):
        x1, y1 = 0, 0
        x2, y2 = self.vworld.canvas_width*2, self.vworld.canvas_height*2
        del_x, del_y = 20, 20
        num_x, num_y = x2 / del_x, y2 / del_y
        # draw center (0,0)
        self.vworld.canvas.create_rectangle(self.vworld.canvas_width-3,self.vworld.canvas_height-3,
                self.vworld.canvas_width+3,self.vworld.canvas_height+3, fill="red")
        # horizontal grid
        for i in range(num_y):
            y = i * del_y
            self.vworld.canvas.create_line(x1, y, x2, y, fill="yellow")
        # verticle grid
        for j in range(num_x):
            x = j * del_x
            self.vworld.canvas.create_line(x, y1, x, y2, fill="yellow")

    def clearCanvas(self, event=None):
        vcanvas = self.vworld.canvas
        vrobot = self.vworld.vrobot
        vcanvas.delete("all")
        poly_points = [0,0,0,0,0,0,0,0]
        vrobot.poly_id = vcanvas.create_polygon(poly_points, fill='blue')
        vrobot.prox_l_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.prox_r_id = vcanvas.create_line(0,0,0,0, fill="red")
        vrobot.floor_l_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")
        vrobot.floor_r_id = vcanvas.create_oval(0,0,0,0, outline="white", fill="white")

    def updateCanvas(self, drawQueue):
        self.vworld.canvas.after(UPDATE_INTERVAL, self.updateCanvas, drawQueue)
        while (drawQueue.qsize() > 0):
            drawCommand = drawQueue.get()
            drawCommand()


class Joystick:
    def __init__(self, comm, m, rCanvas):
        self.gMaxRobotNum = 1
        self.gRobotList = comm.robotList
        self.m = m
        self.vrobots = []
        self.pellet_positions = []
        self.super_pellet_positions = []
        self.score = 0
        for agentIndex in range(3):
            self.vrobots.append(virtual_robot(agentIndex))
            self.vrobots[agentIndex].t = time.time()

        rCanvas.bind_all('<w>', self.joystick_move_north)
        rCanvas.bind_all('<s>', self.joystick_move_south)
        rCanvas.bind_all('<a>', self.joystick_move_west)
        rCanvas.bind_all('<d>', self.joystick_move_east)
        rCanvas.bind_all('<x>', self.stop_move)
        self.canvas = rCanvas
        rCanvas.pack()

    def joystick_move_north(self, event=None):
        global lastMoveDirection
        self.launch_move_north(event=None, direction=lastMoveDirection, robotIndex=0)

    def joystick_move_east(self, event=None):
        global lastMoveDirection
        self.launch_move_east(event=None, direction=lastMoveDirection, robotIndex=0)

    def joystick_move_west(self, event=None):
        global lastMoveDirection
        self.launch_move_west(event=None, direction=lastMoveDirection, robotIndex=0)

    def joystick_move_south(self, event=None):
        global lastMoveDirection
        self.launch_move_south(event=None, direction=lastMoveDirection, robotIndex=0)

    def move_forward(self, robotIndex):
        if self.gRobotList:
            # robot = self.gRobotList[0]
            robot = self.gRobotList[robotIndex]
            vrobot = self.vrobots[robotIndex]
            vrobot.sl = 15
            vrobot.sr = 15

            robot.set_wheel(0, vrobot.sl)
            robot.set_wheel(1, vrobot.sr)

            leftFloor = robot.get_floor(0)
            rightFloor = robot.get_floor(1)

            # move forward until both white
            # while not (leftFloor > BLACK_CUTOFF and rightFloor > BLACK_CUTOFF):

            seenBlackLeft = False
            seenBlackRight = False

            # move forward until both have seen black and at least one is white
            while (leftFloor < BLACK_CUTOFF and rightFloor < BLACK_CUTOFF):
                # print "forward moving inital: left %d right %d" % (leftFloor, rightFloor)
                vrobot.sl = 15
                vrobot.sr = 15
                robot.set_wheel(0, vrobot.sl)
                robot.set_wheel(1, vrobot.sr)
                leftFloor = robot.get_floor(0)
                # if leftFloor < BLACK_CUTOFF:
                #     seenBlackLeft = True
                rightFloor = robot.get_floor(1)
                # if rightFloor < BLACK_CUTOFF:
                #     seenBlackRight = True
                time.sleep(0.01)

            vrobot.sl = 0
            vrobot.sr = 0
            robot.set_wheel(0, 0)
            robot.set_wheel(1, 0)

            # time.sleep(0.2)

            leftFloor = robot.get_floor(0)
            rightFloor = robot.get_floor(1)

            # move til both black
            while not (leftFloor < BLACK_CUTOFF and rightFloor < BLACK_CUTOFF):

                # print "while loop: left %d right %d" % (leftFloor, rightFloor)
                if rightFloor < BLACK_CUTOFF: # right floor sensor sees black, robot turns right
                    vrobot.sl = 15
                    vrobot.sr = -15
                elif leftFloor < BLACK_CUTOFF: # left floor sensor sees black, robot turns left
                    vrobot.sl = -15
                    vrobot.sr = 15
                else:
                    vrobot.sl = 15
                    vrobot.sr = 15
                robot.set_wheel(0, vrobot.sl)
                robot.set_wheel(1, vrobot.sr)

                time.sleep(0.01)

                leftFloor = robot.get_floor(0)
                rightFloor = robot.get_floor(1)

            vrobot.sl = 0
            vrobot.sr = 0
            robot.set_wheel(0, 0)
            robot.set_wheel(1, 0)

            vrobot.t = time.time()
    

    def turn(self, direction, robotIndex):
        ''' direction: -1 for left, 1 for right '''
        if self.gRobotList:
            print "turning robot %d to the %s" % (robotIndex, "left" if direction == - 1 else "right")
            robot = self.gRobotList[robotIndex]
            vrobot = self.vrobots[robotIndex]
            leftFloor = robot.get_floor(0)
            rightFloor = robot.get_floor(1)

            # move forward until both white
            # while not (leftFloor > BLACK_CUTOFF and rightFloor > BLACK_CUTOFF):

            # move forward until something is white
            while leftFloor < BLACK_CUTOFF and rightFloor < BLACK_CUTOFF:
                vrobot.sl = 15
                vrobot.sr = 15
                robot.set_wheel(0, vrobot.sl)
                robot.set_wheel(1, vrobot.sr)
                leftFloor = robot.get_floor(0)
                rightFloor = robot.get_floor(1)
                time.sleep(0.01)

            vrobot.sl = 0
            vrobot.sr = 0
            robot.set_wheel(0, 0)
            robot.set_wheel(1, 0)

            # turn in direction until hits black then hits white again
            
            if direction == -1:
                floorDir = 0
            elif direction == 1:
                floorDir = 1

            turnCount = 1

            if direction == -1:
                #turning left
                if leftFloor < BLACK_CUTOFF:
                    turnCount += 1

            elif direction == 1:
                if rightFloor < BLACK_CUTOFF:
                    turnCount += 1

            floor = robot.get_floor(floorDir)

            for i in range(turnCount):

                seenBlack = False

                while True:
                    if floor < BLACK_CUTOFF:
                        seenBlack = True
                        vrobot.sl = 15 * direction
                        vrobot.sr = -15 * direction
                        robot.set_wheel(0, vrobot.sl)
                        robot.set_wheel(1, vrobot.sr)
                    elif floor > BLACK_CUTOFF and seenBlack:
                        # stop
                        time.sleep(0.3) # move a bit extra
                        vrobot.sl = 0
                        vrobot.sr = 0
                        robot.set_wheel(0, 0)
                        robot.set_wheel(1, 0)
                        break
                    else:
                        vrobot.sl = 15 * direction
                        vrobot.sr = -15 * direction
                        robot.set_wheel(0, vrobot.sl)
                        robot.set_wheel(1, vrobot.sr)

                    time.sleep(0.01)

                    floor = robot.get_floor(floorDir)
            
    def stop_move(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()  


    def launch_move_north(self, event=None, direction="NORTH", robotIndex=0):
        turn_right_thread = threading.Thread(target=self.move_north, args=(direction, robotIndex))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def launch_move_south(self, event=None, direction="NORTH", robotIndex=0):
        turn_right_thread = threading.Thread(target=self.move_south, args=(direction, robotIndex))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def launch_move_east(self, event=None, direction="NORTH", robotIndex=0):
        turn_right_thread = threading.Thread(target=self.move_east, args=(direction, robotIndex))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def launch_move_west(self, event=None, direction="NORTH", robotIndex=0):
        turn_right_thread = threading.Thread(target=self.move_west, args=(direction, robotIndex))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def move_north(self, direction, robotIndex):
        if self.gRobotList:   
            # print "moving north:" 
            # print "last direction: ", direction
            # print "robot index:", robotIndex
            # self.gRobotList[robotIndex].set_wheel_balance(-29)

            robot = self.gRobotList[robotIndex]
            vrobot = self.vrobots[robotIndex]
            if direction == "NORTH":
                self.move_forward(robotIndex)

            elif direction == "EAST":
                self.turn(-1, robotIndex)
                self.move_forward(robotIndex)

            elif direction == "WEST":
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)

            elif direction == "SOUTH":
                self.turn(1, robotIndex)
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)
                # self.move_backward()

            global lastMoveDirection
            lastMoveDirection = "NORTH"


    def move_south(self, direction, robotIndex):
        if self.gRobotList: 
            # print "moving south"
            robot = self.gRobotList[robotIndex]
            vrobot = self.vrobots[robotIndex]
            if direction == "NORTH":
                self.turn(1, robotIndex)
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)
                # self.move_backward()

            elif direction == "EAST":
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)

            elif direction == "WEST":
                self.turn(-1, robotIndex)
                self.move_forward(robotIndex)

            elif direction == "SOUTH":
                self.move_forward(robotIndex)

            global lastMoveDirection
            lastMoveDirection = "SOUTH"

    def move_east(self, direction, robotIndex):
        if self.gRobotList: 
            # print "moving east"
            robot = self.gRobotList[robotIndex]
            vrobot = self.vrobots[robotIndex]
            if direction == "NORTH":
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)

            elif direction == "EAST":
                self.move_forward(robotIndex)

            elif direction == "WEST":
                self.turn(1, robotIndex)
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)
                # self.move_backward()

            elif direction == "SOUTH":
                self.turn(-1, robotIndex)
                self.move_forward(robotIndex)

            global lastMoveDirection
            lastMoveDirection = "EAST"

    def move_west(self, direction, robotIndex):
        if self.gRobotList: 
            # print "moving west"
            robot = self.gRobotList[robotIndex]
            vrobot = self.vrobots[robotIndex]
            if direction == "NORTH":
                self.turn(-1, robotIndex)
                self.move_forward(robotIndex)

            elif direction == "EAST":
                self.turn(1, robotIndex)
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)
                # self.move_backward()

            elif direction == "WEST":
                self.move_forward(robotIndex)

            elif direction == "SOUTH":
                self.turn(1, robotIndex)
                self.move_forward(robotIndex)

            global lastMoveDirection
            lastMoveDirection = "WEST"

    def update_all_virtual_robots(self):
        while not gQuit:
            while len(self.gRobotList) < 3:
                print "waiting for robot to connect"
                time.sleep(0.1)
            print self.gRobotList

            for agentIndex in range(3):
                self.update_virtual_robot(agentIndex)

    def update_virtual_robot(self, robotIndex):
        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 # floor ambient color - if floor is darker, set higher noise
        p_factor = 1.4 # proximity conversion - assuming linear
        d_factor = 1.1 # travel distance conversion
        a_factor = 17 # rotation conversion, assuming linear


        if len(self.gRobotList) >= 3:
            robot = self.gRobotList[robotIndex]
            vrobot = self.vrobots[robotIndex]
            robot.set_wheel_balance(-3)
            t = time.time()
            del_t = t - vrobot.t
            vrobot.t = t # update the tick
            if vrobot.sl == vrobot.sr:
                vrobot.x = vrobot.x + vrobot.sl * del_t * math.sin(vrobot.a) * d_factor
                vrobot.y = vrobot.y + vrobot.sl * del_t * math.cos(vrobot.a) * d_factor
            if vrobot.sl == vrobot.sr:
                vrobot.a = vrobot.a + (vrobot.sl * del_t)/a_factor
            #update sensors
            prox_l = robot.get_proximity(0)
            prox_r = robot.get_proximity(1)
            if (prox_l > noise_prox):
                vrobot.dist_l = (100 - prox_l)*p_factor
            else:
                vrobot.dist_l = False
            if (prox_r > noise_prox):
                vrobot.dist_r = (100 - prox_r)*p_factor
            else:
                vrobot.dist_r = False
                
            floor_l = robot.get_floor(0)
            floor_r = robot.get_floor(1)
            if (floor_l < noise_floor):
                vrobot.floor_l = floor_l
            else:
                vrobot.floor_l = False
            if (floor_r < noise_floor):
                vrobot.floor_r = floor_r
            else:
                vrobot.floor_r = False
        time.sleep(0.05)


def stopProg(event=None):
    global m
    m.quit()
    gQuit = True
    print "Exit"


def draw_virtual_world(virtual_world, joystick):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if joystick.gRobotList is not None:
            virtual_world.draw_food_layout(joystick.pellet_positions)
            virtual_world.update_score(joystick.score)
            for agentIndex in range(3):
                virtual_world.draw_robot(agentIndex)
                virtual_world.draw_prox("left", agentIndex)
                virtual_world.draw_prox("right", agentIndex)
                virtual_world.draw_floor("left", agentIndex)
                virtual_world.draw_floor("right", agentIndex)
        time.sleep(0.1)


def human_turn():
    print "n-for North \n s-for South \n e- for East \n w- for west"
    # wait for console input
    while True:
        move = str(sys.stdin.readline())[0].lower()

        if move in ["w", "west"]:
            action = "WEST"
            print "move west"
            break
        elif move in ["s", "south"]:
            action = "SOUTH"
            print "move south"
            break
        elif move in ["n", "north"]:
            action = "NORTH"
            print "move north"
            break
        elif move in ["e", "east"]:
            action = "EAST"
            print "move east"
            break
        else:
            print "Invalid move"
    return action


# takes in a gameState and returns an action
def ai_turn(gameState):

    def evaluationFunction(gameState):
        score = gameState.getScore()

        pacmanPosition = gameState.positions[0]

        # the closer the closest food is, the better
        food = gameState.getFood()
        minFoodDistance = sys.maxint
        for foodPosition in food:
            distance = util.manhattanDistance(pacmanPosition, foodPosition)
            if distance < minFoodDistance:
                minFoodDistance = distance
        score += 1 * minFoodDistance

        minGhostDistance = sys.maxint
        for ghostPosition in gameState.getGhostPositions():
            distance = util.manhattanDistance(pacmanPosition, ghostPosition)
            if distance < minGhostDistance:
                minGhostDistance = distance
        score += 100 * minGhostDistance
        return score

    def Vopt(gameState, depth, agent_index):
            if gameState.isWin() or gameState.isLose():
                return (gameState.score, None)
            elif depth == 0:
                print "agentIndex %d, score: %d" %(agent_index, evaluationFunction(gameState))
                return (evaluationFunction(gameState), None)

            scores = []
            actions = gameState.getLegalMoves(agent_index)

            if agent_index == 0:
                for action in actions:
                    newState = gameState.generateSuccessor(agent_index, action)
                    scores.append( (Vopt(newState, depth, agent_index + 1)[0] , action) )
            elif agent_index == 1:
                for action in actions:
                    newState = gameState.generateSuccessor(agent_index, action)
                    scores.append( (Vopt(newState, depth, agent_index + 1)[0] , action) )
            else:
                for action in actions:
                    newState = gameState.generateSuccessor(agent_index, action)
                    scores.append( (Vopt(newState, depth - 1, 0)[0] , action) )

            if agent_index == 0:
                max_score = max(scores)[0]
                max_choices = [score for score in scores if score[0] == max_score]
                return random.choice(max_choices)
                # return max(scores)
            else:
                return (sum([score for score, action in scores]) / len(scores), random.choice([action for score, action in scores]))

    utility, action = Vopt(gameState, 4, 0)
    print "utility: ", utility
    return action


def nextTurn(gameState, gameMode):
    '''
    one run of this function corresponds to one 'turn' in the game
    pacman and all ghosts move once, and game state is updated
    '''

    move_threads = []
    
    currentState = gameState
    
    old_agent_directions = gameState.directions[:]

    def move(currentState, agentIndex):
        legalActions = gameState.getLegalMoves(agentIndex)
        print "Agent: %s, legal actions: %s" %(agentIndex, legalActions)
        if agentIndex == 0: # Pacman's move
            if gameMode == "Human":
                while True:
                    action = human_turn()
                    if action in legalActions:
                        break
                    else:
                        print "Action is not legal."
            elif gameMode == "AI":
                gameStateCopy = copy.deepcopy(gameState)
                action = ai_turn(gameStateCopy)
                print "AI wants to turn, ", action
                move = str(sys.stdin.readline())[0].lower()

        else: # Ghosts' moves
            # if currentState.directions[agentIndex] in legalActions:
            #     action = currentState.directions[agentIndex]
            # else:
            action = legalActions[np.random.randint(0, len(legalActions))] # choose action randomly

        currentState = currentState.generateSuccessor(agentIndex, action)
        
        if action == "NORTH":
            move_fn = joystick.launch_move_north
        elif action == "EAST":
            move_fn = joystick.launch_move_east
        elif action == "WEST":
            move_fn = joystick.launch_move_west
        elif action == "SOUTH":
            move_fn = joystick.launch_move_south
    
        print "moving robot %d to the %s. It's old direction was: %s" % (agentIndex, action, old_agent_directions[agentIndex])
        moveThread = threading.Thread(target=move_fn, args=(None, old_agent_directions[agentIndex], agentIndex)) # the none is the event thing
        moveThread.daemon = True
        moveThread.start()

        move_threads.append(moveThread)

        return currentState

    if currentState.boostTimer > 0: # pacman gets to move twice
        move(currentState, 0)

    else:
        for agentIndex in range(3):
            currentState = move(currentState, agentIndex)
            if currentState.isWin() or currentState.isLose():
                break

    # wait for all three movements to be over
    for thread in move_threads:
        thread.join()

    if currentState.boostTimer > 0:
        currentState.boostTimer -= 1

    all_coords = currentState.get_all_coordinates()

    for agentIndex in range(3):
        vAgent = joystick.vrobots[agentIndex]
        vAgent.x, vAgent.y = all_coords[agentIndex]

        if currentState.directions[agentIndex] == "NORTH":
            vAgent.a = 0
        elif currentState.directions[agentIndex] == "EAST":
            vAgent.a = math.pi / 2
        elif currentState.directions[agentIndex] == "WEST":
            vAgent.a = - (math.pi / 2)
        elif currentState.directions[agentIndex] == "SOUTH":
            vAgent.a = math.pi
        else:
            raise Exception("direction not in NSEW")
    joystick.score = currentState.score
    joystick.pellet_positions = currentState.food

    return currentState


def initialize_wheel_balance():
    robots = joystick.gRobotList
    robots[0].set_wheel_balance(7)
    robots[1].set_wheel_balance(4)
    robots[2].set_wheel_balance(-29)


# This is run on a separate thread from the main thread so that we can wait for moves to complete
def run_game(gameMode):
    while len(joystick.gRobotList) < 3:
        print "not enough robots"
        time.sleep(1)

    initialize_wheel_balance()

    gameState = game.GameState()
    print "starting game!"
    print "Win?", gameState.isWin()
    print "Lose?", gameState.isLose()
    while not (gameState.isWin() or gameState.isLose()):
        # turnThread = threading.Thread(target=nextTurn, args=(gameState,))
        # turnThread.daemon = True
        # turnThread.start()

        # multithreading unnecessary for this part? it can be blocked since it's not main thread 
        # and doesn't need to do anything else
        gameState = nextTurn(gameState, gameMode)

    if gameState.isWin():
        print "Congratulations! You won!"
    elif gameState.isLose():
        print "Sorry, you lost..."

    print "Score: ", gameState.score
    stopProg()


def main(argv=None):
    global m, comm
    global sleepTime
    sleepTime = 0.01
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    m = tk.Tk() #root
    drawQueue = Queue.Queue(0)

    global lastMoveDirection
    lastMoveDirection = "NORTH"

    canvas_width = 500 #original: 700 # half width
    canvas_height = 200 # original 380half height
    rCanvas = tk.Canvas(m, bg="black", width=canvas_width*2, height=canvas_height*2)

    global joystick
    
    joystick = Joystick(comm, m, rCanvas)

    # visual elements of the virtual robots
    pacman_points = [0,0,0,0,0,0,0,0]
    blinky_points = [120,120,120,120,120,120,120,120]
    inky_points =[-120,-120,-120,-120,-120,-120,-120,-120]
    poly_points = [pacman_points, blinky_points, inky_points]
    colors = ['yellow', 'red', 'cyan']
    for agentIndex in range(3):
        joystick.vrobots[agentIndex].poly_id = rCanvas.create_polygon(poly_points[agentIndex], fill=colors[agentIndex]) #robots, pacman, blinky(top right), inky(bottom left)
        joystick.vrobots[agentIndex].prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
        joystick.vrobots[agentIndex].prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
        joystick.vrobots[agentIndex].floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
        joystick.vrobots[agentIndex].floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")
    
    time.sleep(1)

    #create the virtual worlds that contains the virtual robot

    vWorld = virtual_world(drawQueue, joystick.vrobots, rCanvas, canvas_width, canvas_height)
    
    ''' objects in the world '''
    rectangles = []
    pellets = []
    super_pellets = []
    
    # pacman map/walls
    rectangles.append([-150, -150, 150, 150]) #overall square
    rectangles.append([-90, 90, -30, 30])
    rectangles.append([90, 90, 30, 30])
    rectangles.append([-90, -90, -30, -30])
    rectangles.append([90, -90, 30, -30])


    #pacman pellets/capsules/food
    for index_x in range(BOARD_SIZE):
        for index_y in range(BOARD_SIZE):
            x_position = -120 + (60*index_x)
            y_position = -120 + (60*index_y)
            if (index_x == 0 and index_y == 4) or (index_x == 4 and index_y == 0):
                new_super_pellet = [x_position, y_position]
                super_pellets.append(new_super_pellet)
            elif not ((index_x == 0 and index_y == 0) or (index_x ==2 and index_y == 2) or (index_x ==4 and index_y ==4) or (index_x == 1 and index_y == 1) or (index_x == 3 and index_y == 1) or (index_x == 1 and index_y == 3) or (index_x==3 and index_y==3)):
                new_pellet = [x_position, y_position]
                pellets.append(new_pellet)

    vWorld.add_score_label([175,-175])

    for pill in pellets:
        vWorld.add_pellet(pill)
    for super_pill in super_pellets:
        vWorld.add_super_pellet(super_pill)
    for rect in rectangles:
        vWorld.add_obstacle(rect)


    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, joystick))
    draw_world_thread.daemon = True
    draw_world_thread.start()

    gui = VirtualWorldGui(vWorld, m)

    gui.drawGrid()
    gui.drawMap()

    rCanvas.after(200, gui.updateCanvas, drawQueue)
    m.mainloop()

    for robot in joystick.gRobotList:
        robot.reset()
    comm.stop()
    comm.join()


if __name__ == "__main__":
    main()

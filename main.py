import Tkinter as tk 
import time 
from HamsterAPI.comm_ble import RobotComm
import math
import threading
from tk_hamster_GUI import *
import numpy as np
import globalVars as g
import graphics
<<<<<<< HEAD
import game, sys
=======
import game
import random
>>>>>>> 9b052373a2251b3e7684a2fc560a57eae0ef4079

UPDATE_INTERVAL = 30

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

        self.button9 = tk.Button(m,text="Exit")
        self.button9.pack(side='left')
        self.button9.bind('<Button-1>', stopProg)
    
        self.button4 = tk.Button(m, text="AI Mode")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.AI_mode)
        
        
        self.button5 = tk.Button(m, text="Human Mode")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.Human_mode)

    def AI_mode(self, event=None):
        AI_thread = threading.Thread(target= AI_game)
        AI_thread.daemon = True
        AI_thread.start()
    
    
    def Human_mode(self, event=None):
        Human_thread= threading.Thread(target=human_game)
        Human_thread.daemon = True
        Human_thread.start()
    
    def startNavigateThread(self, event=None):
        navigation_thread = threading.Thread(target=self.navigate)
        navigation_thread.daemon = True
        navigation_thread.start()

    def resetvRobot(self, event=None):
        self.vworld.vrobot.reset_robot()

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
        self.vrobot = virtual_robot()
        self.vrobot.t = time.time()


        rCanvas.bind_all('<w>', self.launch_move_north)
        rCanvas.bind_all('<s>', self.launch_move_south)
        rCanvas.bind_all('<a>', self.launch_move_west)
        rCanvas.bind_all('<d>', self.launch_move_east)

        rCanvas.bind_all('<x>', self.stop_move)  
        rCanvas.pack()

    def move_forward(self):
        if self.gRobotList:
            robot = self.gRobotList[0]
            self.vrobot.sl = 15
            self.vrobot.sr = 15   

            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            time.sleep(0.5)

            print "initial move forward"

            leftFloor = robot.get_floor(0)
            rightFloor = robot.get_floor(1)

            while not (leftFloor < 40 and rightFloor < 40):
                print "while loop: floorleft %d floorright %d" % (leftFloor, rightFloor)
                if rightFloor < 40: # right floor sensor sees black, robot turns right
                    self.vrobot.sl = 15
                    self.vrobot.sr = -15  
                elif leftFloor < 40: # left floor sensor sees black, robot turns left
                    self.vrobot.sl = -15
                    self.vrobot.sr = 15   
                else:
                    self.vrobot.sl = 15
                    self.vrobot.sr = 15   
                robot.set_wheel(0, self.vrobot.sl)
                robot.set_wheel(1, self.vrobot.sr)

                time.sleep(0.01)

                leftFloor = robot.get_floor(0)
                rightFloor = robot.get_floor(1)

            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0, 0)
            robot.set_wheel(1, 0)

            self.vrobot.t = time.time()
    

    def turn(self, direction):
        ''' direction: -1 for left, 1 for right '''
        if self.gRobotList:
            robot = self.gRobotList[0]

            leftFloor = robot.get_floor(0)
            rightFloor = robot.get_floor(1)

            # move forward until both white
            while not (leftFloor > 40 and rightFloor > 40):
                self.vrobot.sl = 15
                self.vrobot.sr = 15
                robot.set_wheel(0, self.vrobot.sl)
                robot.set_wheel(1, self.vrobot.sr)
                leftFloor = robot.get_floor(0)
                rightFloor = robot.get_floor(1)
                time.sleep(0.01)

            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0, 0)
            robot.set_wheel(1, 0)

            # turn in direction until hits black then hits white again

            seenBlack = False
            if direction == -1:
                floorDir = 0
            elif direction == 1:
                floorDir = 1

            floor = robot.get_floor(floorDir)
            while True:
                if floor < 40:
                    seenBlack = True
                    self.vrobot.sl = 15 * direction
                    self.vrobot.sr = -15 * direction
                    robot.set_wheel(0, self.vrobot.sl)
                    robot.set_wheel(1, self.vrobot.sr)
                elif floor > 40 and seenBlack:
                    # stop
                    time.sleep(0.2) # move a bit extra
                    self.vrobot.sl = 0
                    self.vrobot.sr = 0
                    robot.set_wheel(0, 0)
                    robot.set_wheel(1, 0)
                    break
                else:
                    self.vrobot.sl = 15 * direction
                    self.vrobot.sr = -15 * direction
                    robot.set_wheel(0, self.vrobot.sl)
                    robot.set_wheel(1, self.vrobot.sr)

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

    def launch_move_north(self, event=None):
        turn_right_thread = threading.Thread(target=self.move_north, args=(lastMoveDirection,))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def launch_move_south(self, event=None):
        turn_right_thread = threading.Thread(target=self.move_south, args=(lastMoveDirection,))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def launch_move_east(self, event=None):
        turn_right_thread = threading.Thread(target=self.move_east, args=(lastMoveDirection,))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def launch_move_west(self, event=None):
        turn_right_thread = threading.Thread(target=self.move_west, args=(lastMoveDirection,))
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def move_north(self, direction):
        if self.gRobotList:   
            robot = self.gRobotList[0]

            if direction == "NORTH":
                self.move_forward()

            elif direction == "EAST":
                self.turn(-1)
                self.move_forward()

            elif direction == "WEST":
                self.turn(1)
                self.move_forward()

            elif direction == "SOUTH":
                self.turn(1)
                self.turn(1)
                self.move_forward()
                # self.move_backward()

            global lastMoveDirection
            lastMoveDirection = "NORTH"


    def move_south(self, direction):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            if direction == "NORTH":
                self.turn(1)
                self.turn(1)
                self.move_forward()
                # self.move_backward()

            elif direction == "EAST":
                self.turn(1)
                self.move_forward()

            elif direction == "WEST":
                self.turn(-1)
                self.move_forward()

            elif direction == "SOUTH":
                self.move_forward()

            global lastMoveDirection
            lastMoveDirection = "SOUTH"

    def move_east(self, direction):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            if direction == "NORTH":
                self.turn(1)
                self.move_forward()

            elif direction == "EAST":
                self.move_forward()

            elif direction == "WEST":
                self.turn(1)
                self.turn(1)
                self.move_forward()
                # self.move_backward()

            elif direction == "SOUTH":
                self.turn(-1)
                self.move_forward()

            global lastMoveDirection
            lastMoveDirection = "EAST"

    def move_west(self, direction):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            if direction == "NORTH":
                self.turn(-1)
                self.move_forward()

            elif direction == "EAST":
                self.turn(1)
                self.turn(1)
                self.move_forward()
                # self.move_backward()

            elif direction == "WEST":
                self.move_forward()

            elif direction == "SOUTH":
                self.turn(1)
                self.move_forward()

            global lastMoveDirection
            lastMoveDirection = "WEST"


    def update_virtual_robot(self):
        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 # floor ambient color - if floor is darker, set higher noise
        p_factor = 1.4 # proximity conversion - assuming linear
        d_factor = 1.1 # travel distance conversion
        a_factor = 17 # rotation conversion, assuming linear 

        while not self.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        print "connected to robot in update_virtual_robot"

        while not g.gQuit:
            if self.gRobotList is not None:
                robot = self.gRobotList[0]
                robot.set_wheel_balance(-3)
                t = time.time()
                del_t = t - self.vrobot.t
                self.vrobot.t = t # update the tick
                if self.vrobot.sl == self.vrobot.sr:
                    self.vrobot.x = self.vrobot.x + self.vrobot.sl * del_t * math.sin(self.vrobot.a) * d_factor
                    self.vrobot.y = self.vrobot.y + self.vrobot.sl * del_t * math.cos(self.vrobot.a) * d_factor
                if self.vrobot.sl == -self.vrobot.sr:
                    self.vrobot.a = self.vrobot.a + (self.vrobot.sl * del_t)/a_factor 
                #update sensors
                prox_l = robot.get_proximity(0)
                prox_r = robot.get_proximity(1)
                if (prox_l > noise_prox):
                    self.vrobot.dist_l = (100 - prox_l)*p_factor
                else:
                    self.vrobot.dist_l = False
                if (prox_r > noise_prox):
                    self.vrobot.dist_r = (100 - prox_r)*p_factor
                else:
                    self.vrobot.dist_r = False
                
                floor_l = robot.get_floor(0)
                floor_r = robot.get_floor(1)
                if (floor_l < noise_floor):
                    self.vrobot.floor_l = floor_l
                else:
                    self.vrobot.floor_l = False
                if (floor_r < noise_floor):
                    self.vrobot.floor_r = floor_r
                else:
                    self.vrobot.floor_r = False
            time.sleep(0.05)
def human_game():
    print "w-for North \n s-for South \n a- for East \n d- for west"
    # wait for console input
    while True:
        move =  str(sys.stdin.readline())
        if move[0] == ['w', 'W']:
            print "move north"
        
        elif move[0] in ["s", "S"]:
            print "move south"
        elif move[0] in ["a", "A"]:
            print "move west"
        elif move[0] in ["d", "D"]:
            print "move east"
        else:
            print "done"
            continue

def nextGameTurn():
    # for each agent
    # get legal moves
    # decide on move and launch thread to make the move
    # join threads
    pass    


def stopProg(event=None):
    g.m.quit()
    g.gQuit = True
    print "Exit"


def draw_virtual_world(virtual_world, joystick):
    time.sleep(1) # give time for robot to connect.
    while not g.gQuit:
        if joystick.gRobotList is not None:
            virtual_world.draw_robot()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")
        time.sleep(0.1)

def nextTurn(gameState):
    move_threads = []

    for agentIndex in range(3):
        legalActions = gameState.getLegalMoves(agentIndex)
        action = legalActions[random.randrange(0, len(legalActions))] # choose action randomly

        successor = gameState.generateSuccessor(agentIndex, action)

        if action == "North":
            move = launch_move_north
        elif action == "East":
            move = launch_move_east
        elif action == "West":
            move = launch_move_west
        elif action == "South":
            move = launch_move_south


        moveThread = threading.Thread(target=move, args=(direction,))
        moveThread.daemon = True
        moveThread.start()

        move_threads.append(moveThread)

    for thread in move_threads:
        thread.join()


def run_game():
    gameState = game.GameState()
    while not (game.isWin() or game.isLose()):
        turnThread = threading.Thread(target=nextTurn, args=(gameState))
        turnThread.daemon = True
        turnThread.start()

    if game.isWin():
        print "Congratulations! You won!"
    else:
        print "Sorry, you lost..."

    print "Score: ", game.score

    # return game.score

def main(argv=None): 
    global sleepTime
    sleepTime = 0.01
    g.comm.start()
    
    print 'Bluetooth starts'

    global lastMoveDirection
    lastMoveDirection = "NORTH"

    drawQueue = Queue.Queue(0)

    canvas_width = 500 #original: 700 # half width
    canvas_height = 200 # original 380half height
    rCanvas = tk.Canvas(g.m, bg="white", width=canvas_width*2, height=canvas_height*2)
    joystick = Joystick(g.comm, g.m, rCanvas)



    # visual elements of the virtual robot 
    poly_points = [0,0,0,0,0,0,0,0]
    joystick.vrobot.poly_id = rCanvas.create_polygon(poly_points, fill='blue') #robot
    joystick.vrobot.prox_l_id = rCanvas.create_line(0,0,0,0, fill="red") #prox sensors
    joystick.vrobot.prox_r_id = rCanvas.create_line(0,0,0,0, fill="red")
    joystick.vrobot.floor_l_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white") #floor sensors
    joystick.vrobot.floor_r_id = rCanvas.create_oval(0,0,0,0, outline="white", fill="white")

    time.sleep(1)

    update_vrobot_thread = threading.Thread(target=joystick.update_virtual_robot)
    update_vrobot_thread.daemon = True
    update_vrobot_thread.start()

    #create the virtual worlds that contains the virtual robot

    vWorld = virtual_world(drawQueue, joystick.vrobot, rCanvas, canvas_width, canvas_height)

    ''' objects in the world '''
    rectangles = []

    # pacman map
    rectangles.append([-150, -150, 150, 150]) #overall square
    rectangles.append([-90, 90, -30, 30])
    rectangles.append([90, 90, 30, 30])
    rectangles.append([-90, -90, -30, -30])
    rectangles.append([90, -90, 30, -30])
    rectangles.append([100, 100, 140, 140])
    rectangles.append([-100, -100, -140, -140])
    

    for rect in rectangles:
        vWorld.add_obstacle(rect)


    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, joystick))
    draw_world_thread.daemon = True
    draw_world_thread.start()

    gui = VirtualWorldGui(vWorld, g.m)

    gui.drawGrid()
    gui.drawMap()

    rCanvas.after(200, gui.updateCanvas, drawQueue)
    g.m.mainloop()

    game_thread = threading.Thread(target=run)
    game_thread.start()
    # run()    

    for robot in joystick.gRobotList:
        robot.reset()
    g.comm.stop()
    g.comm.join()


if __name__ == "__main__":
    main()

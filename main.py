import Tkinter as tk 
import time 
from HamsterAPI.comm_ble import RobotComm
import math
import threading
from tk_hamster_GUI import *
import numpy as np
import globalVars as g
import graphics

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

    def startNavigateThread(self, event=None):
        navigation_thread = threading.Thread(target=self.navigate)
        navigation_thread.daemon = True
        navigation_thread.start()

    def resetvRobot(self, event=None):
        self.vworld.vrobot.reset_robot()

    def leastSquares(self, xvalues, yvalues):
        x = np.array(xvalues)
        y = np.array(yvalues)
        A = np.vstack([x, np.ones(len(x))]).T 
        m, c = np.linalg.lstsq(A, y)[0]
        return (m, c)

    def localize(self, face, boundary):
        robot = self.vworld.vrobot
        localization_x_points = robot.localization_x_points
        localization_y_points = robot.localization_y_points
        while len(localization_x_points) <= 10 and len(localization_y_points) <= 10:
            localization_x_points.append(0)
            localization_y_points.append(robot.dist_l)
            localization_x_points.append(40)
            localization_y_points.append(robot.dist_r)
            time.sleep(0.02)

        m, b = self.leastSquares(localization_x_points, localization_y_points)
        theta = math.degrees(math.atan(m))
        # print "theta:", theta
        
        currentAngle = math.degrees(self.vworld.vrobot.a) % 360
        
        angle_difference = math.degrees(robot.a - robot.previous_a) % 360
        # print "newBoxIndex", robot.facing_box_index
        # print "angle difference:", angle_difference

        proximity_avg = (robot.dist_r + robot.dist_l) / float(2)
        distance_from_robot_center = proximity_avg + 20 # 20 is half the length of the robot

        if face == "top":
            self.vworld.vrobot.a = math.radians(theta)
            self.vworld.vrobot.y = boundary - math.cos(math.radians(theta)) * distance_from_robot_center
        if face == "bottom":
            self.vworld.vrobot.a = math.radians(theta + 180)
            self.vworld.vrobot.y = boundary + math.cos(math.radians(theta)) * distance_from_robot_center
        if face == "right":
            self.vworld.vrobot.a = math.radians(theta + 270)
            self.vworld.vrobot.x = boundary + math.cos(math.radians(theta)) * distance_from_robot_center
        if face == "left":
            self.vworld.vrobot.a = math.radians(theta + 90)
            self.vworld.vrobot.x = boundary - math.cos(math.radians(theta)) * distance_from_robot_center

        self.vworld.vrobot.previous_a = self.vworld.vrobot.a

        self.vworld.vrobot.localization_x_points = []
        self.vworld.vrobot.localization_y_points = []

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

        rCanvas.bind_all('<w>', self.launch_move_forward)
        # rCanvas.bind_all('<s>', self.move_down)
        # rCanvas.bind_all('<a>', self.move_left)
        rCanvas.bind_all('<d>', self.launch_turn_right)
        rCanvas.bind_all('<x>', self.stop_move)  
        rCanvas.pack()

    def launch_move_forward(self, event=None):
        move_fwd_thread = threading.Thread(target=self.move_forward)
        move_fwd_thread.daemon = True
        move_fwd_thread.start()
        print "move fwd thread started"
        # move_fwd_thread.join()
        print "move fwd thread finished"

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

    def launch_turn_left(self, event=None):
        turn_left_thread = threading.Thread(target=self.turn_left)
        turn_left_thread.daemon = True
        turn_left_thread.start()
    
    def turn_left(self):
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
            
            # turn right until hit black then hit white again
            
            seenBlack = False
            
            while True:
                if rightFloor < 40:
                    seenBlack = True
                    # turn right
                    self.vrobot.sl = 15
                    self.vrobot.sr = -15
                    robot.set_wheel(0, self.vrobot.sl)
                    robot.set_wheel(1, self.vrobot.sr)
                elif rightFloor > 40 and seenBlack:
                    # stop
                    self.vrobot.sl = 0
                    self.vrobot.sr = 0
                    robot.set_wheel(0, 0)
                    robot.set_wheel(1, 0)
                    break
                else:
                    # turn right
                    self.vrobot.sl = 15
                    self.vrobot.sr = -15
                    robot.set_wheel(0, self.vrobot.sl)
                    robot.set_wheel(1, self.vrobot.sr)
                
                time.sleep(0.01)
                
                leftFloor = robot.get_floor(0)
                rightFloor = robot.get_floor(1)

    def launch_turn_right(self, event=None):
        turn_right_thread = threading.Thread(target=self.turn_right)
        turn_right_thread.daemon = True
        turn_right_thread.start()

    def turn_right(self):
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

            # turn right until hit black then hit white again

            seenBlack = False

            while True:
                if rightFloor < 40:
                    seenBlack = True
                    # turn right
                    self.vrobot.sl = -15
                    self.vrobot.sr = 15
                    robot.set_wheel(0, self.vrobot.sl)
                    robot.set_wheel(1, self.vrobot.sr)
                elif rightFloor > 40 and seenBlack:
                    # stop
                    self.vrobot.sl = 0
                    self.vrobot.sr = 0
                    robot.set_wheel(0, 0)
                    robot.set_wheel(1, 0)
                    break
                else:
                    # turn right
                    self.vrobot.sl = -15
                    self.vrobot.sr = 15
                    robot.set_wheel(0, self.vrobot.sl)
                    robot.set_wheel(1, self.vrobot.sr)

                time.sleep(0.01)

                leftFloor = robot.get_floor(0)
                rightFloor = robot.get_floor(1)
            

    def stop_move(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()  


    def move_north(self, direction, event=None):
        if self.gRobotList:   
            robot = self.gRobotList[0]

            if direction == "NORTH":
                self.move_forward()

            elif direction == "EAST":
                self.turn_left()
                self.move_forward()

            elif direction == "WEST":
                self.turn_right()
                self.move_forward()

            elif direction == "SOUTH":
                self.move_backward()


    def move_south(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            if direction == "NORTH":
                self.move_backward()

            elif direction == "EAST":
                self.turn_right()
                self.move_forward()

            elif direction == "WEST":
                self.turn_left()
                self.move_forward()

            elif direction == "SOUTH":
                self.move_forward()

    def move_east(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            if direction == "NORTH":
                self.turn_right()
                self.move_forward()

            elif direction == "EAST":
                self.move_forward()

            elif direction == "WEST":
                self.move_backward()

            elif direction == "SOUTH":
                self.turn_left()
                self.move_forward()

    def move_west(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            if direction == "NORTH":
                self.turn_left()
                self.move_forward()

            elif direction == "EAST":
                self.move_backward()

            elif direction == "WEST":
                self.move_forward()

            elif direction == "SOUTH":
                self.turn_right()
                self.move_forward()


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


def main(argv=None): 
    global sleepTime
    sleepTime = 0.01
    g.comm.start()
    print 'Bluetooth starts'

    drawQueue = Queue.Queue(0)

    canvas_width = 700 # half width
    canvas_height = 380 # half height
    rCanvas = tk.Canvas(g.m, bg="white", width=canvas_width*2, height=canvas_height*2)
    joystick = Joystick(g.comm, g.m, rCanvas)

    # # visual elements of the virtual robot 
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
    
    

    # Outer walls
    rectangles.append([-120, 20, -40, 60])
    rectangles.append([-120, 60, -80, 140])
    rectangles.append([-80, 100, 80, 140])
    rectangles.append([120, 60, 80, 140])
    rectangles.append([120, 20, 40, 60])
    rectangles.append([120, 100, 400, 140])
    rectangles.append([360, 100, 400, -140])
    rectangles.append([-120, 100, -400, 140])
    rectangles.append([-360, 100, -400, -140])
    rectangles.append([-360, -100, 360, -140])
    rectangles.append([-120, -20, 120, -60])

    # Right inner walls
    rectangles.append([160, 20, 200, -60])
    rectangles.append([240, 60, 320, 20])
    rectangles.append([280, 20, 320, -20])
    rectangles.append([240, -20, 320, -60])

    # Left inner walls
    rectangles.append([-160, 20, -200, -60])
    rectangles.append([-240, 60, -320, 20])
    rectangles.append([-280, 20, -320, -20])
    rectangles.append([-240, -20, -320, -60])

    for rect in graphics.rectangles:
        vWorld.add_obstacle(rect)

    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, joystick))
    draw_world_thread.daemon = True
    draw_world_thread.start()

    gui = VirtualWorldGui(vWorld, g.m)

    gui.drawGrid()
    gui.drawMap()

    rCanvas.after(200, gui.updateCanvas, drawQueue)
    g.m.mainloop()

    for robot in joystick.gRobotList:
        robot.reset()
    g.comm.stop()
    g.comm.join()


if __name__ == "__main__":
    main()

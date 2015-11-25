import Tkinter as tk 
import time 
from HamsterAPI.comm_ble import RobotComm
import math
import threading
from tk_hamster_GUI import *
import numpy as np

UPDATE_INTERVAL = 30

gMaxRobotNum = 1; # max number of robots to control
# gRobotList = None
gQuit = False
m = None

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

        self.button4 = tk.Button(m,text="Trace")
        self.button4.pack(side='left')
        self.button4.bind('<Button-1>', self.toggleTrace)

        self.button5 = tk.Button(m,text="Prox Dots")
        self.button5.pack(side='left')
        self.button5.bind('<Button-1>', self.toggleProx)

        self.button6 = tk.Button(m,text="Floor Dots")
        self.button6.pack(side='left')
        self.button6.bind('<Button-1>', self.toggleFloor)

        self.button7 = tk.Button(m,text="Localize")
        self.button7.pack(side='left')
        self.button7.bind('<Button-1>', self.localize)

        self.button8 = tk.Button(m,text="Navigate")
        self.button8.pack(side='left')
        self.button8.bind('<Button-1>', self.startNavigateThread)

        self.button9 = tk.Button(m,text="Exit")
        self.button9.pack(side='left')
        self.button9.bind('<Button-1>', stopProg)

    def startNavigateThread(self, event=None):
        navigation_thread = threading.Thread(target=self.navigate)
        navigation_thread.daemon = True
        navigation_thread.start()

    def resetvRobot(self, event=None):
        self.vworld.vrobot.reset_robot()

    def toggleTrace(self, event=None):
        if self.vworld.trace:
            self.vworld.trace = False
            self.button4["text"] = "Trace"
        else:
            self.vworld.trace = True
            self.button4["text"] = "No Trace"

    def toggleProx(self, event=None):
        if self.vworld.prox_dots:
            self.vworld.prox_dots = False
            self.button5["text"] = "Prox Dots"
        else:
            self.vworld.prox_dots = True
            self.button5["text"] = "No Prox Dots"

    def toggleFloor(self, event=None):
        if self.vworld.floor_dots:
            self.vworld.floor_dots = False
            self.button6["text"] = "Floor Dots"
        else:
            self.vworld.floor_dots = True
            self.button6["text"] = "No Floor Dots"

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

    def move_up(self, event=None):
        if comm.robotList:
            robot = comm.robotList[0]
            self.vworld.vrobot.sl = 30
            self.vworld.vrobot.sr = 30   
            robot.set_wheel(0,self.vworld.vrobot.sl)
            robot.set_wheel(1,self.vworld.vrobot.sr)
            self.vworld.vrobot.t = time.time()

    def move_down(self, event=None):
        if comm.robotList:   
            robot = comm.robotList[0]
            self.vworld.vrobot.sl = -30
            self.vworld.vrobot.sr = -30   
            robot.set_wheel(0,self.vworld.vrobot.sl)
            robot.set_wheel(1,self.vworld.vrobot.sr)
            self.vworld.vrobot.t = time.time()

    def move_left(self, event=None):
        if comm.robotList: 
            robot = comm.robotList[0]
            self.vworld.vrobot.sl = -15
            self.vworld.vrobot.sr = 15   
            robot.set_wheel(0,self.vworld.vrobot.sl)
            robot.set_wheel(1,self.vworld.vrobot.sr)
            self.vworld.vrobot.t = time.time()       

    def move_right(self, event=None):
        if comm.robotList: 
            robot = comm.robotList[0]
            self.vworld.vrobot.sl = 15
            self.vworld.vrobot.sr = -15  
            robot.set_wheel(0,self.vworld.vrobot.sl)
            robot.set_wheel(1,self.vworld.vrobot.sr) 
            self.vworld.vrobot.t = time.time()      

    def stop_move(self, event=None):
        if comm.robotList: 
            robot = comm.robotList[0]
            self.vworld.vrobot.sl = 0
            self.vworld.vrobot.sr = 0
            robot.set_wheel(0,self.vworld.vrobot.sl)
            robot.set_wheel(1,self.vworld.vrobot.sr)
            self.vworld.vrobot.t = time.time()   

    def face_right(self):
        if 90 <= math.degrees(self.vworld.vrobot.a) % 360 <= 270: # if robot is facing down, turn left
            self.move_left()
            while math.degrees(self.vworld.vrobot.a) % 360 < 270:
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360
            self.stop_move()
        else: # if robot is facing up, turn right
            if math.degrees(self.vworld.vrobot.a) % 360 > 270:
                self.move_right()
                while math.degrees(self.vworld.vrobot.a) % 360 > 270:
                    time.sleep(sleepTime)
                    print "a:", math.degrees(self.vworld.vrobot.a) % 360
                self.stop_move()

            self.move_right()
            while math.degrees(self.vworld.vrobot.a) < 90:
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360
            self.stop_move()

    def face_left(self):
        if 90 <= math.degrees(self.vworld.vrobot.a) % 360 <= 270: # if robot is facing down, turn right
            self.move_right()
            while math.degrees(self.vworld.vrobot.a) % 360 < 270:
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360
            self.stop_move()
        else: # if robot is facing up, turn left
            if math.degrees(self.vworld.vrobot.a) % 360 < 270:
                self.move_left()
                while math.degrees(self.vworld.vrobot.a) % 360 < 270:
                    time.sleep(sleepTime)
                    print "a:", math.degrees(self.vworld.vrobot.a) % 360
                self.stop_move()
            self.move_left()
            while math.degrees(self.vworld.vrobot.a) % 360 > 270:
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360
            self.stop_move()

    def face_up(self):
        if 0 <= math.degrees(self.vworld.vrobot.a) % 360 <= 180: # if robot is facing towards right, turn left
            self.move_left()
            while math.degrees(self.vworld.vrobot.a) % 360 > 5:
                "turning left"
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360 > 5
            self.stop_move()

        else: # if robot is facing towards left, turn right
            self.move_right()
            while math.degrees(self.vworld.vrobot.a) % 360 < 355:
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360
            self.stop_move()

    def face_down(self):
        if 0 <= math.degrees(self.vworld.vrobot.a) % 360 <= 180: # if robot is facing towards right, turn right
            self.move_right()
            while math.degrees(self.vworld.vrobot.a) < 180:
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360
            self.stop_move()

        else: # if robot is facing towards left, turn left
            self.move_left()
            while math.degrees(self.vworld.vrobot.a) % 360 > 180:
                time.sleep(sleepTime)
                print "a:", math.degrees(self.vworld.vrobot.a) % 360
            self.stop_move()

    def move_to_xy(self, x, y):
        global sleepTime
        print "X:", self.vworld.vrobot.x
        print "Y:", self.vworld.vrobot.y
        print "a:", math.degrees(self.vworld.vrobot.a)

        if not (-1 <= (self.vworld.vrobot.x - x) <= 1): # if robot is already at correct x coordinate, skip the following
            if (self.vworld.vrobot.x - x) < 0: # if robot is to the left of destination
                if 265 <= math.degrees(self.vworld.vrobot.a) % 360 <= 275: # if robot is already facing left, just move backwards
                    self.move_down()
                    while (self.vworld.vrobot.x - x) < 0:
                        time.sleep(sleepTime)
                        print "VRobot.x:", self.vworld.vrobot.x
                    self.stop_move()

                else:
                    if not (85 <= math.degrees(self.vworld.vrobot.a) % 360 <= 95): # make sure robot faces right
                        self.face_right()

                    self.move_up()
                    while (self.vworld.vrobot.x - x) < 0:
                        time.sleep(sleepTime)
                        print "VRobot.x:", self.vworld.vrobot.x
                    self.stop_move()
                
            if (self.vworld.vrobot.x - x) > 0: # if robot is to the right of destination
                if 85 <= math.degrees(self.vworld.vrobot.a) % 360 <= 95:
                    self.move_down()
                    while (self.vworld.vrobot.x - x) > 0:
                        time.sleep(sleepTime)
                        print "VRobot.x:", self.vworld.vrobot.x
                    self.stop_move()

                else:
                    if not (265 <= math.degrees(self.vworld.vrobot.a) % 360 <= 275): # make sure robot faces left
                        self.face_left()

                    self.move_up()
                    while (self.vworld.vrobot.x - x) > 0:
                        time.sleep(sleepTime)
                        print "VRobot.x:", self.vworld.vrobot.x
                    self.stop_move()


        if not (-1 <= (self.vworld.vrobot.y - y) <= 1): # if robot is already at correct y coordinate, skip the following
            if (self.vworld.vrobot.y - y) < 0: # if robot is below desination
                if 175 <= math.degrees(self.vworld.vrobot.a) % 360 <= 185: # if robot is facing down already, just move backwards
                    self.move_down()
                    while (self.vworld.vrobot.y - y) < 0:
                        time.sleep(sleepTime)
                        print "VRobot.y:", self.vworld.vrobot.y
                    self.stop_move()

                else: 
                    if not (math.degrees(self.vworld.vrobot.a) % 360 <= 5 or math.degrees(self.vworld.vrobot.a) % 360 >= 355): # make sure robot faces up
                        self.face_up()

                    self.move_up()
                    while (self.vworld.vrobot.y - y) < 0:
                        time.sleep(sleepTime)
                        print "VRobot.y:", self.vworld.vrobot.y
                    self.stop_move()

            if (self.vworld.vrobot.y - y) > 0: # if robot is above destination
                if (math.degrees(self.vworld.vrobot.a) % 360 <= 5 or math.degrees(self.vworld.vrobot.a) % 360 >= 355): # if robot is facing up already, just move backwards
                    self.move_down()
                    while (self.vworld.vrobot.y - y) > 0:
                        time.sleep(sleepTime)
                        print "VRobot.y:", self.vworld.vrobot.y
                    self.stop_move()

                else:
                    if not (175 <= math.degrees(self.vworld.vrobot.a) % 360 <= 185): # make sure robot faces down
                        self.face_down()

                    self.move_up()
                    while (self.vworld.vrobot.y - y) > 0:
                        time.sleep(sleepTime)
                        print "VRobot.y:", self.vworld.vrobot.y
                    self.stop_move()

    def find_last_box(self):
        robot = comm.robotList[0]
        while not gQuit:
            time.sleep(0.01)
            proximity_left = robot.get_proximity(0)
            proximity_right = robot.get_proximity(1)
            print "proximity_left:", proximity_left
            print "proximity_right", proximity_right

            if proximity_right > 60 and proximity_left > 60:
                print "Box found!"
                self.stop_move()
                break
            if proximity_left - proximity_right > 10:
                self.move_left()
            elif proximity_right - proximity_left > 10:
                self.move_right()
            else:
                self.move_up()



    def navigate(self):
        while not comm.robotList:
            print "waiting for robot to connect"
            time.sleep(0.1)
        self.vworld.vrobot.x = 230
        self.vworld.vrobot.a = math.radians(-90)

        print "connected to robot in navigate"
        time.sleep(1)

        print "hello"

        # go to box A
        self.move_to_xy(80, 0)
        time.sleep(1)
        self.localize("right", 40) # localize to box a
        time.sleep(1)

        # go to box C
        self.move_to_xy(80, 120)
        time.sleep(1)
        self.move_to_xy(-60, 120)
        self.face_up()
        self.localize("top", 140)

        # go to center
        self.move_to_xy(-60, 0)
        self.face_left()
        self.find_last_box()
        # print "end"


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

        rCanvas.bind_all('<w>', self.move_up)
        rCanvas.bind_all('<s>', self.move_down)
        rCanvas.bind_all('<a>', self.move_left)
        rCanvas.bind_all('<d>', self.move_right)
        rCanvas.bind_all('<x>', self.stop_move)  
        rCanvas.pack()

    def move_up(self, event=None):
        if self.gRobotList:
            robot = self.gRobotList[0]
            self.vrobot.sl = 30
            self.vrobot.sr = 30   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_down(self, event=None):
        if self.gRobotList:   
            robot = self.gRobotList[0]
            self.vrobot.sl = -30
            self.vrobot.sr = -30   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()

    def move_left(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = -15
            self.vrobot.sr = 15   
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()       

    def move_right(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 15
            self.vrobot.sr = -15  
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr) 
            self.vrobot.t = time.time()      

    def stop_move(self, event=None):
        if self.gRobotList: 
            robot = self.gRobotList[0]
            self.vrobot.sl = 0
            self.vrobot.sr = 0
            robot.set_wheel(0,self.vrobot.sl)
            robot.set_wheel(1,self.vrobot.sr)
            self.vrobot.t = time.time()  

    def update_virtual_robot(self):
        # this is the robot modeling code - below is a very simple and inaccurate
        # model, as example of how to use the GUI toolkit you need to create you
        # own model

        noise_prox = 25 # noisy level for proximity
        noise_floor = 20 # floor ambient color - if floor is darker, set higher noise
        p_factor = 1.4 # proximity conversion - assuming linear
        d_factor = 1.1 # travel distance conversion
        a_factor = 17 # rotation conversion, assuming linear 

        while not self.gRobotList:
            print "waiting for robot to connect"
            time.sleep(0.1)

        print "connected to robot in update_virtual_robot"

        while not gQuit:
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
    global gQuit
    global m
    m.quit()
    gQuit = True
    print "Exit"

def draw_virtual_world(virtual_world, joystick):
    time.sleep(1) # give time for robot to connect.
    while not gQuit:
        if joystick.gRobotList is not None:
            virtual_world.draw_robot()
            virtual_world.draw_prox("left")
            virtual_world.draw_prox("right")
            virtual_world.draw_floor("left")
            virtual_world.draw_floor("right")
        time.sleep(0.1)

def main(argv=None): 
    global m, comm
    global sleepTime
    sleepTime = 0.01
    comm = RobotComm(gMaxRobotNum)
    comm.start()
    print 'Bluetooth starts'
    m = tk.Tk() #root
    drawQueue = Queue.Queue(0)

    #creating tje virtual appearance of the robot
    canvas_width = 700 # half width
    canvas_height = 380 # half height
    rCanvas = tk.Canvas(m, bg="white", width=canvas_width*2, height=canvas_height*2)

    joystick = Joystick(comm, m, rCanvas)

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
    #objects in the world
    rectF = [0, 50, 40, -50]
    rectC = [0, 140, -100, 180]
    rectB = [-100, 180, -140, 80]
    rectE = [0, -140, -100, -180]
    rectD = [-100, -180, -140, -80]
    rectA = [-220, -20, -260, 20]

    vWorld.add_obstacle(rectA)
    vWorld.add_obstacle(rectB)
    vWorld.add_obstacle(rectC)
    vWorld.add_obstacle(rectD)
    vWorld.add_obstacle(rectE)
    vWorld.add_obstacle(rectF)

    draw_world_thread = threading.Thread(target=draw_virtual_world, args=(vWorld, joystick))
    draw_world_thread.daemon = True
    draw_world_thread.start()

    gui = VirtualWorldGui(vWorld, m)

    rCanvas.after(200, gui.updateCanvas, drawQueue)
    m.mainloop()


    for robot in joystick.gRobotList:
        robot.reset()
    comm.stop()
    comm.join()


if __name__ == "__main__":
    main()

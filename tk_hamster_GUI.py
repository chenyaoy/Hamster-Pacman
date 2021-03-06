'''
/* =======================================================================
   (c) 2015, Kre8 Technology, Inc.

   PROPRIETARY and CONFIDENTIAL

   This file contains source code that constitutes proprietary and
   confidential information created by David Zhu

   Kre8 Technology retains the title, ownership and intellectual property rights
   in and to the Software and all subsequent copies regardless of the
   form or media.  Copying or distributing any portion of this file
   without the written permission of Kre8 Technology is prohibited.

   Use of this code is governed by the license agreement,
   confidentiality agreement, and/or other agreement under which it
   was distributed. When conflicts or ambiguities exist between this
   header and the written agreement, the agreement supersedes this file.
   ========================================================================*/
'''

import Tkinter as tk  
import time  
import math
import Queue

class virtual_robot:
    def __init__(self, agentIndex):
        #self.robot = None
        if agentIndex == 0:
            self.x = 0 # x coordinate
            self.y = 0 # y coordinate
        elif agentIndex == 1: #ghost 1 (Blinky)
            self.x = 120 # x coordinate
            self.y = 120 # y coordinate
        elif agentIndex ==2: #ghost 2 (Inky)
            self.x = -120
            self.y = -120
        
        self.l = 20*math.sqrt(2) # half diagonal - robot is 40 mm square
        
        self.a = 0 # angle of the robot, 0 when aligned with verticle axis
        self.dist_l = False
        self.dist_r = False #distance
        self.floor_l = False 
        self.floor_r = False 
        self.sl = 0 # speed of left wheel
        self.sr = 0 # speed of right wheel
        self.t = 0 # last update time
        self.localization_x_points = []
        self.localization_y_points = []
        self.previous_a = 0

    def reset_robots(self, agentIndex):
        if agentIndex == 0:
            self.x = 0 # x coordinate
            self.y = 0 # y coordinate
        elif agentIndex == 1: #ghost 1 (Blinky)
            self.x = 120 # x coordinate
            self.y = 120 # y coordinate
        elif agentIndex ==2: #ghost 2 (Inky)
            self.x = -120
            self.y = -120
        
        self.a = 0 # angle of the robot, 0 when aligned with verticle axis
        self.dist_l = False
        self.dist_r = False
        self.floor_l = False 
        self.floor_r = False     
        self.sl = 0 # speed of left wheel
        self.sr = 0 # speed of right wheel
        self.t = 0 # last update time

    def set_robot_speed(self, w_l, w_r):
        self.sl = w_l
        self.sr = w_r

    def set_robot_a_pos(self, a, x, y):
        self.a = a
        self.x = x
        self.y = y

    def set_robot_prox_dist(self, dist_l, dist_r):
        self.dist_l = dist_l
        self.dist_r = dist_r

    def set_robot_floor (self, floor_l, floor_r):
        self.floor_l = floor_l
        self.floor_r = floor_r

class virtual_world:
    def __init__(self, drawQueue, vrobots=None, canvas=None, canvas_width=0,
                 canvas_height=0, mp=None, trace=False, prox_dots=False,
                 floor_dots=False):
        self.drawQueue = drawQueue
        self.vrobots = vrobots
        self.canvas = canvas
        self.canvas_width = canvas_width
        self.canvas_height = canvas_height
        self.map = mp if mp is not None else []
        self.pellets = []
        self.super_pellets =[]
        self.pellet_ids = [[0 for x in range(5)] for x in range(5)]
        self.trace = trace #leave trace of robot
        self.prox_dots = prox_dots # draw obstacles detected as dots on map
        self.floor_dots = floor_dots
        self.score_label = None
        self.score_points = None

    def add_pellet(self, pill):
        self.pellets.append(pill)
    
    def add_super_pellet(self, super_pill):
        self.super_pellets.append(super_pill)
    
    
    def add_obstacle(self,rect):
        self.map.append(rect)

    def draw_rect(self, x1, y1, x2, y2):
        self.drawQueue.put(lambda: self.canvas.create_rectangle([x1,y1,x2,y2], fill=None, outline='blue', width=10))
        
    def draw_pellet(self, x1, y1, x2, y2, grid_x, grid_y):
        self.pellet_ids[grid_x][grid_y] = self.canvas.create_oval([x1, y1, x2, y2], fill='green')
        self.drawQueue.put(lambda: self.pellet_ids[grid_x][grid_y])
    
    def draw_super_pellet(self, x1, y1, x2, y2, grid_x, grid_y):
        self.pellet_ids[grid_x][grid_y] =self.canvas.create_oval([x1, y1, x2, y2], fill='magenta')
        self.drawQueue.put(lambda: self.pellet_ids[grid_x][grid_y])
    
    def add_score_label(self, coordinates):
        x = self.canvas_width + coordinates[0]
        y = self.canvas_height - coordinates[1]
        rect_x1 =  x- 25
        rect_y1 = y+25
        rect_x2 = x+25
        rect_y2=  y-25
        self.score_label = self.canvas.create_text(x-35, y, text="Score:", font=("Times", 20), fill='white')
        self.score_points = self.canvas.create_text(x, y, text= "0", font=("Times", 20), fill= 'white')
#        self.drawQueue.put(lambda: self.canvas.create_rectange([rect_x1, rect_y1, rect_y1, rect_y2], fill='black'))



    
    def update_score(self, score):
        self.canvas.itemconfig(self.score_points, text= str(score))
    
    def draw_food_layout(self, pellet_list):
        if pellet_list:
            for x in range(5):
                for y in range(5):
                    if not pellet_list[x][y]:
                        if self.pellet_ids[x][y] != 0:
                            self.canvas.delete(self.pellet_ids[x][y])
                            self.pellet_ids[x][y] = 0

    def draw_super_pellets(self, super_pellet_list):
        for super_pellet in super_pellet_list:
            x1 = canvas_width +super_pellet[0] - 15
            y1 = canvas_height - super_pellet[1] +15
            x2 = canvas_width + super_pellet[0] + 15
            y2 = canvas+height - super_pellet[1] - 15
            self.draw_super_pellet(x1, y1, x2, y2)

    def coordinate_to_grid(self, coordinate):
        if coordinate == -120:
            return 0
        elif coordinate == -60:
            return 1
        elif coordinate == 0:
            return 2
        elif coordinate == 60:
            return 3
        elif coordinate == 120:
            return 4

    def draw_map(self):
        canvas_width = self.canvas_width
        canvas_height = self.canvas_height
        for rect in self.map:
            x1 = self.canvas_width + rect[0]
            y1 = self.canvas_height - rect[1]
            x2 = self.canvas_width + rect[2]
            y2 = self.canvas_height - rect[3]
            self.draw_rect(x1, y1, x2, y2)
        for pill in self.pellets:
            x1 = self.canvas_width + pill[0] - 10
            y1 = self.canvas_height - pill[1] + 10
            x2 = self.canvas_width + pill[0] + 10
            y2 = self.canvas_height - pill[1] - 10
            grid_x = self.coordinate_to_grid(pill[0])
            grid_y = self.coordinate_to_grid(pill[1])
            self.draw_pellet(x1,y1, x2, y2, grid_x, grid_y )
        for super_pill in self.super_pellets:
            x1 = canvas_width + super_pill[0] - 15
            y1 = canvas_height - super_pill[1] +15
            x2 = canvas_width + super_pill[0] + 15
            y2 = canvas_height - super_pill[1] - 15
            grid_x = self.coordinate_to_grid(super_pill[0])
            grid_y = self.coordinate_to_grid(super_pill[1])
            self.draw_super_pellet(x1, y1, x2, y2, grid_x, grid_y)

    def draw_robot(self, agentIndex):
        canvas_width = self.canvas_width
        canvas_height = self.canvas_height
        pi4 = 3.1415 / 4 # quarter pi
        vrobot = self.vrobots[agentIndex]
        a1 = vrobot.a + pi4
        a2 = vrobot.a + 3*pi4
        a3 = vrobot.a + 5*pi4
        a4 = vrobot.a + 7*pi4

        x1 = canvas_width + vrobot.l * math.sin(a1) + vrobot.x
        x2 = canvas_width + vrobot.l * math.sin(a2) + vrobot.x
        x3 = canvas_width + vrobot.l * math.sin(a3) + vrobot.x        
        x4 = canvas_width + vrobot.l * math.sin(a4) + vrobot.x

        y1 = canvas_height - vrobot.l * math.cos(a1) - vrobot.y
        y2 = canvas_height - vrobot.l * math.cos(a2) - vrobot.y
        y3 = canvas_height - vrobot.l * math.cos(a3) - vrobot.y
        y4 = canvas_height - vrobot.l * math.cos(a4) - vrobot.y

        points = (x1,y1,x2,y2,x3,y3,x4,y4)
        poly_id = vrobot.poly_id
        self.drawQueue.put(lambda: self.canvas.coords(poly_id, points))

        if (self.trace):
            pi3 = 3.1415/3
            a1 = vrobot.a
            a2 = a1 + 2*pi3
            a3 = a1 + 4*pi3
            x1 = canvas_width + 3 * math.sin(a1) + vrobot.x
            x2 = canvas_width + 3 * math.sin(a2) + vrobot.x
            x3 = canvas_width + 3 * math.sin(a3) + vrobot.x 
            y1 = canvas_height - 3 * math.cos(a1) - vrobot.y
            y2 = canvas_height - 3 * math.cos(a2) - vrobot.y
            y3 = canvas_height - 3 * math.cos(a3) - vrobot.y
            self.drawQueue.put(lambda: self.canvas.create_polygon([x1,y1,x2,y2,x3,y3], outline="blue"))

    def draw_prox(self, side, agentIndex):
        canvas_width = self.canvas_width
        canvas_height = self.canvas_height
        vrobot = self.vrobots[agentIndex]
        if (side == "left"):
            a_e = vrobot.a - 3.1415/5 #emitter location
            prox_dis = vrobot.dist_l
            prox_l_id = vrobot.prox_l_id
        else:
            a_e = vrobot.a + 3.1415/5 #emitter location
            prox_dis = vrobot.dist_r
            prox_l_id = vrobot.prox_r_id
        if (prox_dis):
            x_e = (vrobot.l-4) * math.sin(a_e) + vrobot.x #emiter pos of left sensor
            y_e = (vrobot.l-4) * math.cos(a_e) + vrobot.y #emiter pos of right sensor
            x_p = prox_dis * math.sin(vrobot.a) + x_e
            y_p = prox_dis * math.cos(vrobot.a) + y_e
            if (self.prox_dots):
                self.drawQueue.put(lambda: self.canvas.create_oval(canvas_width+x_p-1, canvas_height-y_p-1, canvas_width+x_p+1, canvas_height-y_p+1, outline='red'))
            points = (canvas_width+x_e, canvas_height-y_e, canvas_width+x_p, canvas_height-y_p)
            self.drawQueue.put(lambda: self.canvas.coords(prox_l_id, points))
        else:
            points = (0,0,0,0)
            self.drawQueue.put(lambda: self.canvas.coords(prox_l_id, points))

    def draw_floor(self, side, agentIndex):
        canvas_width = self.canvas_width
        canvas_height = self.canvas_height
        vrobot = self.vrobots[agentIndex]
        if (side == "left"):
            border = vrobot.floor_l
            floor_id = vrobot.floor_l_id
            a = vrobot.a - 3.1415/7 #rough position of the left floor sensor
        else:
            border = vrobot.floor_r
            floor_id = vrobot.floor_r_id
            a = vrobot.a + 3.1415/7 #rough position of the left floor sensor         
        x_f = (vrobot.l - 12) * math.sin(a) + vrobot.x
        y_f = (vrobot.l - 12) * math.cos(a) + vrobot.y
        points = (canvas_width+x_f-2, canvas_height-y_f-2, canvas_width+x_f+2, canvas_height-y_f+2)
        self.drawQueue.put(lambda: self.canvas.coords(floor_id, points))
        if (border): 
            self.drawQueue.put(lambda: self.canvas.itemconfig(floor_id, outline = "black", fill="black"))
            if (self.floor_dots):
                self.drawQueue.put(lambda: self.canvas.create_oval(canvas_width+x_f-2, canvas_height-y_f-2, canvas_width+x_f+2, canvas_height-y_f+2, fill='black'))
        else:
            self.drawQueue.put(lambda: self.canvas.itemconfig(floor_id, outline = "white", fill="white"))



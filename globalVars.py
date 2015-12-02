from HamsterAPI.comm_ble import RobotComm
import Tkinter as tk 

gMaxRobotNum = 1 # max number of robots to control
comm = RobotComm(gMaxRobotNum)
m = tk.Tk()
gQuit = False
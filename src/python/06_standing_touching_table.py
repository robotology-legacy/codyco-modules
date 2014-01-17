import loader

import rtt_interface as rtt
import rtt_interface_corba
rtt.Logger_Instance().setLogLevel(rtt.Error)
import time
import deploy
deploy.loadTypekitsAndPlugins()
import xdefw.interactive
shell = xdefw.interactive.shell()

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import xde_isir_controller as xic

import pylab as pl
import numpy
import lgsm

#GUI
import Tkinter as tk
#from tk import *

import sys
import os
import inspect
cpath = os.path.dirname(os.path.abspath(inspect.getfile( inspect.currentframe()))) + "/"
sys.path.append(cpath)

# from pycontrol import VHController
from pycontrol_smooth_contact_force import VHController


    
    

pi = numpy.pi
# minimum distance for computing the local distance between shapes in possible collision
local_minimal_distance = 0.02 #0.05
# height of the table
table_z = 0.50
########### CONTROLLER ###############

# formulation of the control problem
# False= use full problem (ddq, Fext, tau) => more stable, but slower (not inverse of mass matrix)
# True= use reduced problem (Fext, tau) => faster
use_reduced_problem_no_ddq = False

# solver for the quadratic problem 
# qld = more accurate
solver = "qld" # "quadprog"

###################### PARAMETERS ##############################


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=local_minimal_distance)
wm.resizeWindow("mainWindow",  640, 480, 50, 50)

##### WORLD

## GROUND
#flat ground
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
#uneven ground
wm.addWorld(groundWorld)

## MOVING WALL
robotWorld = xrl.createWorldFromUrdfFile("resources/moving_table.xml", "moving_wall", [-0.4,0.0,table_z,0.707,0,0.707,0], True, 0.001, 0.01)

wm.addWorld(robotWorld)
dynModel_moving_wall = xrl.getDynamicModelFromWorld(robotWorld)

## ROBOT
rname = "robot"
fixed_base = False
robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, rname, [0,0,0.6,1,0,0,0], fixed_base, 0.001, 0.001)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)

N  = robot.getJointSpaceDim()
# get dynamic model of the robot
dynModel = xrl.getDynamicModelFromWorld(robotWorld)
jmap     = xrl.getJointMapping(xr.icub_simple, robot)

# suppress joint limits of the icub => to allow more movement
# robot.setJointPositionsMin(-4.0*lgsm.ones(N))
# robot.setJointPositionsMax(4.0*lgsm.ones(N))
jl_min = robot.getJointPositionsMin()
jl_max = robot.getJointPositionsMax()
jl_max[jmap[rname+".l_knee"]] = 0.
jl_max[jmap[rname+".r_knee"]] = 0.
# robot.setJointPositionsMin(-4.0*lgsm.ones(N))
robot.setJointPositionsMax(jl_max)
##### SET CONTACT TYPES
#set contact law
# = material 1
# = material 2
# = 0=no friction, 1=cone, 2=cone+friction in rotation
# = mu=coeff of friction
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)
wm.ms.setContactLawForMaterialPair("material.metal", "material.metal", 1, 1.5)

## enable contact with flat ground
robot.enableContactWithBody("ground.ground", True)
# this only displays the contacts ground with feet
wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) 

## enable contact with moving wall
robot.enableContactWithBody("moving_wall.moving_wall", True)
# this displays all the contacts: whole body with wall
#wm.contact.showContacts([(b,"moving_wall.moving_wall") for b in robot.getSegmentNames()]) 
# this displays only the contacts of the hands
wm.contact.showContacts([(rname+"."+b,"moving_wall.moving_wall") for b in ["l_hand", "r_hand"]]) 



##### SET INITIAL STATE OF THE ROBOT
qinit = lgsm.zeros(N)
# correspond to:    l_elbow_pitch     r_elbow_pitch     l_knee             r_knee             l_ankle_pitch      r_ankle_pitch      l_shoulder_roll          r_shoulder_roll
for name, val in [("l_elbow_pitch", pi/2.), ("r_elbow_pitch", pi/2.), ("l_knee", -0.05), ("r_knee", -0.05), ("l_ankle_pitch", -0.05), ("r_ankle_pitch", -0.05), ("l_shoulder_roll", pi/2.), ("r_shoulder_roll", pi/2.)]:
    qinit[jmap[rname+"."+name]] = val    

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))




##### CONTROLLERS

## controller for the moving wall
ctrl_moving_wall = xic.ISIRController(dynModel_moving_wall, "moving_wall", wm.phy, wm.icsync, solver, use_reduced_problem_no_ddq)
gposdes = lgsm.zeros(1)
gveldes = lgsm.zeros(1)
# create full task with a very low weight for reference posture
# lower KP if the spring of the wall is too rigid
fullTask = ctrl_moving_wall.createFullTask("zero_wall", w=1., kp=3000)  
fullTask.set_q(gposdes)
fullTask.set_qdot(gveldes)

## OBSERVER MOVING WALL
tpobs = ctrl_moving_wall.add_updater(xic.observers.TorqueObserver(ctrl_moving_wall))

## controller for the robot
controller = VHController("controller", dt, dynModel,rname, wm.phy, wm.icsync, solver, use_reduced_problem_no_ddq,jmap, tpobs)

# ##### OBSERVERS

# ## OBSERVERS COM
# cpobs = ctrl.add_updater(xic.observers.CoMPositionObserver(ctrl.getModel()))
# fpobs = ctrl.add_updater(xic.observers.FramePoseObserver(ctrl.getModel(), rname+'.waist', lgsm.Displacement()))
# obs_l_foot = ctrl.add_updater(xic.observers.FramePoseObserver(ctrl.getModel(), rname+'.l_foot', lgsm.Displacement()))
# obs_r_foot = ctrl.add_updater(xic.observers.FramePoseObserver(ctrl.getModel(), rname+'.r_foot', lgsm.Displacement()))

# ## OBSERVERS ANKLE
# all_joints_obs = ctrl.add_updater(xic.observers.JointPositionsObserver(ctrl.getModel()))




#######################################################
################ SIMULATE ######################""

# launch the simulation
wm.startAgents()
controller.s.start()
ctrl_moving_wall.s.start()
wm.phy.s.agent.triggerUpdate()
# time.sleep(1.)


class MyTk(tk.Frame):
   def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.parent.title("Parameters")
        self.pack(fill = tk.BOTH, expand = 1)

        menubar = tk.Menu(self.parent)
        self.parent.config(menu = menubar)

        self.label1 = Label(self, border = 25, text="w1")
        self.label2 = Label(self, border = 25, text="w2")
        self.label1.grid(row = 1, column = 1)
        self.label2.grid(row = 2, column = 1)
        self.w1 = Scale(self, from_=0, to=20, length=600, tickinterval=1, orient=tk.HORIZONTAL)
        self.w1.set(1)
        self.w1.grid(row = 1, column = 2)
        self.w2 = Scale(self, from_=0, to=20, length=600, tickinterval=1, orient=tk.HORIZONTAL)
        self.w2.set(10)
        self.w2.grid(row = 2, column = 2)
         
        # File Menu
        fileMenu = tk.Menu(menubar)
        menubar.add_cascade(label = "Menu", menu = fileMenu)

        # Menu Item for Open Image
        fileMenu.add_command(label = "Set values", command = self.onSetValue) 
        
   def setController(self,controller):
        self.controller=controller
              
   def onSetValue(self):
        #set Callback
        print (self.w1.get(), self.w2.get())
        self.controller.CoMTask.setWeight(self.w1.get())
        print "new weights are set"
        

# activate GUI
master = tk.Tk()
master.geometry("300x400+300+300")
gui = MyTk(master)
gui.setController(controller)
master.mainloop()







# shell()
xdefw.interactive.shell_console()()




time.sleep(1.)
wm.stopAgents()
controller.s.stop()
ctrl_moving_wall.s.stop()

####################### STOP SIMU ######################""

# ##### RESULTS
# comtraj = numpy.array(cpobs.get_record())
# wtraj = numpy.array(fpobs.get_record())
# jtraj = numpy.array(all_joints_obs.get_record())
# rf_traj = numpy.array(obs_r_foot.get_record())
# lf_traj = numpy.array(obs_l_foot.get_record())
tpos = tpobs.get_record()
print tpos
# pl.figure()
# pl.plot(comtraj[:,0],label="com x")
# pl.plot(comtraj[:,1],label="com y")
# pl.plot(comtraj[:,2],label="com z")
# pl.legend()
# pl.savefig("results/05_touch_table_5N_COM.pdf", bbox_inches='tight' )

# #pl.figure()
# #pl.plot(wtraj[:,0],label="waist x")
# #pl.plot(wtraj[:,1],label="waist y")
# #pl.plot(wtraj[:,2],label="waist z")
# #pl.legend()

# #pl.figure()
# #pl.plot(rf_traj[:,0],label="right foot x")
# #pl.plot(rf_traj[:,1],label="right foot y")
# #pl.plot(rf_traj[:,2],label="right foot z")
# #pl.legend()

# #pl.figure()
# #pl.plot(lf_traj[:,0],label="left foot x")
# #pl.plot(lf_traj[:,1],label="left foot y")
# #pl.plot(lf_traj[:,2],label="left foot z")
# #pl.legend()

# #pl.figure()
# #pl.plot(jtraj[:,0]*rad2deg,label="0")
# #pl.plot(jtraj[:,1]*rad2deg,label="1")
# #pl.plot(jtraj[:,2]*rad2deg,label="2")
# #pl.plot(jtraj[:,3]*rad2deg,label="3")
# #pl.plot(jtraj[:,4]*rad2deg,label="4")
# #pl.plot(jtraj[:,5]*rad2deg,label="5")
# #pl.title("left leg")
# #pl.legend()

# #pl.figure()
# #pl.plot(jtraj[:,26]*rad2deg,label="26")
# #pl.plot(jtraj[:,27]*rad2deg,label="27")
# #pl.plot(jtraj[:,28]*rad2deg,label="28")
# #pl.plot(jtraj[:,29]*rad2deg,label="29")
# #pl.plot(jtraj[:,30]*rad2deg,label="30")
# #pl.plot(jtraj[:,31]*rad2deg,label="31")
# #pl.legend()
# #pl.title("right leg")

pl.figure()
pl.plot(tpos)
pl.legend()
pl.title("force on wall")
pl.savefig("results/05_touch_table_5N_force.pdf", bbox_inches='tight' )

#the last to show all plots
pl.show()







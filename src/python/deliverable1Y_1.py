#!/xde
#
# Copyright (C) 2013-7 CODYCO Project
# Author: Serena Ivaldi
# email:  serena.ivaldi@isir.upmc.fr
#
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details
#

# NOTES FOR THIS FILE
#
# this file simulates the leg free-falling. for doing this, we need to 
# lift the icub and not put it on the ground
# this is done by fixing the base when we create the robot
#
# robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, robot_name, 
# [x0,y0,z0,1,0,0,0], fixed_base=True, minimal_damping=0.001, composite_offset=0.001)

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import xde_isir_controller as xic
import lgsm
import time
import pylab as pl
import numpy


### PARAMETERS ###############
pi = lgsm.np.pi
rad2deg = 180.0/pi;
# minimum distance for computing the local distance between shapes in possible collision
local_minimal_distance = 0.02 
robot_name = "robot"
fixed_base = True
minimal_damping = 0.001
composite_offset = 0.001
# initial position of the robot in the space
x0=0
y0=0
z0=0.7
# initial joint configuration
init_l_elbow_pitch = pi/8.
init_r_elbow_pitch = pi/8.
init_l_knee = pi/4.
init_r_knee = pi/4.
init_l_ankle_pitch = 0
init_r_ankle_pitch = 0
init_l_shoulder_roll = pi/8.
init_r_shoulder_roll = pi/8.


########### CONTROLLER ###############
# formulation of the control problem
use_reduced_problem_no_ddq = False
# solver for the quadratic problem 
solver = "qld" # "quadprog"




##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)
### GROUND
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(groundWorld)
### ROBOT
#robot on ground
#robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, robot_name, [0,0,0.6,1,0,0,0], fixed_base, 0.001, 0.001)
#robot floating
robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, robot_name, [x0,y0,z0,1,0,0,0], fixed_base, minimal_damping, composite_offset)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(robot_name)
robot.enableGravity(True)
N  = robot.getJointSpaceDim()


dynModel = xrl.getDynamicModelFromWorld(robotWorld)
# NEW WAY: joint map of the robot
jmap     = xrl.getJointMapping(xr.icub_simple, robot)


##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)

robot.enableContactWithBody("ground.ground", True)
wm.contact.showContacts([(robot_name+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) # to display contact



##### SET INITIAL STATE
qinit = lgsm.zeros(N)
# NEW WAY
# correspond to:    l_elbow_pitch     r_elbow_pitch     l_knee             r_knee             l_ankle_pitch      r_ankle_pitch      l_shoulder_roll          r_shoulder_roll
for name, val in [("l_elbow_pitch", init_l_elbow_pitch), ("r_elbow_pitch", init_r_elbow_pitch), ("l_knee", init_l_knee), ("r_knee", pi/4.), ("l_ankle_pitch", 0), ("r_ankle_pitch", 0), ("l_shoulder_roll", pi/8.), ("r_shoulder_roll", pi/8.)]:
    qinit[jmap[robot_name+"."+name]] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))


##### CTRL
# flat ground
#ctrl = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel, robot_name, wm.phy, wm.icsync, "quadprog", True)
#OLD WAY
#ctrl = xic.ISIRCtrl(xic.xic_config.xic_path, dynModel, robot_name, wm.phy, wm.icsync, "qld", False)
#
ctrl = xic.ISIRController(dynModel, robot_name, wm.phy, wm.icsync, "qld", False)


##### SET TASKS
# this ...
#N0 = 6 if fixed_base is False else 0
#partialTask = ctrl.createPartialTask("partial", range(N0, N+N0), 0.0001, kp=9., pos_des=qinit)
# ... is equivalent to that ("INTERNAL" can be omitted):
#OLD WAY
#fullTask = ctrl.createFullTask("full", 0.0001, "INTERNAL", kp=9., pos_des=qinit)
#NW WAY
fullTask = ctrl.createFullTask("full", "INTERNAL", w=0.0001, kp=9., q_des=qinit)

#OLD WAY
#waistTask   = ctrl.createFrameTask("waist", robot_name+'.waist', lgsm.Displacement(), "RXYZ", 1., kp=36., pos_des=lgsm.Displacement(0,0,.58,1,0,0,0))
#NEW WAY
waistTask   = ctrl.createFrameTask("waist", robot_name+'.waist', lgsm.Displacement(), "RXYZ", w=1., kp=36., pose_des=lgsm.Displacement(0,0,.58,1,0,0,0))


#OLD WAY
#back_name   = [robot_name+"."+n for n in ['lap_belt_1', 'lap_belt_2', 'chest']]
#backTask    = ctrl.createPartialTask("back", back_name, 0.001, kp=16., pos_des=lgsm.zeros(3))
#NEW WAY
back_dofs   = [jmap[robot_name+"."+n] for n in ['torso_pitch', 'torso_roll', 'torso_yaw']]
backTask    = ctrl.createPartialTask("back", back_dofs, w=0.001, kp=16., q_des=lgsm.zeros(3))


sqrt2on2 = lgsm.np.sqrt(2.)/2.
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)

ctrl.createContactTask("CLF0", robot_name+".l_foot", lgsm.Displacement([-.039,-.027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF1", robot_name+".l_foot", lgsm.Displacement([-.039, .027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF2", robot_name+".l_foot", lgsm.Displacement([-.039, .027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF3", robot_name+".l_foot", lgsm.Displacement([-.039,-.027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin

ctrl.createContactTask("CRF0", robot_name+".r_foot", lgsm.Displacement([-.039,-.027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF1", robot_name+".r_foot", lgsm.Displacement([-.039, .027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF2", robot_name+".r_foot", lgsm.Displacement([-.039, .027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF3", robot_name+".r_foot", lgsm.Displacement([-.039,-.027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin


#for n in ["CL"+str(s) for s in range(1,5)] + ["CR"+str(s) for s in range(1,5)]:
#    ctrl.s.activateTaskAsConstraint(n)


##### OBSERVERS COM

#OLD WAY --> change in the function and call model from the controller
#fpobs = ctrl.updater.register(xic.observers.FramePoseObserver(dynModel, robot_name+'.waist', lgsm.Displacement()))
#NEW WAY
fpobs = ctrl.add_updater(xic.observers.FramePoseObserver(ctrl.getModel(), robot_name+'.waist', lgsm.Displacement()))


obs_l_foot = ctrl.add_updater(xic.observers.FramePoseObserver(ctrl.getModel(), robot_name+'.l_foot', lgsm.Displacement()))
obs_r_foot = ctrl.add_updater(xic.observers.FramePoseObserver(ctrl.getModel(), robot_name+'.r_foot', lgsm.Displacement()))

##### OBSERVERS ANKLE
all_joints_obs = ctrl.add_updater(xic.observers.JointPositionsObserver(ctrl.getModel()))


##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import xdefw.interactive
#xdefw.interactive.shell_console()()
time.sleep(15.)

wm.stopAgents()
ctrl.s.stop()

##### RESULTS
wtraj = numpy.array(fpobs.get_record())
jtraj = numpy.array(all_joints_obs.get_record())
rf_traj = numpy.array(obs_r_foot.get_record())
lf_traj = numpy.array(obs_l_foot.get_record())

pl.figure()
pl.plot(wtraj[:,0],label="waist x")
pl.plot(wtraj[:,1],label="waist y")
pl.plot(wtraj[:,2],label="waist z")
pl.legend()

pl.figure()
pl.plot(rf_traj[:,0],label="right foot x")
pl.plot(rf_traj[:,1],label="right foot y")
pl.plot(rf_traj[:,2],label="right foot z")
pl.legend()

pl.figure()
pl.plot(lf_traj[:,0],label="left foot x")
pl.plot(lf_traj[:,1],label="left foot y")
pl.plot(lf_traj[:,2],label="left foot z")
pl.legend()

pl.figure()
pl.plot(jtraj[:,0]*rad2deg,label="0")
pl.plot(jtraj[:,1]*rad2deg,label="1")
pl.plot(jtraj[:,2]*rad2deg,label="2")
pl.plot(jtraj[:,3]*rad2deg,label="3")
pl.plot(jtraj[:,4]*rad2deg,label="4")
pl.plot(jtraj[:,5]*rad2deg,label="5")
pl.legend()

pl.figure()
pl.plot(jtraj[:,6]*rad2deg,label="6")
pl.plot(jtraj[:,7]*rad2deg,label="7")
pl.plot(jtraj[:,8]*rad2deg,label="8")
pl.legend()

pl.figure()
pl.plot(jtraj[:,9]*rad2deg,label="9")
pl.plot(jtraj[:,10]*rad2deg,label="10")
pl.plot(jtraj[:,11]*rad2deg,label="11")
pl.legend()

pl.figure()
pl.plot(jtraj[:,26]*rad2deg,label="26")
pl.plot(jtraj[:,27]*rad2deg,label="27")
pl.plot(jtraj[:,28]*rad2deg,label="28")
pl.plot(jtraj[:,29]*rad2deg,label="29")
pl.plot(jtraj[:,30]*rad2deg,label="30")
pl.plot(jtraj[:,31]*rad2deg,label="31")
pl.legend()
pl.title("right leg")

#the last to show all plots
pl.show()




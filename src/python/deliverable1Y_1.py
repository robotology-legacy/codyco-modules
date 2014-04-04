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
#
#
# TYPES OF ICUB
# 
# tests are performed with different types of icub -> without and with compensated and 
# identified viscous friction
# these models of iCub have to be inserted in xde-isir/xde-resources
# 
#icub_ge03_comp_visc_friction  -> iCubGenova03_compensatedViscousFriction.urdf
#icub_ge03_iden_visc_friction -> iCubGenova03_identifiedViscousFriction.urdf
#icub_ge03_no_visc_friction -> iCubGenova03_noViscousFriction.urdf


import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import xde_isir_controller as xic
import lgsm
import time
import pylab as pl
import numpy

# from pycontrol import VHController
from pycontrol_smooth_contact_force import Deliverable1YController

### PARAMETERS ###############
pi = lgsm.np.pi
rad2deg = 180.0/pi;
# minimum distance for computing the local distance between shapes in possible collision
local_minimal_distance = 0.02 
fixed_base = True
minimal_damping = 0.001
composite_offset = 0.001
# initial position of the robot in the space
x0=0
y0=0
z0=0.7
# initial joint configuration
init_l_shoulder_roll = pi/8.
init_r_shoulder_roll = pi/8.
init_l_elbow_pitch = pi/2.
init_r_elbow_pitch = pi/2.
init_l_knee = 0 #pi/4.
init_r_knee = 0.
init_l_ankle_pitch = 0
init_r_ankle_pitch = 0
init_l_hip_pitch = 0.
init_l_hip_yaw = 0.
init_r_hip_pitch = pi/2.
init_r_hip_yaw = 0.


########### CONTROLLER ###############
# formulation of the control problem
use_reduced_problem_no_ddq = False
# solver for the quadratic problem 
solver = "qld" # "quadprog"

##### ROBOT TYPE

icub = xr.icub_simple
#icub = xr.icub_ge03_comp_visc_friction
#icub = xr.icub_ge03_iden_visc_friction
#icub = xr.icub_ge03_no_visc_friction 

robot_name = "robot"
robot1_name = "robot_comp_visc"
robot2_name = "robot_iden_visc"
robot3_name = "robot_no_visc"


##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)
### GROUND
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(groundWorld)
### ROBOT
#robot floating
robotWorld = xrl.createWorldFromUrdfFile(icub, robot_name, [x0,y0,z0,1,0,0,0], fixed_base, minimal_damping, composite_offset)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(robot_name)
robot.enableGravity(True)
N  = robot.getJointSpaceDim()
dynModel = xrl.getDynamicModelFromWorld(robotWorld)
jmap     = xrl.getJointMapping(xr.icub_simple, robot)

print "JOINT MAP"
print jmap


##### NO INTERACTION

##### SET INITIAL STATE OF THE ROBOT
qinit = lgsm.zeros(N)
# NEW WAY
# correspond to:    l_elbow_pitch     r_elbow_pitch     l_knee             r_knee             l_ankle_pitch      r_ankle_pitch      l_shoulder_roll          r_shoulder_roll
for name, val in [ ("r_elbow_pitch", init_r_elbow_pitch), ("l_elbow_pitch", init_l_elbow_pitch),
                  ("r_knee", init_r_knee), ("l_knee", init_l_knee), 
                ("r_hip_pitch",init_r_hip_pitch), ("l_hip_pitch",init_l_hip_pitch),
                ("r_hip_yaw",init_r_hip_yaw),("l_hip_yaw",init_l_hip_yaw),
                ("l_ankle_pitch", init_l_ankle_pitch), ("r_ankle_pitch", init_r_ankle_pitch), 
                ("l_shoulder_roll", init_l_shoulder_roll), ("r_shoulder_roll", init_r_shoulder_roll)]:
    qinit[jmap[robot_name+"."+name]] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))

##### CONTROLLER OF THE ROBOT - the modified controller

## controller for the robot
ctrl = Deliverable1YController("controller", dt, dynModel,robot_name, wm.phy, wm.icsync, solver, use_reduced_problem_no_ddq,jmap)

# old solution
#ctrl = xic.ISIRController(dynModel, robot_name, wm.phy, wm.icsync, "qld", False)
#all_joints_obs = ctrl.add_updater(xic.observers.JointPositionsObserver(ctrl.getModel()))

##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import xdefw.interactive
#xdefw.interactive.shell_console()()
time.sleep(3.04)

wm.stopAgents()
ctrl.s.stop()

q_trj = ctrl.observer_q;
dq_trj = ctrl.observer_dq;


print len(q_trj)
print len(q_trj[0])

Nsamples=len(q_trj)
Njoints=len(q_trj[0])

qtr = numpy.zeros((Njoints,Nsamples))

for m in range(0, Nsamples):
    #print numpy.squeeze(q_trj[m])
    qtr[:, m] = numpy.squeeze(q_trj[m])
    

#print qtr
print numpy.squeeze(q_trj[27,:])

#q_traj=numpy.asarray(q_trj)
#dq_traj=numpy.asarray(dq_trj)

r_leg_dofs   = [jmap[robot_name+"."+n] for n in ['r_hip_roll', 'r_hip_yaw', 'r_hip_pitch','r_knee','r_ankle_roll','r_ankle_pitch']]


#print q_traj[:,jmap[robot_name+".r_hip_roll"]]


##### RESULTS

pl.figure()
pl.plot(qtr[jmap[robot_name+".r_hip_roll"],:]*rad2deg,label="r_hip_roll")
pl.plot(qtr[jmap[robot_name+".r_hip_yaw"],:]*rad2deg,label="r_hip_yaw")
pl.plot(qtr[jmap[robot_name+".r_hip_pitch"],:]*rad2deg,label="r_hip_pitch")
pl.plot(qtr[jmap[robot_name+".r_knee"],:]*rad2deg,label="r_knee")
pl.plot(qtr[jmap[robot_name+".r_ankle_roll"],:]*rad2deg,label="r_ankle_roll")
pl.plot(qtr[jmap[robot_name+".r_ankle_pitch"],:]*rad2deg,label="r_ankle_pitch")
pl.legend()
pl.title("joint position")
#
#pl.figure()
#pl.plot(dq_traj[:,jmap[robot_name+".r_hip_roll"]]*rad2deg,label="r_hip_roll")
#pl.plot(dq_traj[:,jmap[robot_name+".r_hip_yaw"]]*rad2deg,label="r_hip_yaw")
#pl.plot(dq_traj[:,jmap[robot_name+".r_hip_pitch"]]*rad2deg,label="r_hip_pitch")
#pl.plot(dq_traj[:,jmap[robot_name+".r_knee"]]*rad2deg,label="r_knee")
#pl.plot(dq_traj[:,jmap[robot_name+".r_ankle_roll"]]*rad2deg,label="r_ankle_roll")
#pl.plot(dq_traj[:,jmap[robot_name+".r_ankle_pitch"]]*rad2deg,label="r_ankle_pitch")
###pl.plot(jtraj[:,1]*rad2deg,label="1")
###pl.plot(jtraj[:,2]*rad2deg,label="2")
###pl.plot(jtraj[:,3]*rad2deg,label="3")
###pl.plot(jtraj[:,4]*rad2deg,label="4")
###pl.plot(jtraj[:,5]*rad2deg,label="5")
#pl.legend()
#pl.title("joint velocity")
#
#
##the last to show all plots
pl.show()


# SAVE DATA IN FILES

#fw = open('joints', 'w+')
#fw.write('0123456789abcdef')



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

# NOTE
# this script is just used to show some human manikins of different
# size - height and weight - with respect to the robot

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import xde_isir_controller as xic
import lgsm
import time

pi = lgsm.np.pi
rad2deg = 180.0/pi;

# initial position of the human manikins in the space
H_initPos_4 = lgsm.Displacement(-0.5, 0., 0.6, 1., 0, 0, 0.)
H_initPos = lgsm.Displacement(-1., 0., 0.9, 1., 0, 0, 0.)
H_initPos_2 = lgsm.Displacement(-1.5, 0., 1., 1., 0, 0, 0.)
H_initPos_3 = lgsm.Displacement(-2.0, 0., 1.1, 1., 0, 0, 0.)

#color of each manikin
#colors are defined in:
#         xde-precise/xde/share/resources/ogre/materials/xde.material
human1_color = "xde/BlueOpaqueAvatars"
human2_color = "xde/VioletOpaque"
human3_color = "xde/YellowOpaqueAvatars"
human4_color = "xde/GreenOpaqueAvatars"



##### AGENTS
dt = 0.01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.2)
wm.resizeWindow("mainWindow",  640, 480, 1000, 50)


##### GROUND

groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(groundWorld)

##### ROBOT
rname = "robot"
fixed_base = False
robotWorld = xrl.createWorldFromUrdfFile(xr.icub_simple, rname, [0,0,0.6,1,0,0,0], fixed_base, 0.001, 0.001)
wm.addWorld(robotWorld)
robot = wm.phy.s.GVM.Robot(rname)
robot.enableGravity(True)
N  = robot.getJointSpaceDim()


dynModel = xrl.getDynamicModelFromWorld(robotWorld)
jmap     = xrl.getJointMapping(xr.icub_simple, robot)


####################
## Create manikin ##
####################

# Create manikin 1
NAME = 'Pauline'
people = {'Pauline':[1.56, 53.]}
size, mass = people[NAME]
import desc.simple.scene, human
vhWorld = desc.simple.scene.createWorld()
vhName = "manikin"
human.addManikin(vhWorld, vhName, mass, size, H_initPos, .1)
wm.addWorld(vhWorld)
vh = wm.phy.s.GVM.Robot(vhName)
wm.ms.assembleSystem()
vh.enableGravity(False)
wm.graph_scn.SceneInterface.setNodeMaterial(vhName,human1_color,True)

# Create manikin 2
NAME = 'Serena'
people = {'Serena':[1.80, 80.]}
size2, mass2 = people[NAME]
vhWorld2 = desc.simple.scene.createWorld()
vhName2 = "manikin2"
human.addManikin(vhWorld2, vhName2, mass2, size2, H_initPos_2, .1)
wm.addWorld(vhWorld2)
vh2 = wm.phy.s.GVM.Robot(vhName2)
wm.ms.assembleSystem()
vh2.enableGravity(False)
wm.graph_scn.SceneInterface.setNodeMaterial(vhName2,human2_color,True)

# Create manikin 3
NAME = 'Pippo'
people = {'Pippo':[2.0, 100.]}
size3, mass3 = people[NAME]
vhWorld3 = desc.simple.scene.createWorld()
vhName3 = "manikin3"
human.addManikin(vhWorld3, vhName3, mass3, size3, H_initPos_3, .1)
wm.addWorld(vhWorld3)
vh3 = wm.phy.s.GVM.Robot(vhName3)
wm.ms.assembleSystem()
vh3.enableGravity(False)
wm.graph_scn.SceneInterface.setNodeMaterial(vhName3,human3_color,True)

# Create manikin 4
NAME = 'Ciccio'
people = {'Ciccio':[1.0, 18.]}
size4, mass4 = people[NAME]
vhWorld4 = desc.simple.scene.createWorld()
vhName4 = "manikin4"
human.addManikin(vhWorld4, vhName4, mass4, size4, H_initPos_4, .1)
wm.addWorld(vhWorld4)
vh4 = wm.phy.s.GVM.Robot(vhName4)
wm.ms.assembleSystem()
vh4.enableGravity(False)
wm.graph_scn.SceneInterface.setNodeMaterial(vhName4,human4_color,True)

##### SET INTERACTION
wm.ms.setContactLawForMaterialPair("material.metal", "material.concrete", 1, 1.5)

robot.enableContactWithBody("ground.ground", True)
wm.contact.showContacts([(rname+"."+b,"ground.ground") for b in ["l_foot", "r_foot"]]) # to display contact



##### SET INITIAL STATE
qinit = lgsm.zeros(N)
# NEW WAY
# correspond to:    l_elbow_pitch     r_elbow_pitch     l_knee             r_knee             l_ankle_pitch      r_ankle_pitch      l_shoulder_roll          r_shoulder_roll
for name, val in [("l_elbow_pitch", 0.), ("r_elbow_pitch", 0.), ("l_knee", 0.01), ("r_knee", 0.01), ("l_ankle_pitch", -0.05), ("r_ankle_pitch", -0.05), ("l_shoulder_roll", pi/2.), ("r_shoulder_roll", pi/2.)]:
    qinit[jmap[rname+"."+name]] = val

robot.setJointPositions(qinit)
dynModel.setJointPositions(qinit)
robot.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))


##### CONTROL FOR ROBOT
ctrl = xic.ISIRController(dynModel, rname, wm.phy, wm.icsync, "qld", False)


##### SET TASKS
fullTask = ctrl.createFullTask("full", "INTERNAL", w=0.0001, kp=9., q_des=qinit)
waistTask   = ctrl.createFrameTask("waist", rname+'.waist', lgsm.Displacement(), "RXYZ", w=1., kp=36., pose_des=lgsm.Displacement(0,0,.58,1,0,0,0))
back_dofs   = [jmap[rname+"."+n] for n in ['torso_pitch', 'torso_roll', 'torso_yaw']]
backTask    = ctrl.createPartialTask("back", back_dofs, w=0.001, kp=16., q_des=lgsm.zeros(3))

sqrt2on2 = lgsm.np.sqrt(2.)/2.
RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)

ctrl.createContactTask("CLF0", rname+".l_foot", lgsm.Displacement([-.039,-.027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF1", rname+".l_foot", lgsm.Displacement([-.039, .027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF2", rname+".l_foot", lgsm.Displacement([-.039, .027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CLF3", rname+".l_foot", lgsm.Displacement([-.039,-.027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin

ctrl.createContactTask("CRF0", rname+".r_foot", lgsm.Displacement([-.039,-.027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF1", rname+".r_foot", lgsm.Displacement([-.039, .027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF2", rname+".r_foot", lgsm.Displacement([-.039, .027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
ctrl.createContactTask("CRF3", rname+".r_foot", lgsm.Displacement([-.039,-.027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin

##### SIMULATE
ctrl.s.start()

wm.startAgents()
wm.phy.s.agent.triggerUpdate()

#import xdefw.interactive
#xdefw.interactive.shell_console()()
time.sleep(3.)

wm.stopAgents()
ctrl.s.stop()






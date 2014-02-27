#!/xde
#
# Copyright (C) 2013-7 CODYCO Project
# Author: Pauline Maurice
# email:  maurice@isir.upmc.fr
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
# this file is a base for scripting the replay of markers trajectories in XDE

####################################
#                                  #
# Import all modules: configure... #
#                                  #
####################################

import xde_world_manager as xwm
import xde_robot_loader as xrl
import xde_resources as xr


import lgsm
import numpy as np
from math import pi
import time 
import sys


POSITION_MODE = 0
VELOCITY_MODE = 1
POSVEL_MODE = 2

MARKER_FILE = "resources/markers.txt"
POS_FILE = "resources/acquisition_2_filtered.txt"

##############################
## Import initial position  ##
##############################                  
markers = {'pelvis_right_back' : 16, 'pelvis_left_back' : 19}

with open(POS_FILE, 'r') as fin:
	markers_pos = np.genfromtxt(fin, delimiter = '\t')

rightIndex = markers['pelvis_right_back']
leftIndex = markers['pelvis_left_back']

initPos = [(0.001*markers_pos[0,4*rightIndex+2]  + 0.001*markers_pos[0,4*leftIndex+2]) / 2,
			(0.001*markers_pos[0,4*rightIndex+3]  + 0.001*markers_pos[0,4*leftIndex+3]) / 2,
			(0.001*markers_pos[0,4*rightIndex+4]  + 0.001*markers_pos[0,4*leftIndex+4]) / 2 - 0.08] # pelvis markers were approx. 7cm above root origin
initRot = np.arctan2(-(0.001*markers_pos[0,4*leftIndex+2] - 0.001*markers_pos[0,4*rightIndex+2]), 0.001*markers_pos[0,4*leftIndex+3] - 0.001*markers_pos[0,4*rightIndex+3]) 

H_initPos = lgsm.Displacement(initPos[0], initPos[1], initPos[2], np.cos(initRot/2), 0, 0, np.sin(initRot/2))


####################
## Create agents  ##
####################
dt = .01
wm = xwm.WorldManager()
wm.createAllAgents(dt, lmd_max=.01, uc_relaxation_factor=0.01)


###################
## Create ground ##
###################
groundWorld = xrl.createWorldFromUrdfFile(xr.ground, "ground", [0,0,0,1,0,0,0], True, 0.001, 0.001)
wm.addWorld(groundWorld)


####################
## Create manikin ##
####################
# configure manikin morphology
NAME = 'Pauline'
people = {'Pauline':[1.56, 53.]}
size, mass = people[NAME]

# Create manikin
import desc.simple.scene, human, scene
vhWorld = desc.simple.scene.createWorld()
vhName = "manikin"
human.addManikin(vhWorld, vhName, mass, size, H_initPos, .1)

#marker_coord, marker_body = scene.buildCodaCouplings(vhWorld, vhName, MARKER_FILE) # Configure couplings (must be done before adding manikin to world)

wm.addWorld(vhWorld)
vh = wm.phy.s.GVM.Robot(vhName)
wm.ms.assembleSystem()
vh.enableGravity(False)
N  = vh.getJointSpaceDim()

dynModel = xrl.getDynamicModelFromWorld(vhWorld) 

# Set initial state
vh.setJointPositions(lgsm.zeros(N))
dynModel.setJointPositions(lgsm.zeros(N))
vh.setJointVelocities(lgsm.zeros(N))
dynModel.setJointVelocities(lgsm.zeros(N))

#vh.lockSegmentJoints2("Thoracic Spine 4")
#vh.lockSegmentJoints2("Head Center of Gravity")
#vh.lockSegmentJoints2("Right Clavicular")
#vh.lockSegmentJoints2("Left Clavicular")
#
#wm.phy.s.Connectors.OConnectorRobotState.new("ocpos", vhName+"_", vhName)
#wm.phy.s.Connectors.IConnectorPDCouplingList.new("icpdcoupl", "Coda_refs_"+vhName, POSITION_MODE) # can be used as "POSVEL_MODE" also, in this case you need to give the reference position and velocity of each PDCoupling


#######################
## Create DataPlayer ##
#######################
import player
player = player.Player("player", marker_coord, dynModel)
player.loadData(POS_FILE)


################################
## Connect Player and Physics ##
################################

# Synchro physic/data
wm.phy.addCreateInputPort("player_trigger", "double")
wm.icsync.addEvent("player_trigger") # so that physics waits for "player" to update the markers position
player.getPort("output_trigger").connectTo(wm.phy.getPort("player_trigger"))
player.getPort("output_bodytarget").connectTo(wm.phy.getPort("Coda_refs_"+vhName+"_H")) 

wm.phy.getPort(vhName+"_q").connectTo(player.getPort("q"))
wm.phy.getPort(vhName+"_qdot").connectTo(player.getPort("qDot"))
wm.phy.getPort(vhName+"_Hroot").connectTo(player.getPort("Hroot"))
wm.phy.getPort(vhName+"_Troot").connectTo(player.getPort("Troot"))


######################
## Start simulation ##
######################
player.s.start()
wm.startAgents()

wm.phy.s.agent.triggerUpdate()

scene.attachManikin(vh, wm.phy, marker_body, True)
player.startPlaying()

time.sleep(500.) # you are not supposed to need this, but there seems to be a pb on my computer and without the "sleep" the simulation does not run (remains locked at the first iteration)

#wm.stopAgents()
#player.s.stop()



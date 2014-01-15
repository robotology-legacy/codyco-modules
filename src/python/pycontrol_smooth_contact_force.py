
import time
import math
import numpy as np
from numpy import *
import lgsm
import rtt_interface as rtt
import xdefw.rtt

import xde_world_manager as xwm
import xde_robot_loader  as xrl
import xde_resources     as xr
import xde_isir_controller as xic

class VHController(xdefw.rtt.Task):
  
  def __init__(self, name, time_step, model,rname,phy,icsync, solver, use_reduced_problem_no_ddq,jmap):
    super(VHController, self).__init__(rtt.PyTaskFactory.CreateTask(rname))
    self.s.setPeriod(time_step)
    self.t = 0
    self.dt = time_step
	
    self.ctrl = xic.ISIRController(model, rname, phy,icsync, solver, use_reduced_problem_no_ddq)           
    self.model = model

    ###################### PARAMETERS ##############################
    pi = np.pi
    rad2deg = 180.0/pi;
    # height of the waist
    waist_z = 0.58
    # height of contact on the table
    self.table_contact_z = 0.52
    self.table_contact_x = -0.25
    self.table_contact_y_R = 0.1
    self.table_contact_y_L = -0.1


    self.n = self.model.nbInternalDofs()
    head_init = self.model.getSegmentPosition(self.model.getSegmentIndex(rname+".head"))
    com_init = self.model.getCoMPosition()
    com_x = com_init[0,0]
    com_y = com_init[1,0]
    com_z = com_init[2,0]
    qinit = lgsm.zeros(self.n)
    # correspond to:    l_elbow_pitch     r_elbow_pitch     l_knee             r_knee             l_ankle_pitch      r_ankle_pitch        l_shoulder_roll          r_shoulder_roll
    for name, val in [("l_elbow_pitch", np.pi/2.), ("r_elbow_pitch", np.pi/2.), ("l_knee", -0.05), ("r_knee", -0.05), ("l_ankle_pitch", -0.05), ("r_ankle_pitch", -0.05), ("l_shoulder_roll", np.pi/2.), ("r_shoulder_roll", np.pi/2.)]:
      qinit[jmap[rname+"."+name]] = val
    #### WEIGHTS OF THE TASKS

    Weight_head_stabilization_task = 0.5
    Weight_waist_task = 0.4
    Weight_COM_task = 10.0
    Weight_hand_task = 1.0
    Weight_hand_task_or = 4

    # weight of the force task = 1.0 by default because we don't know how to weight it wrt the other tasks
    Weight_force_task = 1.0

    #### FORCE TASK
    # the desired force is expressed in the world frame
    force_desired = lgsm.vector(0,0,0,0,0,-5) # mux, muy, muz, fx, fy,fz


    ## full task: bring joints to the initial position
    #default: Kd= 2*sqrt(Kp)
    self.fullTask = self.ctrl.createFullTask("full", "INTERNAL", w=0.0001, kp=0., kd=9., q_des=qinit)

    ## waist task: better balance
    self.waistTask   = self.ctrl.createFrameTask("waist", rname+'.waist', lgsm.Displacement(), "RXYZ", w=Weight_waist_task, kp=36.,     pose_des=lgsm.Displacement(0,0,.58,1,0,0,0))

    ## back task: to keep the back stable
    back_dofs   = [jmap[rname+"."+n] for n in ['torso_pitch', 'torso_roll', 'torso_yaw']]
    self.backTask    = self.ctrl.createPartialTask("back", back_dofs, w=0.001, kp=16., q_des=lgsm.zeros(3))

    ## head stabilization task: 
    self.headTask   = self.ctrl.createFrameTask("head_task", rname+'.head', lgsm.Displacement(), "RZ", w=Weight_head_stabilization_task, kp=10., pose_des=head_init )
    neck_dofs   = [jmap[rname+"."+n] for n in ['head_roll', 'head_yaw', 'head_pitch']]
    neck_des = lgsm.zeros(3)
    #neck_des[2] = -pi/10
    self.headNeckBlock = self.ctrl.createPartialTask("neck", neck_dofs, w=3., kp=16., q_des=neck_des)

    ## COM task
    self.CoMTask     = self.ctrl.createCoMTask("com", "XY", w=Weight_COM_task, kp=20., pose_des=lgsm.Displacement(t=com_init)) #, kd=0.
    # controller that generates the ZMP reference trajectory for stabilizing the COM
    # COM_reference_position = [[-0.0,0.0]]
    # self.ctrl.add_updater( xic.task_controller.ZMPController( self.CoMTask, self.ctrl.getModel(), COM_reference_position, RonQ=1e-6, horizon=1.8, dt=self.dt, H_0_planeXY=lgsm.Displacement(), stride=3, gravity=9.81) )

    ## contact tasks for the feet
    sqrt2on2 = lgsm.np.sqrt(2.)/2.
    RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
    RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)

    self.ctrl.createContactTask("CLF0", rname+".l_foot", lgsm.Displacement([-.039,-.027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
    self.ctrl.createContactTask("CLF1", rname+".l_foot", lgsm.Displacement([-.039, .027,-.031]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
    self.ctrl.createContactTask("CLF2", rname+".l_foot", lgsm.Displacement([-.039, .027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin
    self.ctrl.createContactTask("CLF3", rname+".l_foot", lgsm.Displacement([-.039,-.027, .099]+ RotLZdown.tolist()), 1.5, 0.) # mu, margin

    self.ctrl.createContactTask("CRF0", rname+".r_foot", lgsm.Displacement([-.039,-.027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
    self.ctrl.createContactTask("CRF1", rname+".r_foot", lgsm.Displacement([-.039, .027, .031]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
    self.ctrl.createContactTask("CRF2", rname+".r_foot", lgsm.Displacement([-.039, .027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin
    self.ctrl.createContactTask("CRF3", rname+".r_foot", lgsm.Displacement([-.039,-.027,-.099]+ RotRZdown.tolist()), 1.5, 0.) # mu, margin

    ## frame tasks for putting the hand in contact with the wall
    # we have to split rotation and xyz posing - because controlling both at same time generates 
    # rototranslation vector (xyz+o4)
    #this is the frame controlled - attached to the hand
    H_hand_tool = lgsm.Displacement(0, 0.0, 0, 1, 0, 0, 0)
    # rotation matrix hand in world frame
    R_right_hand = np.matrix([[1, 0, 0],[0, -1, 0],[0,0,-1]])
    self.o_right_hand = lgsm.Quaternion.fromMatrix(R_right_hand)
    # mano parallela tavolo, palmo verso basso 
    R_left_hand = np.matrix([[-1, 0, 0],[0, -1, 0],[0,0,1]])
    self.o_left_hand = lgsm.Quaternion.fromMatrix(R_left_hand)

    Task_hand_controlled_var = "RXYZ" # "RXYZ" "R" "XY" "Z" ..

    # a frame task to prepare the hand to touch
    self.prepareTouch_R_task = self.ctrl.createFrameTask("prepare_R_touchWall", rname+'.r_hand', H_hand_tool, "XYZ", w=Weight_hand_task, kp=36., pose_des=lgsm.Displacement([self.table_contact_x,self.table_contact_y_R,self.table_contact_z+0.05]+self.o_right_hand.tolist()))

    self.prepareTouch_L_task = self.ctrl.createFrameTask("prepare_L_touchWall", rname+'.l_hand', H_hand_tool, "XYZ", w=Weight_hand_task, kp=36., pose_des=lgsm.Displacement([self.table_contact_x,self.table_contact_y_L,self.table_contact_z+0.05]+self.o_left_hand.tolist()))

    # it is a frame task to displace the hand
    self.touchWall_R_task = self.ctrl.createFrameTask("R_touchWall", rname+'.r_hand', H_hand_tool, "XYZ", w=Weight_hand_task, kp=36., pose_des=lgsm.Displacement([self.table_contact_x,self.table_contact_y_R,self.table_contact_z]+self.o_right_hand.tolist()))

    self.touchWall_L_task = self.ctrl.createFrameTask("L_touchWall", rname+'.l_hand', H_hand_tool, "XYZ", w=Weight_hand_task, kp=36., pose_des=lgsm.Displacement([self.table_contact_x,self.table_contact_y_L,self.table_contact_z]+self.o_left_hand.tolist()))

    # it is a frame task to rotate the hand
    self.touchWall_orient_R_task = self.ctrl.createFrameTask("R_touchWall_orient", rname+'.r_hand', H_hand_tool, "R", w=Weight_hand_task_or, kp=36., pose_des=lgsm.Displacement([self.table_contact_x,self.table_contact_y_R,self.table_contact_z]+self.o_right_hand.tolist()))
    self.touchWall_orient_L_task = self.ctrl.createFrameTask("L_touchWall_orient", rname+'.l_hand', H_hand_tool, "R",w=Weight_hand_task_or, kp=36., pose_des=lgsm.Displacement([self.table_contact_x,self.table_contact_y_L,self.table_contact_z]+self.o_left_hand.tolist()))

    ## force task for touching the table
    self.force_R_EETask = self.ctrl.createForceTask("R_EE", rname+".r_hand", H_hand_tool, w=Weight_force_task)
    self.force_L_EETask = self.ctrl.createForceTask("L_EE", rname+".l_hand", H_hand_tool, w=Weight_force_task)
    # update the task goal
    self.force_R_EETask.setPosition(lgsm.Displacement()) #--> new "expressed in world frame"
    self.force_L_EETask.setPosition(lgsm.Displacement()) #--> new "expressed in world frame"
    self.force_R_EETask.setWrench(force_desired)
    self.force_L_EETask.setWrench(force_desired)

  def startHook(self):
    # deactivate tasks

    self.headNeckBlock.deactivate()
    #self.headTask.deactivate()
    self.force_R_EETask.deactivate()
    self.force_L_EETask.deactivate()
    self.touchWall_orient_R_task.deactivate()
    self.touchWall_orient_L_task.deactivate()
    self.touchWall_R_task.deactivate()
    self.touchWall_L_task.deactivate()
    self.prepareTouch_R_task.deactivate()
    self.prepareTouch_L_task.deactivate()
    self.ctrl.s.start()
    
  def stopHook(self):
    pass
    # self.ctrl.s.stop()
    
  def updateHook(self):
    time = [2., 6., 8., 9., 11., 16., 18., 20.]
    # print "prepare L"
    if self.t<time[0] and not self.prepareTouch_L_task.isActiveAsObjective():
      self.prepareTouch_L_task.activateAsObjective()
      self.prepareTouch_R_task.activateAsObjective()

    elif self.t>=time[0] and self.t<time[1] and not self.touchWall_orient_L_task.isActiveAsObjective():
      # print "activate orientation L" 
      self.touchWall_orient_L_task.activateAsObjective()
      self.touchWall_orient_R_task.activateAsObjective()
      
    elif self.t>=time[1] and self.t<time[2] and not self.touchWall_L_task.isActiveAsObjective():
      # print "go to contact L"
      self.touchWall_L_task.activateAsObjective()
      #touchWall_orient_L_task.activateAsConstraint()
      self.prepareTouch_L_task.deactivate()
      self.touchWall_R_task.activateAsObjective()
      #touchWall_orient_R_task.activateAsConstraint()
      self.prepareTouch_R_task.deactivate()

    elif self.t>=time[2] and self.t<time[3] and not self.force_R_EETask.isActiveAsObjective():
      # print "activate forces" 
      self.force_R_EETask.activateAsObjective()
      self.force_L_EETask.activateAsObjective()
      
    elif self.t>=time[5] and self.t<time[6] and self.touchWall_L_task.isActiveAsObjective():
      #print "deactivate touch" 
      self.touchWall_L_task.deactivate()
      self.touchWall_R_task.deactivate()
      self.prepareTouch_L_task.activateAsObjective()
      self.prepareTouch_R_task.activateAsObjective()
      
    elif self.t>=time[6] and self.t<time[7] and self.prepareTouch_L_task.isActiveAsObjective():
      #print "activate touch 2 "      
      self.touchWall_L_task.setPosition(lgsm.Displacement([self.table_contact_x,self.table_contact_y_L-0.1,self.table_contact_z]+self.o_right_hand.tolist()))
      self.touchWall_R_task.setPosition(lgsm.Displacement([self.table_contact_x,self.table_contact_y_R+0.1,self.table_contact_z]+self.o_right_hand.tolist()))
      
      self.prepareTouch_L_task.deactivate()
      self.prepareTouch_R_task.deactivate()  
      self.touchWall_L_task.activateAsObjective()
      self.touchWall_R_task.activateAsObjective()
    
      
    if self.t>time[2] and self.t<time[3]:
      force_desired = lgsm.vector(0,0,0,0,0,-5.*(self.t-time[2])/(time[3]-time[2]))
      self.force_R_EETask.setWrench(force_desired)
      self.force_L_EETask.setWrench(force_desired)          
    elif self.t>=time[3] and self.t<time[4]: 
      force_desired = lgsm.vector(0,0,0,0,0,-5.*(1.-(self.t-time[3])/(time[4]-time[3])))
      self.force_R_EETask.setWrench(force_desired)
      self.force_L_EETask.setWrench(force_desired)
    elif self.t>=time[4] and self.t<time[5]:
      force_desired = lgsm.vector(0,0,0,0,0,0)
      self.force_R_EETask.setWrench(force_desired)
      self.force_L_EETask.setWrench(force_desired)
    
      
     
    self.t += self.dt
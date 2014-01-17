
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

import controlfsm


class VHController(xdefw.rtt.Task):
  
  def __init__(self, name, time_step, model, rname, phy, icsync, solver, use_reduced_problem_no_ddq, jmap, observer):
    super(VHController, self).__init__(rtt.PyTaskFactory.CreateTask(rname))
    self.s.setPeriod(time_step)
    self.t = 0
    self.dt = time_step
    self.rname = rname
	
    self.ctrl = xic.ISIRController(model, rname, phy,icsync, solver, use_reduced_problem_no_ddq)           
    self.model = model

    ###################### PARAMETERS ##############################
    pi = np.pi
    rad2deg = 180.0/pi;
    # height of the waist
    waist_z = 0.58

    self.n = self.model.nbInternalDofs()
    head_init = self.model.getSegmentPosition(self.model.getSegmentIndex(rname+".head")).copy()
    com_init = self.model.getCoMPosition().copy()
    self.com_init = com_init
    rhand_init = self.model.getSegmentPosition(self.model.getSegmentIndex(rname+".r_hand")).copy()
    lhand_init = self.model.getSegmentPosition(self.model.getSegmentIndex(rname+".l_hand")).copy()
    qinit = lgsm.zeros(self.n)
    # correspond to:    l_elbow_pitch     r_elbow_pitch     l_knee             r_knee             l_ankle_pitch      r_ankle_pitch        l_shoulder_roll          r_shoulder_roll
    for name, val in [("l_elbow_pitch", np.pi/2.), ("r_elbow_pitch", np.pi/2.), ("l_knee", -0.05), ("r_knee", -0.05), ("l_ankle_pitch", -0.05), ("r_ankle_pitch", -0.05), ("l_shoulder_roll", np.pi/2.), ("r_shoulder_roll", np.pi/2.)]:
      qinit[jmap[rname+"."+name]] = val
    #### WEIGHTS OF THE TASKS
    Weight_full_task = 0.001
    Weight_back_task = 10.
    Weight_waist_task = 1.
    Weight_head_task = 10.    
    Weight_COM_task = 1000.
    Weight_hand_task = 100.
    Weight_hand_task_or = 10.
    Weight_force_task = 100.
    
    # Weight_full_task = 0.0001
    # Weight_back_task = 0.001
    # Weight_waist_task = 0.4
    # Weight_head_task = 0.5    
    # Weight_COM_task = 10.
    # Weight_hand_task = 5.
    # Weight_hand_task_or = 4.
    # Weight_force_task = 1.
    
    # the desired force is expressed in the world frame
    force_desired = lgsm.vector(0,0,0,0,0,0) # mux, muy, muz, fx, fy,fz

    #### TASKS
    ## full task: bring joints to the initial position
    #default: Kd= 2*sqrt(Kp)
    self.fullTask = self.ctrl.createFullTask("full", "INTERNAL", w=Weight_full_task, kp=0., KD = 9., q_des=qinit)

    ## waist task: better balance
    self.waistTask   = self.ctrl.createFrameTask("waist", rname+'.waist', lgsm.Displacement(), "R", w=Weight_waist_task, kp=10., pose_des=lgsm.Displacement(0,0,.58,1,0,0,0))

    ## back task: to keep the back stable
    back_dofs   = [jmap[rname+"."+n] for n in ['torso_pitch', 'torso_roll', 'torso_yaw']]
    self.backTask    = self.ctrl.createPartialTask("back", back_dofs, w=Weight_back_task, kp=16., q_des=lgsm.zeros(3))

    ## head stabilization task: 
    self.headTask   = self.ctrl.createFrameTask("head_task", rname+'.head', lgsm.Displacement(), "RZ", w=Weight_head_task, kp=10., pose_des=head_init )

    ## COM task
    self.CoMTask     = self.ctrl.createCoMTask("com", "XY", w=Weight_COM_task, kp=50., pose_des=lgsm.Displacement(t=com_init)) 
 
    ## contact tasks for the feet
    sqrt2on2 = lgsm.np.sqrt(2.)/2.
    RotLZdown = lgsm.Quaternion(-sqrt2on2,0.0,-sqrt2on2,0.0) * lgsm.Quaternion(0.0,1.0,0.0,0.0)
    RotRZdown = lgsm.Quaternion(0.0, sqrt2on2,0.0, sqrt2on2) * lgsm.Quaternion(0.0,1.0,0.0,0.0)

    self.ctrl.createContactTask("CLF0", rname+".l_foot", lgsm.Displacement([-.039,-.027,-.031]+ RotLZdown.tolist()), 1., 0.) # mu, margin
    self.ctrl.createContactTask("CLF1", rname+".l_foot", lgsm.Displacement([-.039, .027,-.031]+ RotLZdown.tolist()), 1., 0.) # mu, margin
    self.ctrl.createContactTask("CLF2", rname+".l_foot", lgsm.Displacement([-.039, .027, .099]+ RotLZdown.tolist()), 1., 0.) # mu, margin
    self.ctrl.createContactTask("CLF3", rname+".l_foot", lgsm.Displacement([-.039,-.027, .099]+ RotLZdown.tolist()), 1., 0.) # mu, margin

    self.ctrl.createContactTask("CRF0", rname+".r_foot", lgsm.Displacement([-.039,-.027, .031]+ RotRZdown.tolist()), 1., 0.) # mu, margin
    self.ctrl.createContactTask("CRF1", rname+".r_foot", lgsm.Displacement([-.039, .027, .031]+ RotRZdown.tolist()), 1., 0.) # mu, margin
    self.ctrl.createContactTask("CRF2", rname+".r_foot", lgsm.Displacement([-.039, .027,-.099]+ RotRZdown.tolist()), 1., 0.) # mu, margin
    self.ctrl.createContactTask("CRF3", rname+".r_foot", lgsm.Displacement([-.039,-.027,-.099]+ RotRZdown.tolist()), 1., 0.) # mu, margin

    ## frame tasks for putting the hand in contact with the table
    #this is the controlled frame - attached to the hand
    H_hand_tool = lgsm.Displacement(0., 0., 0, 1, 0, 0, 0)

    #Task_hand_controlled_var = "RXYZ"  "RXYZ" "R" "XY" "Z" ..
    self.RHand_task = self.ctrl.createFrameTask("rhand", rname+'.r_hand', H_hand_tool, "XYZ", w=Weight_hand_task, kp=36., pose_des=rhand_init)
    self.LHand_task = self.ctrl.createFrameTask("lhand", rname+'.l_hand', H_hand_tool, "XYZ", w=Weight_hand_task, kp=36., pose_des=lhand_init)
    self.orient_RHand_task = self.ctrl.createFrameTask("orient_rhand", rname+'.r_hand', H_hand_tool, "R", w=Weight_hand_task_or, kp=36., pose_des=rhand_init)
    self.orient_LHand_task = self.ctrl.createFrameTask("orient_lhand", rname+'.l_hand', H_hand_tool, "R",w=Weight_hand_task_or, kp=36., pose_des=lhand_init)

    ## force task for touching the table
    self.force_RHand_Task = self.ctrl.createForceTask("force_rhand", rname+".r_hand", H_hand_tool, w=Weight_force_task, kp=16)
    self.force_LHand_Task = self.ctrl.createForceTask("force_lhand", rname+".l_hand", H_hand_tool, w=Weight_force_task, kp=16)
    # update the task goal
    self.force_RHand_Task.setPosition(lgsm.Displacement()) # expressed in world frame
    self.force_LHand_Task.setPosition(lgsm.Displacement()) # expressed in world frame
    self.force_RHand_Task.setWrench(force_desired)
    self.force_LHand_Task.setWrench(force_desired)
    
    self.fsm = controlfsm.build(self, observer)
    self.fi = 0.

  def startHook(self):
    # deactivate tasks
    # self.backTask.deactivate()
    self.force_RHand_Task.deactivate()
    self.force_LHand_Task.deactivate()
    self.LHand_task.deactivate()
    self.RHand_task.deactivate()
    self.orient_RHand_task.deactivate()
    self.orient_LHand_task.deactivate()

    self.ctrl.s.start()
    
  def stopHook(self):
    pass
    # self.ctrl.s.stop()
    
  def updateHook(self):
    self.fsm.update(self.dt) 
    self.t += self.dt

import math
import numpy as np
import lgsm
import lgsm.geom
import fsm.basic as bfsm

table_contact_z = 0.53
table_contact_x = -0.25
table_contact_y_R = 0.1
table_contact_y_L = -0.1
fmax = 10.
delta_com_forward = -0.005
t_duration = 3.
# rotation matrix hand in world frame
R_right_hand = np.matrix([[1, 0, 0],[0, -1, 0],[0,0,-1]])
o_right_hand = lgsm.Quaternion.fromMatrix(R_right_hand)
R_left_hand = np.matrix([[-1, 0, 0],[0, -1, 0],[0,0,1]])
o_left_hand = lgsm.Quaternion.fromMatrix(R_left_hand)
    
def compute_CoM_pos_forward(fd, fi, f, delta): # delta < 0 to lean forward
  # pos = lgsm.Displacement()
  # pos.setTranslation(np.matrix([[delta*np.sin(np.pi*(f-fi)/(2.*(fd-fi)))],[0.],[0.]]))
  pos = np.matrix([[delta*np.sin(np.pi*(f-fi)/(2.*(fd-fi)))],[0.],[0.]])
  return pos
  
def compute_CoM_pos_backward(fd, fi, f, delta): 
  pos = np.matrix([[delta*np.cos(np.pi*(f-fi)/(2.*(fd-fi)))],[0.],[0.]])
  return pos
  
class ControlState(bfsm.FSMState):
  
  def __init__(self, fsm, name, transitions, controller, observer):
    super(ControlState, self).__init__(fsm, name, transitions)
    self.controller = controller
    self.observer = observer
    
class InitState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller,observer):
    super(InitState, self).__init__(fsm, name, transitions, controller,observer)
    
  def doEnter(self, from_state):
    print "Init"
    

class PrepareState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller,observer):
    super(PrepareState, self).__init__(fsm, name, transitions, controller,observer)
    
    self.fi = 0.
  def doEnter(self, from_state):
    print "Prepare"
    self.controller.LHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_L,table_contact_z+0.05]+o_left_hand.tolist()))
    self.controller.RHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_R,table_contact_z+0.05]+o_right_hand.tolist()))

    if not self.controller.LHand_task.isActiveAsObjective():
      self.controller.LHand_task.activateAsObjective()
    if not self.controller.RHand_task.isActiveAsObjective():
      self.controller.RHand_task.activateAsObjective()
    if not self.controller.orient_LHand_task.isActiveAsObjective(): 
      self.controller.orient_LHand_task.activateAsObjective()
    if not self.controller.orient_RHand_task.isActiveAsObjective(): 
      self.controller.orient_RHand_task.activateAsObjective()
    self.controller.fi = self.observer.getPort("tau").read()[0][0,0]
    print self.controller.fi
    
  def doUpdate(self, dt):
    return
  
  def outCondition(self):
    error_L = np.linalg.norm(self.controller.LHand_task.getError())
    error_R = np.linalg.norm(self.controller.RHand_task.getError())
    return np.maximum(error_L,error_R)<.008



class LHandReachingState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller,observer):
    super(LHandReachingState, self).__init__(fsm, name, transitions, controller,observer)
    
    
  def doEnter(self, from_state):
    print "LHandReaching"
    self.controller.LHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_L,table_contact_z]+o_left_hand.tolist()))
    if not self.controller.LHand_task.isActiveAsObjective():
      self.controller.LHand_task.activateAsObjective()
      
  def doUpdate(self, dt):
    return
    
  def outCondition(self):
    error = np.linalg.norm(self.controller.LHand_task.getError())
    return error<.008
    
class RHandReachingState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller,observer):
    super(RHandReachingState, self).__init__(fsm, name, transitions, controller,observer)
    
    
  def doEnter(self, from_state):
    print "RHandReaching"
    self.controller.RHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_R,table_contact_z]+o_right_hand.tolist()))
    if not self.controller.RHand_task.isActiveAsObjective():
      self.controller.RHand_task.activateAsObjective()
      
  def doUpdate(self, dt):
    return
    
  def outCondition(self):
    error = np.linalg.norm(self.controller.RHand_task.getError())
    return error<.008
    
    
class TwoHandsReachingState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller,observer):
    super(TwoHandsReachingState, self).__init__(fsm, name, transitions, controller,observer)
    
    
  def doEnter(self, from_state):
    print "TwoHandsReaching"
    self.controller.LHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_L,table_contact_z]+o_left_hand.tolist()))
    if not self.controller.LHand_task.isActiveAsObjective():
      self.controller.LHand_task.activateAsObjective()
    
    self.controller.RHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_R,table_contact_z]+o_right_hand.tolist()))
    if not self.controller.RHand_task.isActiveAsObjective():
      self.controller.RHand_task.activateAsObjective()      
      
  def outCondition(self):
    error_L = np.linalg.norm(self.controller.LHand_task.getError())
    error_R = np.linalg.norm(self.controller.RHand_task.getError())
    return np.maximum(error_L,error_R)<.008
    
class LHandForceIncreaseState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller,observer):
    super(LHandForceIncreaseState, self).__init__(fsm, name, transitions, controller,observer)
    
    self.t_ini = 0
    self.t_duration = t_duration
    self.fi = 0.
    self.observer = observer
    
    
  def doEnter(self, from_state):
    print "LHandForceIncrease"
    self.t_ini = self.controller.t
    if not self.controller.force_LHand_Task.isActiveAsObjective():
      self.controller.force_LHand_Task.activateAsObjective()
    self.fi = self.observer.getPort("tau").read()[0][0,0]
    
  def doUpdate(self, dt):
      if self.controller.t-self.t_ini <= self.t_duration:
        force_desired = lgsm.vector(0,0,0,0,0,-fmax*(self.controller.t-self.t_ini)/self.t_duration)
        self.controller.force_LHand_Task.setWrench(force_desired)    
        p_com = compute_CoM_pos_forward(fd=-fmax+self.controller.fi, fi=self.fi, f=self.observer.getPort("tau").read()[0][0,0], delta = delta_com_forward)
        p_com = self.controller.com_init+p_com
        self.controller.CoMTask.setPosition(lgsm.Displacement(t=p_com))

class RHandForceIncreaseState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller, observer):
    super(RHandForceIncreaseState, self).__init__(fsm, name, transitions, controller, observer)
    
    self.t_ini = 0
    self.t_duration = t_duration
    self.fi = 0.
    self.observer = observer
    
  def doEnter(self, from_state):
    print "RHandForceIncrease"
    self.t_ini = self.controller.t
    if not self.controller.force_RHand_Task.isActiveAsObjective():
      self.controller.force_RHand_Task.activateAsObjective()
    self.fi = self.observer.getPort("tau").read()[0][0,0]
    
  def doUpdate(self, dt):
      if self.controller.t-self.t_ini <= self.t_duration:
        force_desired = lgsm.vector(0,0,0,0,0,-fmax*(self.controller.t-self.t_ini)/self.t_duration)
        self.controller.force_RHand_Task.setWrench(force_desired)   
        p_com = compute_CoM_pos_forward(fd=-fmax+self.controller.fi, fi=self.fi, f=self.observer.getPort("tau").read()[0][0,0], delta = delta_com_forward)
        p_com = self.controller.com_init+p_com
        self.controller.CoMTask.setPosition(lgsm.Displacement(t=p_com))

class TwoHandsForceIncreaseState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller, observer):
    super(TwoHandsForceIncreaseState, self).__init__(fsm, name, transitions, controller, observer)
    
    self.t_ini = 0
    self.t_duration = t_duration
    self.fi = 0.
    self.observer = observer
    
  def doEnter(self, from_state):
    print "TwoHandsForceIncrease"
    self.t_ini = self.controller.t
    if not self.controller.force_RHand_Task.isActiveAsObjective():
      self.controller.force_RHand_Task.activateAsObjective()
    if not self.controller.force_LHand_Task.isActiveAsObjective():
      self.controller.force_LHand_Task.activateAsObjective()     
    self.fi = self.observer.getPort("tau").read()[0][0,0]
    
  def doUpdate(self, dt):
      if self.controller.t-self.t_ini <= self.t_duration:
        force_desired = lgsm.vector(0,0,0,0,0,-fmax*(self.controller.t-self.t_ini)/self.t_duration)
        self.controller.force_RHand_Task.setWrench(force_desired)   
        self.controller.force_LHand_Task.setWrench(force_desired)   
        p_com = compute_CoM_pos_forward(fd=-fmax+self.controller.fi, fi=self.fi, f=self.observer.getPort("tau").read()[0][0,0], delta = delta_com_forward)
        p_com = self.controller.com_init+p_com
        self.controller.CoMTask.setPosition(lgsm.Displacement(t=p_com))
        
class LHandReleaseState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller, observer):
    super(LHandReleaseState, self).__init__(fsm, name, transitions, controller, observer)
    
    self.t_ini = 0
    self.t_duration = t_duration
    self.fi = 0.
    self.observer = observer
    
  def doEnter(self, from_state):
    print "LHandRelease"
    self.t_ini = self.controller.t
    self.fi = self.observer.getPort("tau").read()[0][0,0]
    
  def doUpdate(self, dt):
      if self.controller.t-self.t_ini <= self.t_duration:
        force_desired = lgsm.vector(0,0,0,0,0,-fmax*(1.-(self.controller.t-self.t_ini)/self.t_duration))
        self.controller.force_LHand_Task.setWrench(force_desired)
        p_com = compute_CoM_pos_backward(fd=self.controller.fi, fi=self.fi, f=self.observer.getPort("tau").read()[0][0,0], delta = 0.*delta_com_forward)
        p_com = self.controller.com_init+p_com
        self.controller.CoMTask.setPosition(lgsm.Displacement(t=p_com))
      else:
        self.controller.LHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_L,table_contact_z+0.05]+o_left_hand.tolist()))

  def outCondition(self):
    if self.controller.t-self.t_ini > self.t_duration:
      error = np.linalg.norm(self.controller.LHand_task.getError())
      return error<.008    
    
class RHandReleaseState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller, observer):
    super(RHandReleaseState, self).__init__(fsm, name, transitions, controller, observer)
    
    self.t_ini = 0
    self.t_duration = t_duration
    self.fi = 0.
    self.observer = observer
    
  def doEnter(self, from_state):
    print "RHandRelease"
    self.t_ini = self.controller.t
    self.fi = self.observer.getPort("tau").read()[0][0,0]
    
  def doUpdate(self, dt):
      if self.controller.t-self.t_ini <= self.t_duration:
        force_desired = lgsm.vector(0,0,0,0,0,-fmax*(1.-(self.controller.t-self.t_ini)/self.t_duration))
        self.controller.force_RHand_Task.setWrench(force_desired)
        p_com = compute_CoM_pos_backward(fd=self.controller.fi, fi=self.fi, f=self.observer.getPort("tau").read()[0][0,0], delta = 0.*delta_com_forward)
        p_com = self.controller.com_init+p_com
        self.controller.CoMTask.setPosition(lgsm.Displacement(t=p_com))
      else:
        self.controller.RHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_R,table_contact_z+0.05]+o_right_hand.tolist()))

  def outCondition(self):
    if self.controller.t-self.t_ini > self.t_duration:
      error = np.linalg.norm(self.controller.RHand_task.getError())
      return error<.008 
      
class TwoHandsReleaseState(ControlState):
  
  def __init__(self, fsm, name, transitions, controller, observer):
    super(TwoHandsReleaseState, self).__init__(fsm, name, transitions, controller, observer)
    
    self.t_ini = 0
    self.t_duration = t_duration
    self.fi = 0.
    self.observer = observer
    
  def doEnter(self, from_state):
    print "TwoHandsRelease"
    self.t_ini = self.controller.t
    self.fi = self.observer.getPort("tau").read()[0][0,0]
    
  def doUpdate(self, dt):
      if self.controller.t-self.t_ini <= self.t_duration:
        force_desired = lgsm.vector(0,0,0,0,0,-fmax*(1.-(self.controller.t-self.t_ini)/self.t_duration))
        self.controller.force_RHand_Task.setWrench(force_desired) 
        self.controller.force_LHand_Task.setWrench(force_desired)
        p_com = compute_CoM_pos_backward(fd=self.controller.fi, fi=self.fi, f=self.observer.getPort("tau").read()[0][0,0], delta = 0.*delta_com_forward)
        p_com = self.controller.com_init+p_com
        self.controller.CoMTask.setPosition(lgsm.Displacement(t=p_com))
      else:
        self.controller.RHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_R,table_contact_z+0.05]+o_right_hand.tolist()))
        self.controller.LHand_task.setPosition(lgsm.Displacement([table_contact_x,table_contact_y_L,table_contact_z+0.05]+o_left_hand.tolist()))

  def outCondition(self):
    if self.controller.t-self.t_ini > self.t_duration:
      error_L = np.linalg.norm(self.controller.LHand_task.getError())
      error_R = np.linalg.norm(self.controller.RHand_task.getError())
      error = np.maximum(error_L,error_R)
      return error<.008    
      
def build(controller, observer):
    fsm = bfsm.FSM("state-init")
    
    InitState(fsm, "state-init", [], controller,observer)
    prepare = PrepareState(fsm, "state-prepare", [], controller, observer)
    rhreaching = RHandReachingState(fsm, "state-RHandReaching", [], controller, observer)
    lhreaching = LHandReachingState(fsm, "state-LHandReaching", [], controller, observer)
    threaching = TwoHandsReachingState(fsm, "state-TwoHandsReaching", [], controller, observer)
    lhforce = LHandForceIncreaseState(fsm, "state-LHandForceIncrease", [], controller, observer)
    rhforce = RHandForceIncreaseState(fsm, "state-RHandForceIncrease", [], controller, observer)
    thforce = TwoHandsForceIncreaseState(fsm, "state-TwoHandsForceIncrease", [], controller, observer)
    lhrelease = LHandReleaseState(fsm, "state-LHandRelease", [], controller, observer)
    rhrelease = RHandReleaseState(fsm, "state-RHandRelease", [], controller, observer)
    threlease = TwoHandsReleaseState(fsm, "state-TwoHandsRelease", [], controller, observer)
    
    fsm.addTransition("state-init", "state-prepare", 1.)    
    fsm.addTransition("state-prepare", "state-RHandReaching", prepare.outCondition)    
    fsm.addTransition("state-RHandReaching", "state-RHandForceIncrease", rhreaching.outCondition)    
    fsm.addTransition("state-RHandForceIncrease", "state-RHandRelease", 4.)    
    fsm.addTransition("state-RHandRelease", "state-LHandReaching", rhrelease.outCondition)
    fsm.addTransition("state-LHandReaching", "state-LHandForceIncrease", lhreaching.outCondition)   
    fsm.addTransition("state-LHandForceIncrease", "state-LHandRelease", 4.)   
    fsm.addTransition("state-LHandRelease", "state-TwoHandsReaching", lhrelease.outCondition)
    fsm.addTransition("state-TwoHandsReaching", "state-TwoHandsForceIncrease", threaching.outCondition)
    fsm.addTransition("state-TwoHandsForceIncrease", "state-TwoHandsRelease", 4.)
    fsm.addTransition("state-TwoHandsRelease", "state-prepare", threlease.outCondition)
    
    return fsm

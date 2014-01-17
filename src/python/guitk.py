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

#controller

#GUI
from Tkinter import *

class MyTk(Frame):
   def __init__(self, parent):
        Frame.__init__(self, parent) 
        self.parent = parent        
        self.parent.title("Parameters")
        self.pack(fill = BOTH, expand = 1)

        menubar = Menu(self.parent)
        self.parent.config(menu = menubar)
        
        self.label_t = Label(self, border = 5, text="TASK")
        self.label_w = Label(self, border = 5, text="WEIGHT")
        self.label_k = Label(self, border = 5, text="KP")
        self.label_t.grid(row = 1, column = 1)
        self.label_w.grid(row = 1, column = 2)
        self.label_k.grid(row = 1, column = 5)

        self.label = [Label(self, border=5, text=l) for l in 
        ["full", "waist","back / torso","head stabiliz","CoM","right_hand (XYZ)",
        "left_hand  (XYZ)","right_hand (R)","left_hand  (R)","right_hand (force)",
        "left_hand  (force)"]]
              
        for irow in range(0,10):
            self.label[irow].grid(row = irow+2, column = 1)
              
        self.weight=[]
        for irow in range(0,10):
            self.weight.append(Scale(self, from_=0, to=20, length=300, tickinterval=0.1, orient=HORIZONTAL))
            self.weight[irow].grid(row = irow+2, column = 2)  
            
        self.weightEntry=[]
        for irow in range(0,10):
            self.weightEntry.append(Spinbox(self,from_=0, to=20, increment=0.1,width=8))
            self.weightEntry[irow].grid(row = irow+2, column = 3)  
            
        self.weightSetButton = Button(self,text="Set these weights", command=self.onClickWeightSetButton)
        self.weightSetButton.grid(row=13,column=3)
        
        self.kp=[]
        for irow in range(0,10):
            self.kp.append(Scale(self, from_=0, to=20, length=400, tickinterval=0.1, orient=HORIZONTAL))
            self.kp[irow].grid(row = irow+2, column = 5) 
                     
        self.kpEntry=[]
        for irow in range(0,10):
            self.kpEntry.append(Spinbox(self,from_=0, to=20, increment=0.1,width=8))
            self.kpEntry[irow].grid(row = irow+2, column = 6) 
            
        self.kpSetButton = Button(self,text="Set these kp", command=self.onClickKpSetButton)
        self.kpSetButton.grid(row=13,column=6)
        
        self.resetValues()
        
        # File Menu
        fileMenu = Menu(menubar)
        menubar.add_cascade(label = "Menu", menu = fileMenu)
        # Menu Item for general menu
        fileMenu.add_command(label = "Reset values", command = self.onResetValues) 
        fileMenu.add_command(label = "Save values", command = self.onSaveValues)
        fileMenu.add_command(label = "Load values", command = self.onLoadValues)
        # File Menu
        simulationMenu = Menu(menubar)
        menubar.add_cascade(label = "Simulation", menu = simulationMenu)
        # Menu Item for general menu
        simulationMenu.add_command(label = "Set values in controller", command = self.onSetValues)   
               
   def onResetValues(self):
       self.resetValues()
       
   def onSaveValues(self):
       print "GUI: Saving values on file"
       
   def onLoadValues(self):
       print "GUI: Loading values from file"
       
   def onClickWeightSetButton(self):
       print "GUI: setting weight values in the sliders"
       for irow in range(0,10):
            self.weight[irow].set(self.weightEntry[irow].get())
                  
   def onClickKpSetButton(self):
       print "GUI: setting kp values in the sliders"
       for irow in range(0,10):
            self.kp[irow].set(self.kpEntry[irow].get())
                
   def resetValues(self):
       print "GUI: Resetting values"
       for irow in range(0,10):
            self.weight[irow].set(1) 
            self.kp[irow].set(1)
                       
   def onSetValues(self):
        print "GUI: current values for weights"
        for irow in range(0,10):
            print (irow, self.weight[irow].get() )            
        # uncomment this only when using xde + isir controller 
        #it=0
#        self.controller.fullTask.setWeight(self.weight[it].get()); it+=1
#        self.controller.waistTask.setWeight(self.weight[it].get()); it+=1
#        self.controller.backTask.setWeight(self.weight[it].get()); it+=1
#        self.controller.headTask.setWeight(self.weight[it].get()); it+=1
#        self.controller.CoMTask.setWeight(self.weight[it].get()); it+=1
#        self.controller.RHand_task.setWeight(self.weight[it].get()); it+=1
#        self.controller.LHand_task.setWeight(self.weight[it].get()); it+=1
#        self.controller.orient_RHand_task.setWeight(self.weight[it].get()); it+=1
#        self.controller.orient_LHand_task.setWeight(self.weight[it].get()); it+=1
#        self.controller.force_RHand_Task.setWeight(self.weight[it].get()); it+=1
#        self.controller.force_LHand_Task.setWeight(self.weight[it].get()); it+=1
        print "GUI: new weights are set in the controller"
        
        print "GUI: current values for KPs"
        for irow in range(0,10):
            print (irow, self.kp[irow].get() )        
        # uncomment this only when using xde + isir controller 
#        it=0
#        self.controller.fullTask.setStiffness(self.kp[it].get()); it+=1
#        self.controller.waistTask.setStiffness(self.kp[it].get()); it+=1
#        self.controller.backTask.setStiffness(self.kp[it].get()); it+=1
#        self.controller.headTask.setStiffness(self.kp[it].get()); it+=1
#        self.controller.CoMTask.setStiffness(self.kp[it].get()); it+=1
#        self.controller.RHand_task.setStiffness(self.kp[it].get()); it+=1
#        self.controller.LHand_task.setStiffness(self.kp[it].get()); it+=1
#        self.controller.orient_RHand_task.setStiffness(self.kp[it].get()); it+=1
#        self.controller.orient_LHand_task.setStiffness(self.kp[it].get()); it+=1
#        self.controller.force_RHand_Task.setStiffness(self.kp[it].get()); it+=1
#        self.controller.force_LHand_Task.setStiffness(self.kp[it].get()); it+=1
        print "GUI: new KPs are set in the controller"
 
   def setController(self,controller):
        self.controller=controller    
     
##### DEBUG ONLY #####  

#comment everything below once you want to use it with the controller  
        
def main():
   #activate GUI
   master = Tk()
   gui = MyTk(master)
   #gui.setController(controller)
   master.mainloop()

# test standalone
if __name__ == '__main__':
    main()
    

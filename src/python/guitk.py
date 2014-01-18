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
# not right now - only when using it with XDE

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
        self.label_w_cur = Label(self, border = 5, text="current")
        self.label_k = Label(self, border = 5, text="KP")
        self.label_k_cur = Label(self, border = 5, text="current")
        self.label_t.grid(row = 1, column = 1)
        self.label_w.grid(row = 1, column = 2)
        self.label_w_cur.grid(row = 1, column = 4)
        self.label_k.grid(row = 1, column = 5)
        self.label_k_cur.grid(row = 1, column = 7)

        self.label = [Label(self, border=5, text=l) for l in 
        ["full", "waist","back / torso","head stabiliz","CoM","right_hand (XYZ)",
        "left_hand  (XYZ)","right_hand (R)","left_hand  (R)","right_hand (force)",
        "left_hand  (force)"]]
        
        self.Ntask = len(self.label)
              
        for irow in range(0,self.Ntask):
            self.label[irow].grid(row = irow+2, column = 1)
              
        self.weight=[]
        for irow in range(0,self.Ntask):
            self.weight.append(Scale(self, from_=0, to=50, length=200, resolution=0.001, orient=HORIZONTAL, command=self.onUpdatedWeightScale))
            self.weight[irow].grid(row = irow+2, column = 2)  
            
        self.weightEntry=[]
        for irow in range(0,self.Ntask):
            self.weightEntry.append(Spinbox(self,from_=0, to=50, increment=0.001,width=8,justify=RIGHT))
            self.weightEntry[irow].grid(row = irow+2, column = 3)  
            
        self.weightSetButton = Button(self,text="Preview weights", command=self.onClickWeightSetButton)
        self.weightSetButton.grid(row=13,column=3)
        
        self.weightSetButton = Button(self,text="Send these weights to XDE", command=self.onClickWeightSendButton)
        self.weightSetButton.grid(row=13,column=2)
        
        self.kp=[]
        for irow in range(0,self.Ntask):
            self.kp.append(Scale(self, from_=0, to=50, length=200, resolution=0.001, orient=HORIZONTAL, command=self.onUpdatedKpScale))
            self.kp[irow].grid(row = irow+2, column = 5) 
                     
        self.kpEntry=[]
        for irow in range(0,self.Ntask):
            self.kpEntry.append(Spinbox(self,from_=0, to=50, increment=0.001,width=8,justify=RIGHT))
            self.kpEntry[irow].grid(row = irow+2, column = 6) 
            
        self.kpSetButton = Button(self,text="Preview kp", command=self.onClickKpSetButton)
        self.kpSetButton.grid(row=13,column=6)
        
        self.kpSendButton = Button(self,text="Send these Kp to XDE", command=self.onClickKpSendButton)
        self.kpSendButton.grid(row=13,column=5)
        
        # some labels to update values
        self.cur_w=[]
        self.cur_w_text=[] 
        for irow in range(0,self.Ntask):
             self.cur_w_text.append(StringVar())
             self.cur_w.append(Label(self, border = 1, textvariable=self.cur_w_text[irow]))
             self.cur_w[irow].grid(row = irow+2, column = 4) 
        self.cur_k=[]
        self.cur_k_text=[] 
        for irow in range(0,self.Ntask):
             self.cur_k_text.append(StringVar())
             self.cur_k.append(Label(self, border = 1, textvariable=self.cur_k_text[irow]))
             self.cur_k[irow].grid(row = irow+2, column = 7) 
        
        self.resetValues()
        
        self.getFromSimuButton = Button(self,text="Get values from simulation", command=self.onClickGetFromSimuButton)
        self.getFromSimuButton.grid(row=4,column=8)
        self.sendFromSimuButton = Button(self,text="Send values to simulation", command=self.onClickSendToSimuButton)
        self.sendFromSimuButton.grid(row=5,column=8)       
        
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
        simulationMenu.add_command(label = "Send values to controller", command = self.onSendValues)   
        simulationMenu.add_command(label = "Get values from controller", command = self.onGetValues) 
               
   def onResetValues(self):
       self.resetValues()
       
   def onSaveValues(self):
       print "GUI: Saving values on file"
       
   def onLoadValues(self):
       print "GUI: Loading values from file"
       
   def onClickWeightSetButton(self):
       print "GUI: setting weight values in the sliders"
       for irow in range(0,self.Ntask):
            self.weight[irow].set(self.weightEntry[irow].get())
                  
   def onClickKpSetButton(self):
       print "GUI: setting kp values in the sliders"
       for irow in range(0,self.Ntask):
            self.kp[irow].set(self.kpEntry[irow].get())
   
   #when the sliders of KP are moved, values are updated
   def onUpdatedKpScale(self, event):
       #print "updates kp"
       for irow in range(0,self.Ntask):
           self.setEntrySpin(self.kpEntry,irow,self.kp[irow].get())

    #when the sliders of the Weights are moved, values are updated
   def onUpdatedWeightScale(self, event):
       #print "updates weights"
       for irow in range(0,self.Ntask):
           self.setEntrySpin(self.weightEntry,irow,self.weight[irow].get()) 
           
   def onClickWeightSendButton(self):
       #send weights to XDE / simulation / controller
       print "GUI: current values for weights"
       for irow in range(0,self.Ntask):
           print (irow, self.weightEntry[irow].get() )            
        # uncomment this only when using xde + isir controller 
        #it=0
#        self.controller.fullTask.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.waistTask.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.backTask.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.headTask.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.CoMTask.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.RHand_task.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.LHand_task.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.orient_RHand_task.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.orient_LHand_task.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.force_RHand_Task.setWeight(self.weightEntry[it].get()); it+=1
#        self.controller.force_LHand_Task.setWeight(self.weightEntry[it].get()); it+=1
       print "GUI: new weights are set in the controller"
       for irow in range(0,self.Ntask):
           self.cur_w_text[irow].set(self.weightEntry[irow].get())
           
   def onClickWeightFromButton(self):
       #get weights from XDE / simulation / controller
       print "GUI: getting values for weights"          
        # uncomment this only when using xde + isir controller 
        #it=0
#        self.weightEntry[it].set(self.controller.fullTask.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.waistTask.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.backTask.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.headTask.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.CoMTask.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.RHand_task.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.LHand_task.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.orient_RHand_task.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.orient_LHand_task.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.force_RHand_Task.getWeight()); it+=1
#        self.weightEntry[it].set(self.controller.force_LHand_Task.getWeight()); it+=1
       for irow in range(0,self.Ntask):
           self.cur_w_text[irow].set(self.weightEntry[irow].get())
           

   def onClickKpSendButton(self):
       #send kp to XDE / simulation / controller
       print "GUI: current values for kp"
       for irow in range(0,self.Ntask):
            print (irow, self.kpEntry[irow].get() )        
        # uncomment this only when using xde + isir controller 
#        it=0
#        self.controller.fullTask.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.waistTask.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.backTask.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.headTask.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.CoMTask.setStiffness(self.kpkpEntryit].get()); it+=1
#        self.controller.RHand_task.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.LHand_task.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.orient_RHand_task.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.orient_LHand_task.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.force_RHand_Task.setStiffness(self.kpEntry[it].get()); it+=1
#        self.controller.force_LHand_Task.setStiffness(self.kpEntry[it].get()); it+=1
       print "GUI: new KPs are set in the controller"   
       for irow in range(0,self.Ntask):
           self.cur_k_text[irow].set(self.kpEntry[irow].get())
           
   def onClickKpFromButton(self):
       #get kp from XDE / simulation / controller
       print "GUI: getting values for kp"       
        # uncomment this only when using xde + isir controller 
#        it=0
#        self.kpEntry[it].set(self.controller.fullTask.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.waistTask.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.backTask.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.headTask.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.CoMTask.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.RHand_task.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.LHand_task.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.orient_RHand_task.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.orient_LHand_task.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.force_RHand_Task.getStiffness()); it+=1
#        self.kpEntry[it].set(self.controller.force_LHand_Task.getStiffness()); it+=1   
       for irow in range(0,self.Ntask):
           self.cur_k_text[irow].set(self.kpEntry[irow].get())           
           
               
   def resetValues(self):
       print "GUI: Resetting values"           
       #full
       self.setEntrySpin(self.weightEntry,0,0.5)
       self.setEntrySpin(self.kpEntry,0,0.)      
       #waist
       self.setEntrySpin(self.weightEntry,1,0.4)
       self.setEntrySpin(self.kpEntry,1,36.)       
       #back / torso
       self.setEntrySpin(self.weightEntry,2,0.001)
       self.setEntrySpin(self.kpEntry,2,16.)     
       #head stabiliz
       self.setEntrySpin(self.weightEntry,3,0.5)
       self.setEntrySpin(self.kpEntry,3,10.)      
       #CoM
       self.setEntrySpin(self.weightEntry,4,10.0)
       self.setEntrySpin(self.kpEntry,4,20.)       
       #right_hand (XYZ)
       self.setEntrySpin(self.weightEntry,5,5.0)
       self.setEntrySpin(self.kpEntry,5,36.)     
       #left_hand  (XYZ)
       self.setEntrySpin(self.weightEntry,6,5.0)
       self.setEntrySpin(self.kpEntry,6,36.)     
       #right_hand (R)
       self.setEntrySpin(self.weightEntry,7,4.0)
       self.setEntrySpin(self.kpEntry,7,16.)      
       #left_hand  (R)
       self.setEntrySpin(self.weightEntry,8,4.0)
       self.setEntrySpin(self.kpEntry,8,16.)       
       #right_hand (force)
       self.setEntrySpin(self.weightEntry,9,1.0)
       self.setEntrySpin(self.kpEntry,9,0.01)        
       #left_hand  (force)
       self.setEntrySpin(self.weightEntry,10,1.0)
       self.setEntrySpin(self.kpEntry,10,0.01)
       #update values on sliders as well
       self.onClickWeightSetButton()
       self.onClickKpSetButton()
     
   def setEntrySpin(self,spin,index,value):
       spin[index].delete(0,7)
       spin[index].insert(0,value)      
                             
   def onSendValues(self):      
       self.onClickWeightSendButton()
       self.onClickKpSendButton()
       
   def onGetValues(self):
       self.onClickWeightFromButton()
       self.onClickKpFromButton()
        
   def onClickSendToSimuButton(self):
       self.onSendValues()
       
   def onClickGetFromSimuButton(self):
       self.onGetValues()
 
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
    

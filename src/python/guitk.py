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

        self.label1 = Label(self, border = 25, text="w1")
        self.label2 = Label(self, border = 25, text="w2")
        self.label1.grid(row = 1, column = 1)
        self.label2.grid(row = 2, column = 1)
        self.w1 = Scale(self, from_=0, to=20, length=600, tickinterval=1, orient=HORIZONTAL)
        self.w1.set(1)
        self.w1.grid(row = 1, column = 2)
        self.w2 = Scale(self, from_=0, to=20, length=600, tickinterval=1, orient=HORIZONTAL)
        self.w2.set(10)
        self.w2.grid(row = 2, column = 2)
        
        self.resetValues()
        
        # File Menu
        fileMenu = Menu(menubar)
        menubar.add_cascade(label = "Menu", menu = fileMenu)

        # Menu Item for Open Image
        fileMenu.add_command(label = "Set values", command = self.onSetValue) 
        fileMenu.add_command(label = "Reset values", command = self.onResetValue) 
        
   def onResetValue(self):
       self.resetValues()
       
   def resetValues(self):
       print "resetting values"
       self.w1.set(1)
       self.w2.set(10)
        
   def setController(self,controller):
        self.controller=controller
               
   def onSetValue(self):
        print (self.w1.get(), self.w2.get())
        # uncomment this only when using xde + isir controller 
        #self.controller.CoMTask.setWeight(self.w1.get())
        print "new weights are set"
     
     
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
    

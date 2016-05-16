#!/xde
#
# Copyright (C) 2013-7 CODYCO Project
# Author: Serena Ivaldi, Joseph Salini
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

class TimeObserver:
    def __init__(self,dt):
        self.dt = dt
        self.time = 0
    
    def update(self,tick):
        self.time += self.dt
        
    def getElapsedTime(self):
        return self.time
        
    def reset(self):
        self.time=0
    
    
    

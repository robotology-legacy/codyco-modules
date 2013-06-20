#!/usr/bin/python
"""Script for facilitating the use of dataDumper with multiple ports

In case of dumping n ports simultaniously, this scripts creates n istances of 
dataDumper, and connects them to the port do dump. The it can be closed 
withc Ctrl+C and it shutdown cleanly all instances of dataDumper.
"""




import sys
import yarp
import subprocess
import signal
import os

procs = [];
port_names = [];
dumper_port_names = [];

yarp.Network.init()


def signal_handler(signal_num, frame):
    """Signal handler, implemented only for closing application with Ctrl+C"""
    print 'Ctrl+C, detected, closing application!'
    for i in range(0,len(port_names)):
        yarp.Network.disconnect(port_names[i],dumper_port_names[i])
        
    for i in range(0,len(procs)):
        procs[i].send_signal(signal.SIGINT)
             
    for i in range(0,len(procs)):
        procs[i].wait()
        
    sys.exit(0)


def main():
    argc = len(sys.argv)
   
    signal.signal(signal.SIGINT, signal_handler)

    
    if( len(sys.argv) == 1 ):
       print "Script used to data various port using dataDumper"
       print "The ports to dump are passed as command line arguments"
    else:
        
        for i in range(1,len(sys.argv)):
            port_name = sys.argv[i]
            if( yarp.Network.exists(port_name) ):
                dumper_port_name = "/dumper" + port_name;
                procs.append(subprocess.Popen(["dataDumper","--name",dumper_port_name]));
                while( not yarp.Network.exists(dumper_port_name) ):
                    #busy waiting
                    pass
                yarp.Network.connect(port_name,dumper_port_name)
                
                port_names.append(port_name);
                dumper_port_names.append(dumper_port_name);

            else:
                print port_name + "not existing "
            
        for i in range(0,len(procs)):
            procs[i].wait()
            

if __name__ == '__main__':
   main()

#!/usr/bin/python

import argparse

def printPortTags(ports, port_type, host, additional_flags):
    for port in ports:
        # print datadumper launch 
        print("\t<module>");
        print("\t\t<name>yarpdatadumper</name>");
        print("\t\t<parameters>--name /dumper"+port+" --type "+port_type+" "+additional_flags+"</parameters>");
        print("\t\t<node>"+host+"</node>");
        print("\t\t<tag>data-dumper"+port.replace('/','-').replace(':','-')+"</tag>"); 
        print("\t</module>");
 
        # print connection between datadumper and port
        print("\t<connection>");
        print("\t\t<from>"+port+"</from>");
        print("\t\t<to>/dumper"+port+"</to>");
        print("\t\t<protocol>udp</protocol>");
        print("\t</connection>");

def main():
    parser = argparse.ArgumentParser(description='Tool for generating a YarpManager XML application for dumping a list of YARP ports using the yarpdatadumper.')
    parser.add_argument('--ports', nargs='+', dest="ports", action='store', required=True, help='list of ports (serializable to bottles) to dump')
    parser.add_argument('--imagePorts', nargs='+', dest="imagePorts", action='store', help='list of ports (of to dump')
    parser.add_argument('--host', nargs=1,  dest="host", action='store', required=True, help='host where to launch the dataDumpers')
    parser.add_argument('--name', nargs=1,  dest="name", action='store', required=True, help='name of the application')
    parser.add_argument('--rxTime', action='store_true',help='pass --rxTime flag to the yarpdatadumpers')
    parser.add_argument('--txTime', action='store_true',help='pass --txTime flag to the yarpdatadumpers')
    parser.add_argument('--addVideo', action='store_true',help='pass --addVideo flag to the yarpdatadumpers')
    args = parser.parse_args()

    print("<application>");
    print("\t<name>"+args.name[0]+"</name>");
    print("\t<dependencies>");
    for port in args.ports:
        print("\t\t<port>"+port+"</port>");
    print("\t</dependencies>");

    additional_flags = "";
    if(args.rxTime):
        additional_flags += " --rxTime";
    if(args.txTime):
        additional_flags += " --txTime";
    if(args.addVideo):
        additional_flags += " --addVideo";

    if( args.ports is not None):
        printPortTags(args.ports,"bottle",args.host[0],additional_flags);
    if( args.imagePorts is not None):
        printPortTags(args.imagePorts,"image",args.host[0],additional_flags);
    
    print("</application>");
  
if __name__ == "__main__":
    main()
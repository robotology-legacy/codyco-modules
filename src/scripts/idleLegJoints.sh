#!/bin/bash

list="4 5"

for a in $list; do
	echo "set icmd cmod $a idl"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a idl"  | yarp rpc /icub/left_leg/rpc:i
done

sleep 0.5

for a in $list; do
	echo "set icmd cmod $a pos"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a pos"  | yarp rpc /icub/left_leg/rpc:i
done

list="1"

for a in $list; do
	echo "set icmd cmod $a idl"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a idl"  | yarp rpc /icub/left_leg/rpc:i
done

sleep 0.5

for a in $list; do
	echo "set icmd cmod $a pos"  | yarp rpc /icub/right_leg/rpc:i
	echo "set icmd cmod $a pos"  | yarp rpc /icub/left_leg/rpc:i
done

echo "!!Now I will calib the wholeBodyDynamics!!"
sleep 0.5
echo "calibStanding all 600"  | yarp rpc /wholeBodyDynamicsTree/rpc:i
sleep 0.5
yarp clean --timeout 0.2

clear all;
robotName = 'icubGazeboSim';
localName = 'torqueControlTests';
 kp = diag([30*ones(1,3),7*ones(1,22)]);
 Ts = 0.01;
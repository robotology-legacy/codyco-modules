**Trajectories genereted the 14th of January 2016 (2nd year Koroibot Review Meeting)**


- icub\_walk\_seq_* references for walkPlayer
- walkingParams.txt : parameters of the generated trajectories
- com\_l\_sole: com w.r.t. l_sole (at initial position) (x,y,z)
- comTraj_.. com w.r.t. frame positioned at the midpoint between the two feet at initial time (time, x, y) (z is fixed.. see walkingParams.txt). Check orientation frame. It can be as root\_frame instead of world\_frame
- torqueBalancing\_comTraj : com trajectory to be read by walkPlayer
- rest: ignore.


**To run:**

- Put in home position.
- walkPlayer --robot icub --period 10 --refSpeedMinJerk 0 --filename icub_walk_seq --execute --torqueBalancingSequence torqueBalancing
- Dump all the ports: remember the `com:o` port. Dump with envelope if using `yarp read` or datadumper to dump the timestamp

***Notes***

- Call the reset command only at the end of the trajectories otherwise the stream of the com may be wrong
- to obtain the final com, simply detect the reset event and past the com accordingly.




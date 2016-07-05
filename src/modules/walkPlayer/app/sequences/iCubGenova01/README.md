**Trajectories genereted the 14th of January 2016 (2nd year Koroibot Review Meeting)**


- icub_walk_seq_* references for walkPlayer
- walkingParams.txt : parameters of the generated trajectories
- com_l_sole: com w.r.t. l_sole (at initial position) (x,y,z)
- comTraj_.. com w.r.t. frame positioned at the midpoint between the two feet at initial time (time, x, y) (z is fixed.. see walkingParams.txt)
- torqueBalancing_comTraj : com trajectory to be read by walkPlayer
- rest: ignore.


**To run:**
- Put in home position.


walkPlayer --robot icub --period 10 --refSpeedMinJerk 0 --filename icub_walk_seq --execute --torqueBalancingSequence torqueBalancing



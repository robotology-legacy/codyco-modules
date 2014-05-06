#wholeBodyDynamics.thrift

/**
* wholeBodyDynamics_IDLServer
*
* Interface.
*/

service wholeBodyDynamics_IDLServer
{
  /**
  * Calibrate the force/torque sensors
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calib(1:string calib_code, 2:i32 nr_of_samples=100)

  /**
  * Calibrate the force/torque sensors when on double support
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the sole).
  * For this calibration the strong assumption of simmetry of the robot and its pose is done.
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calibStanding(1:string calib_code, 2:i32 nr_of_samples=100)

  /**
  * Reset the sensor offset to 0 0 0 0 0 0 (six zeros).
  * @param calib_code argument to specify the sensors to reset (all,arms,legs,feet)
  * @return true/false on success/failure
  */
  bool resetOffset(1:string calib_code)

  /**
  * Quit the module.
  * @return true/false on success/failure
  */
  bool quit();
}

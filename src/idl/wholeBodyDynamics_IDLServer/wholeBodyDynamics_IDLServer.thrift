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
  * Calibrate the force/torque sensors when on single support on left foot
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the left sole).
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calibStandingLeftFoot(1:string calib_code, 2:i32 nr_of_samples=100)

  /**
  * Calibrate the force/torque sensors when on single support on right foot
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the right sole).
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet)
  * @param nr_of_samples number of samples
  * @return true/false on success/failure
  */
  bool calibStandingRightFoot(1:string calib_code, 2:i32 nr_of_samples=100)

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

  // This should be implemented in a separated "simpleLeggedOdometry" interface,
  // but for the time being it is easier to just implement it here
  //service simpleLeggedOdometry_IDLServer
  // {

  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a link that is assumed to be fixed in the odometry.
   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_link the name of the link that should be initially fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  bool resetSimpleLeggedOdometry(1:string initial_world_frame, 2:string initial_fixed_link)

  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_link the name of the new link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  bool changeFixedLinkSimpleLeggedOdometry(1:string new_fixed_link)
  
  /**
   * Set the cutoff frequency (in Hz) for IMU measurements
   * @return true/false on success/failure 
   */
  bool set_imuFilterCutoffInHz(1:double newCutoff);
  
  /**
   * Get the cutoff frequency (in Hz) for IMU measurements
   * @return the cutoff frequency (in Hz)
   */
  double get_imuFilterCutoffInHz();
  
  /**
   * Set the cutoff frequency (in Hz) for FT measurements
   * @return true/false on success/failure 
   */
  bool set_forceTorqueFilterCutoffInHz(1:double newCutoff);
  
  /**
   * Get the cutoff frequency (in Hz) for FT measurements
   * @return the cutoff frequency (in Hz)
   */
  double get_forceTorqueFilterCutoffInHz();

  // } /** simpleLeggedOdometry_IDLServer */
}





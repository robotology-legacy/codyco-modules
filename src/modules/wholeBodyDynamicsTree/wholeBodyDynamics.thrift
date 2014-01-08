#wholeBodyDynamics.thrift

/**
* wholeBodyDynamics_IDLServer
*
* Interface. 
*/

service wholeBodyDynamics_IDLServer
{
  /**
  * Gets the list of available commands
  * @return Bottle containing all available commands
  */
  string help();
  
  /**
  * Calibrate the force/torque sensors 
  * (WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist)
  * @param calib_code argument to specify the sensors to calibrate (all,arms,legs,feets) 
  * @return information about the calibration
  */
  string calib(1:string calib_code)
  
  /**
  * Quit the module.
  * @return true/false on success/failure
  */
  bool quit();  
}

#staticInertiaIdentification.thrift

/**
* staticInertiaIdentification_IDLServer
*
* Interface.
*/

service staticInertiaIdentification_IDLServer
{
  /**
   * Start the estimation.
   * @return true/false on success/failure
   */
  bool start();

  /**
   * stop the estimation.
   * @return true/false on success/failure
   */
  bool stop();

  /**
   * Save the estimated inertial parameters in a URDF file.
   * @param fileName the name of the save file.
   * @param robotName the name of the robot model in the URDF file.
   * @return true/false on success/failure
   */
  bool saveURDF(1:string fileName, 2:string robotName="test_icub");

  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  bool quit();
}

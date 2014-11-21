#insituFTSensorCalibration.thrift

/**
* insituFTSensorCalibration_IDLServer
*
* Interface.
*/

service insituFTSensorCalibration_IDLServer
{
  /**
  * Start the new dataset acquisition.
  * At the end of each dataset acquisition,
  * the module stops until the user changes
  * the added mass on the robot. This command
  * is used to start the dataset acquisition
  * after the added mass is correctly mounted
  * in the robot.
  *
  * @return true/false on success/failure
  */
  bool startNewDatasetAcquisition()


  /**
  * Quit the module.
  * @return true/false on success/failure
  */
  bool quit();
}

#floatingBaseEstimator_IDLServer.thrift

/**
* floatingBaseEstimator_IDLServer
*
* Interface.
*/

service floatingBaseEstimator_IDLServer
{
  /**
   * Reset the odometry world to be (initially) a frame specified in the robot model,
   * and specify a frame that is assumed to be fixed in the odometry.

   * @param initial_world_frame the frame of the robot model that is assume to be initially
   *        coincident with the world/inertial frame.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame/link names are wrong)
   */
  bool resetSimpleLeggedOdometry(1:string initial_world_frame, 2:string initial_fixed_frame);

  /**
   * Change the link that is considered fixed by the odometry.
   * @param new_fixed_frame the name of a frame attached to the link that should be considered fixed
   * @return true/false on success/failure (typically if the frame names are wrong)
   */
  bool changeFixedLinkSimpleLeggedOdometry(1:string new_fixed_frame);

  /**
   * Get the current settings in the form of a string.
   * @return the current settings as a human readable string.
   */
  string getCurrentSettingsString();
}





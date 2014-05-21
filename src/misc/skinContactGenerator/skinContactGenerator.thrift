#skinContactGenerator.thrift

/**
* skinContactGenerator_IDLServer
*
* Interface.
*/
service skinContactGenerator_IDLServer
{
  /**
   * Configure the skinContactGenerator to produce a contact
   * at the origin of a given link.
   */
  bool setContact(1:i32 bodyPart, 2:i32 link);

  /**
   * Configure the skinContactGenerator to produce a contact
   * at a prescribed position of a given link.
   */
  bool setContactForce(1:i32 bodyPart,
                           2:i32 link,
                         3:double f_x,
                         4:double f_y,
                         5:double f_z,
                         6:double p_x,
                         7:double p_y,
                         8:double p_z,
                         9:i32 skinPart=0);

  /**
   * Configure the skinContactGenerator to produce a contact
   * at the origin of a given link.
   */
  bool setContactName(1:string bodyPart, 2:string link);

  /**
   * Quit the module.
   * @return true/false on success/failure
   */
  bool quit();
}

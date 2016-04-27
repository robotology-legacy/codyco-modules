// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_STRUCT_wholeBodyDynamicsSettings
#define YARP_THRIFT_GENERATOR_STRUCT_wholeBodyDynamicsSettings

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <Gravity.h>
#include <KinematicSourceType.h>

class wholeBodyDynamicsSettings;


class wholeBodyDynamicsSettings : public yarp::os::idl::WirePortable {
public:
  // Fields
  KinematicSourceType kinematicSource;
  /**
   * Specify the source of the kinematic information for one link, see KinematicSourceType information for more info.
   */
  std::string fixedFrameName;
  /**
   * If kinematicSource is FIXED_LINK, specify the frame of the robot that we know to be fixed (i.e. not moving with respect to an inertial frame)
   */
  Gravity gravity;
  /**
   * If kinematicSource is FIXED_LINK, specify the gravity vector in the fixedFrame
   */
  double imuFilterCutoff;
  /**
   * Cutoff frequency of the first order filter of the IMU
   */
  double forceTorqueFilterCutoff;
  /**
   * Cutoff frequency of the first order filter of the F/T sensors
   */
  bool useJointVelocity;
  /**
   * Use the joint velocity measurement if this is true, assume they are zero otherwise.
   */
  bool useJointAcceleration;

  // Default constructor
  wholeBodyDynamicsSettings() : kinematicSource((KinematicSourceType)0), fixedFrameName(""), imuFilterCutoff(0), forceTorqueFilterCutoff(0), useJointVelocity(0), useJointAcceleration(0) {
  }

  // Constructor with field values
  wholeBodyDynamicsSettings(const KinematicSourceType kinematicSource,const std::string& fixedFrameName,const Gravity& gravity,const double imuFilterCutoff,const double forceTorqueFilterCutoff,const bool useJointVelocity,const bool useJointAcceleration) : kinematicSource(kinematicSource), fixedFrameName(fixedFrameName), gravity(gravity), imuFilterCutoff(imuFilterCutoff), forceTorqueFilterCutoff(forceTorqueFilterCutoff), useJointVelocity(useJointVelocity), useJointAcceleration(useJointAcceleration) {
  }

  // Copy constructor
  wholeBodyDynamicsSettings(const wholeBodyDynamicsSettings& __alt) : WirePortable(__alt)  {
    this->kinematicSource = __alt.kinematicSource;
    this->fixedFrameName = __alt.fixedFrameName;
    this->gravity = __alt.gravity;
    this->imuFilterCutoff = __alt.imuFilterCutoff;
    this->forceTorqueFilterCutoff = __alt.forceTorqueFilterCutoff;
    this->useJointVelocity = __alt.useJointVelocity;
    this->useJointAcceleration = __alt.useJointAcceleration;
  }

  // Assignment operator
  const wholeBodyDynamicsSettings& operator = (const wholeBodyDynamicsSettings& __alt) {
    this->kinematicSource = __alt.kinematicSource;
    this->fixedFrameName = __alt.fixedFrameName;
    this->gravity = __alt.gravity;
    this->imuFilterCutoff = __alt.imuFilterCutoff;
    this->forceTorqueFilterCutoff = __alt.forceTorqueFilterCutoff;
    this->useJointVelocity = __alt.useJointVelocity;
    this->useJointAcceleration = __alt.useJointAcceleration;
    return *this;
  }

  // read and write structure on a connection
  bool read(yarp::os::idl::WireReader& reader);
  bool read(yarp::os::ConnectionReader& connection);
  bool write(yarp::os::idl::WireWriter& writer);
  bool write(yarp::os::ConnectionWriter& connection);

private:
  bool write_kinematicSource(yarp::os::idl::WireWriter& writer);
  bool nested_write_kinematicSource(yarp::os::idl::WireWriter& writer);
  bool write_fixedFrameName(yarp::os::idl::WireWriter& writer);
  bool nested_write_fixedFrameName(yarp::os::idl::WireWriter& writer);
  bool write_gravity(yarp::os::idl::WireWriter& writer);
  bool nested_write_gravity(yarp::os::idl::WireWriter& writer);
  bool write_imuFilterCutoff(yarp::os::idl::WireWriter& writer);
  bool nested_write_imuFilterCutoff(yarp::os::idl::WireWriter& writer);
  bool write_forceTorqueFilterCutoff(yarp::os::idl::WireWriter& writer);
  bool nested_write_forceTorqueFilterCutoff(yarp::os::idl::WireWriter& writer);
  bool write_useJointVelocity(yarp::os::idl::WireWriter& writer);
  bool nested_write_useJointVelocity(yarp::os::idl::WireWriter& writer);
  bool write_useJointAcceleration(yarp::os::idl::WireWriter& writer);
  bool nested_write_useJointAcceleration(yarp::os::idl::WireWriter& writer);
  bool read_kinematicSource(yarp::os::idl::WireReader& reader);
  bool nested_read_kinematicSource(yarp::os::idl::WireReader& reader);
  bool read_fixedFrameName(yarp::os::idl::WireReader& reader);
  bool nested_read_fixedFrameName(yarp::os::idl::WireReader& reader);
  bool read_gravity(yarp::os::idl::WireReader& reader);
  bool nested_read_gravity(yarp::os::idl::WireReader& reader);
  bool read_imuFilterCutoff(yarp::os::idl::WireReader& reader);
  bool nested_read_imuFilterCutoff(yarp::os::idl::WireReader& reader);
  bool read_forceTorqueFilterCutoff(yarp::os::idl::WireReader& reader);
  bool nested_read_forceTorqueFilterCutoff(yarp::os::idl::WireReader& reader);
  bool read_useJointVelocity(yarp::os::idl::WireReader& reader);
  bool nested_read_useJointVelocity(yarp::os::idl::WireReader& reader);
  bool read_useJointAcceleration(yarp::os::idl::WireReader& reader);
  bool nested_read_useJointAcceleration(yarp::os::idl::WireReader& reader);

public:

  yarp::os::ConstString toString();

  // if you want to serialize this class without nesting, use this helper
  typedef yarp::os::idl::Unwrapped<wholeBodyDynamicsSettings > unwrapped;

  class Editor : public yarp::os::Wire, public yarp::os::PortWriter {
  public:

    Editor() {
      group = 0;
      obj_owned = true;
      obj = new wholeBodyDynamicsSettings;
      dirty_flags(false);
      yarp().setOwner(*this);
    }

    Editor(wholeBodyDynamicsSettings& obj) {
      group = 0;
      obj_owned = false;
      edit(obj,false);
      yarp().setOwner(*this);
    }

    bool edit(wholeBodyDynamicsSettings& obj, bool dirty = true) {
      if (obj_owned) delete this->obj;
      this->obj = &obj;
      obj_owned = false;
      dirty_flags(dirty);
      return true;
    }

    virtual ~Editor() {
    if (obj_owned) delete obj;
    }

    bool isValid() const {
      return obj!=0/*NULL*/;
    }

    wholeBodyDynamicsSettings& state() { return *obj; }

    void begin() { group++; }

    void end() {
      group--;
      if (group==0&&is_dirty) communicate();
    }
    void set_kinematicSource(const KinematicSourceType kinematicSource) {
      will_set_kinematicSource();
      obj->kinematicSource = kinematicSource;
      mark_dirty_kinematicSource();
      communicate();
      did_set_kinematicSource();
    }
    void set_fixedFrameName(const std::string& fixedFrameName) {
      will_set_fixedFrameName();
      obj->fixedFrameName = fixedFrameName;
      mark_dirty_fixedFrameName();
      communicate();
      did_set_fixedFrameName();
    }
    void set_gravity(const Gravity& gravity) {
      will_set_gravity();
      obj->gravity = gravity;
      mark_dirty_gravity();
      communicate();
      did_set_gravity();
    }
    void set_imuFilterCutoff(const double imuFilterCutoff) {
      will_set_imuFilterCutoff();
      obj->imuFilterCutoff = imuFilterCutoff;
      mark_dirty_imuFilterCutoff();
      communicate();
      did_set_imuFilterCutoff();
    }
    void set_forceTorqueFilterCutoff(const double forceTorqueFilterCutoff) {
      will_set_forceTorqueFilterCutoff();
      obj->forceTorqueFilterCutoff = forceTorqueFilterCutoff;
      mark_dirty_forceTorqueFilterCutoff();
      communicate();
      did_set_forceTorqueFilterCutoff();
    }
    void set_useJointVelocity(const bool useJointVelocity) {
      will_set_useJointVelocity();
      obj->useJointVelocity = useJointVelocity;
      mark_dirty_useJointVelocity();
      communicate();
      did_set_useJointVelocity();
    }
    void set_useJointAcceleration(const bool useJointAcceleration) {
      will_set_useJointAcceleration();
      obj->useJointAcceleration = useJointAcceleration;
      mark_dirty_useJointAcceleration();
      communicate();
      did_set_useJointAcceleration();
    }
    const KinematicSourceType get_kinematicSource() {
      return obj->kinematicSource;
    }
    const std::string& get_fixedFrameName() {
      return obj->fixedFrameName;
    }
    const Gravity& get_gravity() {
      return obj->gravity;
    }
    double get_imuFilterCutoff() {
      return obj->imuFilterCutoff;
    }
    double get_forceTorqueFilterCutoff() {
      return obj->forceTorqueFilterCutoff;
    }
    bool get_useJointVelocity() {
      return obj->useJointVelocity;
    }
    bool get_useJointAcceleration() {
      return obj->useJointAcceleration;
    }
    virtual bool will_set_kinematicSource() { return true; }
    virtual bool will_set_fixedFrameName() { return true; }
    virtual bool will_set_gravity() { return true; }
    virtual bool will_set_imuFilterCutoff() { return true; }
    virtual bool will_set_forceTorqueFilterCutoff() { return true; }
    virtual bool will_set_useJointVelocity() { return true; }
    virtual bool will_set_useJointAcceleration() { return true; }
    virtual bool did_set_kinematicSource() { return true; }
    virtual bool did_set_fixedFrameName() { return true; }
    virtual bool did_set_gravity() { return true; }
    virtual bool did_set_imuFilterCutoff() { return true; }
    virtual bool did_set_forceTorqueFilterCutoff() { return true; }
    virtual bool did_set_useJointVelocity() { return true; }
    virtual bool did_set_useJointAcceleration() { return true; }
    void clean() {
      dirty_flags(false);
    }
    bool read(yarp::os::ConnectionReader& connection);
    bool write(yarp::os::ConnectionWriter& connection);
  private:

    wholeBodyDynamicsSettings *obj;

    bool obj_owned;
    int group;

    void communicate() {
      if (group!=0) return;
      if (yarp().canWrite()) {
        yarp().write(*this);
        clean();
      }
    }
    void mark_dirty() {
      is_dirty = true;
    }
    void mark_dirty_kinematicSource() {
      if (is_dirty_kinematicSource) return;
      dirty_count++;
      is_dirty_kinematicSource = true;
      mark_dirty();
    }
    void mark_dirty_fixedFrameName() {
      if (is_dirty_fixedFrameName) return;
      dirty_count++;
      is_dirty_fixedFrameName = true;
      mark_dirty();
    }
    void mark_dirty_gravity() {
      if (is_dirty_gravity) return;
      dirty_count++;
      is_dirty_gravity = true;
      mark_dirty();
    }
    void mark_dirty_imuFilterCutoff() {
      if (is_dirty_imuFilterCutoff) return;
      dirty_count++;
      is_dirty_imuFilterCutoff = true;
      mark_dirty();
    }
    void mark_dirty_forceTorqueFilterCutoff() {
      if (is_dirty_forceTorqueFilterCutoff) return;
      dirty_count++;
      is_dirty_forceTorqueFilterCutoff = true;
      mark_dirty();
    }
    void mark_dirty_useJointVelocity() {
      if (is_dirty_useJointVelocity) return;
      dirty_count++;
      is_dirty_useJointVelocity = true;
      mark_dirty();
    }
    void mark_dirty_useJointAcceleration() {
      if (is_dirty_useJointAcceleration) return;
      dirty_count++;
      is_dirty_useJointAcceleration = true;
      mark_dirty();
    }
    void dirty_flags(bool flag) {
      is_dirty = flag;
      is_dirty_kinematicSource = flag;
      is_dirty_fixedFrameName = flag;
      is_dirty_gravity = flag;
      is_dirty_imuFilterCutoff = flag;
      is_dirty_forceTorqueFilterCutoff = flag;
      is_dirty_useJointVelocity = flag;
      is_dirty_useJointAcceleration = flag;
      dirty_count = flag ? 7 : 0;
    }
    bool is_dirty;
    int dirty_count;
    bool is_dirty_kinematicSource;
    bool is_dirty_fixedFrameName;
    bool is_dirty_gravity;
    bool is_dirty_imuFilterCutoff;
    bool is_dirty_forceTorqueFilterCutoff;
    bool is_dirty_useJointVelocity;
    bool is_dirty_useJointAcceleration;
  };
};

#endif

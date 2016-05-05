// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <wholeBodyDynamics_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class wholeBodyDynamics_IDLServer_calib : public yarp::os::Portable {
public:
  std::string calib_code;
  int32_t nr_of_samples;
  bool _return;
  void init(const std::string& calib_code, const int32_t nr_of_samples);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_calibStanding : public yarp::os::Portable {
public:
  std::string calib_code;
  int32_t nr_of_samples;
  bool _return;
  void init(const std::string& calib_code, const int32_t nr_of_samples);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_calibStandingLeftFoot : public yarp::os::Portable {
public:
  std::string calib_code;
  int32_t nr_of_samples;
  bool _return;
  void init(const std::string& calib_code, const int32_t nr_of_samples);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_calibStandingRightFoot : public yarp::os::Portable {
public:
  std::string calib_code;
  int32_t nr_of_samples;
  bool _return;
  void init(const std::string& calib_code, const int32_t nr_of_samples);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_resetOffset : public yarp::os::Portable {
public:
  std::string calib_code;
  bool _return;
  void init(const std::string& calib_code);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry : public yarp::os::Portable {
public:
  std::string initial_world_frame;
  std::string initial_fixed_link;
  bool _return;
  void init(const std::string& initial_world_frame, const std::string& initial_fixed_link);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry : public yarp::os::Portable {
public:
  std::string new_fixed_link;
  bool _return;
  void init(const std::string& new_fixed_link);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_set_imuFilterCutoffInHz : public yarp::os::Portable {
public:
  double newCutoff;
  bool _return;
  void init(const double newCutoff);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_get_imuFilterCutoffInHz : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_set_forceTorqueFilterCutoffInHz : public yarp::os::Portable {
public:
  double newCutoff;
  bool _return;
  void init(const double newCutoff);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_get_forceTorqueFilterCutoffInHz : public yarp::os::Portable {
public:
  double _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_useIMUAsKinematicSource : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource : public yarp::os::Portable {
public:
  std::string fixedFrame;
  bool _return;
  void init(const std::string& fixedFrame);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool wholeBodyDynamics_IDLServer_calib::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("calib",1,1)) return false;
  if (!writer.writeString(calib_code)) return false;
  if (!writer.writeI32(nr_of_samples)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_calib::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_calib::init(const std::string& calib_code, const int32_t nr_of_samples) {
  _return = false;
  this->calib_code = calib_code;
  this->nr_of_samples = nr_of_samples;
}

bool wholeBodyDynamics_IDLServer_calibStanding::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("calibStanding",1,1)) return false;
  if (!writer.writeString(calib_code)) return false;
  if (!writer.writeI32(nr_of_samples)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_calibStanding::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_calibStanding::init(const std::string& calib_code, const int32_t nr_of_samples) {
  _return = false;
  this->calib_code = calib_code;
  this->nr_of_samples = nr_of_samples;
}

bool wholeBodyDynamics_IDLServer_calibStandingLeftFoot::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("calibStandingLeftFoot",1,1)) return false;
  if (!writer.writeString(calib_code)) return false;
  if (!writer.writeI32(nr_of_samples)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_calibStandingLeftFoot::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_calibStandingLeftFoot::init(const std::string& calib_code, const int32_t nr_of_samples) {
  _return = false;
  this->calib_code = calib_code;
  this->nr_of_samples = nr_of_samples;
}

bool wholeBodyDynamics_IDLServer_calibStandingRightFoot::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("calibStandingRightFoot",1,1)) return false;
  if (!writer.writeString(calib_code)) return false;
  if (!writer.writeI32(nr_of_samples)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_calibStandingRightFoot::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_calibStandingRightFoot::init(const std::string& calib_code, const int32_t nr_of_samples) {
  _return = false;
  this->calib_code = calib_code;
  this->nr_of_samples = nr_of_samples;
}

bool wholeBodyDynamics_IDLServer_resetOffset::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("resetOffset",1,1)) return false;
  if (!writer.writeString(calib_code)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_resetOffset::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_resetOffset::init(const std::string& calib_code) {
  _return = false;
  this->calib_code = calib_code;
}

bool wholeBodyDynamics_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_quit::init() {
  _return = false;
}

bool wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("resetSimpleLeggedOdometry",1,1)) return false;
  if (!writer.writeString(initial_world_frame)) return false;
  if (!writer.writeString(initial_fixed_link)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry::init(const std::string& initial_world_frame, const std::string& initial_fixed_link) {
  _return = false;
  this->initial_world_frame = initial_world_frame;
  this->initial_fixed_link = initial_fixed_link;
}

bool wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("changeFixedLinkSimpleLeggedOdometry",1,1)) return false;
  if (!writer.writeString(new_fixed_link)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry::init(const std::string& new_fixed_link) {
  _return = false;
  this->new_fixed_link = new_fixed_link;
}

bool wholeBodyDynamics_IDLServer_set_imuFilterCutoffInHz::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_imuFilterCutoffInHz",1,2)) return false;
  if (!writer.writeDouble(newCutoff)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_set_imuFilterCutoffInHz::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_set_imuFilterCutoffInHz::init(const double newCutoff) {
  _return = false;
  this->newCutoff = newCutoff;
}

bool wholeBodyDynamics_IDLServer_get_imuFilterCutoffInHz::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_imuFilterCutoffInHz",1,2)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_get_imuFilterCutoffInHz::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_get_imuFilterCutoffInHz::init() {
  _return = (double)0;
}

bool wholeBodyDynamics_IDLServer_set_forceTorqueFilterCutoffInHz::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(3)) return false;
  if (!writer.writeTag("set_forceTorqueFilterCutoffInHz",1,2)) return false;
  if (!writer.writeDouble(newCutoff)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_set_forceTorqueFilterCutoffInHz::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_set_forceTorqueFilterCutoffInHz::init(const double newCutoff) {
  _return = false;
  this->newCutoff = newCutoff;
}

bool wholeBodyDynamics_IDLServer_get_forceTorqueFilterCutoffInHz::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("get_forceTorqueFilterCutoffInHz",1,2)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_get_forceTorqueFilterCutoffInHz::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readDouble(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_get_forceTorqueFilterCutoffInHz::init() {
  _return = (double)0;
}

bool wholeBodyDynamics_IDLServer_useIMUAsKinematicSource::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("useIMUAsKinematicSource",1,1)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_useIMUAsKinematicSource::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_useIMUAsKinematicSource::init() {
  _return = false;
}

bool wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("useFixedFrameAsKinematicSource",1,1)) return false;
  if (!writer.writeString(fixedFrame)) return false;
  return true;
}

bool wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource::init(const std::string& fixedFrame) {
  _return = false;
  this->fixedFrame = fixedFrame;
}

wholeBodyDynamics_IDLServer::wholeBodyDynamics_IDLServer() {
  yarp().setOwner(*this);
}
bool wholeBodyDynamics_IDLServer::calib(const std::string& calib_code, const int32_t nr_of_samples) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_calib helper;
  helper.init(calib_code,nr_of_samples);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::calib(const std::string& calib_code, const int32_t nr_of_samples)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::calibStanding(const std::string& calib_code, const int32_t nr_of_samples) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_calibStanding helper;
  helper.init(calib_code,nr_of_samples);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::calibStanding(const std::string& calib_code, const int32_t nr_of_samples)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_calibStandingLeftFoot helper;
  helper.init(calib_code,nr_of_samples);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_calibStandingRightFoot helper;
  helper.init(calib_code,nr_of_samples);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::resetOffset(const std::string& calib_code) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_resetOffset helper;
  helper.init(calib_code);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::resetOffset(const std::string& calib_code)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::quit() {
  bool _return = false;
  wholeBodyDynamics_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_resetSimpleLeggedOdometry helper;
  helper.init(initial_world_frame,initial_fixed_link);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_changeFixedLinkSimpleLeggedOdometry helper;
  helper.init(new_fixed_link);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::set_imuFilterCutoffInHz(const double newCutoff) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_set_imuFilterCutoffInHz helper;
  helper.init(newCutoff);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::set_imuFilterCutoffInHz(const double newCutoff)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double wholeBodyDynamics_IDLServer::get_imuFilterCutoffInHz() {
  double _return = (double)0;
  wholeBodyDynamics_IDLServer_get_imuFilterCutoffInHz helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double wholeBodyDynamics_IDLServer::get_imuFilterCutoffInHz()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::set_forceTorqueFilterCutoffInHz(const double newCutoff) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_set_forceTorqueFilterCutoffInHz helper;
  helper.init(newCutoff);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::set_forceTorqueFilterCutoffInHz(const double newCutoff)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
double wholeBodyDynamics_IDLServer::get_forceTorqueFilterCutoffInHz() {
  double _return = (double)0;
  wholeBodyDynamics_IDLServer_get_forceTorqueFilterCutoffInHz helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","double wholeBodyDynamics_IDLServer::get_forceTorqueFilterCutoffInHz()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::useIMUAsKinematicSource() {
  bool _return = false;
  wholeBodyDynamics_IDLServer_useIMUAsKinematicSource helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::useIMUAsKinematicSource()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::useFixedFrameAsKinematicSource(const std::string& fixedFrame) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_useFixedFrameAsKinematicSource helper;
  helper.init(fixedFrame);
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool wholeBodyDynamics_IDLServer::useFixedFrameAsKinematicSource(const std::string& fixedFrame)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool wholeBodyDynamics_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "calib") {
      std::string calib_code;
      int32_t nr_of_samples;
      if (!reader.readString(calib_code)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(nr_of_samples)) {
        nr_of_samples = 100;
      }
      bool _return;
      _return = calib(calib_code,nr_of_samples);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibStanding") {
      std::string calib_code;
      int32_t nr_of_samples;
      if (!reader.readString(calib_code)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(nr_of_samples)) {
        nr_of_samples = 100;
      }
      bool _return;
      _return = calibStanding(calib_code,nr_of_samples);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibStandingLeftFoot") {
      std::string calib_code;
      int32_t nr_of_samples;
      if (!reader.readString(calib_code)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(nr_of_samples)) {
        nr_of_samples = 100;
      }
      bool _return;
      _return = calibStandingLeftFoot(calib_code,nr_of_samples);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "calibStandingRightFoot") {
      std::string calib_code;
      int32_t nr_of_samples;
      if (!reader.readString(calib_code)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(nr_of_samples)) {
        nr_of_samples = 100;
      }
      bool _return;
      _return = calibStandingRightFoot(calib_code,nr_of_samples);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "resetOffset") {
      std::string calib_code;
      if (!reader.readString(calib_code)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = resetOffset(calib_code);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "quit") {
      bool _return;
      _return = quit();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "resetSimpleLeggedOdometry") {
      std::string initial_world_frame;
      std::string initial_fixed_link;
      if (!reader.readString(initial_world_frame)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(initial_fixed_link)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = resetSimpleLeggedOdometry(initial_world_frame,initial_fixed_link);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "changeFixedLinkSimpleLeggedOdometry") {
      std::string new_fixed_link;
      if (!reader.readString(new_fixed_link)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = changeFixedLinkSimpleLeggedOdometry(new_fixed_link);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_imuFilterCutoffInHz") {
      double newCutoff;
      if (!reader.readDouble(newCutoff)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_imuFilterCutoffInHz(newCutoff);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_imuFilterCutoffInHz") {
      double _return;
      _return = get_imuFilterCutoffInHz();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "set_forceTorqueFilterCutoffInHz") {
      double newCutoff;
      if (!reader.readDouble(newCutoff)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = set_forceTorqueFilterCutoffInHz(newCutoff);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "get_forceTorqueFilterCutoffInHz") {
      double _return;
      _return = get_forceTorqueFilterCutoffInHz();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeDouble(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "useIMUAsKinematicSource") {
      bool _return;
      _return = useIMUAsKinematicSource();
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "useFixedFrameAsKinematicSource") {
      std::string fixedFrame;
      if (!reader.readString(fixedFrame)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = useFixedFrameAsKinematicSource(fixedFrame);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "help") {
      std::string functionName;
      if (!reader.readString(functionName)) {
        functionName = "--all";
      }
      std::vector<std::string> _return=help(functionName);
      yarp::os::idl::WireWriter writer(reader);
        if (!writer.isNull()) {
          if (!writer.writeListHeader(2)) return false;
          if (!writer.writeTag("many",1, 0)) return false;
          if (!writer.writeListBegin(BOTTLE_TAG_INT, static_cast<uint32_t>(_return.size()))) return false;
          std::vector<std::string> ::iterator _iterHelp;
          for (_iterHelp = _return.begin(); _iterHelp != _return.end(); ++_iterHelp)
          {
            if (!writer.writeString(*_iterHelp)) return false;
           }
          if (!writer.writeListEnd()) return false;
        }
      reader.accept();
      return true;
    }
    if (reader.noMore()) { reader.fail(); return false; }
    yarp::os::ConstString next_tag = reader.readTag();
    if (next_tag=="") break;
    tag = tag + "_" + next_tag;
  }
  return false;
}

std::vector<std::string> wholeBodyDynamics_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("calib");
    helpString.push_back("calibStanding");
    helpString.push_back("calibStandingLeftFoot");
    helpString.push_back("calibStandingRightFoot");
    helpString.push_back("resetOffset");
    helpString.push_back("quit");
    helpString.push_back("resetSimpleLeggedOdometry");
    helpString.push_back("changeFixedLinkSimpleLeggedOdometry");
    helpString.push_back("set_imuFilterCutoffInHz");
    helpString.push_back("get_imuFilterCutoffInHz");
    helpString.push_back("set_forceTorqueFilterCutoffInHz");
    helpString.push_back("get_forceTorqueFilterCutoffInHz");
    helpString.push_back("useIMUAsKinematicSource");
    helpString.push_back("useFixedFrameAsKinematicSource");
    helpString.push_back("help");
  }
  else {
    if (functionName=="calib") {
      helpString.push_back("bool calib(const std::string& calib_code, const int32_t nr_of_samples = 100) ");
      helpString.push_back("Calibrate the force/torque sensors ");
      helpString.push_back("(WARNING: calibrate the sensors when the only external forces acting on the robot are on the torso/waist) ");
      helpString.push_back("@param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet) ");
      helpString.push_back("@param nr_of_samples number of samples ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="calibStanding") {
      helpString.push_back("bool calibStanding(const std::string& calib_code, const int32_t nr_of_samples = 100) ");
      helpString.push_back("Calibrate the force/torque sensors when on double support ");
      helpString.push_back("(WARNING: calibrate the sensors when the only external forces acting on the robot are on the sole). ");
      helpString.push_back("For this calibration the strong assumption of simmetry of the robot and its pose is done. ");
      helpString.push_back("@param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet) ");
      helpString.push_back("@param nr_of_samples number of samples ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="calibStandingLeftFoot") {
      helpString.push_back("bool calibStandingLeftFoot(const std::string& calib_code, const int32_t nr_of_samples = 100) ");
      helpString.push_back("Calibrate the force/torque sensors when on single support on left foot ");
      helpString.push_back("(WARNING: calibrate the sensors when the only external forces acting on the robot are on the left sole). ");
      helpString.push_back("@param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet) ");
      helpString.push_back("@param nr_of_samples number of samples ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="calibStandingRightFoot") {
      helpString.push_back("bool calibStandingRightFoot(const std::string& calib_code, const int32_t nr_of_samples = 100) ");
      helpString.push_back("Calibrate the force/torque sensors when on single support on right foot ");
      helpString.push_back("(WARNING: calibrate the sensors when the only external forces acting on the robot are on the right sole). ");
      helpString.push_back("@param calib_code argument to specify the sensors to calibrate (all,arms,legs,feet) ");
      helpString.push_back("@param nr_of_samples number of samples ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="resetOffset") {
      helpString.push_back("bool resetOffset(const std::string& calib_code) ");
      helpString.push_back("Reset the sensor offset to 0 0 0 0 0 0 (six zeros). ");
      helpString.push_back("@param calib_code argument to specify the sensors to reset (all,arms,legs,feet) ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="quit") {
      helpString.push_back("bool quit() ");
      helpString.push_back("Quit the module. ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="resetSimpleLeggedOdometry") {
      helpString.push_back("bool resetSimpleLeggedOdometry(const std::string& initial_world_frame, const std::string& initial_fixed_link) ");
      helpString.push_back("Reset the odometry world to be (initially) a frame specified in the robot model, ");
      helpString.push_back("and specify a link that is assumed to be fixed in the odometry. ");
      helpString.push_back("@param initial_world_frame the frame of the robot model that is assume to be initially ");
      helpString.push_back("       coincident with the world/inertial frame. ");
      helpString.push_back("@param new_fixed_link the name of the link that should be initially fixed ");
      helpString.push_back("@return true/false on success/failure (typically if the frame/link names are wrong) ");
    }
    if (functionName=="changeFixedLinkSimpleLeggedOdometry") {
      helpString.push_back("bool changeFixedLinkSimpleLeggedOdometry(const std::string& new_fixed_link) ");
      helpString.push_back("Change the link that is considered fixed by the odometry. ");
      helpString.push_back("@param new_fixed_link the name of the new link that should be considered fixed ");
      helpString.push_back("@return true/false on success/failure (typically if the frame/link names are wrong) ");
    }
    if (functionName=="set_imuFilterCutoffInHz") {
      helpString.push_back("bool set_imuFilterCutoffInHz(const double newCutoff) ");
      helpString.push_back("Set the cutoff frequency (in Hz) for IMU measurements ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="get_imuFilterCutoffInHz") {
      helpString.push_back("double get_imuFilterCutoffInHz() ");
      helpString.push_back("Get the cutoff frequency (in Hz) for IMU measurements ");
      helpString.push_back("@return the cutoff frequency (in Hz) ");
    }
    if (functionName=="set_forceTorqueFilterCutoffInHz") {
      helpString.push_back("bool set_forceTorqueFilterCutoffInHz(const double newCutoff) ");
      helpString.push_back("Set the cutoff frequency (in Hz) for FT measurements ");
      helpString.push_back("@return true/false on success/failure ");
    }
    if (functionName=="get_forceTorqueFilterCutoffInHz") {
      helpString.push_back("double get_forceTorqueFilterCutoffInHz() ");
      helpString.push_back("Get the cutoff frequency (in Hz) for FT measurements ");
      helpString.push_back("@return the cutoff frequency (in Hz) ");
    }
    if (functionName=="useIMUAsKinematicSource") {
      helpString.push_back("bool useIMUAsKinematicSource() ");
      helpString.push_back("Use the IMU as the kinematic source of ");
      helpString.push_back("information for the acceleration of one link. ");
    }
    if (functionName=="useFixedFrameAsKinematicSource") {
      helpString.push_back("bool useFixedFrameAsKinematicSource(const std::string& fixedFrame) ");
      helpString.push_back("Use a fixed frame (tipically root_link, l_sole or r_sole) ");
      helpString.push_back("as the source of kinematic information. The assumption ");
      helpString.push_back("is that the specified frame will remain fixed until ");
      helpString.push_back("the kinematic source is changing, and the gravity ");
      helpString.push_back("on this link is specified by the fixedFrameGravity (tipically ");
      helpString.push_back("set to (0,0,-9.81) . ");
    }
    if (functionName=="help") {
      helpString.push_back("std::vector<std::string> help(const std::string& functionName=\"--all\")");
      helpString.push_back("Return list of available commands, or help message for a specific function");
      helpString.push_back("@param functionName name of command for which to get a detailed description. If none or '--all' is provided, print list of available commands");
      helpString.push_back("@return list of strings (one string per line)");
    }
  }
  if ( helpString.empty()) helpString.push_back("Command not found");
  return helpString;
}



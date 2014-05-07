// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <wholeBodyDynamics_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class wholeBodyDynamics_IDLServer_calib : public yarp::os::Portable {
public:
  std::string calib_code;
  int32_t nr_of_samples;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("calib",1,1)) return false;
    if (!writer.writeString(calib_code)) return false;
    if (!writer.writeI32(nr_of_samples)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class wholeBodyDynamics_IDLServer_calibStanding : public yarp::os::Portable {
public:
  std::string calib_code;
  int32_t nr_of_samples;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("calibStanding",1,1)) return false;
    if (!writer.writeString(calib_code)) return false;
    if (!writer.writeI32(nr_of_samples)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class wholeBodyDynamics_IDLServer_resetOffset : public yarp::os::Portable {
public:
  std::string calib_code;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(2)) return false;
    if (!writer.writeTag("resetOffset",1,1)) return false;
    if (!writer.writeString(calib_code)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

class wholeBodyDynamics_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(1)) return false;
    if (!writer.writeTag("quit",1,1)) return false;
    return true;
  }
  virtual bool read(yarp::os::ConnectionReader& connection) {
    yarp::os::idl::WireReader reader(connection);
    if (!reader.readListReturn()) return false;
    if (!reader.readBool(_return)) {
      reader.fail();
      return false;
    }
    return true;
  }
};

bool wholeBodyDynamics_IDLServer::calib(const std::string& calib_code, const int32_t nr_of_samples) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_calib helper;
  helper.calib_code = calib_code;
  helper.nr_of_samples = nr_of_samples;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool wholeBodyDynamics_IDLServer::calib(const std::string& calib_code, const int32_t nr_of_samples)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::calibStanding(const std::string& calib_code, const int32_t nr_of_samples) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_calibStanding helper;
  helper.calib_code = calib_code;
  helper.nr_of_samples = nr_of_samples;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool wholeBodyDynamics_IDLServer::calibStanding(const std::string& calib_code, const int32_t nr_of_samples)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::resetOffset(const std::string& calib_code) {
  bool _return = false;
  wholeBodyDynamics_IDLServer_resetOffset helper;
  helper.calib_code = calib_code;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool wholeBodyDynamics_IDLServer::resetOffset(const std::string& calib_code)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool wholeBodyDynamics_IDLServer::quit() {
  bool _return = false;
  wholeBodyDynamics_IDLServer_quit helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool wholeBodyDynamics_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool wholeBodyDynamics_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
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
    helpString.push_back("resetOffset");
    helpString.push_back("quit");
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



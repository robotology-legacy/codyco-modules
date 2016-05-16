// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <insituFTSensorCalibration_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class insituFTSensorCalibration_IDLServer_startNewDatasetAcquisition : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class insituFTSensorCalibration_IDLServer_quit : public yarp::os::Portable {
public:
  bool _return;
  void init();
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool insituFTSensorCalibration_IDLServer_startNewDatasetAcquisition::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("startNewDatasetAcquisition",1,1)) return false;
  return true;
}

bool insituFTSensorCalibration_IDLServer_startNewDatasetAcquisition::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void insituFTSensorCalibration_IDLServer_startNewDatasetAcquisition::init() {
  _return = false;
}

bool insituFTSensorCalibration_IDLServer_quit::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(1)) return false;
  if (!writer.writeTag("quit",1,1)) return false;
  return true;
}

bool insituFTSensorCalibration_IDLServer_quit::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void insituFTSensorCalibration_IDLServer_quit::init() {
  _return = false;
}

insituFTSensorCalibration_IDLServer::insituFTSensorCalibration_IDLServer() {
  yarp().setOwner(*this);
}
bool insituFTSensorCalibration_IDLServer::startNewDatasetAcquisition() {
  bool _return = false;
  insituFTSensorCalibration_IDLServer_startNewDatasetAcquisition helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool insituFTSensorCalibration_IDLServer::startNewDatasetAcquisition()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool insituFTSensorCalibration_IDLServer::quit() {
  bool _return = false;
  insituFTSensorCalibration_IDLServer_quit helper;
  helper.init();
  if (!yarp().canWrite()) {
    yError("Missing server method '%s'?","bool insituFTSensorCalibration_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool insituFTSensorCalibration_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "startNewDatasetAcquisition") {
      bool _return;
      _return = startNewDatasetAcquisition();
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

std::vector<std::string> insituFTSensorCalibration_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("startNewDatasetAcquisition");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="startNewDatasetAcquisition") {
      helpString.push_back("bool startNewDatasetAcquisition() ");
      helpString.push_back("Start the new dataset acquisition. ");
      helpString.push_back("At the end of each dataset acquisition, ");
      helpString.push_back("the module stops until the user changes ");
      helpString.push_back("the added mass on the robot. This command ");
      helpString.push_back("is used to start the dataset acquisition ");
      helpString.push_back("after the added mass is correctly mounted ");
      helpString.push_back("in the robot. ");
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



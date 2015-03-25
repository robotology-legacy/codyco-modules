// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <eventRepeater.h>
#include <yarp/os/idl/WireTypes.h>



class eventRepeater_sendEvent : public yarp::os::Portable {
public:
  std::string event;
  bool _return;
  void init(const std::string& event);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

class eventRepeater_se : public yarp::os::Portable {
public:
  std::string event;
  bool _return;
  void init(const std::string& event);
  virtual bool write(yarp::os::ConnectionWriter& connection);
  virtual bool read(yarp::os::ConnectionReader& connection);
};

bool eventRepeater_sendEvent::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("sendEvent",1,1)) return false;
  if (!writer.writeString(event)) return false;
  return true;
}

bool eventRepeater_sendEvent::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void eventRepeater_sendEvent::init(const std::string& event) {
  _return = false;
  this->event = event;
}

bool eventRepeater_se::write(yarp::os::ConnectionWriter& connection) {
  yarp::os::idl::WireWriter writer(connection);
  if (!writer.writeListHeader(2)) return false;
  if (!writer.writeTag("se",1,1)) return false;
  if (!writer.writeString(event)) return false;
  return true;
}

bool eventRepeater_se::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  if (!reader.readListReturn()) return false;
  if (!reader.readBool(_return)) {
    reader.fail();
    return false;
  }
  return true;
}

void eventRepeater_se::init(const std::string& event) {
  _return = false;
  this->event = event;
}

eventRepeater::eventRepeater() {
  yarp().setOwner(*this);
}
bool eventRepeater::sendEvent(const std::string& event) {
  bool _return = false;
  eventRepeater_sendEvent helper;
  helper.init(event);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool eventRepeater::sendEvent(const std::string& event)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool eventRepeater::se(const std::string& event) {
  bool _return = false;
  eventRepeater_se helper;
  helper.init(event);
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool eventRepeater::se(const std::string& event)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool eventRepeater::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  bool direct = (tag=="__direct__");
  if (direct) tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "sendEvent") {
      std::string event;
      if (!reader.readString(event)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = sendEvent(event);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "se") {
      std::string event;
      if (!reader.readString(event)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = se(event);
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

std::vector<std::string> eventRepeater::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("sendEvent");
    helpString.push_back("se");
    helpString.push_back("help");
  }
  else {
    if (functionName=="sendEvent") {
      helpString.push_back("bool sendEvent(const std::string& event) ");
      helpString.push_back("Raise an event on the output streaming port ");
      helpString.push_back("@param event event to raise. ");
    }
    if (functionName=="se") {
      helpString.push_back("bool se(const std::string& event) ");
      helpString.push_back("Raise an event on the output streaming port ");
      helpString.push_back("@param event event to raise. ");
      helpString.push_back("\note This is just an shorted alias for the sendEvent method ");
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



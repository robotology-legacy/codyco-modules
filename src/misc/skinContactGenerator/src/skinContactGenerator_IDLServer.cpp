// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#include <skinContactGenerator_IDLServer.h>
#include <yarp/os/idl/WireTypes.h>



class skinContactGenerator_IDLServer_setContact : public yarp::os::Portable {
public:
  int32_t bodyPart;
  int32_t link;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("setContact",1,1)) return false;
    if (!writer.writeI32(bodyPart)) return false;
    if (!writer.writeI32(link)) return false;
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

class skinContactGenerator_IDLServer_setContactForce : public yarp::os::Portable {
public:
  int32_t bodyPart;
  int32_t link;
  double f_x;
  double f_y;
  double f_z;
  double p_x;
  double p_y;
  double p_z;
  int32_t skinPart;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(10)) return false;
    if (!writer.writeTag("setContactForce",1,1)) return false;
    if (!writer.writeI32(bodyPart)) return false;
    if (!writer.writeI32(link)) return false;
    if (!writer.writeDouble(f_x)) return false;
    if (!writer.writeDouble(f_y)) return false;
    if (!writer.writeDouble(f_z)) return false;
    if (!writer.writeDouble(p_x)) return false;
    if (!writer.writeDouble(p_y)) return false;
    if (!writer.writeDouble(p_z)) return false;
    if (!writer.writeI32(skinPart)) return false;
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

class skinContactGenerator_IDLServer_setContactName : public yarp::os::Portable {
public:
  std::string bodyPart;
  std::string link;
  bool _return;
  virtual bool write(yarp::os::ConnectionWriter& connection) {
    yarp::os::idl::WireWriter writer(connection);
    if (!writer.writeListHeader(3)) return false;
    if (!writer.writeTag("setContactName",1,1)) return false;
    if (!writer.writeString(bodyPart)) return false;
    if (!writer.writeString(link)) return false;
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

class skinContactGenerator_IDLServer_quit : public yarp::os::Portable {
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

bool skinContactGenerator_IDLServer::setContact(const int32_t bodyPart, const int32_t link) {
  bool _return = false;
  skinContactGenerator_IDLServer_setContact helper;
  helper.bodyPart = bodyPart;
  helper.link = link;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool skinContactGenerator_IDLServer::setContact(const int32_t bodyPart, const int32_t link)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool skinContactGenerator_IDLServer::setContactForce(const int32_t bodyPart, const int32_t link, const double f_x, const double f_y, const double f_z, const double p_x, const double p_y, const double p_z, const int32_t skinPart) {
  bool _return = false;
  skinContactGenerator_IDLServer_setContactForce helper;
  helper.bodyPart = bodyPart;
  helper.link = link;
  helper.f_x = f_x;
  helper.f_y = f_y;
  helper.f_z = f_z;
  helper.p_x = p_x;
  helper.p_y = p_y;
  helper.p_z = p_z;
  helper.skinPart = skinPart;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool skinContactGenerator_IDLServer::setContactForce(const int32_t bodyPart, const int32_t link, const double f_x, const double f_y, const double f_z, const double p_x, const double p_y, const double p_z, const int32_t skinPart)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool skinContactGenerator_IDLServer::setContactName(const std::string& bodyPart, const std::string& link) {
  bool _return = false;
  skinContactGenerator_IDLServer_setContactName helper;
  helper.bodyPart = bodyPart;
  helper.link = link;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool skinContactGenerator_IDLServer::setContactName(const std::string& bodyPart, const std::string& link)");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}
bool skinContactGenerator_IDLServer::quit() {
  bool _return = false;
  skinContactGenerator_IDLServer_quit helper;
  if (!yarp().canWrite()) {
    fprintf(stderr,"Missing server method '%s'?\n","bool skinContactGenerator_IDLServer::quit()");
  }
  bool ok = yarp().write(helper,helper);
  return ok?helper._return:_return;
}

bool skinContactGenerator_IDLServer::read(yarp::os::ConnectionReader& connection) {
  yarp::os::idl::WireReader reader(connection);
  reader.expectAccept();
  if (!reader.readListHeader()) { reader.fail(); return false; }
  yarp::os::ConstString tag = reader.readTag();
  while (!reader.isError()) {
    // TODO: use quick lookup, this is just a test
    if (tag == "setContact") {
      int32_t bodyPart;
      int32_t link;
      if (!reader.readI32(bodyPart)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(link)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setContact(bodyPart,link);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setContactForce") {
      int32_t bodyPart;
      int32_t link;
      double f_x;
      double f_y;
      double f_z;
      double p_x;
      double p_y;
      double p_z;
      int32_t skinPart;
      if (!reader.readI32(bodyPart)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(link)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(f_x)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(f_y)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(f_z)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(p_x)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(p_y)) {
        reader.fail();
        return false;
      }
      if (!reader.readDouble(p_z)) {
        reader.fail();
        return false;
      }
      if (!reader.readI32(skinPart)) {
        skinPart = 0;
      }
      bool _return;
      _return = setContactForce(bodyPart,link,f_x,f_y,f_z,p_x,p_y,p_z,skinPart);
      yarp::os::idl::WireWriter writer(reader);
      if (!writer.isNull()) {
        if (!writer.writeListHeader(1)) return false;
        if (!writer.writeBool(_return)) return false;
      }
      reader.accept();
      return true;
    }
    if (tag == "setContactName") {
      std::string bodyPart;
      std::string link;
      if (!reader.readString(bodyPart)) {
        reader.fail();
        return false;
      }
      if (!reader.readString(link)) {
        reader.fail();
        return false;
      }
      bool _return;
      _return = setContactName(bodyPart,link);
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

std::vector<std::string> skinContactGenerator_IDLServer::help(const std::string& functionName) {
  bool showAll=(functionName=="--all");
  std::vector<std::string> helpString;
  if(showAll) {
    helpString.push_back("*** Available commands:");
    helpString.push_back("setContact");
    helpString.push_back("setContactForce");
    helpString.push_back("setContactName");
    helpString.push_back("quit");
    helpString.push_back("help");
  }
  else {
    if (functionName=="setContact") {
      helpString.push_back("bool setContact(const int32_t bodyPart, const int32_t link) ");
      helpString.push_back("Configure the skinContactGenerator to produce a contact ");
      helpString.push_back("at the origin of a given link. ");
    }
    if (functionName=="setContactForce") {
      helpString.push_back("bool setContactForce(const int32_t bodyPart, const int32_t link, const double f_x, const double f_y, const double f_z, const double p_x, const double p_y, const double p_z, const int32_t skinPart = 0) ");
      helpString.push_back("Configure the skinContactGenerator to produce a contact ");
      helpString.push_back("at a prescribed position of a given link. ");
    }
    if (functionName=="setContactName") {
      helpString.push_back("bool setContactName(const std::string& bodyPart, const std::string& link) ");
      helpString.push_back("Configure the skinContactGenerator to produce a contact ");
      helpString.push_back("at the origin of a given link. ");
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



// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_eventRepeater
#define YARP_THRIFT_GENERATOR_eventRepeater

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>

class eventRepeater;


class eventRepeater : public yarp::os::Wire {
public:
  eventRepeater();
  /**
   * Raise an event on the output streaming port
   * @param event event to raise.
   */
  virtual bool sendEvent(const std::string& event);
  /**
   * Raise an event on the output streaming port
   * @param event event to raise.
   * \note This is just an shorted alias for the sendEvent method
   */
  virtual bool se(const std::string& event);
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif


/*
 * Copyright (C) 2013  CoDyCo Consortium
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 *
 * Authors: Andrea Del Prete
 * email: andrea.delprete@iit.it
 */

#ifndef __PARAMHELPUTIL_H__
#define __PARAMHELPUTIL_H__

#include <yarp/sig/Vector.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/BufferedPort.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iterator>
#include <map>
#include <vector>


namespace paramHelp
{

/** Suffixes of the ports opened by the ParamHelper class. */
static const char* PORT_IN_STREAM_SUFFIX    = "/stream:i";
static const char* PORT_OUT_STREAM_SUFFIX   = "/stream:o";
static const char* PORT_OUT_INFO_SUFFIX     = "/info:o";
static const char* PORT_IN_INFO_SUFFIX      = "/info:i";
static const char* PORT_RPC_SUFFIX          = "/rpc";
static const char* PORT_OUT_MONITOR_SUFFIX  = "/monitor:o";


/** Convert a generic variable into a string (using an ostringstream). */
template <class T> inline std::string toString(const T& t)
{ 
    std::ostringstream ss; 
    ss << t; 
    return ss.str(); 
}

/** Convert a generic vector into a string (using an ostringstream). */
template <class T> inline std::string toString(const std::vector<T>& v, const char *separator=" ")
{ 
    std::ostringstream s; 
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(s, separator)); 
    return s.str(); 
}

template<class T1>
std::string strcat(const T1 &s)
{ return toString(s); }

template<class T1, class T2>
std::string strcat(const T1 &s1, const T2 &s2)
{ return toString(s1)+toString(s2); }

template<class T1, class T2, class T3>
std::string strcat(const T1 &s1, const T2 &s2, const T3 &s3)
{ return toString(s1)+toString(s2)+toString(s3); }

template<class T1, class T2, class T3, class T4>
std::string strcat(const T1 &s1, const T2 &s2, const T3 &s3, const T4 s4)
{ return toString(s1)+toString(s2)+toString(s3)+toString(s4); }

template<class T1, class T2, class T3, class T4, class T5>
std::string strcat(const T1 &s1, const T2 &s2, const T3 &s3, const T4 s4, const T5 s5)
{ return toString(s1)+toString(s2)+toString(s3)+toString(s4)+toString(s5); }

/** Convert a generic variable into a yarp::os::Value. */
template <class T> inline yarp::os::Value castToValue(const T& t){                          return (*yarp::os::Value::makeValue(toString(t).c_str())); }
template <>        inline yarp::os::Value castToValue<double>(const double& t){             return yarp::os::Value(t); }
template <>        inline yarp::os::Value castToValue<float>(const float& t){               return yarp::os::Value((double)t); }
template <>        inline yarp::os::Value castToValue<int>(const int& t){                   return yarp::os::Value(t); }
template <>        inline yarp::os::Value castToValue<bool>(const bool& t){                 return yarp::os::Value(t); }
template <>        inline yarp::os::Value castToValue<std::string>(const std::string& t){   return (*yarp::os::Value::makeString(t.c_str())); }

/** Convert a yarp::os::Value into the specified type. */
template <class T> inline T           castFromValue(const yarp::os::Value &v){              return (T) std::string(v.toString().c_str()); }
template <>        inline double      castFromValue<double>(const yarp::os::Value &v){      return v.asDouble(); }
template <>        inline float       castFromValue<float>(const yarp::os::Value &v){       return (float)v.asDouble(); }
template <>        inline int         castFromValue<int>(const yarp::os::Value &v){         return v.asInt(); }
template <>        inline bool        castFromValue<bool>(const yarp::os::Value &v){        return v.asBool(); }
template <>        inline std::string castFromValue<std::string>(const yarp::os::Value &v){ return v.asString().c_str(); }

/** Print the specified Bottle. */
inline void printBottle(const yarp::os::Bottle &b)
{
    for(int i=0; i<b.size(); i++)
        printf("%s\n", b.get(i).asString().c_str());
}

}//end namespace paramHelp


#endif




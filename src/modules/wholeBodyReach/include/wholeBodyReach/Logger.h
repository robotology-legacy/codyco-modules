/*
 * Copyright (C) 2014 Koroibot
 * Author: Andrea Del Prete
 * email:  adelpret@laas.fr
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
 */



#ifndef WBR_LOGGER_H
#define WBR_LOGGER_H

#include<Eigen/Core>
#include <wholeBodyReach/wholeBodyReachConstants.h>

namespace wholeBodyReach
{
    
enum MsgType
{
	MSG_STREAM_INFO     = 0,	//
	MSG_STREAM_ERROR    = 1,	//
	MSG_INFO            = 2,	//
    MSG_ERROR           = 3     //
};

/** A simple class for logging messages
*/
class Logger
{
public:
    
	/** Constructor */
	Logger(double timeSample=0.0, double streamPrintPeriod=0.0);

	/** Destructor */
	~Logger(){}

	void countdown();
    
    void sendMsg(std::string msg, MsgType type);
    
    bool setTimeSample(double t);
    bool setStreamPrintPeriod(double s);
    
protected:
    double      _timeSample;        // specify the period of call of the countdown method
    double      _streamPrintPeriod; // specify the time period of the stream prints
    double      _printCountdown;    // every time this is < 0 (i.e. every _streamPrintPeriod sec) print stuff
};

Logger& getLogger();

std::string toString(const Eigen::MatrixRXd &m, int precision=2, const char* endRowStr="\n", int maxRowsPerLine=10);
    
std::string jointToString(const Eigen::VectorXd &j, int precision=1);
    
/** Convert a generic variable into a string. */
template <class T> inline std::string toString(const T& t)
{ std::ostringstream ss; ss << t; return ss.str(); }

/** Convert a generic vector into a string */
template <class T> inline std::string toString(const std::vector<T>& v, const char *separator=" ")
{ std::ostringstream s; std::copy(v.begin(), v.end(), std::ostream_iterator<T>(s, separator)); return s.str(); }
    
};  // end namespace wholeBodyReach

#endif

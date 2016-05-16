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

namespace wholeBodyReach
{
    
/** Enum representing the different kind of messages.
 */
enum MsgType
{
    MSG_STREAM_DEBUG    = 0,    // streaming debug message
	MSG_STREAM_INFO     = 1,	// streaming information message
    MSG_STREAM_WARNING  = 2,    // streaming warning message
	MSG_STREAM_ERROR    = 3,	// streaming error message
    MSG_DEBUG           = 4,    // debug message
	MSG_INFO            = 5,	// information message
    MSG_WARNING         = 6,    // warning message
    MSG_ERROR           = 7     // error message
};
    
enum LoggerVerbosity
{
    VERBOSITY_ALL,
    VERBOSITY_INFO_WARNING_ERROR,
    VERBOSITY_WARNING_ERROR,
    VERBOSITY_ERROR,
    VERBOSITY_NONE
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

    /** Method to be called at every control iteration
     * to decrement the internal Logger's counter. */
	void countdown();
    
    void sendMsg(std::string msg, MsgType type);
    
    /** Set the sampling time at which the method countdown()
     * is going to be called. */
    bool setTimeSample(double t);
    
    /** Set the time period for printing of streaming messages. */
    bool setStreamPrintPeriod(double s);
    
    /** Set the verbosity level of the logger. */
    void setVerbosity(LoggerVerbosity lv);
    
protected:
    LoggerVerbosity _lv;                /// verbosity of the logger
    double          _timeSample;        /// specify the period of call of the countdown method
    double          _streamPrintPeriod; /// specify the time period of the stream prints
    double          _printCountdown;    /// every time this is < 0 (i.e. every _streamPrintPeriod sec) print stuff
    
    bool isStreamMsg(MsgType m)
    { return m==MSG_STREAM_ERROR || m==MSG_STREAM_DEBUG || m==MSG_STREAM_INFO || m==MSG_STREAM_WARNING; }
    
    bool isDebugMsg(MsgType m)
    { return m==MSG_STREAM_DEBUG || m==MSG_DEBUG; }
    
    bool isInfoMsg(MsgType m)
    { return m==MSG_STREAM_INFO || m==MSG_INFO; }
    
    bool isWarningMsg(MsgType m)
    { return m==MSG_STREAM_WARNING || m==MSG_WARNING; }
    
    bool isErrorMsg(MsgType m)
    { return m==MSG_STREAM_ERROR || m==MSG_ERROR; }
};

/** Method to get the logger (singleton). */
Logger& getLogger();
    
};  // end namespace wholeBodyReach

#endif

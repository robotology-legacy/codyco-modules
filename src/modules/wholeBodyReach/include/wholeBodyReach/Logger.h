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
    
};  // end namespace wholeBodyReach

#endif

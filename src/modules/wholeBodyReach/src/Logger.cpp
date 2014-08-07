/*
Copyright (c) 2010-2013 Tommaso Urli

Tommaso Urli    tommaso.urli@uniud.it   University of Udine

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be
included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/

#ifndef WIN32
	#include <sys/time.h>
#else
	#include <Windows.h>
	#include <iomanip>
#endif

#include <iostream>
#include <iomanip>      // std::setprecision
#include "wholeBodyReach/Logger.h"

using namespace std;
using namespace wholeBodyReach;

Logger& wholeBodyReach::getLogger()
{
    static Logger l;
    return l;
}

Logger::Logger(double timeSample, double streamPrintPeriod)
: _timeSample(timeSample),
 _streamPrintPeriod(streamPrintPeriod),
 _printCountdown(0.0)
{}

void Logger::countdown()
{
    if(_printCountdown<0.0)
        _printCountdown = _streamPrintPeriod;
    _printCountdown -= _timeSample;
}

void Logger::sendMsg(string msg, MsgType type)
{
    if(type==MSG_STREAM_INFO && _printCountdown<0.0)
    {
        printf("%s\n", msg.c_str());
        return;
    }
    if(type==MSG_INFO || type==MSG_ERROR)
        printf("%s\n", msg.c_str());
}

bool Logger::setTimeSample(double t)
{
    if(t<=0.0)
        return false;
    _timeSample = t;
    return true;
}

bool Logger::setStreamPrintPeriod(double s)
{
    if(s<=0.0)
        return false;
    _streamPrintPeriod = s;
    return true;
}




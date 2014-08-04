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

//*************************************************************************************************************************
std::string wholeBodyReach::toString(const Eigen::MatrixRXd &m, int precision, const char* endRowStr, int maxColsPerLine)
{
    // if m is a column vector print it as a row vector
    if(m.cols()==1)
        return toString(m.transpose(), precision, endRowStr, maxColsPerLine);
    
    string ret = "";
    if(m.rows()>1 && m.cols()>maxColsPerLine)
    {
        return ret+"("+toString(maxColsPerLine)+" cols)\n" +
        toString(m.leftCols(maxColsPerLine),precision,endRowStr,maxColsPerLine) + "\n" +
        toString(m.rightCols(m.cols()-maxColsPerLine),precision,endRowStr,maxColsPerLine);
    }
    char tmp[350];
    for(int i=0;i<m.rows();i++)
    {
        for(int j=0;j<m.cols();j++)
        {
            sprintf(tmp, "% .*lf\t", precision, m(i,j));
            ret+=tmp;
        }
        ret = ret.substr(0,ret.length()-1);     // remove the last character (tab)
        if(i<m.rows()-1)                          // if it is not the last row
            ret+= endRowStr;
    }
    return ret; //.substr(0, ret.length()-1);
}

std::string wholeBodyReach::jointToString(const Eigen::VectorXd &j, int precision)
{
    if(j.size()!=ICUB_DOFS && j.size()!=ICUB_DOFS+6)
        cout<<"Error in size of joint vector: "<<j.size()<<endl;
    
    int index=0;
    string ret = "";
    char tmp[350];
    if(j.size()==ICUB_DOFS+6)
    {
        ret += "base(";
        for(int i=0;i<6;i++)
        {
            sprintf(tmp, "% .*lf ", precision, j(index));
            ret+=tmp;
            index++;
        }
        ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
        ret += ")\t";
    }
    ret += "torso(";
    for(int i=0;i<3;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tl_arm(";
    for(int i=0;i<5;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tr_arm(";
    for(int i=0;i<5;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tl_leg(";
    for(int i=0;i<6;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret = ret.substr(0, ret.length()-1); // remove the last character (tab)
    ret += ")\tr_leg(";
    for(int i=0;i<6;i++)
    {
        sprintf(tmp, "% .*lf ", precision, j(index));
        ret+=tmp;
        index++;
    }
    ret += ")";
    return ret;
}


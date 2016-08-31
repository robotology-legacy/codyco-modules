#include "JointTorqueControl.h"
#include <yarp/os/Property.h>
#include <yarp/os/LockGuard.h>

#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <yarp/os/Bottle.h>

#include <algorithm>
#include <math.h>
#include <cstring>

#include <yarp/os/Time.h>

#include <Eigen/LU>

#include <iostream>
#include <cmath>

#include <yarp/os/all.h>

RpcServerCallback::RpcServerCallback(yarp::dev::JointTorqueControl* jtcPtr)
: jtc(jtcPtr)
{
}

RpcServerCallback::~RpcServerCallback()
{
   if(jtc != NULL){
       delete jtc;
   }
}

bool RpcServerCallback::read(yarp::os::ConnectionReader& connection)
{
     yarp::os::Bottle request, reply;
     bool ok = request.read(connection);
     if (!ok) return false;
     // process data "request", prepare "reply"
     parseRequestAndWriteReply(request, reply);
     yarp::os::ConnectionWriter *returnToSender = connection.getWriter();
     if (returnToSender!=NULL) {
         reply.write(*returnToSender);
     }
     return true;
}

void RpcServerCallback::parseRequestAndWriteReply(yarp::os::Bottle& request, yarp::os::Bottle& reply)
{
    if(request.size()>0) {
        std::string keyword = request.get(0).asString();

        if (keyword=="help") {
            generateHelpResponse(reply);
        } else if (keyword=="save") {
            saveParametersToFile(request, reply);
        } else if (keyword=="set") {
            setParameterValues(request, reply);
        } else if (keyword=="get") {
            std::string secondKeyword = request.get(1).asString();

            if (secondKeyword == "joint_list") {
                for(int j=0; j < jtc->axesNames.size(); j++) {
                    reply.addString(jtc->axesNames[j]);
                }
            } else if (secondKeyword == "no_joints") {
                reply.addInt(jtc->axes);
            } else if (secondKeyword == "params") {
                int joint_index = request.get(2).asInt();
                addAllJointParamsToReply(joint_index, reply);
            } else {
                getParameterValues(request, reply);
            }

        } else {
            reply.addString("Type [help] for valid commands.");
        }
    }else {
        reply.addString("Type [help] for valid commands.");
    }

}

void RpcServerCallback::generateHelpResponse(yarp::os::Bottle& reply)
{
    std::string replyString = "\n===============\nValid Commands\n===============\nNote: Omit the brackets [] when writing your commands.\n\n";
    replyString += "[save] [location] - saves all current parameters to a txt file at the specified [location] (defaults to ./).\n";
    replyString += "[get] [joint_list] - returns a list of strings with the joint labels.\n";
    replyString += "[get] [no_joints] - returns the number of joints in the part.\n";
    replyString += "[get] [params] [joint_index] - get all parameter values for a joint index.\n";
    replyString += "[get] [param_id] [joint_index] - get the parameter value for a joint index.\n";
    replyString += "[set] [joint_index] [param_id] [param_value] [param_id] [param_value] ... - set the parameter value(s) for a joint index.\n";
    replyString += "\n  Valid [param_id] values:\n";
    replyString += "  --> kp - The proportional torque error gain.\n";
    replyString += "  --> ki - The integral torque error gain.\n";
    replyString += "  --> kd - The derivative torque error gain.\n";
    replyString += "  --> max_int - Maximum allowable integral gain windup (spooling).\n";
    replyString += "  --> max_pwm - Maximum allowable PWM signal.\n";
    replyString += "  --> kv - Propotional constant multiplied by the joint velocity (back emf).\n";
    replyString += "  --> kcp - Stiction up (qd > 0) proportional constant multiplied to the coulomb friction, or sign(qd) function.\n";
    replyString += "  --> kcn - Stiction down (qd < 0) proportional constant multiplied to the coulomb friction, or sign(qd) function.\n";
    replyString += "  --> coulombVelThr - Joint velocity at which to apply the sign(qd) estimate of coulomb friction. Below this threshold the coulomb friction is calculated as (qd/coulombVelThr)^3\n";
    replyString += "  --> kff - Feedforward proportional gain on the desired torque.\n";
    replyString += "  --> frictionCompensation - Coefficient from 0-1 which dictates how much of the calculated friction compensation is added to the PI control.\n";

    std::cout << replyString << std::endl;
    reply.addVocab(yarp::os::Vocab::encode("many"));
    reply.addString(replyString);
}

void RpcServerCallback::saveParametersToFile(yarp::os::Bottle& request, yarp::os::Bottle& reply)
{
    std::string filePath;
    if (request.size()>1) {
        filePath = request.get(1).asString();
    } else {
        filePath = "./";
    }
    filePath += "/jtc_parameters/";
    std::string fileName = jtc->simplePartName + "_params.txt";
    boost::filesystem::create_directory(filePath);
    filePath += fileName;

    std::ofstream saveFile;
    saveFile.open(filePath.c_str());
    saveFile << formatJointParamsForSave();
    saveFile.close();

    std::string replyString("Parameters saved to: " + boost::filesystem::canonical(filePath).string());
    std::cout << replyString << std::endl;
    reply.addString(replyString);
}


void RpcServerCallback::setParameterValues(yarp::os::Bottle& request, yarp::os::Bottle& reply)
{
    int joint_index = request.get(1).asInt();
    for(int i=2; i<request.size(); ++i)
    {
        std::string key = request.get(i).asString();
        if(key == "kp") {
            std::cout << "Changing param: " << "kp" <<" --- OLD: " << jtc->jointTorqueLoopGains[joint_index].kp;
            jtc->jointTorqueLoopGains[joint_index].kp = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->jointTorqueLoopGains[joint_index].kp << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].kp);
        }
        else if(key == "ki") {
            std::cout << "Changing param: " << "ki" <<" --- OLD: " << jtc->jointTorqueLoopGains[joint_index].ki;
            jtc->jointTorqueLoopGains[joint_index].ki = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->jointTorqueLoopGains[joint_index].ki << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].ki);
        }
        else if(key == "kd") {
            std::cout << "Changing param: " << "kd" <<" --- OLD: " << jtc->jointTorqueLoopGains[joint_index].kd;
            jtc->jointTorqueLoopGains[joint_index].kd = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->jointTorqueLoopGains[joint_index].kd << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].kd);
        }
        else if(key == "max_int") {
            std::cout << "Changing param: " << "max_int" <<" --- OLD: " << jtc->jointTorqueLoopGains[joint_index].max_int;
            jtc->jointTorqueLoopGains[joint_index].max_int = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->jointTorqueLoopGains[joint_index].max_int << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].max_int);
        }
        else if(key == "max_pwm") {
            std::cout << "Changing param: " << "max_pwm" <<" --- OLD: " << jtc->jointTorqueLoopGains[joint_index].max_pwm;
            jtc->jointTorqueLoopGains[joint_index].max_pwm = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->jointTorqueLoopGains[joint_index].max_pwm << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].max_pwm);
        }
        else if(key == "kv") {
            std::cout << "Changing param: " << "kv" <<" --- OLD: " << jtc->motorParameters[joint_index].kv;
            jtc->motorParameters[joint_index].kv = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->motorParameters[joint_index].kv << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kv);
        }
        else if(key == "kcp") {
            std::cout << "Changing param: " << "kcp" <<" --- OLD: " << jtc->motorParameters[joint_index].kcp;
            jtc->motorParameters[joint_index].kcp = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->motorParameters[joint_index].kcp << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kcp);
        }
        else if(key == "kcn") {
            std::cout << "Changing param: " << "kcn" <<" --- OLD: " << jtc->motorParameters[joint_index].kcn;
            jtc->motorParameters[joint_index].kcn = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->motorParameters[joint_index].kcn << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kcn);
        }
        else if(key == "coulombVelThr") {
            std::cout << "Changing param: " << "coulombVelThr" <<" --- OLD: " << jtc->motorParameters[joint_index].coulombVelThr;
            jtc->motorParameters[joint_index].coulombVelThr = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->motorParameters[joint_index].coulombVelThr << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].coulombVelThr);
        }
        else if(key == "kff") {
            std::cout << "Changing param: " << "kff" <<" --- OLD: " << jtc->motorParameters[joint_index].kff;
            jtc->motorParameters[joint_index].kff = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->motorParameters[joint_index].kff << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kff);
        }
        else if(key == "frictionCompensation") {
            std::cout << "Changing param: " << "frictionCompensation" <<" --- OLD: " << jtc->motorParameters[joint_index].frictionCompensation;
            jtc->motorParameters[joint_index].frictionCompensation = request.get(++i).asDouble();
            std::cout << "    NEW: " << jtc->motorParameters[joint_index].frictionCompensation << std::endl;
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].frictionCompensation);
        }
        else {
            reply.addString(key);
            reply.addDouble(0.0);
        }

    }
}

void RpcServerCallback::getParameterValues(yarp::os::Bottle& request, yarp::os::Bottle& reply)
{
    for(int i=1; i<request.size(); ++i)
    {
        std::string key = request.get(i).asString();
        int joint_index = request.get(++i).asInt();
        if(key == "kp") {
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].kp);
        }
        else if(key == "ki") {
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].ki);
        }
        else if(key == "kd") {
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].kd);
        }
        else if(key == "max_int") {
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].max_int);
        }
        else if(key == "max_pwm") {
            reply.addString(key);
            reply.addDouble(jtc->jointTorqueLoopGains[joint_index].max_pwm);
        }
        else if(key == "kv") {
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kv);
        }
        else if(key == "kcp") {
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kcp);
        }
        else if(key == "kcn") {
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kcn);
        }
        else if(key == "coulombVelThr") {
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].coulombVelThr);
        }
        else if(key == "kff") {
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].kff);
        }
        else if(key == "frictionCompensation") {
            reply.addString(key);
            reply.addDouble(jtc->motorParameters[joint_index].frictionCompensation);
        }
        else {
            reply.addString(key);
            reply.addDouble(0.0);
        }

    }
}

std::string RpcServerCallback::formatJointParamsForSave()
{
    std::stringstream formattedStream;

    formattedStream << "kp\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->jointTorqueLoopGains[j].kp << "\t";}
    formattedStream << ")\n";

    formattedStream << "ki\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->jointTorqueLoopGains[j].ki << "\t";}
    formattedStream << ")\n";

    formattedStream << "kd\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->jointTorqueLoopGains[j].kd << "\t";}
    formattedStream << ")\n";

    formattedStream << "maxInt\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->jointTorqueLoopGains[j].max_int << "\t";}
    formattedStream << ")\n";

    formattedStream << "maxPwm\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->jointTorqueLoopGains[j].max_pwm << "\t";}
    formattedStream << ")\n";

    formattedStream << "bemf\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->motorParameters[j].kv << "\t";}
    formattedStream << ")\n";

    formattedStream << "stictionUp\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->motorParameters[j].kcp << "\t";}
    formattedStream << ")\n";

    formattedStream << "stictionDown\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->motorParameters[j].kcn << "\t";}
    formattedStream << ")\n";

    formattedStream << "coulombVelThr\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->motorParameters[j].coulombVelThr << "\t";}
    formattedStream << ")\n";

    formattedStream << "kff\t\t\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->motorParameters[j].kff << "\t";}
    formattedStream << ")\n";

    formattedStream << "frictionCompensation\t=\t(\t";
    for(int j=0; j<jtc->axes; ++j) {formattedStream << jtc->motorParameters[j].frictionCompensation << "\t";}
    formattedStream << ")\n";

    return formattedStream.str();

}


void RpcServerCallback::addAllJointParamsToReply(int joint_index, yarp::os::Bottle& reply)
{
    reply.addString("kp");
    reply.addDouble(jtc->jointTorqueLoopGains[joint_index].kp);
    reply.addString("ki");
    reply.addDouble(jtc->jointTorqueLoopGains[joint_index].ki);
    reply.addString("kd");
    reply.addDouble(jtc->jointTorqueLoopGains[joint_index].kd);
    reply.addString("max_int");
    reply.addDouble(jtc->jointTorqueLoopGains[joint_index].max_int);
    reply.addString("max_pwm");
    reply.addDouble(jtc->jointTorqueLoopGains[joint_index].max_pwm);
    reply.addString("kv");
    reply.addDouble(jtc->motorParameters[joint_index].kv);
    reply.addString("kcp");
    reply.addDouble(jtc->motorParameters[joint_index].kcp);
    reply.addString("kcn");
    reply.addDouble(jtc->motorParameters[joint_index].kcn);
    reply.addString("coulombVelThr");
    reply.addDouble(jtc->motorParameters[joint_index].coulombVelThr);
    reply.addString("kff");
    reply.addDouble(jtc->motorParameters[joint_index].kff);
    reply.addString("frictionCompensation");
    reply.addDouble(jtc->motorParameters[joint_index].frictionCompensation);
}





using namespace std;
using namespace yarp::os;

namespace yarp {
namespace dev {

template <class T>
bool contains(std::vector<T>  &v, T  &x)
{
    return ! (std::find(v.begin(), v.end(), x) == v.end());
}

template <class T>
int findAndReturnIndex(std::vector<T>  &v, T &x)
{
    typename std::vector<T>::iterator it = std::find(v.begin(), v.end(), x);
    if(it == v.end())
    {
        return -1;
    }
    else
    {
        return std::distance(v.begin(), it);
    }
}


void JointTorqueControl::startHijackingTorqueControlIfNecessary(int j)
{
    if( !this->hijackingTorqueControl[j] )
    {
        desiredJointTorques(j) = measuredJointTorques(j);
        this->hijackingTorqueControl[j] = true;
    }
}

void JointTorqueControl::stopHijackingTorqueControlIfNecessary(int j)
{

    if( this->hijackingTorqueControl[j] )
    {
        this->hijackingTorqueControl[j] = false;
    }
}

bool JointTorqueControl::isHijackingTorqueControl(int j)
{
    return this->hijackingTorqueControl[j];
}


JointTorqueControl::JointTorqueControl():
                    PassThroughControlBoard(), RateThread(10)
{
}

JointTorqueControl::~JointTorqueControl()
{
}

bool checkVectorExistInConfiguration(yarp::os::Bottle & bot,
                                     const std::string & name,
                                     const int expected_vec_size)
{
    //std::cerr << " checkVectorExistInConfiguration(" << name
    //         << " , " << expected_vec_size << " )" << std::endl;
    //std::cerr << "bot.check(name) : " << bot.check(name) << std::endl;
    //std::cerr << "bot.find(name).isList(): " << bot.find(name).isList() << std::endl;
    //std::cerr << "bot.find(name).asList()->size() : " << bot.find(name).asList()->size()<< std::endl;

    return (bot.check(name) &&
            bot.find(name).isList() &&
            bot.find(name).asList()->size() == expected_vec_size );
}

bool JointTorqueControl::loadGains(yarp::os::Searchable& config)
{
    if( !config.check("TRQ_PIDS") )
    {
        yError("No TRQ_PIDS group find, initialization failed");
        return false;
    }

    yarp::os::Bottle & bot = config.findGroup("TRQ_PIDS");

    bool gains_ok = true;

    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"kff",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"kp",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"ki",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"maxPwm",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"maxInt",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"stictionUp",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"stictionDown",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"bemf",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"coulombVelThr",this->axes);
    gains_ok = gains_ok && checkVectorExistInConfiguration(bot,"frictionCompensation",this->axes);


    if( !gains_ok )
    {
        yError("TRQ_PIDS group is missing some information, initialization failed");
        return false;
    }

    for(int j=0; j < this->axes; j++)
    {
        jointTorqueLoopGains[j].reset();
        motorParameters[j].reset();

        jointTorqueLoopGains[j].kp        = bot.find("kp").asList()->get(j).asDouble();
        jointTorqueLoopGains[j].ki        = bot.find("ki").asList()->get(j).asDouble();
        jointTorqueLoopGains[j].max_pwm   = bot.find("maxPwm").asList()->get(j).asDouble();
        jointTorqueLoopGains[j].max_int   = bot.find("maxInt").asList()->get(j).asDouble();
        motorParameters[j].kff            = bot.find("kff").asList()->get(j).asDouble();
        motorParameters[j].kcp            = bot.find("stictionUp").asList()->get(j).asDouble();
        motorParameters[j].kcn            = bot.find("stictionDown").asList()->get(j).asDouble();
        motorParameters[j].kv             = bot.find("bemf").asList()->get(j).asDouble();
        motorParameters[j].coulombVelThr  = bot.find("coulombVelThr").asList()->get(j).asDouble();
        motorParameters[j].frictionCompensation  = bot.find("frictionCompensation").asList()->get(j).asDouble();
        if (motorParameters[j].frictionCompensation > 1 || motorParameters[j].frictionCompensation < 0) {
            motorParameters[j].frictionCompensation = 0;
            yWarning("[TRQ_PIDS] frictionCompensation parameter is outside the admissible range [0, 1]. FrictionCompensation reset to 0.0");
        }


    }

    return true;

}


bool JointTorqueControl::loadCouplingMatrix(yarp::os::Searchable& config,
                                            CouplingMatrices & coupling_matrix,
                                            std::string group_name)
{
    if( !config.check(group_name) )
    {
        coupling_matrix.reset(this->axes);
        return true;
    }
    else
    {
        yarp::os::Bottle couplings_bot = config.findGroup(group_name);

        if(!checkVectorExistInConfiguration(couplings_bot,"axesNames",this->axes) )
        {
            std::cerr << "JointTorqueControl: error in loading axesNames parameter" << std::endl;
            return false;
        }
        if(!checkVectorExistInConfiguration(couplings_bot,"motorNames",this->axes) )
        {
            std::cerr << "JointTorqueControl: error in loading motorNames parameter" << std::endl;
            return false;
        }

        std::vector<std::string> motorNames(this->axes);

        // Made axesNames a class variable for use with the params port.
        axesNames.resize(this->axes);

        for(int j=0; j < this->axes; j++)
        {
            motorNames[j] = couplings_bot.find("motorNames").asList()->get(j).asString();
            axesNames[j]  = couplings_bot.find("axesNames").asList()->get(j).asString();
        }

        for(int axis_id = 0; axis_id < (int)this->axes; axis_id++)
        {
            //search if coupling information is provided for each axis, if not return false
            std::string axis_name = axesNames[axis_id];
            if( !couplings_bot.check(axis_name) ||
                !(couplings_bot.find(axis_name).isList()) )
            {
                std::cerr << "[ERR] " << group_name << " group found, but no coupling found for joint "
                        << axis_name << ", exiting" << std::endl;
                        return false;
            }

            //Check coupling configuration is well formed
            yarp::os::Bottle * axis_coupling_bot = couplings_bot.find(axis_name).asList();

            for(int coupled_motor=0; coupled_motor < axis_coupling_bot->size(); coupled_motor++ )
            {
                if( !(axis_coupling_bot->get(coupled_motor).isList()) ||
                    !(axis_coupling_bot->get(coupled_motor).asList()->size() == 2) ||
                    !(axis_coupling_bot->get(coupled_motor).asList()->get(0).isDouble()) ||
                    !(axis_coupling_bot->get(coupled_motor).asList()->get(1).isString()) )
                {
                    std::cerr << "[ERR] " << group_name << " group found, but coupling for axis "
                        << axis_name << " is malformed" << std::endl;
                    return false;
                }

                std::string motorName = axis_coupling_bot->get(coupled_motor).asList()->get(1).asString().c_str();
                if( !contains(motorNames,motorName) )
                {
                    std::cerr << "[ERR] " << group_name << "group found, but motor name  "
                    << motorName << " is not part of the motor list" << std::endl;
                    return false;
                }

            }

            // Zero the row of the selected axis in velocity coupling matrix
            coupling_matrix.fromJointVelocitiesToMotorVelocities.row(axis_id).setZero();

            // Get non-zero coefficient of the coupling matrices
            for(int coupled_motor=0; coupled_motor < axis_coupling_bot->size(); coupled_motor++ )
            {
                double coeff = axis_coupling_bot->get(coupled_motor).asList()->get(0).asDouble();
                std::string motorName = axis_coupling_bot->get(coupled_motor).asList()->get(1).asString();
                int motorIndex = findAndReturnIndex(motorNames,motorName);

                if( motorIndex == -1 )
                {
                    return false;
                }
                coupling_matrix.fromJointVelocitiesToMotorVelocities(axis_id,motorIndex) = coeff;
            }

        }

        std::cerr << "loadCouplingMatrix DEBUG: " << std::endl;
        std::cerr << "loaded kinematic coupling matrix from group " << group_name << std::endl;
        std::cerr << coupling_matrix.fromJointVelocitiesToMotorVelocities << std::endl;
//         std::cerr << "loaded torque coupling matrix from group " << group_name << std::endl;
//         std::cerr << coupling_matrix.fromJointTorquesToMotorTorques << std::endl;

        coupling_matrix.fromJointTorquesToMotorTorques       = coupling_matrix.fromJointVelocitiesToMotorVelocities.transpose();
        coupling_matrix.fromMotorTorquesToJointTorques       = coupling_matrix.fromJointTorquesToMotorTorques.inverse();
        coupling_matrix.fromJointVelocitiesToMotorVelocities = coupling_matrix.fromJointVelocitiesToMotorVelocities.inverse();
        // Compute the torque coupling matrix



        return true;
    }

}

bool JointTorqueControl::open(yarp::os::Searchable& config)
{
    //Workaround: writeStrict option is not documented but
    //            is necessary for getting correctly working
    //            single joint streaming, hardcoding it to on
    yarp::os::Property pass_through_controlboard_config;
    pass_through_controlboard_config.fromString(config.toString());
    pass_through_controlboard_config.put("writeStrict","on");
    PassThroughControlBoard::open(pass_through_controlboard_config);
    this->getAxes(&axes);
    hijackingTorqueControl.assign(axes,false);
    controlModesBuffer.resize(axes);
    motorParameters.resize(axes);
    jointTorqueLoopGains.resize(axes);
    measuredJointPositions.resize(axes,0.0);
    measuredJointVelocities.resize(axes,0.0);
    measuredMotorVelocities.resize(axes,0.0);
    desiredJointTorques.resize(axes,0.0);
    measuredJointTorques.resize(axes,0.0);
    measuredJointPositionsTimestamps.resize(axes,0.0);
    jointControlOutput.resize(axes,0.0);
    jointTorquesError.resize(axes,0.0);
    oldJointTorquesError.resize(axes,0.0);
    derivativeJointTorquesError.resize(axes,0.0);
    integralJointTorquesError.resize(axes,0.0);
    integralState.resize(axes,0.0);
    jointControlOutputBuffer.resize(axes,0.0);

    //Start control thread
    this->setRate(config.check("controlPeriod",10,"update period of the torque control thread (ms)").asInt());

    //Load Gains configurations
    bool ret = this->loadGains(config);


    //Load coupling matrices
    couplingMatrices.reset(this->axes);
    ret = ret &&  this->loadCouplingMatrix(config,couplingMatrices,"FROM_MOTORS_TO_JOINTS_KINEMATIC_COUPLINGS");


    couplingMatricesFirmware.reset(this->axes);
    ret = ret &&  this->loadCouplingMatrix(config,couplingMatricesFirmware,"FROM_MOTORS_TO_JOINTS_KINEMATIC_COUPLINGS_FIRMWARE");

    std::cerr << "fromJointTorquesToMotorTorques matrix" << std::endl;
    std::cerr << couplingMatrices.fromJointTorquesToMotorTorques << std::endl;
    std::cerr << "fromJointVelocitiesToMotorVelocities matrix " << std::endl;
    std::cerr << couplingMatrices.fromJointVelocitiesToMotorVelocities << std::endl;

    std::cerr << "fromJointTorquesToMotorTorques matrix firmware" << std::endl;
    std::cerr << couplingMatricesFirmware.fromJointTorquesToMotorTorques << std::endl;
    std::cerr << "fromJointVelocitiesToMotorVelocities  matrix firmware" << std::endl;
    std::cerr << couplingMatricesFirmware.fromJointVelocitiesToMotorVelocities << std::endl;
    std::cerr << "fromMotorTorquesToJointTorques matrix firmware" << std::endl;
    std::cerr << couplingMatricesFirmware.fromMotorTorquesToJointTorques << std::endl;


    streamingOutput = config.check("streamingOutput");
    std::cerr << "streamingOutput = " << streamingOutput << std::endl;

    partName = config.find("name").asString();
    paramsRpcPort.open(partName+"/params:i");
    paramsRpcCallback = new RpcServerCallback(this);
    paramsRpcPort.setReader(*paramsRpcCallback);

    simplePartName = "";
    // get simple part name:
    for (std::string::reverse_iterator rit=partName.rbegin(); rit!=partName.rend(); ++rit) {
        if(*rit == '/') {
            break;
        } else {
            simplePartName.insert(simplePartName.begin(), *rit);
        }
    }

    if (streamingOutput)
    {
        ret = ret && config.check("name");
        ret = ret && config.find("name").isString();
        portForStreamingPWM.open(partName + "/output_pwms");
        portForReadingRefTorques.open(partName +"/input_torques");
    }

    if( ret )
    {
        ret = ret && this->start();
    }

    return ret;

}


bool JointTorqueControl::close()
{
    this->RateThread::stop();
    paramsRpcPort.close();
    return PassThroughControlBoard::close();
}

//CONTROL MODE
bool JointTorqueControl::setPositionMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_POSITION);
}

bool JointTorqueControl::setVelocityMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_VELOCITY);
}


bool JointTorqueControl::getControlMode(int j, int *mode)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    yarp::os::LockGuard lock(globalMutex);


    if( isHijackingTorqueControl(j) )
    {
        *mode = VOCAB_CM_TORQUE;
        return true;
    }
    else
    {
        return proxyIControlMode2->getControlMode(j,mode);
    }
}

bool JointTorqueControl::setTorqueMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_TORQUE);
}

bool JointTorqueControl::getControlModes(int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    yarp::os::LockGuard lock(globalMutex);

    bool ret = proxyIControlMode2->getControlModes(modes);
    for(int j=0; j < this->axes; j++ )
    {
        if( this->isHijackingTorqueControl(j) )
        {
            if( modes[j] == VOCAB_CM_OPENLOOP )
            {
                modes[j] = VOCAB_CM_TORQUE;
            }
        }
    }
    return ret;
}

bool JointTorqueControl::setOpenLoopMode(int j)
{
    return this->setControlMode(j,VOCAB_CM_OPENLOOP);
}

bool JointTorqueControl::setImpedancePositionMode(int j)
{
    return false;
}

bool JointTorqueControl::setImpedanceVelocityMode(int j)
{
    return false;
}

// CONTROL MODE 2
bool JointTorqueControl::getControlModes(const int n_joint, const int *joints, int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    yarp::os::LockGuard lock(globalMutex);

    bool ret = proxyIControlMode2->getControlModes(n_joint,joints,modes);

    for(int i=0; i < n_joint; i++ )
    {
        int j = joints[i];
        if( this->isHijackingTorqueControl(j) )
        {
            if( modes[i] == VOCAB_CM_OPENLOOP )
            {
                modes[i] = VOCAB_CM_TORQUE;
            }
        }
    }

    return ret;
}

bool JointTorqueControl::setControlMode(const int j, const int mode)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    yarp::os::LockGuard lock(globalMutex);

    int new_mode = mode;
    if( new_mode == VOCAB_CM_TORQUE )
    {
        if (!streamingOutput)
        {
            this->startHijackingTorqueControlIfNecessary(j);
            new_mode = VOCAB_CM_OPENLOOP;
        }
    }
    else
    {
        // The if is not really necessary
        // but it is for clearly state the semantics of
        // this operation (if a joint was in (hijacking) torque mode
        // and you switch to another mode stop hijacking
        // \todo TODO FIXME consider also the case if someone
        // else changes the control mode of a hijacked joint in the proxy
        // controlBoard: we should add a check in the update of the thread
        // that stop the hijacking if a joint control modes is not anymore
        // in openloop
        if (!streamingOutput)
        {
            this->stopHijackingTorqueControlIfNecessary(j);
        }
    }

    if (!streamingOutput)
    {
       return proxyIControlMode2->setControlMode(j, new_mode);
    }
    else
    {
       return true;
    }
}

bool JointTorqueControl::setControlModes(const int n_joint, const int *joints, int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    yarp::os::LockGuard lock(globalMutex);

    for(int i=0; i < n_joint; i++ )
    {
        int j = joints[i];
        if( modes[i] == VOCAB_CM_TORQUE )
        {
            if (!streamingOutput)
            {
                this->startHijackingTorqueControlIfNecessary(j);
                modes[i] = VOCAB_CM_OPENLOOP;
            }
        }
        else
        {
            if (!streamingOutput)
            {
                this->stopHijackingTorqueControlIfNecessary(j);
            }
        }
    }

    if (!streamingOutput)
    {
       return proxyIControlMode2->setControlModes(n_joint,joints,modes);
    }
    else
    {
        return true;
    }
}

bool JointTorqueControl::setControlModes(int *modes)
{
    if( !proxyIControlMode2 )
    {
        return false;
    }

    yarp::os::LockGuard lock(globalMutex);

    for(int j=0; j < this->axes; j++ )
    {
        if( modes[j] == VOCAB_CM_TORQUE )
        {
            if (!streamingOutput)
            {
                this->startHijackingTorqueControlIfNecessary(j);
                modes[j] = VOCAB_CM_OPENLOOP;
            }
        }
        else
        {
            if (!streamingOutput)
            {
                this->stopHijackingTorqueControlIfNecessary(j);
            }
        }
    }

    if (!streamingOutput)
    {
       return proxyIControlMode2->setControlModes(modes);
    }
    else
    {
       return true;
    }
}



// Various methods of different interfaces related to control mode switching
bool JointTorqueControl::setTorqueMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_TORQUE);
    return this->setControlModes(controlModesBuffer.data());
}


bool JointTorqueControl::setPositionMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_POSITION);
    return this->setControlModes(controlModesBuffer.data());
}

bool JointTorqueControl::setVelocityMode()
{
    controlModesBuffer.assign(axes,VOCAB_CM_VELOCITY);
    return this->setControlModes(controlModesBuffer.data());
}

//to add if necessary: setVelocityMode, setPositionDirectMode

//TORQUE CONTROL
bool JointTorqueControl::setRefTorque(int j, double t)
{
    yarp::os::LockGuard(this->globalMutex);
    desiredJointTorques[j] = t;
    return true;
}

bool JointTorqueControl::setRefTorques(const double *t)
{
    yarp::os::LockGuard(this->globalMutex);
    ::memcpy(desiredJointTorques.data(),t,this->axes*sizeof(double));
    return true;
}


bool JointTorqueControl::getRefTorque(int j, double *t)
{
    yarp::os::LockGuard(this->globalMutex);
    *t = desiredJointTorques[j];
    return true;
}

bool JointTorqueControl::getRefTorques(double *t)
{
    yarp::os::LockGuard(this->globalMutex);
    memcpy(t,desiredJointTorques.data(),this->axes*sizeof(double));
    return true;
}

bool JointTorqueControl::getTorque(int j, double *t)
{
    yarp::os::LockGuard(this->globalMutex);
    *t = measuredJointTorques[j];
    return true;
}

bool JointTorqueControl::getTorques(double *t)
{
    yarp::os::LockGuard(this->globalMutex);
    memcpy(t,measuredJointTorques.data(),this->axes*sizeof(double));
    return true;
}

bool JointTorqueControl::getBemfParam(int j, double *bemf)
{
    yarp::os::LockGuard(this->globalMutex);
    *bemf = motorParameters[j].kv;
    return true;
}

bool JointTorqueControl::setBemfParam(int j, double bemf)
{
    yarp::os::LockGuard(this->globalMutex);
    motorParameters[j].kv = bemf;
    return true;
}

bool JointTorqueControl::setTorquePid(int j, const Pid &pid)
{
    //WARNING: the PID structure mixes up motor and joint information
    //WARNING: THIS COULD MAPPING COULD CHANGE AT ANY TIME
    yarp::os::LockGuard(this->globalMutex);
    // Joint level torque loop gains
    jointTorqueLoopGains[j].kp      = pid.kp;
    jointTorqueLoopGains[j].kd      = pid.kd;
    jointTorqueLoopGains[j].ki      = pid.ki;
    jointTorqueLoopGains[j].max_int = pid.max_int;

    // Motor level friction compensation parameters
    motorParameters[j].kcp = pid.stiction_up_val;
    motorParameters[j].kcn = pid.stiction_down_val;
    motorParameters[j].kff = pid.kff;

    return true;
}

bool JointTorqueControl::getTorqueRange(int j, double *min, double *max)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorqueRanges(double *min, double *max)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::setTorquePids(const Pid *pids)
{
    yarp::os::LockGuard lock(globalMutex);

    bool ret = true;
    for(int j=0; j < this->axes; j++)
    {
        ret = ret && this->setTorquePid(j,pids[j]);
    }
    return ret;
}

bool JointTorqueControl::setTorqueErrorLimit(int j, double limit)
{
    //yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::setTorqueErrorLimits(const double *limits)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorqueError(int j, double *err)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorqueErrors(double *errs)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorquePidOutput(int j, double *out)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorquePidOutputs(double *outs)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorquePid(int j, Pid *pid)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorquePids(Pid *pids)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorqueErrorLimit(int j, double *limit)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::getTorqueErrorLimits(double *limits)
{
    yarp::os::LockGuard(this->globalMutex);
    return false;
}

bool JointTorqueControl::resetTorquePid(int j)
{
    //yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::disableTorquePid(int j)
{
    yWarning("JointTorqueControl::enableTorquePid not implemented");
    //yarp::os::LockGuard(this->controlMutex);
    return false;
}

bool JointTorqueControl::enableTorquePid(int j)
{
    //yarp::os::LockGuard(this->controlMutex);
    yWarning("JointTorqueControl::enableTorquePid not implemented");
    return false;
}

bool JointTorqueControl::setTorqueOffset(int j, double v)
{
    //yarp::os::LockGuard(this->controlMutex);
    yWarning("JointTorqueControl::setTorqueOffset not implemented");
    return false;
}

// HIJACKED CONTROL THREAD
bool JointTorqueControl::threadInit()
{
    std::cerr << "Init " << std::endl;
    return true;
}

void JointTorqueControl::readStatus()
{
    this->PassThroughControlBoard::getEncodersTimed(measuredJointPositions.data(),measuredJointPositionsTimestamps.data());
    this->PassThroughControlBoard::getEncoderSpeeds(measuredJointVelocities.data());
    this->PassThroughControlBoard::getTorques(measuredJointTorques.data());
}

/** Saturate the specified value between the specified bounds. */
inline double saturation(const double x, const double xMax, const double xMin)
{
    return x > xMax ? xMax : (x < xMin ? xMin : x);
}

double JointTorqueControl::sign(double x)
{
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

void JointTorqueControl::threadRelease()
{
    yarp::os::LockGuard guard(globalMutex);
}

inline Eigen::Map<Eigen::MatrixXd> toEigen(yarp::sig::Vector & vec)
{
    return Eigen::Map<Eigen::MatrixXd>(vec.data(),1,vec.size());
}

inline Eigen::Map<Eigen::VectorXd> toEigenVector(yarp::sig::Vector & vec)
{
    return Eigen::Map<Eigen::VectorXd>(vec.data(), vec.size());
}

void JointTorqueControl::computeOutputMotorTorques()
{
    //Compute joint level torque PID
    double dt = this->getRate() * 0.001;

    for(int j=0; j < this->axes; j++ )
    {
        JointTorqueLoopGains &gains     = jointTorqueLoopGains[j];
        jointTorquesError[j]            = measuredJointTorques[j] - desiredJointTorques[j];
        derivativeJointTorquesError[j]  = (jointTorquesError[j] - oldJointTorquesError[j]) / dt;

        oldJointTorquesError[j] = jointTorquesError[j];

        integralState[j]            = saturation(integralState[j] + gains.ki * dt * jointTorquesError(j), gains.max_int, -gains.max_int );
        jointControlOutputBuffer[j] = desiredJointTorques[j] - gains.kp*jointTorquesError[j] - gains.kd*derivativeJointTorquesError[j] - integralState[j];
    }

    toEigenVector(jointControlOutput) = couplingMatrices.fromJointTorquesToMotorTorques * toEigenVector(jointControlOutputBuffer);
    toEigenVector(measuredMotorVelocities) = couplingMatrices.fromJointVelocitiesToMotorVelocities * toEigenVector(measuredJointVelocities);

    // Evaluation of coulomb friction with smoothing close to zero velocity
    double coulombFriction;
    double coulombVelThre;
    for (int j = 0; j < this->axes; j++)
    {
        MotorParameters motorParam = motorParameters[j];
        coulombVelThre =  motorParam.coulombVelThr;
        //viscous friction compensation
        if (fabs(measuredMotorVelocities[j]) >= coulombVelThre)
        {
            coulombFriction = sign(measuredMotorVelocities[j]);
        }
        else
        {
            coulombFriction = pow(measuredMotorVelocities[j] / coulombVelThre, 3);
        }
        if (measuredMotorVelocities[j] > 0 )
        {
            coulombFriction = motorParam.kcp*coulombFriction;
        }
        else
        {
            coulombFriction = motorParam.kcn*coulombFriction;
        }
        jointControlOutput[j] = motorParam.kff*jointControlOutput[j] + motorParameters[j].frictionCompensation * (motorParam.kv*measuredMotorVelocities[j] + coulombFriction);
    }

    if (streamingOutput)
    {
        yarp::sig::Vector& output = portForStreamingPWM.prepare();
        output = jointControlOutput;
        portForStreamingPWM.write();
    }

    toEigenVector(jointControlOutput) = couplingMatricesFirmware.fromMotorTorquesToJointTorques * toEigenVector(jointControlOutput);

    bool isNaNOrInf = false;
    for(int j = 0; j < this->axes; j++)
    {

        jointControlOutput[j] = saturation(jointControlOutput[j], jointTorqueLoopGains[j].max_pwm, -jointTorqueLoopGains[j].max_pwm);
        if (isnan(jointControlOutput[j]) || isinf(jointControlOutput[j])) { //this is not std c++. Supported in C99 and C++11
            jointControlOutput[j] = 0;
            isNaNOrInf = true;
        }
    }
    if (isNaNOrInf) {
        yWarning("Inf or NaN found in control output");
    }

}

void JointTorqueControl::run()
{
    // The control mutex protect concurrent access also to the
    // hijacked control board methods, so it should protect also
    // the readStatus method
    yarp::os::LockGuard lock(globalMutex);

    //Read status (position, velocity, torque) from the controlboard
    this->readStatus();

    if (!streamingOutput)
    {
        bool true_value = true;
        if (!contains(hijackingTorqueControl,true_value) )
        {
            return;
        }
    }

    // if in streamingOutput mode read the reference torques from a port
    if( streamingOutput )
    {
        yarp::os::Bottle * ref_torques = portForReadingRefTorques.read(false);
        if( ref_torques != 0 )
        {
            for(int i = 0; i < ref_torques->size() && i < desiredJointTorques.size(); i++ )
            {
                desiredJointTorques[i] = (*ref_torques).get(i).asDouble();
            }
        }
    }

    //update output torques
    computeOutputMotorTorques();

    if(!streamingOutput)
    {

        //Send resulting output
        bool false_value = false;
        if( !contains(hijackingTorqueControl,false_value) )
        {
            this->setRefOutputs(jointControlOutput.data());
        }
        else
        {
            for(int j=0; j < this->axes; j++)
            {
                if( hijackingTorqueControl[j] )
                {
                    this->setRefOutput(j,jointControlOutput[j]);
                }
            }
        }

    }
}


}
}

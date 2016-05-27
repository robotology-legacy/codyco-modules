/*
 * Copyright (C) 2016 Fondazione Istituto Italiano di Tecnologia
 * Author: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include "VirtualAnalogRemapper.h"
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>

#include <map>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;

VirtualAnalogRemapper::VirtualAnalogRemapper()
{

}

VirtualAnalogRemapper::~VirtualAnalogRemapper()
{

}

bool VirtualAnalogRemapper::open(Searchable& config)
{
    yarp::os::Property prop;
    prop.fromString(config.toString());

    if( !( prop.check("axesNames") && prop.find("axesNames").isList() ) )
    {
        yError("VirtualAnalogRemapper: Missing required axesNames string parameter");
        return false;
    }

    Bottle * axesNamesBot = prop.find("axesNames").asList();

    m_axesNames.resize(axesNamesBot->size());
    for(int jnt=0; jnt < axesNamesBot->size(); jnt++)
    {
        m_axesNames[jnt] = axesNamesBot->get(jnt).asString();
    }

    if( !( prop.check("alwaysUpdateAllSubDevices") && prop.find("alwaysUpdateAllSubDevices").isBool() ) )
    {
        yError("VirtualAnalogRemapper: Missing required alwaysUpdateAllSubDevices bool parameter");
        return false;
    }

    m_alwaysUpdateAllSubDevices = prop.find("alwaysUpdateAllSubDevices").asBool();

    // Waiting for attach now
}

bool VirtualAnalogRemapper::attachAll(const PolyDriverList& p)
{
    // For the moment we assume that the controlboard device is the
    // same that implements the IVirtualAnalogSensor interface (this is how things
    // are implemented in embObjMotionControl) In general we at least assume that
    // all the devices that implement IVirtualAnalogSensor also implement IAxisInfo
    // to provide a name for the virtual analog axis
    std::vector<IVirtualAnalogSensor *> virtualAnalogList;
    std::vector<IAxisInfo *>            axisInfoList;
    for(size_t devIdx = 0; devIdx < (size_t)p.size(); devIdx++)
    {
        IVirtualAnalogSensor * pVirtualAnalogSens = 0;
        IAxisInfo            * pAxisInfo          = 0;
        if( p[devIdx]->poly->view(pVirtualAnalogSens) )
        {
            virtualAnalogList.push_back(pVirtualAnalogSens);
            if( !(p[devIdx]->poly->view(pAxisInfo)) )
            {
                yError() << "VirtualAnalogRemapper : attachAll error: device "
                         << p[devIdx]->key << " exposes a IVirtualAnalogSensor, but not a IAxisInfo interface,"
                         << " impossible to map the list of joint names to the IVirtualAnalogSensor interface";
                return false;

            }
            else
            {
                axisInfoList.push_back(pAxisInfo);
            }
        }
    }

    // Find the axisName ---> device , localAxis mapping
    std::map<std::string, IVirtualAnalogSensor *> axisName2virtualAnalogSensorPtr;
    std::map<std::string, IAxisInfo *> axisName2IAxisInfoPtr;
    std::map<std::string, int> axisName2localAxis;

    for(size_t devIdx = 0; devIdx < virtualAnalogList.size(); devIdx++)
    {
        int nrOfVirtualAxes = virtualAnalogList[devIdx]->getChannels();
        for(int localAxis=0; localAxis < nrOfVirtualAxes; localAxis++)
        {
            yarp::os::ConstString axisName;
            axisInfoList[devIdx]->getAxisName(localAxis,axisName);

            std::string axisNameStd = axisName.c_str();

            axisName2virtualAnalogSensorPtr[axisNameStd] = virtualAnalogList[devIdx];
            axisName2IAxisInfoPtr[axisNameStd] = axisInfoList[devIdx];
            axisName2localAxis[axisNameStd] = localAxis;
        }
    }

    // Save the axis ---> device, localAxis mapping
    remappedAxes.resize(m_axesNames.size());

    for(size_t axis = 0; axis < remappedAxes.size(); axis++)
    {
        std::string jointName = m_axesNames[axis];

        // if the name is not exposed in the VirtualAnalogSensors, raise an errpr
        if( axisName2virtualAnalogSensorPtr.find(jointName) == axisName2virtualAnalogSensorPtr.end() )
        {
            remappedAxes[axis].dev = 0;
            remappedAxes[axis].devInfo = 0;
            remappedAxes[axis].localAxis = 0;

            yError() << "VirtualAnalogRemapper: channel " << jointName << " not found in the subdevices, exiting.";
            detachAll();
            return false;
        }
        else
        {
            remappedAxes[axis].dev = axisName2virtualAnalogSensorPtr[jointName];
            remappedAxes[axis].devInfo = axisName2IAxisInfoPtr[jointName];
            remappedAxes[axis].localAxis = axisName2localAxis[jointName];
        }
    }

    // if m_alwaysUpdateAllSubDevices is set to false, then we are ok with just this information
    // (because we will use just the single axis updateMeasure) otherwise, we must check that all
    // the channels of the underling subdevices are part of m_axesNames, so we can always use the
    // updateMeasure (that is more efficient if network communication is involved)
    if( m_alwaysUpdateAllSubDevices )
    {
        // Let's count the total number of channels: if it is different from the axesNames size, give an error
        // because some channel is not covered
        int totalChannels = 0;
        for(size_t subdev = 0; subdev < virtualAnalogList.size(); subdev++)
        {
            totalChannels += virtualAnalogList[subdev]->getChannels();
        }

        if( totalChannels != m_axesNames.size() )
        {
            yError() << "VirtualAnalogRemapper: alwaysUpdateAllSubDevices is set to true, but axesNames has only " << m_axesNames.size() << " listed, while there are " << totalChannels << " in the subdevices";
            detachAll();
            return false;
        }

        // Let's fill the remappedSubdevices structure
        remappedSubdevices.resize(virtualAnalogList.size());

        for(size_t subdev = 0; subdev < virtualAnalogList.size(); subdev++)
        {
            remappedSubdevices[subdev].dev = virtualAnalogList[subdev];
            remappedSubdevices[subdev].measureBuffer.resize(remappedSubdevices[subdev].dev->getChannels());
            remappedSubdevices[subdev].local2globalIdx.resize(remappedSubdevices[subdev].dev->getChannels());

            // Let's fill the local2globalIdx vector by searching on all the vector
            for(int localIndex = 0; localIndex < remappedSubdevices[subdev].local2globalIdx.size(); localIndex++)
            {
                for(size_t globalAxis = 0; globalAxis < remappedAxes.size(); globalAxis++)
                {
                    if( (remappedAxes[globalAxis].dev == remappedSubdevices[subdev].dev) &&
                        (remappedAxes[globalAxis].localAxis == localIndex) )
                    {
                        remappedSubdevices[subdev].local2globalIdx[localIndex] = globalAxis;
                    }
                }
            }
        }
    }

    return true;
}


bool VirtualAnalogRemapper::detachAll()
{
    remappedAxes.resize(0);
    remappedSubdevices.resize(0);

    return true;
}


bool VirtualAnalogRemapper::close()
{
    detachAll();
    return true;
}

bool VirtualAnalogRemapper::updateMeasure(Vector& measure)
{
    bool ret = true;

    if( measure.size() != this->getChannels() )
    {
        yError() << "VirtualAnalogClient: updateMeasure failed : input measure has size " << measure.size() << " while the client is configured with " << this->getChannels() << " channels";
        return false;
    }

    if( m_alwaysUpdateAllSubDevices )
    {
        // use multiple axis method
        for(int subdevIdx=0; subdevIdx < remappedSubdevices.size(); subdevIdx++)
        {
            IVirtualAnalogSensor * dev = this->remappedSubdevices[subdevIdx].dev;

            // Update the measure buffer
            for(int localIndex = 0; localIndex < this->remappedSubdevices[subdevIdx].measureBuffer.size(); localIndex++)
            {
                int globalIndex = this->remappedSubdevices[subdevIdx].local2globalIdx[localIndex];
                this->remappedSubdevices[subdevIdx].measureBuffer[localIndex] = measure[globalIndex];
            }

            bool ok = dev->updateMeasure(this->remappedSubdevices[subdevIdx].measureBuffer);
            ret = ok && ret;
        }
    }
    else
    {
        // use single axis method
        for(int jnt=0; jnt < this->getChannels(); jnt++)
        {
            IVirtualAnalogSensor * dev = this->remappedAxes[jnt].dev;
            int localAxis = this->remappedAxes[jnt].localAxis;

            bool ok = dev->updateMeasure(localAxis,measure[jnt]);
            ret = ok && ret;
        }
    }

    return ret;
}

bool VirtualAnalogRemapper::updateMeasure(int ch, double& measure)
{
    if( ch < 0 || ch >= this->getChannels() )
    {
        yError() << "VirtualAnalogRemapper: updateMeasure failed : requested channel " << ch << " while the client is configured with " << this->getChannels() << " channels";
    }

    // In this case we need to use the single axis method
    IVirtualAnalogSensor * dev = this->remappedAxes[ch].dev;
    int localAxis = this->remappedAxes[ch].localAxis;

    return dev->updateMeasure(localAxis,measure);
}

int VirtualAnalogRemapper::getChannels()
{
    return this->m_axesNames.size();
}

int VirtualAnalogRemapper::getState(int ch)
{
    if( ch < 0 || ch >= this->getChannels() )
    {
        yError() << "VirtualAnalogRemapper: getState failed : requested channel " << ch << " while the client is configured with " << this->getChannels() << " channels";
    }

    // In this case we need to use the single axis method
    IVirtualAnalogSensor * dev = this->remappedAxes[ch].dev;
    int localAxis = this->remappedAxes[ch].localAxis;

    return dev->getState(localAxis);
}

bool VirtualAnalogRemapper::getAxisName(int axis, ConstString& name)
{
    if( axis < 0 || axis >= this->getChannels() )
    {
        yError() << "VirtualAnalogRemapper: getAxisName failed : requested axis " << axis << " while the remapper is configured with " << this->getChannels() << " channels";
        return false;
    }

    // In this case we need to use the single axis method
    IAxisInfo * dev = this->remappedAxes[axis].devInfo;
    int localAxis = this->remappedAxes[axis].localAxis;

    return dev->getAxisName(localAxis,name);
}

bool VirtualAnalogRemapper::getJointType(int axis, JointTypeEnum& type)
{
    if( axis < 0 || axis >= this->getChannels() )
    {
        yError() << "VirtualAnalogRemapper: getJointType failed : requested axis " << axis << " while the remapper is configured with " << this->getChannels() << " channels";
        return false;
    }

    // In this case we need to use the single axis method
    IAxisInfo * dev = this->remappedAxes[axis].devInfo;
    int localAxis = this->remappedAxes[axis].localAxis;

    return dev->getJointType(localAxis,type);
}

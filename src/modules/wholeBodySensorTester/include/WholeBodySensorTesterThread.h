/**
 * Copyright (C) 2017
 * @author: Naveen Kuppuswamy
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

#ifndef WHOLEBODYSENSORTESTERTHREAD_H
#define WHOLEBODYSENSORTESTERTHREAD_H

#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <math.h>

#include <wbi/wbiUtil.h>
#include <wbi/iWholeBodySensors.h>

namespace wbi {
    class iWholeBodySensors;
}

enum ThreadState {
    THREAD_PAUSE,
    THREAD_RUN,
    THREAD_STOP
};

// ******************** THE THREAD
class WholeBodySensorTesterThread: public yarp::os::RateThread {

private:
    yarp::os::Stamp timestamp;
    ThreadState threadState;
    bool scanAndDisplaySensorData();
    unsigned int maxAccelerometers;
    unsigned int maxGyroscopes;
public:
    wbi::iWholeBodySensors *wbs;

    bool setMaxSensors(unsigned int maxAcc, unsigned int maxGyro);
    WholeBodySensorTesterThread(int period=5);
    ~WholeBodySensorTesterThread();
    void attachWholeBodySensor(wbi::iWholeBodySensors *s);
    bool threadInit();
    bool setThreadState(const ThreadState &ts);
    void run();

    yarp::os::Semaphore mutex;
};

#endif /* end of include guard: WHOLEBODYSENSORTESTERTHREADR_H */

// -*- mode:C++ { } tab-width:4 { } c-basic-offset:4 { } indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2015 RBCS
 * Authors: Silvio Traversaro
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#ifndef _BALANCINGTEST_H_
#define _BALANCINGTEST_H_

#include <YarpTestCase.h>

#include <yarp/os/RpcClient.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlMode2.h>


/**
 * BalancingTest : simply launch the torqueBalancing and verify that
 * after ${duration} seconds the controller is still running (i.e.
 * it did not change the control mode back to position)
 *
 */
class BalancingTest : public YarpTestCase {

    yarp::os::RpcClient rpcPort;
    yarp::dev::PolyDriver monitorDD;
    yarp::dev::IControlMode2 * ctrlMode;
    yarp::dev::IEncoders     * iEncs;

    double duration;
    double pollingStep;

public:
    BalancingTest();
    virtual ~BalancingTest();

    virtual bool setup(yarp::os::Property& property);

    virtual void tearDown();

    virtual void run();
};

#endif //_BALANCINGTEST_H

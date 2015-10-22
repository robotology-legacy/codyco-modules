#include "WholeBodyEstimatorThread.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace wbi;
using namespace iDynTree;

WholeBodyEstimatorThread::WholeBodyEstimatorThread (ResourceFinder &rf, iWholeBodySensors* wbs, int period) : RateThread(period),
                                                                                                              m_rfCopy(rf),
                                                                                                              m_wbs(wbs),
                                                                                                              m_run_mutex_acquired(false)
{
    m_joint_status = 0;
}

bool WholeBodyEstimatorThread::threadInit()
{
    
    m_floatingBaseLeggedOdometry = new LeggedOdometry;
    if ( !m_floatingBaseLeggedOdometry->init(m_rfCopy, m_wbs) )
    {
        yError("[WholeBodyEstimatorThread::threadInit()] Problem initializing floating base legged odometry. Doh!");
        return false;
    } else {
        yInfo("[WholeBodyEstimatorThread::threadInit()] Floating Base Legged Odometry was succesfully initialized! Good luck with the rest ;)");
    }
    
    return true;
}

void WholeBodyEstimatorThread::run()
{
    if ( this->m_run_mutex_acquired )
    {
        yError() << "[WholeBodyEstimator::run] run_mutex already acquired at the beginning of run method.\n";
        yError() << "this could cause some problems, please report an issue at https://github.com/robotology/codyco-modules/issues/new";
    }
    
    run_mutex.lock();
    this->m_run_mutex_acquired = true;
    
    m_floatingBaseLeggedOdometry->run();
    
    this->m_run_mutex_acquired = false;
    run_mutex.unlock();
}

void WholeBodyEstimatorThread::threadRelease()
{
    m_floatingBaseLeggedOdometry->release();
    if (m_floatingBaseLeggedOdometry != NULL)
    {
        delete m_floatingBaseLeggedOdometry;
        m_floatingBaseLeggedOdometry = 0;
    }
}

//void WholeBodyEstimatorThread::readRobotStatus()
//{
//    // Last two arguments specify not retrieving timestamps and not to wait to get a sensor measurement
//    if ( !m_wbs->readSensors(wbi::SENSOR_ENCODER_POS, m_joint_status->getJointPosKDL().data.data(), NULL, false) )
//    {
//        yError("[WholeBodyEstimatorThread::readRobotStatus()] Encoders could not be read!");
//    }
//    
//    // Update yarp vectors.
//    m_joint_status->updateYarpBuffers();
//}
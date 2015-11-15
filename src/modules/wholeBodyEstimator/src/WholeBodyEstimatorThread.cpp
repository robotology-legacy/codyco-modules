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
    // Identify Estimators specified in config file
    // Find ESTIMATORS_LIST and read the list of estimators present.
    if ( !fillEstimatorsMap() )
    {
        yError("[wholeBodyEstimatorThread::threadInit()] Problem reading queried estimators from configuration file.");
        return false;
    } else {
        if ( !fillEstimatorsList() )
        {
            yError("[WholeBodyEstimatorThread::threadInit()] Problem creating estimators list");
            return false;
        } else {
            yInfo("[WholeBodyEstimatorThread::threadInit()] Estimators map and list filled correctly. ");
            return true;
        }
    }
    
    m_floatingBaseLeggedOdometry = new LeggedOdometry;
    if ( !m_floatingBaseLeggedOdometry->init(m_rfCopy, m_wbs) )
    {
        yError("[WholeBodyEstimatorThread::threadInit()] Problem initializing floating base legged odometry. Doh!");
        return false;
    } else {
        yInfo("[WholeBodyEstimatorThread::threadInit()] Floating Base Legged Odometry was succesfully initialized! Good luck with the rest ;)");
    }
    
    m_quaternionEKFInstance = new QuaternionEKF;
    if ( !m_quaternionEKFInstance->init(m_rfCopy, m_wbs) )
    {
        yError("[wholeBodyEstimatorThread::threadInit()] Problem initializing the quaternionEKF estimator.");
        return false;
    } else {
        yInfo("[wholeBodyEstimatorThread::threadInit()] QuaternionEKF instance started successfully.");
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
    
    // ###### Floating base legged odometry ######
    run_mutex.lock();
    this->m_run_mutex_acquired = true;
    
    m_floatingBaseLeggedOdometry->run();
    
    this->m_run_mutex_acquired = false;
    run_mutex.unlock();
    
    // ###### QuaternionEKF ######################
    run_mutex.lock();
    this->m_run_mutex_acquired = true;
    
    m_quaternionEKFInstance->run();
    
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

bool WholeBodyEstimatorThread::fillEstimatorsMap()
{
    // Read from rf the estimators list
    yarp::os::Bottle estimatorsListBottle = m_rfCopy.findGroup("ESTIMATORS_LIST");
    
    // Fill estimators map
    //
    // e.g. [QuaternionEKF  | 0]
    //      [LeggedOdometry | 1]
    for (int i=0; i<estimatorsListBottle.size(); i++)
    {
        yarp::os::Value val = estimatorsListBottle.pop();
        m_estimatorsMap[val.toString()] = i;
    }
    
}
bool WholeBodyEstimatorThread::fillEstimatorsList()
{
    // For each estimator create a pointer to the corresponding class and add it to the
    // vector list. Beware that the name of the classes of the estimators must correspond
    // to the ones in ESTIMATORS_LIST
    std::map<std::string, int>::iterator it;
    for (it=m_estimatorsMap.begin(); it!=m_estimatorsMap.end(); ++it)
    {
        //m_estimatorsList[ it->second ] = it->first;
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
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
        yError("[wholeBodyEstimatorThread::threadInit()] Problem1 queried estimators from configuration file.");
        return false;
    } else {
        if ( !fillEstimatorsList() )
        {
            yError("[WholeBodyEstimatorThread::threadInit()] Problem creating estimators list");
            return false;
        } else {
            yInfo("[WholeBodyEstimatorThread::threadInit()] Estimators map and list filled correctly. ");
        }
    }
    
    // Initialize each estimator
    std::vector<IEstimator*>::iterator it;
    unsigned int k = 1;
    for (it = this->m_estimatorsList.begin(); it < this->m_estimatorsList.end(); ++it)
    {
        if ( !(*it)->init(m_rfCopy, m_wbs) )
        {
            //TODO: Every derived class should have access to their name
            yError("[WholeBodyEstimatorThread::threadInit()] Estimator could not be initialized");
            return false;
        } else {
            yInfo("[WholeBodyEstimatorThread::threadInit()] Estimator %i initialized correctly", k);
        }
        k++;
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
    
    // run each estimator
    std::vector<IEstimator*>::iterator it;
    for (it = this->m_estimatorsList.begin(); it < this->m_estimatorsList.end(); ++it)
    {
        (*it)->run();
    }

    this->m_run_mutex_acquired = false;
    run_mutex.unlock();
    
}

void WholeBodyEstimatorThread::threadRelease()
{
    // Delete each estimator
    unsigned int k = 1;
    std::vector<IEstimator*>::iterator it;
    for (it = this->m_estimatorsList.begin(); it < this->m_estimatorsList.end(); ++it)
    {
        yDebug("[WholeBodyEstimatorThread::threadRelease] Calling threadRelease for estimator %i ", k);
        (*it)->release();
        k++;
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
    if (!estimatorsListBottle.isNull())
    {
        for (int i=0; i<estimatorsListBottle.size(); i++)
        {
            yarp::os::Value val = estimatorsListBottle.pop();
            m_estimatorsMap[val.toString()] = i;
        }
    } else {
        yError("[wholeBodyEstimatorThread::fillEstimatorMap()] Bottle empty!");
        return false;
    }
    
    return true;
    
}
bool WholeBodyEstimatorThread::fillEstimatorsList()
{
    // For each estimator create a pointer to the corresponding class and add it to the
    // vector list. Beware that the name of the classes of the estimators must correspond
    // to the ones in ESTIMATORS_LIST
    std::map<std::string, int>::iterator it;
    for (it=m_estimatorsMap.begin(); it!=m_estimatorsMap.end(); ++it)
    {
        // This line is pretty much doing:
        // m_estimatorList[i] = new <class-name-from-map>
        m_estimatorsList.push_back( EstimatorsFactory::create(it->first) );
    }
    
    return true;
    
}
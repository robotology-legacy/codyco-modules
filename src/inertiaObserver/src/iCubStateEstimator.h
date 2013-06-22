#ifndef ICUB_STATE_ESTIMATOR
#define ICUB_STATE_ESTIMATOR

enum iCubLimb { ICUB_HEAD = 0, ICUB_RIGHT_ARM = 1, ICUB_LEFT_ARM = 2, ICUB_TORSO = 3, ICUB_RIGHT_LEG = 4, ICUB_LEFT_LEG =  5};
enum iCubFT { ICUB_FT_RIGHT_ARM, ICUB_FT_LEFT_ARM, ICUB_FT_RIGHT_LEG, ICUB_FT_LEFT_LEG };

#include <yarp/sig/Vector.h>
#include <iCub/ctrl/adaptWinPolyEstimator.h>
#include <iCub/ctrl/filters.h>
#include <yarp/os/api.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/api.h>
#include <yarp/os/Semaphore.h>

#include <iCub/skinDynLib/skinContactList.h>

#include <iostream>
#include <map>
#include <vector>
#include <algorithm>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;
using namespace iCub::skinDynLib;

using namespace std;


typedef struct
{
    double time;
    skinContactList data;
} AWSkinPolyElement;


typedef std::deque<AWSkinPolyElement> AWSkinPolyList;


/**
 * Class for having the configuration (position, velocity, acceleration, 
 *  FT sensor) of the iCub at an arbitrary instant, given the single measurents
 * Actual estimation of velocity and acceleration is done only when requested.  
 * 
 * \note Using the same units of measurments, so degrees for angles
 */
class iCubStateEstimator
{
    friend class inertiaObserver_thread;
    private:
		//Values used in wholeBodyObserver, velocityObserver
        
        const static double still_threshold = 1.0;
        
        const static unsigned window_length = 110;
        
        map<iCubLimb,Semaphore*> posListMutex;
		map<iCubLimb,AWPolyList *> posList;
        
        map<iCubLimb,Semaphore*> pwmListMutex;
		map<iCubLimb,AWPolyList *> pwmList;
		
		map<iCubFT,AWPolyList *> FTList;
        map<iCubFT,Semaphore*> FTListMutex;
        
        AWSkinPolyList * ContactList;
        Semaphore *  ContactMutex;
		
        map<iCubLimb,double> last_ts_linEst;
        map<iCubLimb,double> last_ts_quadEst;
        
        int NVelAll, NAccAll;
        double DVelAll, DAccAll;
        
        map<iCubLimb,int> NVel;
        map<iCubLimb,Vector> DVel;
              
        map<iCubLimb,int> NAcc;
        map<iCubLimb,Vector> DAcc;
        
               
        map<iCubLimb,Vector> xVel;
        map<iCubLimb,Vector> tVel;
        
        map<iCubLimb,Vector> xAcc;
        map<iCubLimb,Vector> tAcc;
        
        
        map<iCubLimb,bool> firstTime;
        
        map<iCubLimb,Vector> winLenVel;
        map<iCubLimb,Vector> winLenAcc;
        
        map<iCubLimb,AWLinEstimator*> linEst;
        map<iCubLimb,AWQuadEstimator*> quadEst;
        
        map<iCubLimb,bool> isStillFlag;
        
        vector<iCubLimb> vectorLimbs;
        vector<iCubFT> vectorFT;
        
        bool useNonCausalEst;
        
        Vector estimate(AWPolyList & elemList, const double time, Vector & winLen, const unsigned N, const Vector & D, Vector & x, Vector & t, const unsigned int order, double & return_time);
        Vector fitCoeff(const Vector & x, Vector & y, const unsigned int i1, const unsigned int i2, const unsigned int order);
        double eval(const Vector & coeff, double x);

        
    protected: 
        void waitOnFTMutex(iCubFT ft);
        void postOnFTMutex(iCubFT ft);
        void waitOnPosMutex(iCubLimb limb);
        void postOnPosMutex(iCubLimb limb);
        void waitOnPwmMutex(iCubLimb limb);
        void postOnPwmMutex(iCubLimb limb);
        void waitOnContactMutex();
        void postOnContactMutex();
        
        
    public:
        iCubStateEstimator();
        ~iCubStateEstimator();
        
        /**
         * Get a position estimate of a limb for timestamp time
         * @param limb the limb whose position is requested
         * @param pos the reference to the vector contining the output sample
         * @param time the timestamp of the requested sample
         * @return the timestamp of the returned sample it all went well, -1.0 otherwise
         */
        double getPos(iCubLimb limb, Vector & pos,const double time);
        
        double getPwm(iCubLimb limb, Vector & pwm, const double time);
        
        /**
         * Get a velocity estimate of a limb for timestamp time
         * @param limb the limb whose velocity is requested
         * @param vel the reference to the vector contining the output sample
         * @param time the timestamp of the requested sample
         * @return the timestamp of the returned sample it all went well, -1.0 otherwise
         */
        double getVel(iCubLimb limb, Vector & vel,const double time);
        
        /**
         * Get a acceleration estimate of a limb for timestamp time
         * @param limb the limb whose acceleration is requested
         * @param acc the reference to the vector contining the output sample
         * @param time the timestamp of the requested sample
         * @return the timestamp of the returned sample it all went well, -1.0 otherwise
         */
        double getAcc(iCubLimb limb, Vector & acc,const double time);
        
        /**
         * Get the FT measure for timestamp time
         * @param ft_sensor the ft_sensor whose measure is requested
         * @param result the reference to the vector contining the output sample
         * @param time the timestamp of the requested sample, or -1.0 to
         *        get the last available sample 
         * @return the timestamp of the returned sample it all went well, -1.0 otherwise
         */
        double getFT(iCubFT ft_sensor, Vector & result, const double time = -1.0);
        
        /**
         * Get the voltage measure for timestamp time
         * @return the timestamp of the returned sample it all went well, -1.0 otherwise
         */
        //double getVoltage(iCubLimb ft_sensor, Vector & result, const double time = -1.0);
        
        
        /**
         * Get a inertial for timestamp time
         * @param inertial the reference to the vector contining the output sample
         * @param time the timestamp of the requested sample
         * @return time the timestamp of the returned sample it all went well, -1.0 otherwise
         */
        double getInertial(Vector & inertial,const double time);
        
        double getContact(skinContactList & dynList, const double time);

        
        /**
         * Submit a position sample
         * 
         * @return true if the sample was submitted, false otherwise
         */
        bool submitPos(iCubLimb limb, const Vector & pos, double time);
        
        /**
         * Submit a Force Torque sensor sample
         * 
         * @return true if the sample was submitted, false otherwise
         */
        bool submitFT(iCubFT ft, const Vector & FT, double time);
        
        
        bool submitPwm(iCubLimb limb, const Vector & pwm, double time);
        /**
         * Submit a Voltage sensor sample
         * 
         * @return true if the sample was submitted, false otherwise
         */
        //bool submitVoltage(iCubLimb limb, const Vector & voltage, double time);
        
        bool submitContact(const skinContactList & skinList, double time);
         
        /**
         * Submit a inertial sensor sample
         * 
         * @return true if the sample was submitted, false otherwise
         */
        bool submitInertial(const Vector & inertial, double time);
        
        /**
         * Return true if a limb was still for the window length 
         * (if window_length = 50 and T = 10 ms, for the last half second)
         * There is an internal flag to check if a limb is still or not. 
         * If the flag is not set (limb not still) a check on the window 
         * state is done, and if the limb is found still, the flag is raised. 
         * Then it can be put to false only by incoming data, 
         * that is checked if different or not from the available data.
         */ 
        bool isStill(iCubLimb limb);
        
        /**
         * Test it two vectors are equal, up to a numerical threshold
         * 
         */
        bool static areEqual(const Vector& a,const Vector& b,const double threshold,const int considered_joints = -1);
        
        /**
         * 
         * Reset all the internal buffers
         * 
         */
        bool reset();
        
        /**
         * Returns a pointer to a deque of the last submitted FT measurments, 
         * in inverse order (the last arrived is at the beginning of the deque).
         */
        AWPolyList * getFTdeque(iCubFT limb);
        
        /**
         * 
         */
        static bool greater_elem(AWPolyElement el1, AWPolyElement el2);
        static bool greater_elem_skin(AWSkinPolyElement el1, AWSkinPolyElement el2);

};


/**
 *
 * A class which handles the incoming position data, that on arrival are 
 * submitted to the iCubStateEstimator object
 * 
 */
class posCollector : public BufferedPort<Bottle>
{
private:
    iCubStateEstimator * p_state_estimator;
    iCubLimb limb;
    double start_ts;

    virtual void onRead(Bottle &b);

public:
    posCollector(iCubLimb _limb,iCubStateEstimator * _p_state_estimator);

    ~posCollector();
};

class pwmCollector : public BufferedPort<Bottle>
{
private:
    iCubStateEstimator * p_state_estimator;
    iCubLimb limb;
    double start_ts;

    virtual void onRead(Bottle &b);

public:
    pwmCollector(iCubLimb _limb,iCubStateEstimator * _p_state_estimator);

    ~pwmCollector();
};





class skinCollector : public BufferedPort<skinContactList>
{
private:
    iCubStateEstimator * p_state_estimator;

    double start_ts;

    virtual void onRead(skinContactList &b);

public:
    skinCollector(iCubStateEstimator * _p_state_estimator);

    ~skinCollector();
};


/**
 *
 * A class which handles the incoming wrench data, that on arrival are 
 * submitted to the iCubStateEstimator object
 * 
 */
class FTCollector : public BufferedPort<Bottle>
{
private:
    iCubStateEstimator * p_state_estimator;
    iCubFT limb;
    double start_ts;

    Filter * filter;

    virtual void onRead(Bottle &b);

public:
    FTCollector(iCubFT _limb,iCubStateEstimator * _p_state_estimator);

    ~FTCollector();
};

/**
 *
 * A class which handles the incoming inertial data, that on arrival are 
 * submitted to the iCubStateEstimator object
 * 
 */
class inertialCollector : public BufferedPort<Bottle>
{
private:
    iCubStateEstimator * p_state_estimator;
    double start_ts;


    virtual void onRead(Bottle &b);

public:
    inertialCollector(iCubStateEstimator * _p_state_estimator);

    ~inertialCollector();
};



/**
 * A class for Matrix online mean
 */

#endif /* ICUB_STATE_ESTIMATOR */

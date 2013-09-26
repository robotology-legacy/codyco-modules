#include "iCubStateEstimator.h"
#include <yarp/sig/Vector.h>

#include <yarp/math/Math.h>
#include <yarp/math/api.h>
#include <cmath>

#include <iomanip>
#include <deque>


void posCollector::onRead(Bottle &b)
{
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz=b.size();
        Vector x(sz);

        for (unsigned int i=0; i<sz; i++)
            x[i]=b.get(i).asDouble();

        double pos_time;

        // for the esteem the time stamp
        // is required. If not present within the
        // packet, the current machine time is 
        // attached to it.
        if (info.isValid())
            pos_time=info.getTime();
        else
            pos_time=Time::now();
        
        if( x.size() == 0 ) {
            std::cerr << "Time stamp of zero sized vector " << pos_time << endl;
        }
        YARP_ASSERT(x.size() > 0);
        p_state_estimator->submitPos(limb,x,pos_time);
}

posCollector::posCollector(iCubLimb _limb,iCubStateEstimator * _p_state_estimator)
{
        limb = _limb;
        p_state_estimator = _p_state_estimator;
        start_ts = Time::now();
}

posCollector::~posCollector()
{
        
}

void pwmCollector::onRead(Bottle &b)
{
       
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz=b.size();
        Vector x(sz);

        for (unsigned int i=0; i<sz; i++)
            x[i]=b.get(i).asDouble();

        double pwm_time;

        // for the esteem the time stamp
        // is required. If not present within the
        // packet, the current machine time is 
        // attached to it.
        if (info.isValid())
            pwm_time=info.getTime();
        else
            pwm_time=Time::now();
        
        if( x.size() == 0 ) {
            std::cerr << "Time stamp of zero sized vector " << pwm_time << endl;
        }
        YARP_ASSERT(x.size() > 0);
        p_state_estimator->submitPwm(limb,x,pwm_time);
        
        /*
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz=b.size()-1;
        Vector x(sz);

        for (unsigned int i=1; i<sz; i++)
            x[i]=b.get(i).asDouble();

        double pwm_time;
        
        pwm_time = b.get(0).asDouble(); 
        
        if( x.size() == 0 ) {
            std::cerr << "Time stamp of zero sized vector " << pwm_time << endl;
        }
        YARP_ASSERT(x.size() > 0);
        p_state_estimator->submitPwm(limb,x,pwm_time);*/
}

pwmCollector::pwmCollector(iCubLimb _limb,iCubStateEstimator * _p_state_estimator)
{
        limb = _limb;
        p_state_estimator = _p_state_estimator;
        start_ts = Time::now();
}

pwmCollector::~pwmCollector()
{
        
}

void FTCollector::onRead(Bottle &b)
{
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz=b.size();
        Vector x(sz);

        for (unsigned int i=0; i<sz; i++)
            x[i]=b.get(i).asDouble();

        double pos_time;

        // for the esteem the time stamp
        // is required. If not present within the
        // packet, the actual machine time is 
        // attached to it.
        if (info.isValid())
            pos_time=info.getTime();
        else
            pos_time=Time::now();

        //Minus because of the definition of the FT measurment in the sensor and in iDyn !!!
        p_state_estimator->submitFT(limb,-1*filter->filt(x),pos_time);
    
}

FTCollector::FTCollector(iCubFT _limb,iCubStateEstimator * _p_state_estimator)
{
        bool no_filter = false;
        limb = _limb;
        p_state_estimator = _p_state_estimator;
        start_ts = Time::now();
        Vector num,den;
        //filter = new FirstOrderLowPassFilter(10.0,0.01,Vector(6,0.0));
        if( no_filter ) { 
            num = Vector(1);
            den = Vector(1);
            num[0] = den[0] = 1.0;
        } else {
            //Filter (2nd order Butterworth filter with cut frequency of 10 hz
            num = Vector(3);
            den = Vector(3);
            num[0] =  0.020083;     
            num[1] = 0.040167;
            num[2] = 0.020083;
            den[0] = 1.000000;
            den[1] = -1.56102;
            den[2] =  0.64135;

        }
        filter = new Filter(num,den,Vector(6,0.0));


}

FTCollector::~FTCollector()
{
}



void skinCollector::onRead(skinContactList &b)
{
        Stamp info;
        BufferedPort<skinContactList>::getEnvelope(info);

        double pos_time;

        // for the esteem the time stamp
        // is required. If not present within the
        // packet, the actual machine time is 
        // attached to it.
        if (info.isValid())
            pos_time=info.getTime();
        else
            pos_time=Time::now();

        //Minus because of the definition of the FT measurment in the sensor and in iDyn !!!
        p_state_estimator->submitContact(b,pos_time);
    
}

skinCollector::skinCollector(iCubStateEstimator * _p_state_estimator)
{
        p_state_estimator = _p_state_estimator;
        start_ts = Time::now();
    
}

skinCollector::~skinCollector()
{
}

void inertialCollector::onRead(Bottle &b)
{
        Stamp info;
        BufferedPort<Bottle>::getEnvelope(info);

        size_t sz=b.size();
        Vector x(sz);

        for (unsigned int i=0; i<sz; i++)
            x[i]=b.get(i).asDouble();

        double pos_time;

        // for the esteem the time stamp
        // is required. If not present within the
        // packet, the actual machine time is 
        // attached to it.
        if (info.isValid())
            pos_time=info.getTime();
        else
            pos_time=Time::now();
            
        //Minus because of the definition of the FT measurment in the sensor and in iDyn !!!
        p_state_estimator->submitInertial(x,pos_time);
    
}

inertialCollector::inertialCollector(iCubStateEstimator * _p_state_estimator)
{
        p_state_estimator = _p_state_estimator;
        start_ts = Time::now();
}

inertialCollector::~inertialCollector()
{
}

iCubStateEstimator::iCubStateEstimator()
{
        useNonCausalEst = true;
        
        vectorLimbs.push_back(ICUB_HEAD);
        vectorLimbs.push_back(ICUB_RIGHT_ARM);
        vectorLimbs.push_back(ICUB_LEFT_ARM);
        vectorLimbs.push_back(ICUB_TORSO);
        vectorLimbs.push_back(ICUB_RIGHT_LEG);
        vectorLimbs.push_back(ICUB_LEFT_LEG);
        
        vectorFT.push_back(ICUB_FT_RIGHT_ARM);
        vectorFT.push_back(ICUB_FT_LEFT_ARM);
        vectorFT.push_back(ICUB_FT_RIGHT_LEG);
        vectorFT.push_back(ICUB_FT_LEFT_LEG);
        
        for(vector<iCubLimb>::size_type i = 0; i != vectorLimbs.size(); i++) {
            posList[vectorLimbs[i]] = new AWPolyList;
            posListMutex[vectorLimbs[i]] = new Semaphore;
            
            pwmList[vectorLimbs[i]] = new AWPolyList;
            pwmListMutex[vectorLimbs[i]] = new Semaphore;
            
            if( !useNonCausalEst ) {
                NVelAll = 16;
                DVelAll = 1.0;
                NAccAll = 25;
                DAccAll = 1.0;
                linEst[vectorLimbs[i]] = new AWLinEstimator(NVelAll,DVelAll);
                quadEst[vectorLimbs[i]] = new AWQuadEstimator(NAccAll,DAccAll);
                last_ts_linEst[vectorLimbs[i]] = last_ts_quadEst[vectorLimbs[i]] = -1.0;
            } else {
                NVelAll = 25;
                DVelAll = 1.0;
                NAccAll = 41;
                DAccAll = 1.0;
                NVel[vectorLimbs[i]] = NVelAll;
                //DVel[vectorLimbs[i]] = DVelAll;
                NAcc[vectorLimbs[i]] = NAccAll;
                //DAcc[vectorLimbs[i]] = DAccAll;
                winLenVel[vectorLimbs[i]] = Vector(0);
                winLenAcc[vectorLimbs[i]] = Vector(0);
                xVel[vectorLimbs[i]] = Vector(0);
                tVel[vectorLimbs[i]] = Vector(0);
                xAcc[vectorLimbs[i]] = Vector(0);
                tAcc[vectorLimbs[i]] = Vector(0);
            }
            isStillFlag[vectorLimbs[i]] = false;
            firstTime[vectorLimbs[i]] = true;
		}
        
        for(vector<iCubFT>::size_type i = 0; i != vectorFT.size(); i++) {
            FTList[vectorFT[i]] = new AWPolyList;
            FTListMutex[vectorFT[i]] = new Semaphore;
		}
        
        ContactList = new AWSkinPolyList;
        ContactMutex = new Semaphore;
        

}

iCubStateEstimator::~iCubStateEstimator()
{
    for(vector<iCubLimb>::size_type i = 0; i != vectorLimbs.size(); i++) {
        if( posList[vectorLimbs[i]] ) {
            delete posList[vectorLimbs[i]];
        }
        if( posListMutex[vectorLimbs[i]] ) {
            delete posListMutex[vectorLimbs[i]];
        }
        if( pwmList[vectorLimbs[i]] ) {
            delete pwmList[vectorLimbs[i]];
        }
        if( pwmListMutex[vectorLimbs[i]] ) {
            delete pwmListMutex[vectorLimbs[i]];
        }
        if( !useNonCausalEst ) {
            if( linEst[vectorLimbs[i]] ) {
                delete linEst[vectorLimbs[i]];
            }
            if( quadEst[vectorLimbs[i]] ) {
                delete quadEst[vectorLimbs[i]];
            }
        } 
    }
    
    for(vector<iCubFT>::size_type i = 0; i != vectorFT.size(); i++) {
        if( FTList[vectorFT[i]] ) {
            delete FTList[vectorFT[i]];
        }
        if( FTListMutex[vectorFT[i]] ) {
            delete FTListMutex[vectorFT[i]];
        }
    }
    
    if( ContactMutex ) {
        delete ContactMutex;
    }
    
    if( ContactList ) {
        delete ContactList;
    }

}
        
double iCubStateEstimator::getPos(iCubLimb limb, Vector & pos, const double time)
{
    AWPolyList * p_list;
    double return_time = -1.0;
    p_list = (posList[limb]);
    
    posListMutex[limb]->wait();
    

    if( p_list->size() <= 2 ) {
		pos = Vector(0);
		return_time = -1.0;
	} else if( time < (p_list->back()).time  ) {
		//Requested instant out of available samples
		pos = Vector(0);
		return_time = -1.0;
	} else if( time > (p_list->front()).time ) {
		/**
         *
         * \todo Return last available sample? it time is too distant? TODO 
         * 
         */
		pos = Vector(0);
		return_time = -1.0;
	} else {
        //scan list to found the two sample with respect to which the request time sample is in the middle 
        AWPolyList::iterator curr, next;
        double curr_time, next_time;
        for(curr = p_list->begin(); curr != p_list->end(); curr++ ) {
            curr_time = (*curr).time;
            if( curr != p_list->begin() ) {
                if( time <= next_time && time >= curr_time ) {
                    next = curr-1;
                    Vector deltaData,nextData,currData;
                    nextData = next->data;
                    currData = curr->data;
                    deltaData = nextData - currData;
                    pos = ((*(next)).data-curr->data)*((time-curr_time)/(next_time-curr_time)) + curr->data;
                    return_time = time;
                    break;
                }
            }
            next_time = curr_time;
        }
        if( curr == p_list->end() ) {
            pos = Vector(0);
            return_time = -1.0;
        } 
    }
    
    
    posListMutex[limb]->post();
    
    return return_time;
}

 
double iCubStateEstimator::getPwm(iCubLimb limb, Vector & pwm, const double time)
{
    AWPolyList * p_list;
    double return_time = -1.0;
    p_list = (pwmList[limb]);
    
    pwmListMutex[limb]->wait();
    

    if( p_list->size() <= 2 ) {
		pwm = Vector(0);
		return_time = -1.0;
	} else if( time < (p_list->back()).time  ) {
		//Requested instant out of available samples
		pwm = Vector(0);
		return_time = -1.0;
	} else if( time > (p_list->front()).time ) {
		/**
         *
         * \todo Return last available sample? it time is too distant? TODO 
         * 
         */
		pwm = Vector(0);
		return_time = -1.0;
	} else {
        //scan list to found the two sample with respect to which the request time sample is in the middle 
        AWPolyList::iterator curr, next;
        double curr_time, next_time;
        for(curr = p_list->begin(); curr != p_list->end(); curr++ ) {
            curr_time = (*curr).time;
            if( curr != p_list->begin() ) {
                if( time <= next_time && time >= curr_time ) {
                    next = curr-1;
                    Vector deltaData,nextData,currData;
                    nextData = next->data;
                    currData = curr->data;
                    deltaData = nextData - currData;
                    pwm = ((*(next)).data-curr->data)*((time-curr_time)/(next_time-curr_time)) + curr->data;
                    return_time = time;
                    break;
                }
            }
            next_time = curr_time;
        }
        if( curr == p_list->end() ) {
            pwm = Vector(0);
            return_time = -1.0;
        } 
    }
    
    
    pwmListMutex[limb]->post();
    
    return return_time;
}

double iCubStateEstimator::getVel(iCubLimb limb, Vector & vel, const double time)
{
    AWPolyList * p_list;
    AWLinEstimator * p_lin_est;
    double * p_last_ts;
    double return_time = -1.0;
    AWPolyList::iterator curr;

    posListMutex[limb]->wait();
    
    p_list = (posList[limb]);
    p_lin_est = linEst[limb];
    p_last_ts = &(last_ts_linEst[limb]);
    
    //cout << "getVel called, p_lin_est winLen : " << p_lin_est->getWinLen().toString() << endl;
	if( p_list->size() <= (unsigned) NVel[limb] ) {
		vel = Vector(0);
		return_time = -1.0;
	} else if( time < (p_list->back()).time  ) {
		//Requested instant out of available samples
		vel = Vector(0);
		return_time = -1.0;
	} else if( time > (p_list->front()).time ) {
		//Return last available sample? it time is too distant? TODO 
		vel = Vector(0);
		return_time = -1.0;
	} else {
        if( useNonCausalEst) {
            return_time = time;
            vel = estimate(*p_list,time,winLenVel[limb],NVel[limb],DVel[limb],xVel[limb],tVel[limb],1,return_time);
        } else {
            if( *p_last_ts == time ) {
                vel = p_lin_est->estimate();
                return_time = time;
            } else if( *p_last_ts == -1.0 || *p_last_ts > time ) {
                p_lin_est->reset();
                for(curr = p_list->end()-1; time >= curr->time; curr--) {
                    p_lin_est->feedData(*curr);
                    *p_last_ts = curr->time;
                    if(curr == p_list->begin() ) {
                        YARP_ASSERT(curr->time == time);
                        break;
                    }
                }
                vel = p_lin_est->estimate();
                return_time = *p_last_ts;
            } else if( *p_last_ts < time ) {
                for(curr = p_list->begin(); curr != p_list->end(); curr++) {
                    if( curr->time <= *p_last_ts ) {
                        YARP_ASSERT(curr != p_list->begin());
                        break;
                    }
                }
                do {
                curr--;
                if( time < curr->time ) {
                    vel = p_lin_est->estimate();
                    return_time = *p_last_ts;
                    break;
                }
                p_lin_est->feedData(*curr);
                *p_last_ts = curr->time;
                } while (curr != p_list->begin() );
            
                if( curr == p_list->begin() ) {
                    if( time == curr->time ) {
                        YARP_ASSERT(curr == p_list->begin());
                        p_lin_est->feedData(*curr);
                        *p_last_ts = curr->time;
                        vel = p_lin_est->estimate();
                        return_time = *p_last_ts;
                    }
                }
                
            }
        }
    }
    posListMutex[limb]->post();
    
    return return_time;
}

double iCubStateEstimator::getAcc(iCubLimb limb, Vector & acc,const double time)
{
	AWPolyList * p_list;
    AWQuadEstimator * p_quad_est;
    double * p_last_ts;
    double return_time = -1.0;
    AWPolyList::iterator curr;

    posListMutex[limb]->wait();

    p_list = (posList[limb]);
    p_quad_est = quadEst[limb];
    p_last_ts = &(last_ts_quadEst[limb]);
    

    //cout << "getAcc called, p_quad_est winLen : " << p_quad_est->getWinLen().toString() << endl;
    if( p_list->size() <= (unsigned) NAcc[limb] ) {
		cout << "Too little size acceleration list\n";
		acc = Vector(0);
		return_time = -1.0;
	} else if( time < (p_list->back()).time  ) {
		//Requested instant out of available samples
		//cout << "Acceleration Requested instant out of available samples\n";
		acc = Vector(0);
		return_time = -1.0;
	} else if( time > (p_list->front()).time ) {
		//Return last available sample? it time is too distant? TODO 
		//cout << "Requested acceleration too recent\n";
		acc = Vector(0);
		return_time = -1.0;
	} else {
        if( useNonCausalEst ) {
            acc = estimate(*p_list,time,winLenAcc[limb],NAcc[limb],DAcc[limb],xAcc[limb],tAcc[limb],2,return_time);
        } else {
            if( time == *p_last_ts ) {
                acc = p_quad_est->estimate();
                return_time = time;
            } else if( *p_last_ts == -1.0 || *p_last_ts > time ) {
                p_quad_est->reset();
                for(curr = p_list->end()-1; curr->time <= time; curr--) {
                    p_quad_est->feedData(*curr);
                    *p_last_ts = curr->time;
                    if(curr == p_list->begin() ) {
                        YARP_ASSERT(curr->time == time);
                        break;
                    }
                }
                acc = p_quad_est->estimate();
                return_time = *p_last_ts;
            } else if(time > *p_last_ts) {
                for(curr = p_list->begin(); curr != p_list->end(); curr++) {
                    if( curr->time <= *p_last_ts ) {
                        YARP_ASSERT(curr != p_list->begin());
                        break;
                    }
                }
                do {
                curr--;
                if( curr->time > time ) {
                    acc = p_quad_est->estimate();
                    return_time = *p_last_ts;
                    break;
                }
                p_quad_est->feedData(*curr);
                *p_last_ts = curr->time;
                } while (curr != p_list->begin() );
                if( curr == p_list->begin() ) {
                    if( time == curr->time ) {
                        YARP_ASSERT(curr == p_list->begin());
                        p_quad_est->feedData(*curr);
                        *p_last_ts = curr->time;
                        acc = p_quad_est->estimate();
                        return_time = *p_last_ts;
                    }
                }
            }
        }
    }
	
    posListMutex[limb]->post();
    return return_time;
}    

double iCubStateEstimator::getFT(iCubFT ft, Vector & result, const double time)
{
    double result_time;
    AWPolyList * p_list;
    p_list = FTList[ft];
    
    FTListMutex[ft]->wait();
    
    if( p_list->size() >= 1 && time == -1.0 ) {
		result = p_list->front().data;
		result_time = (p_list->front()).time;
	} else if( p_list->size() <= 2 ) {
		result = Vector(0);
		result_time = -1.0;
	} else if( time < (p_list->back()).time  ) {
		//Requested instant out of available samples
		result = Vector(0);
		result_time = -1.0;
	} else if( time > (p_list->front()).time ) {
		//Return last available sample? it time is too distant? TODO 
		result = Vector(0);
		result_time = -1.0;
	} else {
        //scan list to found the two sample with respect to which the request time sample is in the middle 
        AWPolyList::iterator curr;
        double curr_time, next_time;
        for(curr = p_list->begin(); curr != p_list->end(); curr++ ) {
            curr_time = curr->time;
            if( curr != p_list->begin() ) {
                if( time == next_time ) {
                    result = (*(curr-1)).data;
                    result_time = time;
                } else if( time < next_time && time > curr_time ) {
                    result = ((*(curr-1)).data-(*curr).data)*((time-curr_time)/(next_time-curr_time)) + (*curr).data;
                    result_time = time;
                }
            }
            next_time = curr_time;
        }
        result = Vector(0);
        result_time = -1.0;
    }
    
    FTListMutex[ft]->post();
    return result_time;
}


double iCubStateEstimator::getContact(skinContactList & skinList, const double time)
{
    double result_time;
    AWSkinPolyList * p_list;
    p_list = ContactList;
    
    ContactMutex->wait();
    
    if( p_list->size() >= 1 && time == -1.0 ) {
		skinList = p_list->front().data;
		result_time = (p_list->front()).time;
	} else if( p_list->size() <= 2 ) {
		//result = Vector(0);
		result_time = -1.0;
	} else if( time < (p_list->back()).time  ) {
		//Requested instant out of available samples
		//result = Vector(0);
		result_time = -1.0;
	} else if( time > (p_list->front()).time ) {
		//Return last available sample? it time is too distant? TODO 
		//result = Vector(0);
		result_time = -1.0;
	} else {
        //result = Vector(0);
        result_time = -1.0;
        //scan list to found the two sample with respect to which the request time sample is in the middle 
        AWSkinPolyList::iterator curr;
        double curr_time, next_time;
        for(curr = p_list->begin(); curr != p_list->end(); curr++ ) {
            curr_time = curr->time;
            if( curr != p_list->begin() ) {
                if( time == next_time ) {
                    skinList = (*(curr-1)).data;
                    result_time = time;
                } else if( time < next_time && time > curr_time ) {
                    if( abs(time-next_time) < abs(time-curr_time) ) {
                        //result = ((*(curr-1)).data-(*curr).data)*((time-curr_time)/(next_time-curr_time)) + (*curr).data;
                        skinList =  (*(curr-1)).data;
                        result_time = next_time;
                    } else {
                        skinList = curr->data;
                        result_time = curr_time;
                    }
                }
            }
            next_time = curr_time;
        }

    }
    
    ContactMutex->post();
    return result_time;
}


double getInertial(Vector & inertial, double time)
{
    inertial = Vector(0);
    return -1.0;
}


bool iCubStateEstimator::submitPos(iCubLimb limb, const Vector & pos, double time)
{
    AWPolyElement el;
    int considered_joints = -1;
    YARP_ASSERT(pos.size() > 0);

    if( limb == ICUB_RIGHT_ARM || limb == ICUB_LEFT_ARM ) {
        considered_joints = 7;
    }
    
    el.data = pos;
    el.time = time;
    
    if( useNonCausalEst && firstTime[limb] ) {
        DVel[limb] = Vector(pos.size(),DVelAll);
        DAcc[limb] = Vector(pos.size(),DAccAll);
        winLenVel[limb].resize(pos.size(),NVel[limb]);
        winLenAcc[limb].resize(pos.size(),NAcc[limb]);
        xVel[limb].resize(NVel[limb]);
        tVel[limb].resize(NAcc[limb]);
        xAcc[limb].resize(NVel[limb]);
        tAcc[limb].resize(NAcc[limb]);
        firstTime[limb] = false;
    }
    
    if( firstTime[limb] ) firstTime[limb] = false;
    
    posListMutex[limb]->wait();
     
    //check to keep the list in descending ordered 
    if( posList[limb]->size() == 0 || el.time >= posList[limb]->front().time ) {
        //standard case
        posList[limb]->push_front(el);
    } else {
        //an ordered insert would be more efficient, but is a very rare possibility
        //so it is easier to do in this way
        posList[limb]->push_front(el);
        sort(posList[limb]->begin(),posList[limb]->end(),greater_elem);
    }    
    
    
    if( posList[limb]->size() > window_length ) {
        posList[limb]->pop_back();
    }
    //While submitting, control if the limb is still still
    if( isStillFlag[limb] ) {
        YARP_ASSERT( el.data.size() == (posList[limb]->back()).data.size() );
        if( !areEqual(el.data,(posList[limb]->back()).data,still_threshold,considered_joints) ) {
            isStillFlag[limb] = false;
        }
    }
    
    posListMutex[limb]->post();
    
    return true;
}


bool iCubStateEstimator::submitPwm(iCubLimb limb, const Vector & pwm, double time)
{
    AWPolyElement el;
    int considered_joints = -1;
    YARP_ASSERT(pwm.size() > 0);

    if( limb == ICUB_RIGHT_ARM || limb == ICUB_LEFT_ARM ) {
        considered_joints = 7;
    }
    
    el.data = pwm;
    el.time = time;
    
    pwmListMutex[limb]->wait();
     
    //check to keep the list in descending ordered 
    if( pwmList[limb]->size() == 0 || el.time >= pwmList[limb]->front().time ) {
        //standard case
        pwmList[limb]->push_front(el);
    } else {
        //an ordered insert would be more efficient, but is a very rare possibility
        //so it is easier to do in this way
        pwmList[limb]->push_front(el);
        sort(pwmList[limb]->begin(),pwmList[limb]->end(),greater_elem);
    }    
    
    
    if( pwmList[limb]->size() > window_length ) {
        pwmList[limb]->pop_back();
    }
    
    pwmListMutex[limb]->post();
    
    return true;
}


bool iCubStateEstimator::submitFT(iCubFT ft, const Vector & FT, double time)
{
    AWPolyElement el;
    el.data = FT;
    el.time = time;
    
    FTListMutex[ft]->wait();
    
    //check to keep the list in descending ordered 
    if( FTList[ft]->size() == 0 || el.time >= FTList[ft]->front().time ) {
        //standard case
        FTList[ft]->push_front(el);
    } else {
        //an ordered insert would be more efficient, but is a very rare possibility
        //so it is easier to do in this way
        FTList[ft]->push_front(el);
        sort(FTList[ft]->begin(),FTList[ft]->end(),greater_elem);
    }
    
    if( FTList[ft]->size() > window_length ) {
        FTList[ft]->pop_back();
    }
    
    FTListMutex[ft]->post();
    
    return true;
}

bool iCubStateEstimator::submitContact(const skinContactList & skinList, double time)
{
    AWSkinPolyElement el;
    el.data = skinList;
    el.time = time;
    
    ContactMutex->wait();
    
    //check to keep the list in descending ordered 
    if( ContactList->size() == 0 || el.time >= ContactList->front().time ) {
        //standard case
        ContactList->push_front(el);
    } else {
        //an ordered insert would be more efficient, but is a very rare possibility
        //so it is easier to do in this way
        ContactList->push_front(el);
        sort(ContactList->begin(),ContactList->end(),greater_elem_skin);
    }
    
    if( ContactList->size() > window_length ) {
        ContactList->pop_back();
    }
    
    ContactMutex->post();
    
    return true;
}

bool iCubStateEstimator::submitInertial(const Vector & inertial, double time)
{
    return false;
}


AWPolyList * iCubStateEstimator::getFTdeque(iCubFT limb)
{
	return FTList[limb];
}

AWPolyList * iCubStateEstimator::getPWMdeque(iCubLimb limb)
{
    return pwmList[limb];
}


bool iCubStateEstimator::reset()
{
    for(vector<iCubLimb>::size_type i = 0; i != vectorLimbs.size(); i++) {
            posListMutex[vectorLimbs[i]]->wait();
            posList[vectorLimbs[i]]->clear();
            posListMutex[vectorLimbs[i]]->post();
            
            pwmListMutex[vectorLimbs[i]]->wait();
            pwmList[vectorLimbs[i]]->clear();
            pwmListMutex[vectorLimbs[i]]->post();
            
            if( !useNonCausalEst ) {
                linEst[vectorLimbs[i]]->reset();
                quadEst[vectorLimbs[i]]->reset();
                last_ts_linEst[vectorLimbs[i]] = last_ts_quadEst[vectorLimbs[i]] = -1.0;
            } else {
                int NVelAll = 25;
                int DVelAll = 1.0;
                int NAccAll = 41;
                int DAccAll = 1.0;
                NVel[vectorLimbs[i]] = NVelAll;
                DVel[vectorLimbs[i]] = DVelAll;
                NAcc[vectorLimbs[i]] = NAccAll;
                DAcc[vectorLimbs[i]] = DAccAll;
                winLenVel[vectorLimbs[i]] = Vector(0);
                winLenAcc[vectorLimbs[i]] = Vector(0);
                xVel[vectorLimbs[i]] = Vector(0);
                tVel[vectorLimbs[i]] = Vector(0);
                xAcc[vectorLimbs[i]] = Vector(0);
                tAcc[vectorLimbs[i]] = Vector(0);
            }
            firstTime[vectorLimbs[i]] = true;
            isStillFlag[vectorLimbs[i]] = false;
    }
    for(vector<iCubFT>::size_type i = 0; i != vectorFT.size(); i++) {
        FTListMutex[vectorFT[i]]->wait();
        FTList[vectorFT[i]]->clear();
        FTListMutex[vectorFT[i]]->post();

    }

    return true;
}

bool iCubStateEstimator::isStill(iCubLimb limb)
{
    AWPolyList * p_list;
    
    bool return_value;
    
    int considered_joints = -1;
    
   if( limb == ICUB_RIGHT_ARM || limb == ICUB_LEFT_ARM ) {
        considered_joints = 7;
    }

    posListMutex[limb]->wait();
    
    p_list = (posList[limb]);
    //Wait for some time before declaring arm still
    if( p_list->size() < window_length ) {
        return_value = false;
    } else if( isStillFlag[limb] ) {
        //fprintf(stderr,"isStill returned true\n");
        return_value = true;
    } else {
        YARP_ASSERT( (p_list->front()).data.size() == posList[limb]->back().data.size() );
        if( !areEqual((p_list->front()).data,(p_list->back()).data,still_threshold,considered_joints) ) {
            //fprintf(stderr,"isStill returned false 1\n");
            return_value = false;
        } else {
            AWPolyList::iterator curr;
            for(curr = p_list->begin(); curr != p_list->end(); curr++ ) {
                YARP_ASSERT(curr->data.size() > 0 );
                if( curr->data.size() != (p_list->front()).data.size() ) {
                    std::cerr << "curr->data.size() " << curr->data.size() << std::endl;
                    std::cerr << "curr->time " << curr->time << std::endl;
                }
                YARP_ASSERT( curr->data.size() == (p_list->front()).data.size() );
                if( !areEqual((p_list->front()).data,curr->data,still_threshold,considered_joints ) ) {
                    //fprintf(stderr,"isStill returned false 2\n");
                    return_value = false;
                    break;
                }
            }
            //if now curr == p_list->end(), this mean that no different value was found in the list
            if( curr == p_list->end() ) {
                isStillFlag[limb] = true;
                //fprintf(stderr,"isStill returned true\n");
                return_value = true;
            }
        }
    }
    
    posListMutex[limb]->post();

    

    return return_value;
}

bool iCubStateEstimator::areEqual(const Vector& a,const Vector& b,const double threshold,const int considered_joints)
{
    if(a.size() != b.size() ) {
        std::cerr << "areEqual: size mismatch a " << a.size() << " " << b.size() << std::endl;
        YARP_ASSERT(false);
    }
    Vector delta = a - b;
    if( considered_joints <= 0 ) {
        for(size_t i=0; i < delta.size(); i++ ) {
            if( abs(delta[i]) > threshold ) {
                return false;
            }
        }
    } else {
        for(int i=0; i < (int)delta.size() && i < considered_joints; i++ ) {
            if( abs(delta[i]) > threshold ) {
                return false;
            }
        }
    }
    return true;
}

bool iCubStateEstimator::greater_elem(AWPolyElement el1, AWPolyElement el2) {
    return (el1.time < el2.time);
}

bool iCubStateEstimator::greater_elem_skin(AWSkinPolyElement el1, AWSkinPolyElement el2) {
    return (el1.time < el2.time);
}

void iCubStateEstimator::waitOnFTMutex(iCubFT ft) {
    if( FTListMutex[ft] ) {
        FTListMutex[ft]->wait();
    } 
}

void iCubStateEstimator::postOnFTMutex(iCubFT ft) {
    if( FTListMutex[ft] ) {
        FTListMutex[ft]->post();
    } 
}

void iCubStateEstimator::waitOnPosMutex(iCubLimb limb) {
    if( posListMutex[limb] ) {
        posListMutex[limb]->wait();
    } 
}

void iCubStateEstimator::postOnPosMutex(iCubLimb limb) {
    if( posListMutex[limb] ) {
        posListMutex[limb]->post();
    } 
}

void iCubStateEstimator::waitOnPwmMutex(iCubLimb limb) {
    if( pwmListMutex[limb] ) {
        pwmListMutex[limb]->wait();
    } 
}

void iCubStateEstimator::postOnPwmMutex(iCubLimb limb) {
    if( pwmListMutex[limb] ) {
        pwmListMutex[limb]->post();
    } 
}


void iCubStateEstimator::waitOnContactMutex() {
    if( ContactMutex ) {
        ContactMutex->wait();
    } 
}

void iCubStateEstimator::postOnContactMutex() {
    if( ContactMutex ) {
        ContactMutex->post();
    } 
}


Vector iCubStateEstimator::estimate(AWPolyList & elemList, const double time, Vector & winLen, const unsigned N, const Vector & D, Vector & x, Vector & t, const unsigned int order, double & return_time)
{
    YARP_ASSERT(order == 1 || order == 2);
    //Search for the closest samples 
    AWPolyList::iterator curr, next;
    AWPolyList::iterator central_sample;
    double curr_time, next_time;
    yarp::sig::Vector coeff;
    int i;
    unsigned int central_sample_index;
    
    unsigned int dim;
    dim = elemList[0].data.size();
    
    for(curr = (&elemList)->begin(), i = 0; curr != elemList.end(); curr++ , i++) {

        curr_time = (*curr).time;
        if( curr != elemList.begin() ) {
            if( time <= next_time && time >= curr_time ) {
                //time between two samples
                if( (next_time - time) > (time - curr_time) ) {
                    //the closesest sample is the oldest
                    central_sample = curr;
                    central_sample_index = i;
                    return_time = curr_time;
                } else {
                    //the closesest sample it the newer one
                    central_sample = curr-1;
                    central_sample_index = i-1;
                    return_time = next_time;
                }
                break;
            }
        }
        next_time = curr_time;
    }
    
    
    
    YARP_ASSERT(N % 2 == 1);
    
    unsigned int lateral_samples;
    lateral_samples = N/2;
    
    if( central_sample_index < lateral_samples || (elemList.size()-central_sample_index) < lateral_samples ) {
        //Not sufficient samples arount the central samples
        return_time = -1.0;
        return Vector(0);
    }
    
    t.resize(N);
    x.resize(N);
    
    for (unsigned int j=0; j<N; j++)
        t[j]=elemList[central_sample_index-lateral_samples+j].time-elemList[central_sample_index-lateral_samples].time;
    
    Vector esteem(dim);
    
    for (unsigned int i=0; i<dim; i++)
    {
        // retrieve the data vector
        for (unsigned int j=0; j<N; j++)
            x[j]=elemList[central_sample_index-lateral_samples+j].data[i];

        // change the window length of two units, back and forth
        unsigned int n1=(unsigned int)((winLen[i]>(1+2))?(winLen[i]-2):(1+2));
        
        unsigned int n2=(unsigned int)((winLen[i]<N)?(winLen[i]+2):N);

        //std::cout << "AWPolyEstimator: order " << order << " i " << i <<  " winLen[i] " << winLen[i] <<  " n1 " << n1 << " n2 " << n2 << std::endl;
        
        // cycle upon all possibile window's length
        for (unsigned int n=n1; n<=n2; n = n+2)
        {
            YARP_ASSERT(n%2 == 1);
            int i1,i2,n_lateral;
            n_lateral = n/2;
            i1 = N/2 - n/2;
            i2 = N/2 + n/2;
            // find the regressor's coefficients
            coeff=fitCoeff(t,x,i1,i2,order);
            
            YARP_ASSERT(coeff.size() == order+1);
            
            bool _stop=false;

            // test the regressor upon all the elements
            // belonging to the actual window
            for (unsigned int k=(unsigned int)i1; k<=(unsigned int)i2; k++)
            {
                if (fabs(x[k]-eval(coeff,t[k]))>D(i))
                {
                    // exit if the max deviation is not verified
                    _stop=true;
                    break;
                }
            }

            // set the new window's length in case of
            // crossing of max deviation threshold
            if (_stop)
            {
                winLen[i]=n;
                break;
            }
        }

        if( order == 1 ) {
            esteem[i]= coeff[1];
        } else {
            esteem[i] = 2.0*coeff[2];
        }
    }
    return esteem;
}

Vector iCubStateEstimator::fitCoeff(const Vector & x, Vector & y, const unsigned int i1, const unsigned int i2, const unsigned int order)
{
    YARP_ASSERT( order == 1 || order == 2);
    unsigned int M =i2-i1+1;

    
    
    if( order == 1 ) {
        
        
        double sum_xi  =0.0;
        double sum_xixi=0.0;
        double sum_yi  =0.0;
        double sum_xiyi=0.0;
    
        for (unsigned int i=i1; i<=i2; i++)
        {
            sum_xi  +=x[i];
            sum_xixi+=x[i]*x[i];
            sum_yi  +=y[i];
            sum_xiyi+=x[i]*y[i];
        }
    
        double den=M*sum_xixi-sum_xi*sum_xi;
    
        Vector c(2);
    
        // the bias
        c[0]=(sum_yi*sum_xixi-sum_xi*sum_xiyi) / den;
    
        // the linear coefficient
        c[1]=(M*sum_xiyi-sum_xi*sum_yi) / den;
    
        return c;
    } else {
        
        Matrix R(M,order+1);
        Vector _y(M);
    
        for (unsigned int i=i1; i<=i2; i++)
        {
            double _x=x[i];
    
            R(i-i1,0)=1.0;
    
            for (unsigned int j=1; j<=order; j++)
            {
                R(i-i1,j)=_x;
                _x*=_x;
            }
    
            _y[i-i1]=y[i];
        }
    
        //Calculating plain pseudo inverse of R
        Matrix RTR = R.transposed()*R;
        
        //Hard coded 3x3 symmetric matrix inverse
        double detRTR = 	  RTR(0,0)*RTR(1,1)*RTR(2,2)
                            + RTR(1,0)*RTR(2,1)*RTR(0,2)
                            + RTR(2,0)*RTR(0,1)*RTR(1,2)
                            - RTR(0,0)*RTR(2,1)*RTR(1,2)
                            - RTR(2,0)*RTR(1,1)*RTR(0,2)
                            - RTR(1,0)*RTR(0,1)*RTR(2,2);
        
        Matrix invRTR = zeros(3,3);
        invRTR(0,0) = RTR(2,2)*RTR(1,1) -  RTR(2,1)*RTR(1,2);
        invRTR(0,1) = RTR(2,1)*RTR(0,2) -  RTR(2,2)*RTR(0,1);
        invRTR(0,2) = RTR(1,2)*RTR(0,1) -  RTR(1,1)*RTR(0,2);
        invRTR(1,0) = invRTR(0,1);
        invRTR(1,1) = RTR(2,2)*RTR(0,0) -  RTR(2,0)*RTR(0,2);
        invRTR(1,2) = RTR(1,0)*RTR(0,2) -  RTR(1,2)*RTR(0,0);
        invRTR(2,0) = invRTR(0,2);
        invRTR(2,1) = invRTR(1,2);
        invRTR(2,2) = RTR(1,1)*RTR(0,0) -  RTR(1,0)*RTR(0,1);
        
        invRTR = (1/detRTR)*invRTR;
        
        Vector ret = invRTR*(R.transposed()*_y);
        
        return ret;
    }
}

double iCubStateEstimator::eval(const Vector & coeff, double x)
{
    double y=coeff[0];
    
    unsigned int order = coeff.size()-1;

    for (unsigned int i=1; i<=order; i++)
    {
        y+=coeff[i]*x;
        x*=x;
    }

    return y;
}

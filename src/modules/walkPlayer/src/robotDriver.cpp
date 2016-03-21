#include "robotDriver.h"

using namespace yarp::math;
using namespace yarp::sig;
using namespace yarp::os;
using namespace yarp::dev;
using namespace std;

robotDriver::robotDriver() {
    drvOptions_ll.clear();
    drvOptions_rl.clear();
    drvOptions_to.clear();
    drv_ll  = 0;
    drv_rl  = 0;
    drv_to  = 0;
    ipos_ll = 0;
    ipid_ll = 0;
    iimp_ll = 0;
    iint_ll = 0;
    ienc_ll = 0;
    ipos_rl = 0;
    ipid_rl = 0;
    iimp_rl = 0;
    iint_rl = 0;
    ienc_rl = 0;
    ipos_to = 0;
    ipid_to = 0;
    ienc_to = 0;
    icmd_ll = 0;
    icmd_rl = 0;
    icmd_to = 0;
    idir_ll = 0;
    idir_rl = 0;
    verbose = 1;
    connected=false;
    iCub::iDyn::version_tag tag;
    icub_dyn = new iCub::iDyn::iCubWholeBody(tag);
}

yarp::sig::Matrix robotDriver::compute_transformations (actionStruct act) {
    for (int i=0; i<6; i++) {
        icub_dyn->lowerTorso->left->setAng(i,act.q_left_leg[i]);
        icub_dyn->lowerTorso->right->setAng(i,act.q_right_leg[i]);
        icub_dyn->lowerTorso->left->setDAng(i,0.0);
        icub_dyn->lowerTorso->right->setDAng(i,0.0);
        icub_dyn->lowerTorso->left->setD2Ang(i,0.0);
        icub_dyn->lowerTorso->right->setD2Ang(i,0.0);
    }
    for (int i=0; i<3; i++) {
        icub_dyn->lowerTorso->up->setAng(i, act.q_torso[i]);
        icub_dyn->lowerTorso->up->setD2Ang(i,0.0);
    }

    yarp::sig::Matrix Hl= icub_dyn->lowerTorso->HLeft  * icub_dyn->lowerTorso->left->getH();
    yarp::sig::Matrix Hr= icub_dyn->lowerTorso->HRight * icub_dyn->lowerTorso->right->getH();

    //cout << endl<< "HL to string:" << endl << Hl.toString() << endl;

    yarp::sig::Matrix Hlr = SE3inv(icub_dyn->lowerTorso->left->getH()) * SE3inv(icub_dyn->lowerTorso->HLeft) * icub_dyn->lowerTorso->HRight * icub_dyn->lowerTorso->right->getH();

    //cout << endl<< "HLR to string:" << endl << Hlr.toString() << endl;

    return Hlr;
}

bool robotDriver::configure(const Property &copt) {
    bool ret=true;
    Property &options=const_cast<Property &> (copt);

    drvOptions_ll.put("device","remote_controlboard");
    drvOptions_rl.put("device","remote_controlboard");
    drvOptions_to.put("device","remote_controlboard");

    string remote;
    string local;
    remote = string("/") + string(options.find("robot").asString()) + string("/left_leg");
    local  = string("/") + string("walkPlayer") + string("/left_leg");
    drvOptions_ll.put("remote",remote.c_str());
    drvOptions_ll.put("local",local.c_str());
    remote = string("/") + string(options.find("robot").asString()) + string("/right_leg");
    local  = string("/") + string("walkPlayer") + string("/right_leg");
    drvOptions_rl.put("remote",remote.c_str());
    drvOptions_rl.put("local",local.c_str());
    remote = string("/") + string(options.find("robot").asString()) + string("/torso");
    local  = string("/") + string("walkPlayer") + string("/torso");
    drvOptions_to.put("remote",remote.c_str());
    drvOptions_to.put("local",local.c_str());

    if (verbose)
    {
        cerr << "right leg driver options:\n" << drvOptions_rl.toString().c_str() << endl;
        cerr << "left  leg driver options:\n" << drvOptions_ll.toString().c_str() << endl;
        cerr << "torso     driver options:\n" << drvOptions_to.toString().c_str() << endl;
    }

    return ret;
    }

bool robotDriver::init() {
    //return true; //@@@@@

    this->drv_ll=new PolyDriver(this->drvOptions_ll);
    this->drv_rl=new PolyDriver(this->drvOptions_rl);
    this->drv_to=new PolyDriver(this->drvOptions_to);

    if (this->drv_ll->isValid() && this->drv_rl->isValid() && this->drv_to->isValid())
        connected = this->drv_ll->view(ipos_ll) && this->drv_ll->view(ienc_ll) && this->drv_ll->view(ipid_ll) && this->drv_ll->view(iimp_ll) && this->drv_ll->view(iint_ll) && this->drv_ll->view(icmd_ll) && this->drv_ll->view(idir_ll) &&
                    this->drv_rl->view(ipos_rl) && this->drv_rl->view(ienc_rl) && this->drv_rl->view(ipid_rl) && this->drv_rl->view(iimp_rl) && this->drv_rl->view(iint_rl) &&this->drv_rl->view(icmd_rl) && this->drv_rl->view(idir_rl) &&
                    this->drv_to->view(ipos_to) && this->drv_to->view(ienc_to) && this->drv_to->view(ipid_to) && this->drv_to->view(icmd_to) && this->drv_to->view(idir_to);
    else
        connected=false;

    if (!connected)
    {
        if (this->drv_ll)
        {
            delete this->drv_ll;
            this->drv_ll=0;
        }
        if (this->drv_rl)
        {
            delete this->drv_rl;
            this->drv_rl=0;
        }
        if (this->drv_to)
        {
            delete this->drv_to;
            this->drv_to=0;
        }
    }



    //set the intial reference speeds
    double speeds_arm[6];
    double speeds_to[3];
    InteractionModeEnum tempMode[6] = {VOCAB_IM_STIFF,VOCAB_IM_STIFF,VOCAB_IM_STIFF,VOCAB_IM_STIFF,VOCAB_IM_STIFF,VOCAB_IM_STIFF};

    for (int i=0; i<6; i++)speeds_arm[i] = 20.0;
    for (int i=0; i<3; i++) speeds_to[i] = 20.0;

    //FIXME: The following for loop is just for testing joint by joint what happens when setInteractionMode() is used. Delete or comment this block and uncomment line 148  to do it as previously done in a batched way for the left leg.
    for (unsigned int i=0; i<6; i++) {
        icmd_ll->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
        this->iint_ll->setInteractionMode(i, tempMode[i]);
    }
    
//    this->iint_ll->setInteractionModes(tempMode);
    this->iint_rl->setInteractionModes(tempMode);

    this->ipos_ll->setRefSpeeds(speeds_arm);
    this->ipos_rl->setRefSpeeds(speeds_arm);
    this->ipos_to->setRefSpeeds(speeds_to);


    return connected;
}

robotDriver::~robotDriver() {
    if (this->drv_ll)
    {
        delete this->drv_ll;
        this->drv_ll=0;
    }
    if (this->drv_rl)
    {
        delete this->drv_rl;
        this->drv_rl=0;
    }
    if (this->drv_to)
    {
        delete this->drv_to;
        this->drv_to=0;
    }
}
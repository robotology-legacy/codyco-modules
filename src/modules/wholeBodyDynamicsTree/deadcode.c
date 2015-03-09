 class wholeBodyDynamicsStatesInterface : public wbi::iWholeBodyStates
    {
    protected:
        yarpWholeBodySensors                *sensors;       // interface to access the robot sensors
        yarpWholeBodyDynamicsEstimator      *estimator;     // estimation thread
        yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>                *skin_contacts_port; //port to the skin contacts
        wbi::IDList                    emptyList;      ///< empty list of IDs to return in case of error
        //double                      estWind;      // time window for the estimation

        virtual bool lockAndReadSensor(const wbi::SensorType st, int numeric_id, double *data, double time, bool blocking);
        virtual bool lockAndReadSensors(const wbi::SensorType st, double *data, double time, bool blocking);
        virtual bool lockAndAddSensor(const wbi::SensorType st, const wbi::ID &sid);
        virtual int lockAndAddSensors(const wbi::SensorType st, const wbi::IDList &sids);
        virtual bool lockAndRemoveSensor(const wbi::SensorType st, const wbi::ID &sid);
        virtual wbi::IDList lockAndGetSensorList(const wbi::SensorType st);
        virtual int lockAndGetSensorNumber(const wbi::SensorType st);

        bool lockAndReadExternalForces(iCub::skinDynLib::skinContactList & external_forces_list);
        bool lockAndReadExternalForceTorque();


        /** Get the velocity of the specified motor. */
        bool getMotorVel(const int numeric_id, double *data, double time, bool blocking);
        /** Get the velocities of all the robot motors. */
        bool getMotorVel(double *data, double time, bool blocking);


    public:
        // *** CONSTRUCTORS ***
        wholeBodyDynamicsStatesInterface(const char* _name,
                                         int estimator_period,
                                         yarp::os::Property & _wbi_yarp_conf);


        ~wholeBodyDynamicsStatesInterface(){ close(); }

        virtual bool init();
        virtual bool close();

        /** Add the specified estimate so that it can be read.
         * @param st Type of estimate.
         * @param sid Id of the estimate.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual bool addEstimate(const wbi::EstimateType st, const wbi::ID &sid);

        /** Add the specified estimates so that they can be read.
         * @param st Type of estimates.
         * @param sids Ids of the estimates.
         * @return True if the estimate has been added, false otherwise (e.g. the estimate has been already added).
         */
        virtual int addEstimates(const wbi::EstimateType st, const wbi::IDList &sids);

        /** Remove the specified estimate.
         * @param st Type of the estimate to remove.
         * @param j Id of the estimate to remove.
         * @return True if the estimate has been removed, false otherwise.
         */
        virtual bool removeEstimate(const wbi::EstimateType st, const wbi::ID &sid);

        /** Get a copy of the estimate list of the specified estimate type.
         * @param st Type of estimate.
         * @return A copy of the estimate list. */
        virtual const wbi::IDList& getEstimateList(const wbi::EstimateType st);

        /** Get the number of estimates of the specified type.
         * @return The number of estimates of the specified type. */
        virtual int getEstimateNumber(const wbi::EstimateType st);

        /** Get the estimate of the specified quantity at the specified time.
         * @param et Type of estimate to get.
         * @param sid Numeric id of the estimate
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimate(const wbi::EstimateType et,  const int estimate_numeric_id, double *data, double time=-1.0, bool blocking=true);

        /** Get all the estimates of the specified estimate type at the specified time.
         * @param et Type of estimate to get.
         * @param data Output data vector.
         * @param time Time at which to estimate the quantity.
         * @param blocking If true, perform a blocking read before estimating, otherwise the estimate is based on the last reading.
         * @return True if all the estimate succeeded, false otherwise.
         */
        virtual bool getEstimates(const wbi::EstimateType et, double *data, double time=-1.0, bool blocking=true);

        /** Set the value of the specified parameter of the estimation algorithm
         * of the specified estimate type.
         * @param et Estimation type (e.g. joint velocity, motor torque).
         * @param ep Parameter to set.
         * @param value Value of the parameter to set.
         * @return True if the operation succeeded, false otherwise. */
        virtual bool setEstimationParameter(const wbi::EstimateType et, const wbi::EstimationParameter ep, const void *value);

        /////////////////////////////////////////////////////
        ///< Implementation specific methods
        /////////////////////////////////////////////////////
        bool setEstimationOffset(const wbi::EstimateType et, const wbi::ID & sid, const double *value);

        bool getEstimationOffset(const wbi::EstimateType et, const wbi::ID & sid, double *value);


        /** Get the estimated external force/torques
         *
         * \note temporary interface, should be substituted by properly defining an external force/torque estimate
         * @param external_forces_list list of estimated external wrenches
         * @return True if the operation succeeded, false otherwise.
         */
        bool getEstimatedExternalForces(iCub::skinDynLib::skinContactList & external_forces_list);

        //////////////////////////////////////////////////////
        //// Diagnostic related methods
        //////////////////////////////////////////////////////
    };



// *********************************************************************************************************************
// *********************************************************************************************************************
//
// *********************************************************************************************************************
// *********************************************************************************************************************
wholeBodyDynamicsStatesInterface::wholeBodyDynamicsStatesInterface(const char* _name,
                                                                   int estimator_period,
                                                   yarp::os::Property & _wbi_yarp_conf)
{
    sensors = new yarpWholeBodySensors(_name, _wbi_yarp_conf);              // sensor interface
    skin_contacts_port = new yarp::os::BufferedPort<iCub::skinDynLib::skinContactList>;
    skin_contacts_port->open(string("/"+string(_name)+"/skin_contacts:i").c_str());
    estimator = new yarpWholeBodyDynamicsEstimator(estimator_period, sensors, skin_contacts_port, _wbi_yarp_conf);  // estimation thread
}

bool wholeBodyDynamicsStatesInterface::init()
{
    bool ok = sensors->init();              // initialize sensor interface
    if( !ok )
    {
        yError() << "wholeBodyDynamicsStatesInterface::init() failed: error in sensor initialization.";
        close();
        return false;
    }

    if( !ok )
    {
        yError() << "wholeBodyDynamicsStatesInterface::init() failed: error in estimator initialization.";
        close();
        return false;
    }

    return ok; // start estimation thread
}

bool wholeBodyDynamicsStatesInterface::close()
{
    std::cout << "[INFO]wholeBodyDynamicsStatesInterface::close() : closing estimator thread" << std::endl;
    if(estimator) estimator->stop();  // stop estimator BEFORE closing sensor interface
    std::cout << "[INFO]wholeBodyDynamicsStatesInterface::close() : closing sensor interface" << std::endl;
    bool ok = (sensors ? sensors->close() : true);
    std::cout << "[INFO]wholeBodyDynamicsStatesInterface::close() : closing skin_contacts_port" << std::endl;
    if( skin_contacts_port )
    {
        skin_contacts_port->close();
        delete skin_contacts_port;
        skin_contacts_port = 0;
    }
    std::cout << "wholeBodyDynamicsStatesInterface::close() : deleting sensor interface" << std::endl;
    if(sensors) { delete sensors; sensors = 0; }
    std::cout << "wholeBodyDynamicsStatesInterface::close() : deleting estimator thread" << std::endl;
    if(estimator) { delete estimator; estimator = 0; }
    return ok;
}

bool wholeBodyDynamicsStatesInterface::addEstimate(const EstimateType et, const ID &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndAddSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:             return lockAndAddSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

int wholeBodyDynamicsStatesInterface::addEstimates(const EstimateType et, const IDList &sids)
{
    //\todo TODO properly handle dependencies
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE:             return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_POS:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_VEL:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_ACC:                return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndAddSensors(SENSOR_ENCODER, sids);
    case ESTIMATE_MOTOR_PWM:                return lockAndAddSensors(SENSOR_PWM, sids);
    case ESTIMATE_IMU:                      return lockAndAddSensors(SENSOR_IMU, sids);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndAddSensors(SENSOR_FORCE_TORQUE, sids);
    default: break;
    }
    return false;
}

bool wholeBodyDynamicsStatesInterface::removeEstimate(const EstimateType et, const ID &sid)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE:             return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_POS:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_VEL:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_ACC:                return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE:             return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return lockAndRemoveSensor(SENSOR_ENCODER, sid);
    case ESTIMATE_MOTOR_PWM:                return lockAndRemoveSensor(SENSOR_PWM, sid);
    case ESTIMATE_IMU:                      return lockAndRemoveSensor(SENSOR_IMU, sid);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return lockAndRemoveSensor(SENSOR_FORCE_TORQUE, sid);
    default: break;
    }
    return false;
}

const IDList& wholeBodyDynamicsStatesInterface::getEstimateList(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorList(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorList(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorList(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorList(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return emptyList;
}

int wholeBodyDynamicsStatesInterface::getEstimateNumber(const EstimateType et)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE:             return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_POS:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_VEL:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_ACC:                return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE:             return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return sensors->getSensorNumber(SENSOR_ENCODER);
    case ESTIMATE_MOTOR_PWM:                return sensors->getSensorNumber(SENSOR_PWM);
    case ESTIMATE_IMU:                      return sensors->getSensorNumber(SENSOR_IMU);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return sensors->getSensorNumber(SENSOR_FORCE_TORQUE);
    default: break;
    }
    return 0;
}

bool wholeBodyDynamicsStatesInterface::getEstimate(const EstimateType et, const int numeric_id, double *data, double time, bool blocking)
{
    wbi::ID sid;
    switch(et)
    {
    case ESTIMATE_JOINT_POS:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:
        return estimator->lockAndCopyVectorElement(numeric_id,estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:
        return false;
    case ESTIMATE_MOTOR_VEL:
        return getMotorVel(numeric_id, data, time, blocking);
    case ESTIMATE_MOTOR_ACC:
        return false;
    case ESTIMATE_MOTOR_TORQUE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:
        return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:
        return lockAndReadSensor(SENSOR_PWM, numeric_id, data, time, blocking);
    case ESTIMATE_IMU:
        return estimator->lockAndCopyElementVectorFromVector(numeric_id, estimator->estimates.lastIMUs, data);
    case ESTIMATE_FORCE_TORQUE_SENSOR:
        return estimator->lockAndCopyElementVectorFromVector(numeric_id, estimator->estimates.lastForceTorques, data);
    case ESTIMATE_EXTERNAL_FORCE_TORQUE:
        return estimator->lockAndCopyExternalForceTorque(numeric_id,data);
    default: break;
    }
    return false;
}

bool wholeBodyDynamicsStatesInterface::getEstimates(const EstimateType et, double *data, double time, bool blocking)
{
    switch(et)
    {
    case ESTIMATE_JOINT_POS:                return estimator->lockAndCopyVector(estimator->estimates.lastQ, data);
    case ESTIMATE_JOINT_VEL:                return estimator->lockAndCopyVector(estimator->estimates.lastDq, data);
    case ESTIMATE_JOINT_ACC:                return estimator->lockAndCopyVector(estimator->estimates.lastD2q, data);
    case ESTIMATE_JOINT_TORQUE:             return estimator->lockAndCopyVector(estimator->estimates.lastTauJ, data);
    case ESTIMATE_JOINT_TORQUE_DERIVATIVE:  return estimator->lockAndCopyVector(estimator->estimates.lastDtauJ, data);
    case ESTIMATE_MOTOR_POS:                return false;
    case ESTIMATE_MOTOR_VEL:                return getMotorVel(data, time, blocking);
    case ESTIMATE_MOTOR_ACC:                return false;
    case ESTIMATE_MOTOR_TORQUE:             return estimator->lockAndCopyVector(estimator->estimates.lastTauM, data);
    case ESTIMATE_MOTOR_TORQUE_DERIVATIVE:  return estimator->lockAndCopyVector(estimator->estimates.lastDtauM, data);
    case ESTIMATE_MOTOR_PWM:                return lockAndReadSensors(SENSOR_PWM, data, time, blocking);
    case ESTIMATE_IMU:                      return estimator->lockAndCopyVectorOfVectors(estimator->estimates.lastIMUs, data);
    case ESTIMATE_FORCE_TORQUE_SENSOR:      return estimator->lockAndCopyVectorOfVectors(estimator->estimates.lastForceTorques, data);
    default: break;
    }
    return false;
}

bool wholeBodyDynamicsStatesInterface::setEstimationParameter(const EstimateType et, const EstimationParameter ep, const void *value)
{
    return estimator->lockAndSetEstimationParameter(et, ep, value);
}

bool wholeBodyDynamicsStatesInterface::setEstimationOffset(const EstimateType et, const ID & sid, const double *value)
{
    return estimator->lockAndSetEstimationOffset(et,sid,value);
}

bool wholeBodyDynamicsStatesInterface::getEstimationOffset(const EstimateType et, const ID & sid, double *value)
{
    return estimator->lockAndGetEstimationOffset(et,sid,value);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                    IMPLEMENTATION SPECIFIC METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************


bool wholeBodyDynamicsStatesInterface::getEstimatedExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    return lockAndReadExternalForces(external_forces_list);
}

// *********************************************************************************************************************
// *********************************************************************************************************************
//                                          PRIVATE METHODS
// *********************************************************************************************************************
// *********************************************************************************************************************

bool wholeBodyDynamicsStatesInterface::getMotorVel(double *data, double time, bool blocking)
{
    bool res = estimator->lockAndCopyVector(estimator->estimates.lastDq, data);    ///< read joint vel
    if(!res) return false;
    IDList idList = lockAndGetSensorList(SENSOR_ENCODER);
    return true;
}

bool wholeBodyDynamicsStatesInterface::getMotorVel(const int numeric_id, double *data, double time, bool blocking)
{
    ///< read joint vel
    return estimator->lockAndCopyVectorElement(numeric_id, estimator->estimates.lastDq, data);
}

bool wholeBodyDynamicsStatesInterface::lockAndReadSensors(const SensorType st, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensors(st, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool wholeBodyDynamicsStatesInterface::lockAndReadSensor(const SensorType st, const int numeric_id, double *data, double time, bool blocking)
{
    estimator->mutex.wait();
    bool res = sensors->readSensor(st, numeric_id, data, 0, blocking);
    estimator->mutex.post();
    return res;
}

bool wholeBodyDynamicsStatesInterface::lockAndReadExternalForces(iCub::skinDynLib::skinContactList & external_forces_list)
{
    estimator->mutex.wait();
    external_forces_list = estimator->estimatedLastSkinDynContacts;
    estimator->mutex.post();
    return true;
}

bool wholeBodyDynamicsStatesInterface::lockAndAddSensor(const SensorType st, const ID &sid)
{
    estimator->mutex.wait();
    bool res = sensors->addSensor(st, sid);
    estimator->mutex.post();
    return res;
}

int wholeBodyDynamicsStatesInterface::lockAndAddSensors(const SensorType st, const IDList &sids)
{
    estimator->mutex.wait();
    int res = sensors->addSensors(st, sids);
    estimator->mutex.post();
    return res;
}

bool wholeBodyDynamicsStatesInterface::lockAndRemoveSensor(const SensorType st, const ID &sid)
{
    estimator->mutex.wait();
    bool res = sensors->removeSensor(st, sid);
    estimator->mutex.post();
    return res;
}

IDList wholeBodyDynamicsStatesInterface::lockAndGetSensorList(const SensorType st)
{
    estimator->mutex.wait();
    IDList res = sensors->getSensorList(st);
    estimator->mutex.post();
    return res;
}

int wholeBodyDynamicsStatesInterface::lockAndGetSensorNumber(const SensorType st)
{
    estimator->mutex.wait();
    int res = sensors->getSensorNumber(st);
    estimator->mutex.post();
    return res;
}

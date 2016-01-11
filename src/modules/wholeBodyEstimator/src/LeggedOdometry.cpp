
#include "LeggedOdometry.h"

#include "kdl/frames_io.hpp"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wbi;
using namespace iDynTree;

REGISTERIMPL(LeggedOdometry);

LeggedOdometry::LeggedOdometry()
{
}

LeggedOdometry::~LeggedOdometry()
{

}

bool LeggedOdometry::init(ResourceFinder &rf, wbi::iWholeBodySensors *wbs)
{
    m_sensors = wbs;
    
    // ########################## ODOMETRY PARAMETERS PARSING ########################################
    yarp::os::Bottle & odometry_group = rf.findGroup("LeggedOdometry");
    if ( odometry_group.isNull() )
    {
        yError("[LeggedOdometry::init] LeggedOdometry not found");
        return false;
    } else {
        yInfo("[LeggedOdometry::init()] LeggedOdometry group was found");
    }
    
    // Module name
    yarp::os::Bottle & module_params = rf.findGroup("module_parameters");
    if ( module_params.isNull() )
    {
        yError("LeggedOdometry::init() module_parameters not found");
        return false;
    } else {
        yInfo("LeggedOdometry::init() module_parameters was found");
    }
    m_module_name = module_params.find("name").asString();
    
    if( odometry_group.isNull()  )
    {
        this->odometry_enabled = false;
        return true;
    }
    
    if( !odometry_group.check("initial_world_frame") ||
       !odometry_group.check("initial_fixed_link") ||
       !odometry_group.check("floating_base_frame") ||
       !odometry_group.find("initial_world_frame").isString() ||
       !odometry_group.find("initial_fixed_link").isString() ||
       !odometry_group.find("floating_base_frame").isString() )
    {
        yError() << " LeggedOdometry group found but malformed, exiting";
        this->odometry_enabled = false;
        return false;
    }
    
    std::string initial_world_frame = odometry_group.find("initial_world_frame").asString();
    std::string initial_fixed_link = odometry_group.find("initial_fixed_link").asString();
    std::string floating_base_frame = odometry_group.find("floating_base_frame").asString();
    
    // Loading additional options
    
    // Check if com should be streamed
    if ( !(this->com_streaming_enabled = odometry_group.check("stream_com")) )
    {
        yInfo() << "COM won't be streamed since stream_com was found false in the configuration file";
    }
    
    // Check if additional frames (besides the floating base) should be streamed
    if( odometry_group.check("additional_frames") &&
       odometry_group.find("additional_frames").isList() )
    {
        this->frames_streaming_enabled = true;
    }
    else
    {
        this->frames_streaming_enabled = false;
    }
    
    // ######################### PREPARING ARGUMENTS FOR iDynTree object creation ########################
    
    // Retrieving URDF
    yarp::os::Property yarpWbiOptions;
    //Get wbi options from the canonical file
    if( !rf.check("wbi_conf_file") )
    {
        yError("[LeggedOdometry] Impossible to initialize LeggedOdometry: wbi_conf_file option missing");
        return false;
    }
    std::string wbiConfFile = rf.findFile("wbi_conf_file");
    yarpWbiOptions.fromConfigFile(wbiConfFile);

    
    // URDF File Path
    std::string urdf_file = yarpWbiOptions.find("urdf").asString().c_str();
    std::string urdf_file_path = rf.findFileByName(urdf_file.c_str());
    
    // DOF Serialization
    std::vector<std::string> dof_serialization;
    IDList torque_estimation_list = m_sensors->getSensorList(wbi::SENSOR_ENCODER);
    for(int dof=0; dof < (int)torque_estimation_list.size(); dof++)
    {
        ID wbi_id;
        torque_estimation_list.indexToID(dof,wbi_id);
        dof_serialization.push_back(wbi_id.toString());
    }
    
    // FT Serialization
    std::vector<std::string> ft_serialization;
    IDList ft_sensor_list =  m_sensors->getSensorList(wbi::SENSOR_FORCE_TORQUE);
    for(int ft=0; ft < (int)ft_sensor_list.size(); ft++)
    {
        ID wbi_id;
        ft_sensor_list.indexToID(ft,wbi_id);
        ft_serialization.push_back(wbi_id.toString());
    }

    // As done in TorqueEstimationTree.cpp
    // iCub Tree model
    KDL::Tree icub_kdl;
    bool ret;
    ret = iDynTree::treeFromUrdfFile(urdf_file_path, icub_kdl);
    if( !ret )
    {
        yError() << " LeggedOdometry: error in parsing URDF";
        return false;
    }
    
    // ft_names
    //Construct F/T sensor name list from URDF gazebo extensions
    std::vector< ::iDynTree::FTSensorData > ft_sensors;
    ret = ::iDynTree::ftSensorsFromUrdfFile(urdf_file_path, ft_sensors);
    
    if( !ret )
    {
        yError() << " LeggedOdometry: error in loading ft_sensors";
        return false;
    }
    std::vector< std::string > ft_names(ft_serialization.size());
    std::vector< KDL::Frame > child_sensor_transforms(ft_serialization.size());
    KDL::Frame kdlFrame;
    
    for(std::size_t serialization_id=0; serialization_id < ft_serialization.size(); serialization_id++)
    {
        if( 0 == ft_sensors.size() )
        {
            std::cerr << "[ERR] LeggedOdometry: ft sensor " << ft_serialization[serialization_id] << " not found in model file." << std::endl;
            return false;
        }
        for(std::size_t ft_sens=0; ft_sens < ft_sensors.size(); ft_sens++ )
        {
            std::string ft_sens_name = ft_sensors[ft_sens].reference_joint;
            std::size_t ft_sens_id;
            
            
            if( ft_serialization[serialization_id] == ft_sens_name)
            {
                ft_sens_id = serialization_id;
                
                ft_names[ft_sens_id] = ft_sens_name;
                //TODO: FIXME properly address also parent and child cases
                //                  and measure_direction
                if( ft_sensors[ft_sens].frame == ::iDynTree::FTSensorData::SENSOR_FRAME )
                {
                    child_sensor_transforms[ft_sens_id] = KDL::Frame(ft_sensors[ft_sens].sensor_pose.M);
                }
                else
                {
                    child_sensor_transforms[ft_sens_id] = KDL::Frame::Identity();
                }
                
                break;
            }
            
            if( ft_sens == ft_sensors.size() -1 )
            {
                std::cerr << "[ERR] LeggedOdometry: ft sensor " << ft_sens_name << " not found in model file." << std::endl;
                return false;
            }
        }
    }
    
    std::cerr << "[INFO] LeggedOdometry init: loaded urdf with " << dof_serialization.size()
    << "dofs and " << ft_names.size() << " fts ( " << ft_serialization.size() <<  ") " << std::endl;
    
    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);
    
    //Setting a custom dof serialization
    //TODO: Quite a hack, substitute with proper)
    if( dof_serialization.size() != 0 )
    {
        YARP_ASSERT(dof_serialization.size() == serial.getNrOfDOFs());
        for(std::size_t dof=0; dof < dof_serialization.size(); dof++)
        {
            std::string dof_string = dof_serialization[dof];
            YARP_ASSERT(serial.getDOFID(dof_string) != -1);
            YARP_ASSERT(serial.getJunctionID(dof_string) != -1);
        }
        
        for(std::size_t dof=0; dof < dof_serialization.size(); dof++)
        {
            std::string dof_string = dof_serialization[dof];
            std::cout << "[DEBUG] LeggedOdometry: Setting id of dof " << dof_string << " to " << dof << std::endl;
            serial.setDOFNameID(dof_string, (int)dof);
            serial.setJunctionNameID(dof_string, (int)dof);
        }
    }
    
    // Manually defined
    std::string imu_link_name = "imu_frame";
    
    // ################################################################################################################
    
    
    // iCub model creation
    // Assuming iCub standing on two feet
    icub_model = new iCub::iDynTree::DynTree(icub_kdl, ft_names, imu_link_name, serial);
    
    // Allocate model
    KDL::CoDyCo::UndirectedTree undirected_tree = this->icub_model->getKDLUndirectedTree();
    bool ok = this->odometry_helper.init(undirected_tree,
                                         initial_world_frame,
                                         initial_fixed_link);
    this->current_fixed_link_name = initial_fixed_link;
    //externalWrenchTorqueEstimator->current_fixed_link_name = initial_fixed_link;
    
    // Get floating base frame index
    this->odometry_floating_base_frame_index = odometry_helper.getDynTree().getFrameIndex(floating_base_frame);
    
    ok = ok && (this->odometry_floating_base_frame_index >= 0 &&
                this->odometry_floating_base_frame_index < odometry_helper.getDynTree().getNrOfFrames());
    
    if( !ok )
    {
        yError() << "Odometry initialization failed, please check your parameters";
        return false;
    }
    
    yInfo() << " SIMPLE_LEGGED_ODOMETRY initialized with initial world frame coincident with "
    << initial_world_frame << " and initial fixed link " << initial_fixed_link;
    
    this->odometry_enabled = true;
    world_H_floatingbase.resize(4,4);
    floatingbase_twist.resize(6,0.0);
    floatingbase_acctwist.resize(6,0.0);
    
    // Joints in icub_model
    m_joint_status = new iDynTree::RobotJointStatus;
    m_joint_status->setNrOfDOFs(icub_model->getNrOfDOFs());
    m_joint_status->zero();
    icub_model->setAng(m_joint_status->getJointPosYARP());
    
    // Get id of frames to stream
    if( this->frames_streaming_enabled )
    {
        yarp::os::Bottle * frames_bot = odometry_group.find("additional_frames").asList();
        yDebug() << "frames_bot " << frames_bot->toString();
        if( frames_bot == NULL ||
           frames_bot->isNull() )
        {
            yError("additional_frames list malformed");
            return false;
        }
        
        for(int i=0; i < frames_bot->size(); i++ )
        {
            std::string frame_name = frames_bot->get(i).asString();
            int frame_index = odometry_helper.getDynTree().getFrameIndex(frame_name);
            if( frame_index < 0 )
            {
                yError("additional_frames: frame %s not found in the model",frame_name.c_str());
                return false;
            }
            
            frames_to_stream.push_back(frame_name);
            frames_to_stream_indices.push_back(frame_index);
        }
        
        buffer_bottles.resize(frames_to_stream.size());
        
        for(int i=0; i < buffer_bottles.size(); i++ )
        {
            yInfo("wholeBodyDynamicsTree: streaming world position of frame %s",frames_to_stream[i].c_str());
            buffer_bottles[i].addList();
        }
        
        buffer_transform_matrix.resize(4,4);
    }
    
    
    
    // Open ports
    port_floatingbasestate = new BufferedPort<Bottle>;
    port_floatingbasestate->open(std::string("/"+m_module_name+"/floatingbasestate:o"));
    
    if( this->com_streaming_enabled )
    {
        port_com = new BufferedPort<Vector>;
        port_com->open(std::string("/"+m_module_name+"/com:o"));
    }
    
    if( this->frames_streaming_enabled )
    {
        port_frames = new BufferedPort<Property>;
        port_frames->open(std::string("/"+m_module_name+"/frames:o"));
    }
    return true;
}

void LeggedOdometry::run()
{
    
    if( this->odometry_enabled )
    {
        readRobotStatus();
        
        // Read joint position, velocity and accelerations into the odometry helper model
        // This could be avoided by using the same geometric model
        // for odometry, force/torque estimation and sensor force/torque calibration
        odometry_helper.setJointsState(m_joint_status->getJointPosKDL(),
                                       m_joint_status->getJointVelKDL(),
                                       m_joint_status->getJointAccKDL());
        
        // Get floating base position in the world
        KDL::Frame world_H_floatingbase_kdl = odometry_helper.getWorldFrameTransform(this->odometry_floating_base_frame_index);
        
        // Publish the floating base position on the port
        KDLtoYarp_position(world_H_floatingbase_kdl,this->world_H_floatingbase);
        
        yarp::os::Bottle & bot = port_floatingbasestate->prepare();
        bot.clear();
        bot.addList().read(this->world_H_floatingbase);
        bot.addList().read(this->floatingbase_twist);
        bot.addList().read(this->floatingbase_acctwist);
        
        port_floatingbasestate->write();
        
        
        if( this->com_streaming_enabled )
        {
            // Stream com in world frame
            KDL::Vector com = odometry_helper.getDynTree().getCOMKDL();
            
            yarp::sig::Vector & com_to_send = port_com->prepare();
            com_to_send.resize(3);
            
            KDLtoYarp(com,com_to_send);
            
            port_com->write();
        }
        
        if( this->frames_streaming_enabled )
        {
            // Stream frames in a property (highly inefficient! clean as soon as possible)
            Property& output = port_frames->prepare();
            
            for(int i =0; i < frames_to_stream.size(); i++ )
            {
                KDL::Frame frame_to_publish = odometry_helper.getWorldFrameTransform(frames_to_stream_indices[i]);
                
                KDLtoYarp_position(frame_to_publish,buffer_transform_matrix);
                
                buffer_bottles[i].get(0).asList()->read(buffer_transform_matrix);
                output.put(frames_to_stream[i].c_str(),buffer_bottles[i].get(0));
            }
            
            port_frames->write();
        }
        
        // save the current link considered as fixed by the odometry
        current_fixed_link_name = odometry_helper.getCurrentFixedLink();
    }
}

void LeggedOdometry::readRobotStatus()
{
    // Last two arguments specify not retrieving timestamps and not to wait to get a sensor measurement
    if ( !m_sensors->readSensors(wbi::SENSOR_ENCODER_POS, m_joint_status->getJointPosKDL().data.data(), NULL, false) )
    {
        yError("[LeggedOdometry::readRobotStatus()] Encoders could not be read!");
    }

    // Update yarp vectors.
    m_joint_status->updateYarpBuffers();
}

void LeggedOdometry::release()
{
    if( this->odometry_enabled )
    {
        closePort(port_floatingbasestate);
    }
    
    if( this->frames_streaming_enabled )
    {
        closePort(port_frames);
    }
    
    if( this->com_streaming_enabled )
    {
        closePort(port_com);
    }
    
    if ( this->icub_model )
    {
        delete icub_model;
        icub_model = 0;
    }
    
    if ( this->m_sensors )
    {
        delete m_sensors;
        m_sensors = 0;
    }
    
    if ( this->m_joint_status )
    {
        delete m_joint_status;
        m_joint_status = 0;
    }
    
    
}

void LeggedOdometry::closePort(Contactable *_port)
{
    if (_port)
    {
        _port->interrupt();
        _port->close();
        
        delete _port;
        _port = 0;
    }
}

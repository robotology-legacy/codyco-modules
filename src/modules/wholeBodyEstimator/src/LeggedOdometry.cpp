
#include "LeggedOdometry.h"

#include "kdl/frames_io.hpp"

using namespace yarp::os;
using namespace yarp::sig;
using namespace wbi;

LeggedOdometry::LeggedOdometry(iWholeBodySensors *wbs) : IEstimator(wbs), m_sensors(wbs)
{
    
}

LeggedOdometry::~LeggedOdometry()
{

}

bool LeggedOdometry::init(ResourceFinder &rf)
{
    // ########################## ODOMETRY PARAMETERS PARSING ########################################
    yarp::os::Bottle & odometry_group = rf.findGroup("SIMPLE_LEGGED_ODOMETRY");
    
    if( odometry_group.isNull()  )
    {
        yInfo() << " SIMPLE_LEGGED_ODOMETRY group not found, odometry disabled";
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
        yError() << " SIMPLE_LEGGED_ODOMETRY group found but malformed, exiting";
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
    
    // URDF File Path
    std::string urdf_file = rf.find("urdf").asString().c_str();
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
    ret = iDynTree::treeFromUrdfFile(urdf_file, icub_kdl);
    if( !ret )
    {
        yError() << " LeggedOdometry: error in parsing URDF";
        return false;
    }
    
    // ft_names
    //Construct F/T sensor name list from URDF gazebo extensions
    std::vector< ::iDynTree::FTSensorData > ft_sensors;
    ret = ::iDynTree::ftSensorsFromUrdfFile(urdf_file, ft_sensors);
    
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
            assert(false);
            return;
        }
        for(std::size_t ft_sens=0; ft_sens < ft_sensors.size(); ft_sens++ )
        {
            std::string ft_sens_name = ft_sensors[ft_sens].reference_joint;
            std::size_t ft_sens_id;
            
            
            if( ft_serialization[serialization_id] == ft_sens_name)
            {
                ft_sens_id = serialization_id;
                
                ft_names[ft_sens_id] = ft_sens_name;
                // \todo TODO FIXME properly address also parent and child cases
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
                assert(false);
                return;
            }
        }
    }
    
    std::cerr << "[INFO] LeggedOdometry init: loaded urdf with " << dof_serialization.size()
    << "dofs and " << ft_names.size() << " fts ( " << ft_serialization.size() <<  ") " << std::endl;
    
    //Define an explicit serialization of the links and the DOFs of the iCub
    //The DOF serialization done in icub_kdl construction is ok
    KDL::CoDyCo::TreeSerialization serial = KDL::CoDyCo::TreeSerialization(icub_kdl);
    
    //Setting a custom dof serialization (\todo TODO FIXME : quite a hack, substitute with proper)
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
    
//    // Open ports
//    port_floatingbasestate = new BufferedPort<Bottle>;
//    port_floatingbasestate->open(string("/"+moduleName+"/floatingbasestate:o"));
//    
//    if( this->com_streaming_enabled )
//    {
//        port_com = new BufferedPort<Vector>;
//        port_com->open(string("/"+moduleName+"/com:o"));
//    }
//    
//    if( this->frames_streaming_enabled )
//    {
//        port_frames = new BufferedPort<Property>;
//        port_frames->open(string("/"+moduleName+"/frames:o"));
//    }
//
    return true;
}

void LeggedOdometry::run()
{
    
}

void LeggedOdometry::release()
{
    
}
/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_format_io/urdf_import.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include <tinyxml.h>

using namespace kdl_format_io;
using namespace KDL;
using namespace std;

using namespace urdf;
using namespace boost;


int main(int argc, char* argv[])
{
    bool status = true;
        bool ret = true;
        
    bool verbose = true;
    
    if( argc != 4 ) {
        std::cerr << "Usage: \t urdf_get_limits urdf_limits.xml urdf_input.xml urdf_output.xml" << std::endl;
        std::cerr << "In which the urdf_output.xml is the same model of urdf_input.xml, but with the limits of urdf_limits.xml" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string file_name_urdf_limits(argv[1]);
    std::string file_name_urdf_input(argv[2]);
    std::string file_name_urdf_output(argv[3]);


    boost::shared_ptr<urdf::ModelInterface> urdf_limits;
    boost::shared_ptr<urdf::ModelInterface> urdf_input;

    std::ifstream t_um(file_name_urdf_limits.c_str());
    std::stringstream buffer_um;
    buffer_um << t_um.rdbuf();
    urdf_limits = parseURDF(buffer_um.str());
    
    std::ifstream t_im(file_name_urdf_input.c_str());
    std::stringstream buffer_im;
    buffer_im << t_im.rdbuf();
    urdf_input = parseURDF(buffer_im.str());
    if( !ret ) {
        std::cerr << "Fatal error in URDF xml parsing" << std::endl;
        return EXIT_FAILURE;
    }

    
    //Iterate over the joints of urdf_input, and copy the limits
    for( std::map<std::string, boost::shared_ptr<urdf::Joint> >::iterator it = urdf_input->joints_.begin();
         it != urdf_input->joints_.end(); it++ )
    {
        std::string input_name = it->first;
        std::cout << "Processing joint " << it->first << std::endl;
        
        boost::shared_ptr<const urdf::Joint> limits_joints_ptr = urdf_limits->getJoint(input_name);
        
        
        if( limits_joints_ptr ) {
            std::cout << "Tryng to copy limits of joint " << input_name << std::endl;
         
            it->second->limits.reset(new urdf::JointLimits);
            *(it->second->limits) = *(limits_joints_ptr->limits);
              
        } else {
            std::cout << "No joint found with name " << input_name << std::endl;
        }
    }
    
    TiXmlDocument* xml_doc;
    
    xml_doc = exportURDF(urdf_input);
    
    if( ! xml_doc->SaveFile(file_name_urdf_output) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::cerr << "URDF file successfully created" << std::endl;
    
    return EXIT_SUCCESS;
}

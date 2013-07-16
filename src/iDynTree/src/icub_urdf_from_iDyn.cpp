/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include "idyn2kdl_icub.h"
#include <iostream>
#include <cstdlib>

#include <iCub/iDyn/iDyn.h>
#include <iCub/iDyn/iDynBody.h>

#include <kdl/tree.hpp>

#include <kdl_urdf/kdl_export.hpp>

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <tinyxml.h>

#include <boost/function.hpp>


#define NAME "icub_urdf_from_iDyn"

using namespace urdf;

void printTree(boost::shared_ptr<const Link> link,int level = 0)
{
  level+=2;
  int count = 0;
  for (std::vector<boost::shared_ptr<Link> >::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
  {
    if (*child)
    {
      for(int j=0;j<level;j++) std::cout << "  "; //indent
      std::cout << "child(" << (count++)+1 << "):  " << (*child)->name  << " FrameToTip " << urdf_export_helpers::values2str((*child)->parent_joint->parent_to_joint_origin_transform.position) << std::endl;
      // first grandchild
      printTree(*child,level);
    }
    else
    {
      for(int j=0;j<level;j++) std::cout << " "; //indent
      std::cout << "root link: " << link->name << " has a null child!" << *child << std::endl;
    }
  }

}


int main(int argc, char* argv[])
{
    bool status = true;
    if( argc != 2 ) {
        std::cerr << "Usage: \t icub_urdf_from_iDyn output.xml" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string file_name(argv[1]);
    
    iCub::iDyn::version_tag icub_type;

    icub_type.head_version = 2;
    icub_type.legs_version = 2;    
    iCub::iDyn::iCubWholeBody icub_idyn(icub_type);

    KDL::Tree icub_kdl;
    
    
    if( ! toKDL(icub_idyn,icub_kdl) ) {
        std::cerr << "Fatal error in iDyn - KDL conversion" << std::endl;
        return EXIT_FAILURE;
    }
    
    boost::shared_ptr<urdf::ModelInterface> icub_ptr(new urdf::ModelInterface);
    
    std::cout << "iCub KDL::Tree: " << std::endl;
    std::cout << icub_kdl << std::endl;
    
    
    if( ! kdl_export::treeToUrdfModel(icub_kdl,"test_icub",*icub_ptr) ) {
        std::cerr << "Fatal error in KDL - URDF conversion" << std::endl;
        return EXIT_FAILURE;
    }
    
    boost::shared_ptr<const Link> root_link=icub_ptr->getRoot();
    printTree(root_link);
    
    TiXmlDocument*  xml_doc =  exportURDF(icub_ptr);
    if( ! xml_doc->SaveFile(argv[1]) ) {
        std::cerr << "Fatal error in URDF xml saving" << std::endl;
    }
    
    std::cerr << "URDF file successfully created" << std::endl;
    
    return EXIT_SUCCESS;
}

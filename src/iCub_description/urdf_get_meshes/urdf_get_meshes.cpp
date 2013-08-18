/**
 * Copyright  (C)  2013 CoDyCo Project
 * Author: Silvio Traversaro
 * website: http://www.codyco.eu
 */
 
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

#include <kdl_urdf/kdl_import.hpp>

#include <iostream>
#include <sstream>
#include <fstream>

#include <tinyxml.h>

using namespace kdl_import;
using namespace KDL;
using namespace std;

using namespace urdf;
using namespace boost;

// construct vector
urdf::Vector3 toUrdf(const KDL::Vector & v) 
{
    return urdf::Vector3(v.x(), v.y(), v.z());
}

// construct rotation
urdf::Rotation toUrdf(const KDL::Rotation & r) 
{
    double x,y,z,w;
    r.GetQuaternion(x,y,z,w);
    return urdf::Rotation(x,y,z,w);
}

// construct pose
urdf::Pose toUrdf(const KDL::Frame & p)
{
    urdf::Pose ret;
    ret.rotation = toUrdf(p.M);
    ret.position = toUrdf(p.p);
    return ret;
}

// construct vector
KDL::Vector toKdl(urdf::Vector3 v)
{
  return KDL::Vector(v.x, v.y, v.z);
}

// construct rotation
KDL::Rotation toKdl(urdf::Rotation r)
{
  return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
}

// construct pose
KDL::Frame toKdl(urdf::Pose p)
{
  return Frame(toKdl(p.rotation), toKdl(p.position));
}

int main(int argc, char* argv[])
{
    bool status = true;
        bool ret = true;
        
    bool verbose = true;
    
    if( argc != 4 ) {
        std::cerr << "Usage: \t urdf_get_meshes urdf_meshes.xml urdf_input.xml urdf_output.xml" << std::endl;
        std::cerr << "In which the urdf_output.xml is the same model of urdf_input.xml, but with the meshes of urdf_meshes.xml" << std::endl;
        return EXIT_FAILURE;
    }
    
    std::string file_name_urdf_meshes(argv[1]);
    std::string file_name_urdf_input(argv[2]);
    std::string file_name_urdf_output(argv[3]);


    boost::shared_ptr<urdf::ModelInterface> urdf_meshes;
    boost::shared_ptr<urdf::ModelInterface> urdf_input;

    std::ifstream t_um(file_name_urdf_meshes.c_str());
    std::stringstream buffer_um;
    buffer_um << t_um.rdbuf();
    urdf_meshes = parseURDF(buffer_um.str());
    
    std::ifstream t_im(file_name_urdf_input.c_str());
    std::stringstream buffer_im;
    buffer_im << t_im.rdbuf();
    urdf_input = parseURDF(buffer_im.str());
    if( !ret ) {
        std::cerr << "Fatal error in URDF xml parsing" << std::endl;
        return EXIT_FAILURE;
    }

    KDL::Tree kdl_meshes, kdl_input;
    ret = ret && treeFromUrdfModel(*urdf_meshes,kdl_meshes,false);
    ret = ret && treeFromUrdfModel(*urdf_input,kdl_input,false);
    if( !ret ) {
        std::cerr << "Fatal error in URDF KDL conversion saving" << std::endl;
        return EXIT_FAILURE;
    }
    
    if(kdl_meshes.getNrOfJoints() != kdl_input.getNrOfJoints())
    {
        std::cerr << "Fatal error, the two models have a different number of DOFs" << std::endl;
        return EXIT_FAILURE;     
    }
    
    JntArray q(kdl_meshes.getNrOfJoints());
    SetToZero(q);
    
    TreeFkSolverPos_recursive meshes_solver(kdl_meshes);
    TreeFkSolverPos_recursive input_solver(kdl_input);
    
    //Iterate over the links of urdf_input, and copy the mesh
    std::vector<boost::shared_ptr<Link> > input_links;
    
    urdf_input->getLinks(input_links);
    
    std::cout << "Found " << input_links.size() << " links in input URDF " << std::endl;
    for(int i=0; i < input_links.size(); i++ )
    {
        KDL::Frame base_link_meshes, base_link_input;
        KDL::Frame link_visual_old, link_collision_old;
        KDL::Frame link_visual_new, link_collision_new;
        std::string input_name = input_links[i]->name;
    
        std::cout << "Processing link " << input_name << std::endl;
        
        boost::shared_ptr<const Link> mesh_link_ptr = urdf_meshes->getLink(input_name);
        
        
        if( mesh_link_ptr ) {
            std::cout << "Tryng to copy meshes of link " << input_name << std::endl;
            //if there is a link with the same name, copy the mesh
            //supporting only single mesh
            if( mesh_link_ptr->collision || mesh_link_ptr->visual ) {
                int retur;
                retur = meshes_solver.JntToCart(q,base_link_meshes,input_name);
                if( retur < 0 ) { return EXIT_FAILURE; } 
                retur = input_solver.JntToCart(q,base_link_input,input_name);
                if( retur < 0 ) { return EXIT_FAILURE; } 
            }
            if( mesh_link_ptr->collision ) {
                link_collision_old = toKdl(mesh_link_ptr->collision->origin);
                link_collision_new = base_link_input.Inverse()*base_link_meshes*link_collision_old;
                
                std::cout << "Old collision origin " << link_collision_old << endl;
                std::cout << "New collision origin " << link_collision_new << endl;

                
                input_links[i]->collision.reset(new urdf::Collision);
                
                input_links[i]->collision->geometry.reset(new urdf::Geometry);
                
                input_links[i]->collision->origin = toUrdf(link_collision_new);
                
                switch( mesh_link_ptr->collision->geometry->type ) {
                    case Geometry::SPHERE:
                        input_links[i]->collision->geometry.reset(new urdf::Sphere);
                        *(input_links[i]->collision->geometry) = *(mesh_link_ptr->collision->geometry);
                    break;
                    case Geometry::BOX:
                        input_links[i]->collision->geometry.reset(new urdf::Box);
                        *(input_links[i]->collision->geometry) = *(mesh_link_ptr->collision->geometry);
                    break;
                    case Geometry::CYLINDER:
                        input_links[i]->collision->geometry.reset(new urdf::Cylinder);
                        *(input_links[i]->collision->geometry) = *(mesh_link_ptr->collision->geometry);
                    break;
                    case Geometry::MESH:
                        input_links[i]->collision->geometry.reset(new urdf::Mesh);
                        (static_pointer_cast<urdf::Mesh>(input_links[i]->collision->geometry))->filename = (static_pointer_cast<urdf::Mesh>(mesh_link_ptr->collision->geometry))->filename;
                        (static_pointer_cast<urdf::Mesh>(input_links[i]->collision->geometry))->scale = (static_pointer_cast<urdf::Mesh>(mesh_link_ptr->collision->geometry))->scale;
                    break;
                }
            
                
            }
            if( mesh_link_ptr->visual ) {
                link_visual_old = toKdl(mesh_link_ptr->visual->origin);
                link_visual_new = base_link_input.Inverse()*base_link_meshes*link_visual_old;
                
                input_links[i]->visual.reset(new urdf::Visual);
                
                input_links[i]->visual->origin = toUrdf(link_visual_new);
                
                input_links[i]->visual->material_name = mesh_link_ptr->visual->material_name;
                
                input_links[i]->visual->material.reset(new urdf::Material);
                
                *(input_links[i]->visual->material) = *(mesh_link_ptr->visual->material);
                                
                input_links[i]->visual->geometry.reset(new urdf::Geometry);

                switch( mesh_link_ptr->visual->geometry->type ) {
                    case Geometry::SPHERE:
                        input_links[i]->visual->geometry.reset(new urdf::Sphere);
                        *(input_links[i]->visual->geometry) = *(mesh_link_ptr->visual->geometry);
                    break;
                    case Geometry::BOX:
                        input_links[i]->visual->geometry.reset(new urdf::Box);
                        *(input_links[i]->visual->geometry) = *(mesh_link_ptr->visual->geometry);
                    break;
                    case Geometry::CYLINDER:
                        input_links[i]->visual->geometry.reset(new urdf::Cylinder);
                        *(input_links[i]->visual->geometry) = *(mesh_link_ptr->visual->geometry);
                    break;
                    case Geometry::MESH:
                        input_links[i]->visual->geometry.reset(new urdf::Mesh);
                        (static_pointer_cast<urdf::Mesh>(input_links[i]->visual->geometry))->filename = (static_pointer_cast<urdf::Mesh>(mesh_link_ptr->visual->geometry))->filename;
                        (static_pointer_cast<urdf::Mesh>(input_links[i]->visual->geometry))->scale = (static_pointer_cast<urdf::Mesh>(mesh_link_ptr->visual->geometry))->scale;
                    break;
                }
                

            }
        } else {
            std::cout << "No link found with name " << input_name << std::endl;
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

#ifndef _UTILITIES
#define _UTILITIES

#include <rbdl/rbdl.h>

#define PI 3.14159265359
#define DEG2RAD(r) (r*PI/180)
#define RAD2DEG(r) (r*180/PI)

void readFromCSV(std::string filename, std::vector<RigidBodyDynamics::Math::VectorNd>& mat);
void writeOnCSV(RigidBodyDynamics::Math::VectorNd time, std::vector< RigidBodyDynamics::Math::VectorNd >& data, std::string file_name, const char* header);
void writeOnCSV(std::vector<RigidBodyDynamics::Math::VectorNd>& data, std::string file_name);

#endif
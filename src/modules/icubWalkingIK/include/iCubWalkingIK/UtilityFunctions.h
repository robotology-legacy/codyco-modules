#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Core>
#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <cmath>

#define DEG2RAD(r) (r*M_PI/180)
#define RAD2DEG(r) (r*180/M_PI)

inline Eigen::Matrix3d rotx (const double &xrot) {
    double s, c;
    s = sin (xrot);
    c = cos (xrot);
    Eigen::Matrix3d tmp;
    tmp << 1., 0., 0.,
           0.,  c,  s,
           0., -s,  c;
    return tmp;
}

inline Eigen::Matrix3d roty (const double &yrot) {
    double s, c;
    s = sin (yrot);
    c = cos (yrot);
    Eigen::Matrix3d tmp;
    tmp <<  c, 0., -s,
           0., 1., 0.,
            s, 0., c;
    return tmp;
}

inline Eigen::Matrix3d rotz (const double &zrot) {
    double s, c;
    s = sin (zrot);
    c = cos (zrot);
    Eigen::Matrix3d tmp;
    tmp << c, s, 0.,
          -s, c, 0.,
          0., 0., 1.;
    return tmp;
}


void readFromCSV(std::string filename, std::vector<Eigen::VectorXd>& mat);
void writeOnCSV(Eigen::VectorXd time, std::vector< Eigen::VectorXd >& data, std::string file_name, const char* header);
void writeOnCSV(std::vector<Eigen::VectorXd>& data, std::string file_name);

#endif
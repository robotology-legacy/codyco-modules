/*
* Copyright (C) 2014 ...
* Author: ...
* email: ...
* Permission is granted to copy, distribute, and/or modify this program
* under the terms of the GNU General Public License, version 2 or any
* later version published by the Free Software Foundation.
*
* A copy of the license can be found at
* http://www.robotcub.org/icub/license/gpl.txt
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
* Public License for more details
*/

#ifndef INSITU_FT_CALIBRATION_DATASET_H
#define INSITU_FT_CALIBRATION_DATASET_H

#include <yarp/os/Bottle.h>
#include <string>

class FTCalibrationDataset
{
public:
    FTCalibrationDataset();
    ~FTCalibrationDataset();

    bool fromBottle(const yarp::os::Bottle & bot);
    std::string dataset_name;
    double added_mass;
};

#endif

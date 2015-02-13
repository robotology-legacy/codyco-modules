/*
* Copyright (C) 2015 ...
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

#include "ftcalibrationdataset.h"

FTCalibrationDataset::FTCalibrationDataset():
                        added_mass(0),
                        dataset_name("")
{

}


FTCalibrationDataset::~FTCalibrationDataset()
{

}

bool FTCalibrationDataset::fromBottle(const yarp::os::Bottle &bot)
{
    if( bot.size() != 2
        || !(bot.get(0).isString())
        || !(bot.get(1).isList())
        || !(bot.get(1).asList()->size() == 2)
        || !(bot.get(1).asList()->get(0).isString())
        || !(bot.get(1).asList()->get(0).asString() == "mass")
        || !(bot.get(1).asList()->get(1).isDouble())
    )
    {
        return false;
    }

    dataset_name = bot.get(0).asString();
    added_mass   = bot.get(1).asList()->get(1).asDouble();
    return true;
}
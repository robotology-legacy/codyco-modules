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
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

#include <string.h>
#include <iostream>
#include <fstream>
#include <iomanip>

#include "ftcalibrationdataset.h"

#include <InSituFTCalibration/offset_estimator.h>
#include <InSituFTCalibration/force_calibration_matrix_estimator.h>

#include <InSituFTCalibration/eigen_wrappers.h>

#include <Eigen/Dense>


using namespace yarp::os;
using namespace std;

#include <cstdlib>


#define MAXBUFSIZE  ((int) 1e6)

double toDouble(std::string str)
{
    stringstream ss(str);
    double ret;
    ss >> ret;
    return ret;
}

Eigen::MatrixXd readMatrix(const char *filename, bool verbose=false)
{
    int cols = 0, rows = 0;
    double * buff = (double *) calloc(MAXBUFSIZE,sizeof(double));

    // Read numbers from file into buffer.
    ifstream infile;
    infile.open(filename);

    if( !infile.is_open() )
    {
        std::cerr << "[ERR] impossible open " << filename << std::endl;
        Eigen::MatrixXd result(0,0);
        return result;
    }

    while (! infile.eof())
    {
        if( verbose && (rows % 1000 == 0) )
        {
            std::cout << "[INFO] readMatrix reading line " << rows
                      << " of file " << filename << std::endl;
        }
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream lineStream(line);
        std::string cell;
        while(std::getline(lineStream,cell,','))
        {
            buff[cols*rows+temp_cols++] = toDouble(cell);
        }
        //while(! stream.eof())
        //    stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();

    //rows--;

    // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    free(buff);
    return result;
}


int main (int argc, char * argv[])
{
    //Creating and preparing the Resource Finder
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("insituFTSensorCalibration.ini"); //default config file name.
    rf.setDefaultContext("insituFTSensorCalibration"); //when no parameters are given to the module this is the default context
    rf.configure(argc,argv);

    if (rf.check("help"))
    {
        cout<< "Utility for running batch test of dataset produced by insituFTSensorCalibration module"<<endl;
        cout<< "Possible parameters" << endl << endl;
        return 0;
    }

    InSituFTCalibration::ForceCalibrationMatrixEstimator estimator;
    std::vector<FTCalibrationDataset> training_datasets;
    std::vector<InSituFTCalibration::ForceTorqueOffsetEstimator *> estimator_datasets;
    std::vector<std::string> datasets_filenames;

    //Load datasets
    yarp::os::Bottle & bot_datasets_files = rf.findGroup("batch_datasets");
    yarp::os::Bottle & bot = rf.findGroup("training_datasets");


    if( bot.size() != bot_datasets_files.size() )
    {
        yError() << "datasets size mismatch";
        return -1;
    }

    int nrOfDatasets = bot.size()-1;

    for(int dataset = 0; dataset < nrOfDatasets; dataset++)
    {
        FTCalibrationDataset ft_dataset;
        if( !(bot.get(dataset+1).isList()) )
        {
            std::cerr << "[ERR] insituFTSensorCalibrationBatch: error in loading training_datasets group " << std::endl;
            return false;
        }
        if( !(ft_dataset.fromBottle(*(bot.get(dataset+1).asList()))) )
        {
            std::cerr << "[ERR] insituFTSensorCalibrationBatch: error in loading training_datasets group " << std::endl;
            return false;
        }
        training_datasets.push_back(ft_dataset);
        estimator_datasets.push_back(0);

        if( !(bot_datasets_files.get(dataset+1).isList()) ||
            !(bot_datasets_files.get(dataset+1).asList()->get(0).isString())  )
        {
            std::cerr << "[ERR] insituFTSensorCalibrationBatch: error in loading batch_datasets  group " << std::endl;
            std::cerr << "[ERR] " << bot_datasets_files.toString() << std::endl;
            return false;
        }

        std::string dataset_filename = bot_datasets_files.get(dataset+1).asList()->get(0).asString().c_str();
        datasets_filenames.push_back(dataset_filename);

    }

    std::cout << "[INFO] Loaded training datasets: " << std::endl;
    for(int currentDataset=0; currentDataset < training_datasets.size(); currentDataset++ )
    {
        std::cout << "[INFO] Dataset " << currentDataset << " Name " << training_datasets[currentDataset].dataset_name
                  << " File " << datasets_filenames[currentDataset] << std::endl;
    }


    for(int currentDataset=0; currentDataset < training_datasets.size(); currentDataset++ )
    {
        estimator.addCalibrationDataset(training_datasets[currentDataset].dataset_name,
                                        training_datasets[currentDataset].added_mass);

        estimator.getCalibrationDataset(training_datasets[currentDataset].dataset_name,
                                        estimator_datasets[currentDataset]);

        //Load matrix from file
        std::cout << "[INFO] loading dataset from " << datasets_filenames[currentDataset] << std::endl;
        Eigen::MatrixXd datasetMatrix = readMatrix(datasets_filenames[currentDataset].c_str());

        int cols = datasetMatrix.cols();
        int nrOfSamples = datasetMatrix.rows();

        if( cols != 9 )
        {
            yError() << "Error: loaded dataset does not have 9 columns";
        }

        for( int sample = 0; sample < nrOfSamples; sample++ )
        {
            Eigen::Vector3d acc;
            Eigen::Matrix<double,6,1> raw_ft;

            raw_ft = datasetMatrix.block<1,6>(sample,0).transpose();
            acc = datasetMatrix.block<1,3>(sample,6).transpose();

            estimator_datasets[currentDataset]->addMeasurements(InSituFTCalibration::wrapVec(raw_ft),
                                                                InSituFTCalibration::wrapVec(acc));

        }

    }

    std::string err;
    bool ret = estimator.computeForceCalibrationMatrixEstimation(err);
    if( !ret )
    {
        yError("insituFTSensorCalibrationBatch ForceCalibrationMatrixEstimation failed %s",err.c_str());
        return false;
    }

    Eigen::Matrix<double,3,6> force_calibration_matrix;
    estimator.getEstimatedForceCalibrationMatrix(InSituFTCalibration::wrapMat(force_calibration_matrix));

    std::cout << "[INFO] Force calibration matrix: " << std::endl;
    std::cout << force_calibration_matrix << std::endl;

    // validate the obtained result against an external file
    // the result in the external file was obtained with a previous
    // version of the software or with an alternative implementation
    double tol = 1e-3;
    if( rf.check("validateForceMatrix") )
    {
        std::string external_validation_file = rf.find("validateForceMatrix").asString();
        Eigen::MatrixXd external_force_calibration_matrix = readMatrix(external_validation_file.c_str());

        if( external_force_calibration_matrix.rows() != 3 ||
            external_force_calibration_matrix.cols() != 6 )
         {
            std::cerr << "[ERR] Validation failed." << std::endl;
            std::cerr << "[ERR] Validation matrix has unexpected size " << external_force_calibration_matrix.rows() << " "
                      << external_force_calibration_matrix.cols() << std::endl;
            return 1;
         }

         for(int i=0; i < 3; i++ )
         {
            for(int j=0; j < 6; j++ )
            {
                if( fabs(external_force_calibration_matrix(i,j)-force_calibration_matrix(i,j)) > tol )
                {
                    std::cerr << "[ERR] Validation failed." << std::endl;
                    return 1;
                }
            }
         }

         std::cerr << "[ERR] Validation successfull." << std::endl;

    }


    return 0;
}



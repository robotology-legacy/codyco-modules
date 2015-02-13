#!/usr/bin/python

import argparse
import numpy as np

# Function to convert a decimal number in [-1.0, 1.0) into 1.15 hex format
# ported from convert_onedotfifteen.m script file in the Matlab/Octave
# script for calibrating with bench data the FTSens sensor
def convert_onedotfifteen(indecimal):
    assert(indecimal >= -1.0)
    assert(indecimal < 1.0);
    quant = int(round((2**15)*indecimal)); # Quantize to 15 bits and round to the nearest integer;
    
    if( quant == 0 ):
        return "0000";

    hex_out=format(quant & 0xFFFF,'X'); # Convert the decimal number to hex

    return hex_out; # Strip the trailing 0x, following the standard of the FTSens calibration matrix file

# Inverse function of the convert_onedotfifteen function
# TODO fix
def inv_convert_onedotfifteen(hex):
    hex_py = "0x"+hex.lower();
    
    unsigned_decimal = int(hex_py,0);

    if( unsigned_decimal & 0x8000 == 0 ):
        #positive
        return float(unsigned_decimal)/(2**15);
    else:
        #negative
        return float(unsigned_decimal-2**16)/(2**15);

class CalibrationMatrix:
    def __init__(self):
        self.raw_calib_matrix = np.zeros([6,6]);
        self.gain         = 0.0;
        self.sensor_full_scale = np.zeros([6]);

    def __init__(self, filename):
        self.fromFile(filename);

    def fromFile(self, filename):
        self.raw_calib_matrix = np.zeros([6,6]);
        self.gain         = 0.0;
        self.sensor_full_scale = np.zeros([6]);

        calib_matrix_file = open(filename, 'r')
        for i, row in enumerate(calib_matrix_file):
            if( i < 36 ):
                self.raw_calib_matrix[i/6,i%6] = inv_convert_onedotfifteen(row.rstrip());
            elif( i == 36 ):
                self.gain = float(row.rstrip());
            elif( i < 44):
                self.sensor_full_scale[i-37] = float(row.rstrip())

    def toFile(self, filename):
        calib_matrix_file = open(filename, 'w')
        for row in range(0,6):
            for col in range(0,6):
                calib_matrix_file.write(convert_onedotfifteen(self.raw_calib_matrix[row,col]));
                calib_matrix_file.write('\r\n')
        calib_matrix_file.write(str(int(self.gain)));
        calib_matrix_file.write('\r\n')
        for row in range(0,6):
            calib_matrix_file.write(str(int(self.sensor_full_scale[row])));
            calib_matrix_file.write('\r\n')

    def printMatrix(self):
        np.set_printoptions(precision=3)
        print("Calib matrix: ") 
        print(str(self.getCalibMatrix()));
        print("Detail:");
        print("Raw calib matrix: ")
        print(str(self.raw_calib_matrix));
        print("Gain        : " + str(self.gain));
        print("Full Scale  : " + str(self.sensor_full_scale));

    def setNewFullScale(self,new_full_scale):
        assert(new_full_scale.size == 6);
        '''Set a new sensor full scale for the matrix, without modifyng the calibration matrix.
           To mantain the matrix equal while changing the full scale, we have
           to change bot the fullscale and also the raw calibration matrix.'''
        for row in range(0,6):
            full_scale_ratio = self.sensor_full_scale[row]/new_full_scale[row];
            self.raw_calib_matrix[row,:] = full_scale_ratio*self.raw_calib_matrix[row,:];
            self.sensor_full_scale[row] = new_full_scale[row];

    def setNewForceCalibrationSubMatrix(self,new_force_submatrix):
        assert(new_force_submatrix.shape == (3,6) );
        '''Set a new force calibration submatrix (the first three rows of the calibration matrix)
           without changing the sensor full scale and the torque calibration matrix'''
        for row in range(0,3):
            self.raw_calib_matrix[row,0:6] = new_force_submatrix[row,0:6]/self.sensor_full_scale[row];

    def setNewCalibrationMatrix(self,new_calib_matrix):
        '''Set a new calibration matrix
           without changing the sensor full scale and gain'''
        assert(new_force_submatrix.shape == (6,6) );
        for row in range(0,6):
            self.raw_calib_matrix[row,0:6] = new_calib_matrix[row,0:6]/self.sensor_full_scale[row];    
        

    def getCalibMatrix(self):
        '''Get the 6x6 calibration matrix, taking into account the sensor fullscale
           and the raw calibration matrix'''
        return self.gain*np.dot(np.diag(self.sensor_full_scale),self.raw_calib_matrix);

def op_print(args):
    # Parse file
    print_matrix = CalibrationMatrix(args.input);

    # Print data
    print_matrix.printMatrix();    

def op_updateFullScale(args):
    inputMatrix = CalibrationMatrix(args.input);
    originalMatrix = CalibrationMatrix(args.original);
    inputMatrix.setNewFullScale(originalMatrix.sensor_full_scale);
    inputMatrix.toFile(args.output);

def op_processInSituForce(args):
    # We will take the force matrix from the insituForceMatrix
    # and the torque calibration matrix from the original one
    insituForceMatrix = CalibrationMatrix(args.input);
    originalMatrix = CalibrationMatrix(args.original);

    # The insituForceMatrix is calibrated with respect to the
    # existing calibration, so we have to multiply it 
    # for the existing calibration matrix to get a raw->FT matrix
    newForceCalibrationMatrix = np.dot(insituForceMatrix.getCalibMatrix(),originalMatrix.getCalibMatrix());
  
    newForceCalibrationMatrix = newForceCalibrationMatrix[0:3,0:6];

    originalMatrix.setNewForceCalibrationSubMatrix(newForceCalibrationMatrix);

    originalMatrix.toFile(args.output);

def op_processInSituForceTorque(args):
    # We will take the force and torque matrix from the insituForceMatrix
    # however, the insitu calibration was performed on the top of original 
    # calibration, so we have to transform the original -> new transformation
    # to a raw -> new transformation
    insituForceMatrix = CalibrationMatrix(args.input);
    originalMatrix = CalibrationMatrix(args.original);

    # The insituForceMatrix is calibrated with respect to the
    # existing calibration, so we have to multiply it 
    # for the existing calibration matrix to get a raw->FT matrix
    newForceTorqueCalibrationMatrix = np.dot(insituForceMatrix.getCalibMatrix(),originalMatrix.getCalibMatrix());
  
    originalMatrix.setNewCalibrationMatrix(newForceTorqueCalibrationMatrix);

    originalMatrix.toFile(args.output);

def op_copyForceSubmatrix(args):
    # We will take the force matrix from the input matrix
    # and the torque calibration matrix from the original one
    inputMatrix = CalibrationMatrix(args.input);
    originalMatrix = CalibrationMatrix(args.original);

    newForceCalibrationMatrix = inputMatrix.getCalibMatrix();
    newForceCalibrationMatrix = newForceCalibrationMatrix[0:3,0:6];

    originalMatrix.setNewForceCalibrationSubMatrix(newForceCalibrationMatrix);

    originalMatrix.toFile(args.output);


def op_copy(args):
    inputMatrix = CalibrationMatrix(args.input);
    inputMatrix.toFile(args.output);

def main():
    parser = argparse.ArgumentParser(description='Tool for manipulate calibration matrices for the IIT FTSens 6-Axis Force/Torque sensor')
    parser.add_argument('--in', nargs='?', dest="input", action='store', default="calib_matrix.dat", help='input calibration matrix xml file (default: calib_matrix.dat)')
    parser.add_argument('--original', nargs='?', dest="original", action='store', default="SN001.dat", help='original calibration matrix xml file (default: SN001.dat)')
    parser.add_argument('--out', nargs='?', dest="output", action='store', default="output_calib_matrix.dat", help='output calibration matrix (default: output_calib_matrix.dat)')
    parser.add_argument('operation', nargs='?', action='store', choices=['print', 'updateFullScale', 'copyForceSubmatrix', 'processInSituForce', 'processInSituForceTorque', 'copy'], default='print', help='operation to perform on the input calibration matrix     :  * print : print the calib matrix file in input (default: print)')
    args = parser.parse_args()

    if( args.operation == "print" ):
        op_print(args)

    if( args.operation == "updateFullScale" ):
        op_updateFullScale(args)
  
    if( args.operation == "copyForceSubmatrix" ):
        op_copyForceSubmatrix(args)  

    if( args.operation == "processInSituForce" ):
        op_processInSituForce(args)

    if( args.operation == "processInSituForceTorque" ):
        op_processInSituForceTorque(args)


    if( args.operation == "copy" ):
        op_copy(args);
 



if __name__ == "__main__":
    main()

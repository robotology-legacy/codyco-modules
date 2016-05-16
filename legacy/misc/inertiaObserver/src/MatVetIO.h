#ifndef __MATRIX_VECTOR_IO__
#define __MATRIX_VECTOR_IO__

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>

#include <string>

/**
 * This function writes the content of a Vector to a given file
 * Uses internally gsl_vector_fwrite function
 * Since the data is written in the native binary format it may not be portable between different architectures.
 * @return true if the file is written, false otherwise
 */
bool Vector_write(const std::string file_name, const yarp::sig::Vector & vec);


/**
 * This function reads the content of a Vector to a given file
 * Uses internally gsl_vector_fread function
 * Since the data is written in the native binary format it may not be portable between different architectures.
 * File format: first 4 bytes: unsigned length of the vector, the rest of the file in gsl format, handled by gsl_vector_fread
 * @return true if the Vector is read, false otherwise
 */
bool Vector_read(const std::string file_name, yarp::sig::Vector & vec);

/**
 * This function writes the content of a Matrix to a given file
 * Uses internally gsl_matrix_fwrite function
 * Since the data is written in the native binary format it may not be portable between different architectures.
 * File format: first 4 bytes: unsigned number of rows of the matrix, successive 4 bytes: unsigned number of columns of the matrix, the rest of the file in gsl format, handled by gsl_matrix_fwrite
 * @return true if the file is written, false otherwise
 */
bool Matrix_write(const std::string file_name, const yarp::sig::Matrix & mat);

/**
 * This function reads the content of a Matrix to a given file
 * Uses internally gsl_matrix_fread function
 * Since the data is written in the native binary format it may not be portable between different architectures.
 * File format: first 4 bytes: unsigned number of rows of the matrix, successive 4 bytes: unsigned number of columns of the matrix, the rest of the file in gsl format, handled by gsl_matrix_fread
 * @return true if the Matrix is read, false otherwise
 */
bool Matrix_read(const std::string file_name, yarp::sig::Matrix & mat);


#endif

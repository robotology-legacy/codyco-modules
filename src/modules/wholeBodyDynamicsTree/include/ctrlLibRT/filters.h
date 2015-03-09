/*
 * Copyright (C) 2014 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Silvio Traversaro
 * email:  silvio.traversaro@iit.it
 * website: www.robotcub.org
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

/**
 * \defgroup Filters Filters
 *
 * @ingroup ctrlLib
 *
 * Classes for filtering.
 *
 * \author Ugo Pattacini
 *
 */

#ifndef RT_FILTERS_H
#define RT_FILTERS_H

#include<Eigen/Dense>

#include <yarp/sig/Vector.h>


namespace iCub
{

namespace ctrl
{

namespace realTime
{

/**
* \ingroup Filters
*
* IIR and FIR.
*/
class Filter
{
protected:
   Eigen::VectorXd b;
   Eigen::VectorXd a;
   yarp::sig::Vector y;

   Eigen::MatrixXd uold; ///< Matrix of past inputs: each column is a past sample
   int uold_last_column_sample;

   Eigen::MatrixXd yold; ///< Matrix of past outpts: each columns is a past output
   int yold_last_column_sample;

   size_t n;
   size_t m;

public:
   /**
   * Creates a filter with specified numerator and denominator
   * coefficients.
   * @param num vector of numerator elements given as increasing
   *            power of z^-1.
   * @param den vector of denominator elements given as increasing
   *            power of z^-1.
   * @param y0 initial output.
   * @note den[0] shall not be 0.
   */
   Filter(const yarp::sig::Vector &num, const yarp::sig::Vector &den,
          const yarp::sig::Vector &y0);

   /**
   * Internal state reset.
   * @param y0 new internal state.
   */
   void init(const yarp::sig::Vector &y0);

   /**
   * Internal state reset for filter with zero gain.
   * @param y0 new internal state.
   * @param u0 expected next input.
   * @note The gain of a digital filter is the sum of the coefficients of its
   *       numerator divided by the sum of the coefficients of its denumerator.
   */
   void init(const yarp::sig::Vector &y0, const yarp::sig::Vector &u0);

   /**
   * Returns the current filter coefficients.
   * @param num vector of numerator elements returned as increasing
   *            power of z^-1.
   * @param den vector of denominator elements returned as
   *            increasing power of z^-1.
   */
   void getCoeffs(yarp::sig::Vector &num, yarp::sig::Vector &den);

   /**
   * Sets new filter coefficients.
   * @param num vector of numerator elements given as increasing
   *            power of z^-1.
   * @param den vector of denominator elements given as increasing
   *            power of z^-1.
   * @note den[0] shall not be 0.
   * @note the internal state is reinitialized to the current
   *       output.
   */
   void setCoeffs(const yarp::sig::Vector &num, const yarp::sig::Vector &den);

   /**
   * Modifies the values of existing filter coefficients without
   * varying their lengths.
   * @param num vector of numerator elements given as increasing
   *            power of z^-1.
   * @param den vector of denominator elements given as increasing
   *            power of z^-1.
   * @return true/false on success/fail.
   * @note den[0] shall not be 0.
   * @note the adjustment is carried out iff num.size() and
   *       den.size() match the existing numerator and denominator
   *       lengths.
   */
   bool adjustCoeffs(const yarp::sig::Vector &num, const yarp::sig::Vector &den);

   /**
   * Performs filtering on the actual input.
   * @param u reference to the actual input.
   * @return a reference the corresponding output.
   * @note the returned reference is valid till any new call to filt.
   */
   const yarp::sig::Vector & filt(const yarp::sig::Vector &u);

   /**
   * Return the reference to the current filter output.
   * @return reference to the filter output.
   */
   const yarp::sig::Vector & output() const { return y; }
};

}

}

}

#endif


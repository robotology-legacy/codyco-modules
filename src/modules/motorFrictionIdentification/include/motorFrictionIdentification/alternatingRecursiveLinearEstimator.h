/* 
 * Copyright (C) 2013 CoDyCo
 * Author: Andrea Del Prete
 * email:  andrea.delprete@iit.it
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

#ifndef _MOTOR_FRICTION_ESTIMATOR
#define _MOTOR_FRICTION_ESTIMATOR

#include <Eigen/Core>                               // import most common Eigen types
#include <Eigen/Cholesky>
#include <motorFrictionIdentification/recursiveLinearEstimator.h>

namespace motorFrictionIdentification
{

/** Class for performing online (i.e. recursive) estimation of the parameters
 * of a linear model, alternating between estimating two subsets of the parameters (while
 * considering the other subset as known).
 * This estimation problem has a particular structure. The parameters are divided
 * into two groups \f$ x = [x_1 x_2]\f$. The first group of parameters \f$x_1\f$ has to be 
 * estimated using a subset of the samples, while the other parameters \f$x_2\f$ 
 * have to be estimated using the remaining samples. Given the model:
 * \f[
 * \begin{bmatrix} \Phi_1 & \Phi_{12} \\ \Phi_{21} & \Phi_2 \end{bmatrix} 
 * \begin{bmatrix} x_1 \\ x_2 \end{bmatrix} = 
 * \begin{bmatrix}b_1 \\ b_2\end{bmatrix}
 * \f]
 * we want to estimate the parameters as:
 * \f[
 * \hat{x}_1 = \Phi_1^+ (b_1 - \Phi_{12}\hat{x}_2)
 * \f]
 * and
 * \f[
 * \hat{x}_2 = \Phi_2^+ (b_2 - \Phi_{21}\hat{x}_1)
 * \f]
 * Actually, given that the estimation is recursive, each time we update the estimation 
 * of (e.g.) \f$x_1\f$ we are gonna use the current estimation of \f$x_2\f$, which is 
 * changing over time, so the order in which we feed the samples to the estimator affects
 * the final estimates.
 */
class AlternatingRecursiveLinearEstimator
{
protected:
    RecursiveLinearEstimator        estimator1;     ///< estimator of the first group of parameters
    RecursiveLinearEstimator        estimator2;     ///< estimator of the second group of parameters
    Eigen::VectorXd                 x;              ///< current parameter estimate
    Eigen::VectorXd                 x1;             ///< current group 1 parameter estimate
    Eigen::VectorXd                 x2;             ///< current group 2 parameter estimate
    Eigen::MatrixXd                 sigma1;         ///< current group 1 parameter estimate covariance
    Eigen::MatrixXd                 sigma2;         ///< current group 2 parameter estimate covariance
    int                             n1;             ///< dimension of the first group of parameters
    int                             n2;             ///< dimension of the second group of parameters

    /** Checks whether the input is of the desired dimensionality.
     * @param input A sample input.
     * @return True if the dimensionality is correct. */
    inline bool checkDomainSize(const Eigen::VectorXd& input) const { return input.size()==n1+n2; }

    /** Resize all matrices and vectors based on the current domain and codomain sizes. */
    void resizeAllVariables();

public:

    /** Constructor. */
    AlternatingRecursiveLinearEstimator(unsigned int numParam1=1, unsigned int numParam2=1);

    /** Provide the estimator with an example of the desired linear mapping
     * to be used for updating the estimate of the parameters of the first group.
     * @param input A sample input.
     * @param output The corresponding output. */
    void feedSampleForGroup1(const Eigen::VectorXd &input, const double &output);

    /** Provide the estimator with an example of the desired linear mapping
     * to be used for updating the estimate of the parameters of the second group.
     * @param input A sample input.
     * @param output The corresponding output. */
    void feedSampleForGroup2(const Eigen::VectorXd &input, const double &output);

    /** Update the current estimation of the parameters. */
    void updateParameterEstimation();

    /** Given an input predicts the corresponding output using the current parameter
     * estimate (remember to call updateParameterEstimation before).
     * @param input A sample input.
     * @param output Output vector containing the predicted model output. */
    void predictOutput(const Eigen::VectorXd &input, double &output) const;

    /** Reset the status of the estimator. */
    inline void reset(){ resizeAllVariables(); }

    /** Get the current estimate of the parameters x (remember to call updateParameterEstimation before).
     * @param xEst Output vector containing the current estimate of the parameters. */
    void getCurrentParameterEstimate(Eigen::VectorXd &xEst) const;

    /** Get the current estimate of the parameters x (remember to call updateParameterEstimation before).
     * @param xEst Output vector containing the current estimate of the parameters. 
     * @param sigma Covariance matrix. */
    void getCurrentParameterEstimate(Eigen::VectorXd &xEst, Eigen::MatrixXd &sigma) const;

    /** Returns the size (dimensionality) of the first group of parameters.
     * @return The size of the first group of parameters. */
    unsigned int getGroup1ParamSize() const { return this->n1; }
    
    /** Returns the size (dimensionality) of the second group of parameters.
     * @return The size of the second group of parameters. */
    unsigned int getGroup2ParamSize() const { return this->n2; }

    /** Set the size of the parameters of the first group and reset the status of the estimator.
     * @param size The desired size of the parameters of the first group. */
    inline void setGroup1ParamSize(unsigned int size) { n1 = size; resizeAllVariables(); }

    /** Set the size of the parameters of the second group and reset the status of the estimator.
     * @param size The desired size of the parameters of the second group. */
    inline void setGroup2ParamSize(unsigned int size) { n2 = size; resizeAllVariables(); }

};

}   // end namespace 

#endif

//unsigned int        m;   ///< The dimensionality of the output domain (codomain)
/** Checks whether the output is of the desired dimensionality.
    * @param output A sample output.
    * @return True if the dimensionality is correct. */
//inline bool checkCoDomainSize(const Eigen::VectorXd& output){ return output.size()==m; }

/** Validates whether the input and output are of the desired dimensionality.
    * @param input A sample input.
    * @param output The corresponding output. */
//bool checkDomainCoDomainSizes(const Eigen::VectorXd& input, const Eigen::VectorXd& output)
//{ return checkDomainSize(input) && checkCoDomainSize(output); }

/** Returns the size (dimensionality) of the output domain (codomain).
    * @return The size of the codomain. */
//unsigned int getCoDomainSize() const { return this->m; }

/** Mutator for the codomain size.
    * @param size The desired codomain size. */
//virtual void setCoDomainSize(unsigned int size) {this->m = size; }
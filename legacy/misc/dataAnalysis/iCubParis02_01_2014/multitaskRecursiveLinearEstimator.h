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

#ifndef _MULTITASK_RECURSIVE_LINEAR_ESTIMATOR
#define _MULTITASK_RECURSIVE_LINEAR_ESTIMATOR

#include <Eigen/Core>                               // import most common Eigen types
#include <Eigen/Cholesky>

/** Class for performing online (i.e. recursive) estimation of parameters
 * according to a linear model of the form:
 * \f[
 * \phi x = y,
 * \f]
 * where\f$ \phi \in R^{m\times n}\f$ is the regressor matrix, \f$ x \in R^n \f$ is 
 * the parameter vector and \f$ y \in R^m \f$ is the output vector. 
 * The parameters \f$ x \f$ are estimated in the least-square sense as:
 * \f[
 * \hat{x} = (\phi^T \phi)^{-1} \phi^T y
 * \f]
 * The estimation is performed online, that is the samples are provided as they become
 * available, and the parameter estimate is updated every time. After \f$ t \f$ input-output
 * samples (\f$ \Phi_t, Y_t\f$) have been fed into the estimator, the parameter estimate is:
 * \f[
 * \hat{x}_t = (\underbrace{\Phi_t^T \Phi_t}_{A_t})^{-1} \underbrace{\Phi_t^T Y_t}_{b_t}
 * \f]
 * To avoid storing all the samples in memory the estimator only need to store \f$ A_t \in R^{n \times n}\f$
 * and \f$ b_t \in R^n\f$, which have constant size. Actually, to improve the numerical accuracy
 * of the estimation, the Cholesky decomposition of \f$ A_t \f$ is stored, which is a triangular matrix
 * \f$ R_t \in R^{n \times n} \f$ such that \f$ A_t = R_t^T R_t \f$. A rank-1 update rule is used to
 * incrementally update the Cholesky decomposition.
 */
class multiTaskRecursiveLinearEstimator
{
protected:
    unsigned int                    n;      ///< The number of parameters
    unsigned int                    m;      ///< The number of outputs
    Eigen::VectorXd          sigma_oe;      ///< Standard deviation of the outputs (default: 1)
    Eigen::LDLT<Eigen::MatrixXd>    R;      ///< Cholesky factor of the inverse covariance matrix (i.e. A).
    Eigen::VectorXd                 x;      ///< current parameter estimate
    Eigen::VectorXd                 b;      ///< current projected output
    int                     sampleCount;    ///< Number of samples during last training routine

    /** Checks whether the input is of the desired dimensionality.
     * @param input A sample input.
     * @return True if the dimensionality is correct. */
    inline bool checkDomainSize(const Eigen::MatrixXd& input) const { return input.rows()==m && input.cols()==n; }
    
    /** Checks whether the output is of the desired dimensionality.
    * @param output A sample output.
    * @return True if the dimensionality is correct. */
    inline bool checkCoDomainSize(const Eigen::VectorXd& output){ return output.size()==m; }

    /** Validates whether the input and output are of the desired dimensionality.
    * @param input A sample input.
    * @param output The corresponding output. */
    bool checkDomainCoDomainSizes(const Eigen::MatrixXd& input, const Eigen::VectorXd& output)
    { return checkDomainSize(input) && checkCoDomainSize(output); }

    /** Resize all matrices and vectors based on the current domain and codomain sizes. */
    void resizeAllVariables(double lambda=1.0);

public:

    /** Constructor.
     * @param nParam The number of parameters to estimate.*/
    multiTaskRecursiveLinearEstimator(unsigned int nParam = 1, unsigned int nOutputs = 1, double lambda = 1.0);

    /** Provide the estimator with an example of the desired linear mapping.
     * @param input A sample input.
     * @param output The corresponding output. */
    void feedSample(const Eigen::MatrixXd &input, const Eigen::VectorXd &output);
    
    /** Provide the estimator with an example of the desired linear mapping 
     *  and update the estimated parameter
     * @param input A sample input.
     * @param output The corresponding output. */
    void feedSampleAndUpdate(const Eigen::MatrixXd &input, const Eigen::VectorXd &output);

    /** Update the current estimation of the parameters. */
    void updateParameterEstimate();

    /** Given an input predicts the corresponding output using the current parameter estimate.
     * @param input A sample input.
     * @param output Output vector containing the predicted model output. 
     * @note Remember to call updateParameterEstimate before.*/
    void predictOutput(const Eigen::MatrixXd &input, Eigen::VectorXd &output) const;

    /** Reset the status of the estimator. */
    inline void reset(){ resizeAllVariables(); }

    /** Get the current estimate of the parameters x.
     * @param xEst Output vector containing the current estimate of the parameters. 
     * @note Remember to call updateParameterEstimate before. */
    void getParameterEstimate(Eigen::VectorXd &xEst) const;
    
    /** Get the current estimate of the parameters x.
     * @param xEst Output vector containing the current estimate of the parameters. 
     * @note Remember to call updateParameterEstimate before. */
    const Eigen::VectorXd & getParameterEstimate() const;

    /** Get the current estimate of the parameters x and the covariance matrix.
     * @param xEst Output vector containing the current estimate of the parameters. 
     * @param sigma Output covariance matrix. 
     * @note Remember to call updateParameterEstimate before. */
    void getParameterEstimate(Eigen::VectorXd &xEst, Eigen::MatrixXd &sigma) const;

    /** Get the current covariance matrix.
     * @param sigma Output covariance matrix. 
     * @note Remember to call updateParameterEstimate before. */
    void getCovarianceMatrix(Eigen::MatrixXd &sigma) const;

    /** Get the current state of this estimator under the form of the matrix \f$A\f$ and
     * the vector \f$b\f$, which are defined by this equation:
     * \f[
     * \underbrace{\Phi_t^T \Phi_t}_{A_t} \hat{x}_t = \underbrace{\Phi_t^T Y_t}_{b_t}
     * \f]
     * \f$A\f$ and \f$b\f$ can be used to resume the estimation from the current state
     * at a later time.
     * @param A Output matrix filled with the inverse of the covariance matrix.
     * @param b Output vector filled with the right-hand side of the normal LS equation. */
    void getEstimationState(Eigen::MatrixXd &A, Eigen::VectorXd &b) const;

    /** Set the state of this estimator under the form of the matrix \f$A\f$ and
     * the vector \f$b\f$, which are defined by this equation:
     * \f[
     * \underbrace{\Phi_t^T \Phi_t}_{A_t} \hat{x}_t = \underbrace{\Phi_t^T Y_t}_{b_t}
     * \f]
     * \f$A\f$ and \f$b\f$ can be retrieved through the method getEstimationState.
     * @param A Inverse of the covariance matrix.
     * @param b Right-hand side vector of the normal LS equation. */
    void setEstimationState(const Eigen::MatrixXd &A, const Eigen::VectorXd &b);

    /** 
     * Get the current value of output error standard deviation.
     * The output standard deviation is used to weight the regression 
     * (the outputs (and corresponding regressor rows) are divided by
     *  the output error standard deviation and then used for estimation)
     * 
     * @param sigma_oe The standard deviation of the output error
     */
    void getOutputErrorStandardDeviation(Eigen::VectorXd &_sigma_oe);
    
    /** 
     * Get the current value of output error standard deviation.
     * The output standard deviation is used to weight the regression 
     * (the outputs (and corresponding regressor rows) are divided by
     *  the output error standard deviation and then used for estimation)
     * 
     * @param sigma_oe The standard deviation of the output error
     * 
     */
    void setOutputErrorStandardDeviation(const Eigen::VectorXd &_sigma_oe);

    
    /** Returns the size (dimensionality) of the input domain.
     * @return The size of the input domain. */
    inline unsigned int getParamSize() const { return this->n; }

    /** Set the size of the parameter vector and reset the status of the estimator.
     * @param size The desired size of the parameter vector. */
    inline void setParamSize(unsigned int size) { n = size; resizeAllVariables(); }
  

    /** Returns the size (dimensionality) of the output domain (codomain).
    * @return The size of the codomain. */
    unsigned int getOutputSize() const { return this->m; }

    /** Mutator for the codomain size.
    * @param size The desired codomain size. */
    virtual void setOutputSize(unsigned int size) {this->m = size; }
};

 
#endif


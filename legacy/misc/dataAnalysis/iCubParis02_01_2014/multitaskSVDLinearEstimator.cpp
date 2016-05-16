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

#include "multitaskSVDLinearEstimator.h"
#include <cstdio>
#include <iostream>

using namespace std;
using namespace Eigen;

multiTaskSVDLinearEstimator::multiTaskSVDLinearEstimator(unsigned int nParam, unsigned int nOutputs, double lambda) 
    : n(nParam), m(nOutputs), A(n,n), svd_A(n,n), sampleCount(0)
{ 
    resizeAllVariables(lambda);
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::feedSample(const MatrixXd &input, const VectorXd &output)
{
    bool res;
    /*
    std::cout << "m : " << m << std::endl;
    std::cout << "input total: " << std::endl << input << std::endl;
    std::cout << "output total: " << std::endl << output << std::endl;
    std::cout << "sigma_oe    : " << std::endl << sigma_oe << std::endl;
    */
    
    assert(checkDomainCoDomainSizes(input,output));
    assert(input.rows() == m);
    for(int out=0; out < m; out++ ) {
        //std::cout << "out: " << out << " m: " << m << std::endl;
        ///< update the Cholesky decomposition of the inverse covariance matrix
        assert(input.row(out).transpose().rows() == A.rows());
        assert((input.row(out)/sigma_oe(out)).transpose().rows() == A.rows());
        //std::cout << "input ( row "  << out << " ) : " << std::endl << input.row(out)/sigma_oe(out) << std::endl;
        //std::cout << " adding   ( row "  << out << " ) : "<< std::endl << input.row(out).transpose()*input.row(out)/(sigma_oe(out)*sigma_oe(out)) << std::endl;
        A += input.row(out).transpose()*input.row(out)/(sigma_oe(out)*sigma_oe(out));
        //std::cout << "A: " << std::endl << A << std::endl;
        ///< update the right hand side of the equation
        b += input.row(out)*(output(out)/(sigma_oe(out)*sigma_oe(out)));
    }
    
    if( sampleCount > 2*n ) {
        svd_A = svd_A.compute(A, ComputeFullU | ComputeFullV);
        /*
        res = svd_A.computeU();
        assert(res);
        res = svd_A.computeV();
        assert(res);
        */
    }
    sampleCount++;
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::feedSampleAndUpdate(const MatrixXd &input, const VectorXd &output)
{
    feedSample(input,output);
    updateParameterEstimate();
    return;
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::predictOutput(const MatrixXd &input, VectorXd &output) const
{
    assert(checkDomainSize(input));
    output = input*x;
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::getParameterEstimate(VectorXd &xEst) const
{
    assert(checkDomainSize(xEst));
    xEst = x;
}

/*************************************************************************************************/
const VectorXd & multiTaskSVDLinearEstimator::getParameterEstimate() const
{
    return x;
}


/*************************************************************************************************/
void multiTaskSVDLinearEstimator::getCovarianceMatrix(MatrixXd &sigma) const
{
    /*
    assert(sigma.cols()==n && sigma.rows()==n);
    ///< if there are not enough sample to perform the estimation set covariance a very high value
    ///< @todo Rather than checking the # of sample I should check the rank of the inverse covariance matrix A
    if(sampleCount<(int)n)  
    {
        sigma = MatrixXd::Constant(n,n,1e10);
        return;
    }
    ///< Invert A by solving n times the system: A*x=e_i, 
    ///< where e_i is a vector with all elements equal to 0, except for the i-th element, which is equal to 1
    VectorXd e_i(n);
    for(unsigned int i=0; i<n; i++)
    {
        e_i.setZero();
        e_i[i] = 1.0;
        if(!R.solveInPlace(e_i))
            printf("Error while computing covariance matrix in loop %d\n", i);
        sigma.col(i) = e_i;

        // If the covariance is exactly zero it means that there are not enough samples to estimate
        // the relative parameter, so actually the covariance is infinite
        if(sigma(i,i)==0.0)
            sigma(i,i) = 1e10;
    }*/
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::getParameterEstimate(VectorXd &xEst, MatrixXd &sigma) const
{
    assert(sigma.cols()==n && sigma.rows()==n);
    getParameterEstimate(xEst);
    getCovarianceMatrix(sigma);
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::getEstimationState(MatrixXd &_A, VectorXd &bOut) const
{
    assert(_A.cols()==n && _A.rows()==n);
    assert(b.size()==n);
    _A = A;
    bOut = b;
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::setEstimationState(const MatrixXd &Anew, const VectorXd &bNew)
{
    assert(A.cols()==n && A.rows()==n);
    assert(b.size()==n);
    A = Anew;
    b = bNew;
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::getOutputErrorStandardDeviation(Eigen::VectorXd &sigma_oe_output)
{
    sigma_oe_output = sigma_oe;
}
  
/*************************************************************************************************/
void multiTaskSVDLinearEstimator::setOutputErrorStandardDeviation(const Eigen::VectorXd &sigma_oe_input)
{
    assert(sigma_oe.size() == m);
    assert(sigma_oe_input.size() == m);
    sigma_oe = sigma_oe_input;
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::updateParameterEstimate()
{
    if( sampleCount > 3*n ) {
        x = svd_A.solve(b);
    }
}

/*************************************************************************************************/
void multiTaskSVDLinearEstimator::resizeAllVariables(double lambda)
{
    A = lambda*MatrixXd::Identity(n,n);
    svd_A = Eigen::JacobiSVD<Eigen::MatrixXd>(n,n);
    b.resize(n);
    b.setZero();
    x.resize(n);
    x.setZero();
    sigma_oe.resize(m);
    sigma_oe.setOnes();
}

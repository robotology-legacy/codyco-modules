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

#include <motorFrictionIdentification/alternatingRecursiveLinearEstimator.h>
#include <cstdio>

using namespace std;
using namespace Eigen;
using namespace motorFrictionIdentification;

AlternatingRecursiveLinearEstimator::AlternatingRecursiveLinearEstimator(unsigned int nParam1, unsigned int nParam2) 
    : n1(nParam1), n2(nParam2)
{ 
    resizeAllVariables();
}

/*************************************************************************************************/
void AlternatingRecursiveLinearEstimator::feedSampleForGroup1(const VectorXd &input, const double &output)
{
    assert(checkDomainSize(input));
    estimator1.feedSample(input.segment(0,n1), output-input.segment(n1,n2).dot(x.segment(n1,n2)));
}

/*************************************************************************************************/
void AlternatingRecursiveLinearEstimator::feedSampleForGroup2(const VectorXd &input, const double &output)
{
    assert(checkDomainSize(input));
    estimator2.feedSample(input.segment(n1,n2), output-input.segment(0,n1).dot(x.segment(0,n1)));
}

/*************************************************************************************************/
void AlternatingRecursiveLinearEstimator::predictOutput(const VectorXd &input, double &output) const
{
    assert(checkDomainSize(input));
    output = input.dot(x);
}

/*************************************************************************************************/
void AlternatingRecursiveLinearEstimator::getCurrentParameterEstimate(VectorXd &xEst) const
{
    assert(checkDomainSize(xEst));
    xEst = x;
}

/*************************************************************************************************/
void AlternatingRecursiveLinearEstimator::getCurrentParameterEstimate(VectorXd &xEst, MatrixXd &sigma) const
{
    assert(sigma.cols()==n1+n2 && sigma.rows()==n1+n2);
    getCurrentParameterEstimate(xEst);
    sigma.block(0,0,n1,n1) = sigma1;
    sigma.block(n1,n1,n2,n2) = sigma2;
}

/*************************************************************************************************/
void AlternatingRecursiveLinearEstimator::updateParameterEstimation()
{
    estimator1.updateParameterEstimation();
    estimator2.updateParameterEstimation();
    estimator1.getCurrentParameterEstimate(x1, sigma1);
    estimator2.getCurrentParameterEstimate(x2, sigma2);
    x.segment(0,n1) = x1;
    x.segment(n1,n2) = x2;
}

/*************************************************************************************************/
void AlternatingRecursiveLinearEstimator::resizeAllVariables()
{
    estimator1.setParamSize(n1);
    estimator2.setParamSize(n2);
    x1.resize(n1);
    x2.resize(n2);
    x.resize(n1+n2);
    sigma1.resize(n1,n1);
    sigma2.resize(n2,n2);
}

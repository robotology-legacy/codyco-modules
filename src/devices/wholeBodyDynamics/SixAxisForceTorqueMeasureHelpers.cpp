#include "SixAxisForceTorqueMeasureHelpers.h"
#include <iDynTree/Core/EigenHelpers.h>

namespace wholeBodyDynamics
{

SixAxisForceTorqueMeasureProcessor::SixAxisForceTorqueMeasureProcessor()
{
    // Initialize the affice function to be the identity
    toEigen(m_secondaryCalibrationMatrix).setIdentity();
    toEigen(m_offset).setZero();
}

iDynTree::Matrix6x6& SixAxisForceTorqueMeasureProcessor::secondaryCalibrationMatrix()
{
    return m_secondaryCalibrationMatrix;
}


const iDynTree::Matrix6x6& SixAxisForceTorqueMeasureProcessor::secondaryCalibrationMatrix() const
{
    return m_secondaryCalibrationMatrix;
}

iDynTree::Wrench& SixAxisForceTorqueMeasureProcessor::offset()
{
    return m_offset;
}

const iDynTree::Wrench& SixAxisForceTorqueMeasureProcessor::offset() const
{
    return m_offset;
}

iDynTree::Wrench SixAxisForceTorqueMeasureProcessor::filt(const iDynTree::Wrench& input) const
{
    Eigen::Matrix<double,6,1> retEig = toEigen(m_secondaryCalibrationMatrix)*toEigen(input)-toEigen(m_offset);

    iDynTree::Wrench ret;
    fromEigen(ret,retEig);

    return ret;
}

}



#ifndef __MODEL3T_H__
#define __MODEL3T_H__


#include "orcisir/ISIRModel.h"

class Model3T: public orcisir::ISIRModel
{
public:
    Model3T(const std::string& robotName);
    virtual ~Model3T();


    virtual const Eigen::VectorXd&      getJointPositions()     const;
    virtual const Eigen::VectorXd&      getJointVelocities()    const;
    virtual const Eigen::Displacementd& getFreeFlyerPosition()  const;
    virtual const Eigen::Twistd&        getFreeFlyerVelocity()  const;

    virtual int                      nbSegments()           const;
    virtual const Eigen::VectorXd&   getActuatedDofs()      const;
    virtual const Eigen::VectorXd&   getJointLowerLimits()  const;
    virtual const Eigen::VectorXd&   getJointUpperLimits()  const;

    //CoM
    virtual double                                        getMass()             const;
    virtual const Eigen::Vector3d&                        getCoMPosition()      const;
    virtual const Eigen::Vector3d&                        getCoMVelocity()      const;
    virtual const Eigen::Vector3d&                        getCoMJdotQdot()      const;
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobian()      const;
    virtual const Eigen::Matrix<double,3,Eigen::Dynamic>& getCoMJacobianDot()   const;

    //dynamic/static equation terms
    virtual const Eigen::MatrixXd&   getInertiaMatrix()         const;
    virtual const Eigen::MatrixXd&   getInertiaMatrixInverse()  const;
    virtual const Eigen::MatrixXd&   getDampingMatrix()         const;
    virtual const Eigen::VectorXd&   getNonLinearTerms()        const;
    virtual const Eigen::VectorXd&   getLinearTerms()           const;
    virtual const Eigen::VectorXd&   getGravityTerms()          const;

    //segment data
    virtual const Eigen::Displacementd&                     getSegmentPosition(int index)           const;
    virtual const Eigen::Twistd&                            getSegmentVelocity(int index)           const;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&   getSegmentJacobian(int index)           const;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&   getSegmentJdot(int index)               const;
    virtual const Eigen::Twistd&                            getSegmentJdotQdot(int index)           const;
    virtual const Eigen::Matrix<double,6,Eigen::Dynamic>&   getJointJacobian(int index)             const;
    virtual double                                          getSegmentMass(int index)               const;
    virtual const Eigen::Vector3d&                          getSegmentCoM(int index)                const;
    virtual const Eigen::Matrix<double, 6, 6>&              getSegmentMassMatrix(int index)         const;
    virtual const Eigen::Vector3d&                          getSegmentMomentsOfInertia(int index)   const;
    virtual const Eigen::Rotation3d&                        getSegmentInertiaAxes(int index)        const;


protected:
    virtual void  doSetJointPositions(const Eigen::VectorXd& q);
    virtual void  doSetJointVelocities(const Eigen::VectorXd& q_dot);
    virtual void  doSetFreeFlyerPosition(const Eigen::Displacementd& H_root);
    virtual void  doSetFreeFlyerVelocity(const Eigen::Twistd& T_root);

    virtual int                 doGetSegmentIndex(const std::string& name)  const;
    virtual const std::string&  doGetSegmentName(int index)                 const;

    virtual void update();

private:
    struct Pimpl;
    boost::shared_ptr<Pimpl> pimpl; //where to save all the data



//additional methods
public:
    void testConstantData();
    void testVariableData();

};


extern "C"
{
    orcisir::ISIRModel* Create(const std::string& robotName)
    {
        return new Model3T(robotName);
    }
}


#endif




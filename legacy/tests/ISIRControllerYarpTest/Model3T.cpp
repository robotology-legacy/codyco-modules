#include "Model3T.h"

using namespace Eigen;

#include <string>
#include <sstream>

#include <iostream>


#include <Eigen/Lgsm>
#include<Eigen/StdVector> //Mandatory for vector fixed-size Eigen element (like Twistd)
                          //else it raise assertion of bad alignement

#ifndef __EIGENDEF_FOR_VECTOR__
#define __EIGENDEF_FOR_VECTOR__
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Twistd)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Matrix<double, 6, 6>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Rotation3d)
#endif



struct Model3T::Pimpl
{
public:
    VectorXd      q;
    VectorXd      dq;
    Displacementd Hroot;
    Twistd        Troot;

    VectorXd actuatedDofs;
    VectorXd lowerLimits;
    VectorXd upperLimits;

    Vector3d                     comPosition;
    Vector3d                     comVelocity;
    Vector3d                     comJdotQdot;
    Matrix< double,3, Dynamic >  comJacobian;
    Matrix< double,3, Dynamic >  comJacobianDot;

    MatrixXd M;
    MatrixXd Minv;
    MatrixXd B;
    VectorXd n;
    VectorXd l;
    VectorXd g;

    std::vector< Displacementd >            segPosition;
    std::vector< Twistd >                   segVelocity;
    std::vector< Matrix<double,6,Dynamic> > segJacobian;
    std::vector< Matrix<double,6,Dynamic> > segJdot;
    std::vector< Twistd >                   segJdotQdot;
    std::vector< Matrix<double,6,Dynamic> > jointJacobian;
    std::vector< double >                   segMass;
    std::vector< Vector3d >                 segCoM;
    std::vector< Matrix<double, 6, 6> >     segMassMatrix;
    std::vector< Matrix<double, 6, 6> >     segNLEffects;
    std::vector< Vector3d >                 segMomentsOfInertia;
    std::vector< Rotation3d >               segInertiaAxes;
    std::vector< std::string >              segName;

    Pimpl(const std::string& robotName)
    {
        q     = VectorXd::Zero(3);
        dq    = VectorXd::Zero(3);
        Hroot = Displacementd(0,0,0);   //CONSTANT
        Troot = Twistd(0,0,0,0,0,0);    //CONSTANT

        actuatedDofs = VectorXd::Ones(3);           //CONSTANT
        lowerLimits  = VectorXd::Constant(3, -10.); //CONSTANT
        upperLimits  = VectorXd::Constant(3,  10.); //CONSTANT

        comPosition     = Vector3d(0,0,0);
        comVelocity     = Vector3d(0,0,0);
        comJdotQdot     = Vector3d(0,0,0);
        comJacobian     = Matrix<double,3,3>::Zero();   //CONSTANT
        comJacobianDot  = Matrix<double,3,3>::Zero();   //CONSTANT

        M    = MatrixXd::Zero(3,3);
        Minv = MatrixXd::Zero(3,3);
        B    = MatrixXd::Identity(3,3) * 0.001;
        n    = VectorXd::Zero(3);
        l    = VectorXd::Zero(3);
        g    = VectorXd::Zero(3);

        for (int i=0; i<4; i++)
        {
            segPosition.push_back(Displacementd(0,0,0));
            segVelocity.push_back(Twistd(0,0,0,0,0,0));
            segJacobian.push_back( Matrix<double,6,3>::Zero() );    //CONSTANT
            segJdot.push_back( Matrix<double,6,3>::Zero() );        //CONSTANT
            segJdotQdot.push_back(Twistd(0,0,0,0,0,0));             //CONSTANT
            jointJacobian.push_back( Matrix<double,6,3>::Zero() );  //CONSTANT
            segMass.push_back(1.);                                  //CONSTANT
            segCoM.push_back( Vector3d(0,0,0) );                    //CONSTANT
            segMassMatrix.push_back( Matrix<double,6,6>::Zero() );  //CONSTANT
            segMomentsOfInertia.push_back( Vector3d(0.1,0.1,0.1) ); //CONSTANT
            segInertiaAxes.push_back( Rotation3d(1,0,0,0) );        //CONSTANT

            std::stringstream name;
            name <<robotName<<".segment_"<<i;
            segName.push_back( name.str() );                        //CONSTANT

            segNLEffects.push_back( Matrix<double,6,6>::Zero() );  //CONSTANT
        }


        ////////////////////////////////////////////////////////////////////////

        segJacobian[0]   << 0,0,0,   0,0,0,   0,0,0,   0,0,0,   0,0,0,   0,0,0;
        jointJacobian[0] << 0,0,0,   0,0,0,   0,0,0,   0,0,0,   0,0,0,   0,0,0;

        segJacobian[1]   << 0,0,0,   0,0,0,   0,0,0,   1,0,0,   0,0,0,   0,0,0;
        jointJacobian[1] << 0,0,0,   0,0,0,   0,0,0,   1,0,0,   0,0,0,   0,0,0;

        segJacobian[2]   << 0,0,0,   0,0,0,   0,0,0,   1,0,0,   0,1,0,   0,0,0;
        jointJacobian[2] << 0,0,0,   0,0,0,   0,0,0,   1,0,0,   0,1,0,   0,0,0;

        segJacobian[3]   << 0,0,0,   0,0,0,   0,0,0,   1,0,0,   0,1,0,   0,0,1;
        jointJacobian[3] << 0,0,0,   0,0,0,   0,0,0,   1,0,0,   0,1,0,   0,0,1;

        for (int i=0; i<4; i++)
        {
            segMassMatrix[i].diagonal() << segMomentsOfInertia[i][0], segMomentsOfInertia[i][1], segMomentsOfInertia[i][2], segMass[i], segMass[i], segMass[i];
        }

        // We assume that the gravity is along -Z and is -9.81!
        g << 0, 0, 9.80665*segMass[3];

        for (int i=1; i<4; i++)
        {
            M += segJacobian[i].transpose() * segMassMatrix[i] * segJacobian[i];
        }

        Minv = M.inverse();

        comJacobian = (   segMass[0]*segJacobian[0].bottomRows<3>()
                        + segMass[1]*segJacobian[1].bottomRows<3>()
                        + segMass[2]*segJacobian[2].bottomRows<3>()
                        + segMass[3]*segJacobian[3].bottomRows<3>()
                        )/( segMass[0]+segMass[1]+segMass[2]+segMass[3] );
    }

};




Model3T::Model3T(const std::string& robotName)
    : orcisir::ISIRModel(robotName, 3, false)
    , pimpl( new Pimpl(robotName))
{
    update();
}

Model3T::~Model3T()
{

}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
const VectorXd&      Model3T::getJointPositions()     const
{
    return pimpl->q;
}

const VectorXd&      Model3T::getJointVelocities()    const
{
    return pimpl->dq;
}

const Displacementd& Model3T::getFreeFlyerPosition()  const
{
    return pimpl->Hroot;
}

const Twistd&        Model3T::getFreeFlyerVelocity()  const
{
    return pimpl->Troot;
}

int               Model3T::nbSegments()           const
{
    return 4;
}

const VectorXd&   Model3T::getActuatedDofs()      const
{
    return pimpl->actuatedDofs;
}

const VectorXd&   Model3T::getJointLowerLimits()  const
{
    return pimpl->lowerLimits;
}

const VectorXd&   Model3T::getJointUpperLimits()  const
{
    return pimpl->upperLimits;
}

//CoM
double                          Model3T::getMass()             const
{
    double mass=0;
    for (int i=0; i<nbSegments(); i++)
    {
        mass += getSegmentMass(i);
    }
    return mass;
}

const Vector3d&                 Model3T::getCoMPosition()      const
{
    return pimpl->comPosition;
}

const Vector3d&                 Model3T::getCoMVelocity()      const
{
    return pimpl->comVelocity;
}

const Vector3d&                 Model3T::getCoMJdotQdot()      const
{
    return pimpl->comJdotQdot;
}

const Matrix<double,3,Dynamic>& Model3T::getCoMJacobian()      const
{
    return pimpl->comJacobian;
}

const Matrix<double,3,Dynamic>& Model3T::getCoMJacobianDot()   const
{
    return pimpl->comJacobianDot;
}

//dynamic/static equation terms
const MatrixXd&   Model3T::getInertiaMatrix()         const
{
    return pimpl->M;
}

const MatrixXd&   Model3T::getInertiaMatrixInverse()  const
{
    return pimpl->Minv;
}

const MatrixXd&   Model3T::getDampingMatrix()         const
{
    return pimpl->B;
}

const VectorXd&   Model3T::getNonLinearTerms()        const
{
    return pimpl->n;
}

const VectorXd&   Model3T::getLinearTerms()           const
{
    return pimpl->l;
}

const VectorXd&   Model3T::getGravityTerms()          const
{
    return pimpl->g;
}

//segment data
const Displacementd&              Model3T::getSegmentPosition(int index)           const
{
    return pimpl->segPosition[index];
}

const Twistd&                     Model3T::getSegmentVelocity(int index)           const
{
    return pimpl->segVelocity[index];
}

const Matrix<double,6,Dynamic>&   Model3T::getSegmentJacobian(int index)           const
{
    return pimpl->segJacobian[index];
}

const Matrix<double,6,Dynamic>&   Model3T::getSegmentJdot(int index)               const
{
    return pimpl->segJdot[index];
}

const Twistd&                     Model3T::getSegmentJdotQdot(int index)           const

{
    return pimpl->segJdotQdot[index];
}

const Matrix<double,6,Dynamic>&   Model3T::getJointJacobian(int index)             const

{
    return pimpl->jointJacobian[index];
}

double                            Model3T::getSegmentMass(int index)               const

{
    return pimpl->segMass[index];
}

const Vector3d&                   Model3T::getSegmentCoM(int index)                const

{
    return pimpl->segCoM[index];
}

const Matrix<double, 6, 6>&       Model3T::getSegmentMassMatrix(int index)         const

{
    return pimpl->segMassMatrix[index];
}

const Vector3d&                   Model3T::getSegmentMomentsOfInertia(int index)   const

{
    return pimpl->segMomentsOfInertia[index];
}


const Rotation3d&                 Model3T::getSegmentInertiaAxes(int index)        const

{
    return pimpl->segInertiaAxes[index];
}



void  Model3T::doSetJointPositions(const VectorXd& q)
{
    pimpl->q = q;
    update();
}

void  Model3T::doSetJointVelocities(const VectorXd& q_dot)
{
    pimpl->dq = q_dot;
    update();
}

void  Model3T::doSetFreeFlyerPosition(const Displacementd& H_root)
{
    pimpl->Hroot = H_root;

    update();
}

void  Model3T::doSetFreeFlyerVelocity(const Twistd& T_root)
{
    pimpl->Troot = T_root;

    update();
}

int                 Model3T::doGetSegmentIndex(const std::string& name)  const
{
    for ( int i=0; i<nbSegments(); i++ )
    {
        if (pimpl->segName[i] == name)
        {
            return i;
        }
    }

    throw( std::string("error, cannot find segment with name ") + name );
}

const std::string&  Model3T::doGetSegmentName(int index)                 const
{
    return pimpl->segName[index];
}




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Model3T::update()
{
    // update segments positions
    pimpl->segPosition[1].x() =pimpl->q[0];

    pimpl->segPosition[2].x() =pimpl->q[0];
    pimpl->segPosition[2].y() =pimpl->q[1];

    pimpl->segPosition[3].x() =pimpl->q[0];
    pimpl->segPosition[3].y() =pimpl->q[1];
    pimpl->segPosition[3].z() =pimpl->q[2];

    // update segments velocities
    pimpl->segVelocity[1].vx() =pimpl->dq[0];

    pimpl->segVelocity[2].vx() =pimpl->dq[0];
    pimpl->segVelocity[2].vy() =pimpl->dq[1];

    pimpl->segVelocity[3].vx() =pimpl->dq[0];
    pimpl->segVelocity[3].vy() =pimpl->dq[1];
    pimpl->segVelocity[3].vz() =pimpl->dq[2];

    //
    pimpl->comPosition = ( pimpl->segMass[0] * pimpl->segPosition[0].getTranslation()
                         + pimpl->segMass[1] * pimpl->segPosition[1].getTranslation()
                         + pimpl->segMass[2] * pimpl->segPosition[2].getTranslation()
                         + pimpl->segMass[3] * pimpl->segPosition[3].getTranslation()
                         ) / getMass();

    pimpl->comVelocity = ( pimpl->segMass[0] * pimpl->segVelocity[0].getLinearVelocity()
                         + pimpl->segMass[1] * pimpl->segVelocity[1].getLinearVelocity()
                         + pimpl->segMass[2] * pimpl->segVelocity[2].getLinearVelocity()
                         + pimpl->segMass[3] * pimpl->segVelocity[3].getLinearVelocity()
                         ) / getMass();


    updateMatricesForReducedProblem(); //To update the matrices for the reduced problem of parent class ISIRModel
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

void Model3T::testConstantData()
{
    std::cout<<"nb segments: "<<nbSegments()<<"\n";     //CONSTANT
    std::cout<<"actuated: "<<getActuatedDofs()<<"\n";   //CONSTANT
    std::cout<<"lower: "<<getJointLowerLimits()<<"\n";  //CONSTANT
    std::cout<<"upper: "<<getJointUpperLimits()<<"\n";  //CONSTANT

    std::cout<<"total mass: "<<getMass()<<"\n";

    std::cout<<"M: "<<getInertiaMatrix()<<"\n";
    std::cout<<"Minv: "<<getInertiaMatrixInverse()<<"\n";
    std::cout<<"B: "<<getDampingMatrix()<<"\n";
    std::cout<<"l: "<<getLinearTerms()<<"\n";

    for (int i=0; i<nbSegments(); i++)
    {
        std::cout<<"name ("<<i<<"): "<<getSegmentName(i)<<"\n";
        std::cout<<"index ("<<i<<"): "<<getSegmentIndex(getSegmentName(i))<<"\n";
        std::cout<<"Jseg ("<<i<<"): "<<getSegmentJacobian(i)<<"\n";
        std::cout<<"dJseg ("<<i<<"): "<<getSegmentJdot(i)<<"\n";
        std::cout<<"dJsegdq ("<<i<<"): "<<getSegmentJdotQdot(i)<<"\n";
        std::cout<<"Jjoint ("<<i<<"): "<<getJointJacobian(i)<<"\n";
        std::cout<<"segmass ("<<i<<"): "<<getSegmentMass(i)<<"\n";
        std::cout<<"segcom ("<<i<<"): "<<getSegmentCoM(i)<<"\n";
        std::cout<<"segMassMat ("<<i<<"): "<<getSegmentMassMatrix(i)<<"\n";
        std::cout<<"segMoI ("<<i<<"): "<<getSegmentMomentsOfInertia(i)<<"\n";
        std::cout<<"segIAxes ("<<i<<"): "<<getSegmentInertiaAxes(i)<<"\n";
    }
}



void Model3T::testVariableData()
{
    std::cout<<"q: "<<getJointPositions()<<"\n";
    std::cout<<"dq: "<<getJointVelocities()<<"\n";
    //std::cout<<"Hroot: "<<getFreeFlyerPosition()<<"\n";   // meaningless because fixed root
    //std::cout<<"Troot: "<<getFreeFlyerVelocity()<<"\n";   //

    std::cout<<"compos: "<<getCoMPosition()<<"\n";
    std::cout<<"comvel: "<<getCoMVelocity()<<"\n";

    for (int i=0; i<nbSegments(); i++)
    {
        std::cout<<"segpos ("<<i<<"): "<<getSegmentPosition(i)<<"\n";
        std::cout<<"segvel ("<<i<<"): "<<getSegmentVelocity(i)<<"\n";
    }

    std::cout<<"n: "<<getNonLinearTerms()<<"\n";
    std::cout<<"g: "<<getGravityTerms()<<"\n";
}







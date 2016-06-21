#ifndef INVERSEKINEMATICSNLP_H
#define INVERSEKINEMATICSNLP_H

#include <IpTNLP.hpp>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/ClassicalAcc.h>
#include <iDynTree/Core/Transform.h>

#include <map> //if moving to c++ use unordered map here which is faster

namespace kinematics {
    class InverseKinematicsNLP;
    class InverseKinematicsData;
}

class kinematics::InverseKinematicsNLP : public Ipopt::TNLP {

    struct FrameInfo {
        iDynTree::Transform transform;
        iDynTree::MatrixDynSize jacobian;
        iDynTree::MatrixFixSize<4, 3> quaternionDerivativeMap;
    };
    typedef std::map<int, FrameInfo> FrameInfoMap;

    InverseKinematicsData& m_data;

    bool updateState(const Ipopt::Number * x);

    //Buffers and variables used in the optimization
    iDynTree::MatrixDynSize transformWithQuaternionJacobianBuffer;
    iDynTree::VectorDynSize dofsSizeZero;
    iDynTree::Twist baseZeroVelocity;
    iDynTree::ClassicalAcc baseZeroAcceleration;
    iDynTree::MatrixFixSize<4, 3> quaternionDerivativeMapBuffer;
    iDynTree::MatrixFixSize<3, 4> quaternionDerivativeInverseMapBuffer;

    FrameInfoMap constraintsInfo;
    FrameInfoMap targetsInfo;

    double jointCostWeight;

    void initializeInternalData(Ipopt::Index n, Ipopt::Index m);

    enum ComputeContraintJacobianOption {
        ComputeContraintJacobianOptionLinearPart = 1,
        ComputeContraintJacobianOptionAngularPart = 1 << 1,
    };

    void computeConstraintJacobian(const iDynTree::MatrixDynSize& transformJacobian,
                                   const iDynTree::MatrixFixSize<4, 3>& quaternionDerivativeMapBuffer,
                                   const iDynTree::MatrixFixSize<3, 4>& quaternionDerivativeInverseMapBuffer,
                                   const int computationOption,
                                   iDynTree::MatrixDynSize& constraintJacobianBuffer);

    void omegaToRPYParameters(const iDynTree::Vector3& rpyAngles,
                              iDynTree::Matrix3x3& map);

public:
    InverseKinematicsNLP(InverseKinematicsData& data);

#pragma mark - IpOpt methods

    virtual ~InverseKinematicsNLP();

    virtual bool get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                              Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style);

    virtual bool get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                 Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u);

    virtual bool get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                    bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                    Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda);

    virtual bool eval_f(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Number& obj_value);

    virtual bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                             Ipopt::Number* grad_f);

    virtual bool eval_g(Ipopt::Index n, const Ipopt::Number* x,
                        bool new_x, Ipopt::Index m, Ipopt::Number* g);

    virtual bool eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                            Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                            Ipopt::Index *jCol, Ipopt::Number* values);

    virtual bool eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                        Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                        bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                        Ipopt::Index* jCol, Ipopt::Number* values);

    virtual void finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                   const Ipopt::Number* x, const Ipopt::Number* z_L,
                                   const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                   const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                   const Ipopt::IpoptData* ip_data,
                                   Ipopt::IpoptCalculatedQuantities* ip_cq);

    void testDerivatives(const iDynTree::VectorDynSize& derivativePoint, int frameIndex, double epsilon, double tolerance, int parametrization);
};

#endif /* end of include guard: INVERSEKINEMATICSNLP_H */

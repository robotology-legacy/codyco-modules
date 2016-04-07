#include "Solver.h"

#include "SolverData.h"
#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>
#include <IpTNLPAdapter.hpp>
#include <cassert>
#include <iDynTree/HighLevel/DynamicsComputations.h>
#include <iDynTree/Core/Transform.h>
#include <Eigen/Core>

using namespace Ipopt;

Solver::Solver(SolverData &data)
: m_data(data) {}

bool Solver::updateState(const Ipopt::Number *x)
{
    for (unsigned index = 0; index < m_data.variableToDoFMapping.size(); ++index) {
        m_data.allJoints(m_data.variableToDoFMapping[index]) = x[index];
    }
    return m_data.dynamics.setRobotState(m_data.allJoints,
                                         m_data.dofsSizeZero,
                                         m_data.dofsSizeZero,
                                         m_data.world_gravity);
}

bool Solver::get_nlp_info(Ipopt::Index& n, Ipopt::Index& m, Ipopt::Index& nnz_jac_g,
                                                   Ipopt::Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = m_data.optimVariableSize;
    m = m_data.constraintsSize;

    nnz_jac_g = m * n; //for now just treat everything as dense
    nnz_h_lag = n * n; //for now just treat everything as dense

    index_style = C_STYLE;
    return true;
}

bool Solver::get_bounds_info(Ipopt::Index n, Ipopt::Number* x_l, Ipopt::Number* x_u,
                                                      Ipopt::Index m, Ipopt::Number* g_l, Ipopt::Number* g_u)
{
    for (Index i = 0; i < n; i++) {
        x_l[i] = -2e+19; //nlp_lower_bound_inf; //I cannot find the definition of this variable!
        x_u[i] =  2e+19;
    }
    //CoM constraints
    g_l[0] = g_u[0] = m_data.comDes[0];
    g_l[1] = g_u[1] = m_data.comDes[1];
    g_l[2] = g_u[2] = m_data.comDes[2];

    if (m_data.feetInContact == BOTH_FEET_IN_CONTACT) {
        // relative transform position constraint
        iDynTree::Position position = m_data.right_X_left.getPosition();
        iDynTree::Rotation rotation = m_data.right_X_left.getRotation();
        g_l[3] = g_u[3] = position(0);
        g_l[4] = g_u[4] = position(1);
        g_l[5] = g_u[5] = position(2);

        //TODO:        g_l[3] = g_u[3] = rotation.getQuaterion(0);
        g_l[6] = g_u[6] = rotation(0,0);
        g_l[7] = g_u[7] = rotation(0,0);
        g_l[8] = g_u[8] = rotation(0,0);
        g_l[9] = g_u[9] = rotation(0,0);
    }

    return true;
}

bool Solver::get_starting_point(Ipopt::Index n, bool init_x, Ipopt::Number* x,
                                                         bool init_z, Ipopt::Number* z_L, Ipopt::Number* z_U,
                                                         Ipopt::Index m, bool init_lambda, Ipopt::Number* lambda)
{
    //Let's assert what we expect: only initial x_0
    assert(init_x);
    assert(!init_z);
    assert(!init_lambda);

    for (Index i = 0; i < n; ++i) {
        x[i] = m_data.qDes[i];
    }
    return true;
}

bool Solver::eval_f(Ipopt::Index n, const Ipopt::Number* x,
                                             bool new_x, Ipopt::Number& obj_value)
{
    //What to do if new_x == false?
    //Computing objective: 1/2 * || q - q_des ||^2
    if (new_x) {
        if (!updateState(x)) return false;
    }

    Eigen::Map<const Eigen::VectorXd> q(x, n);
    m_data.qError = q - m_data.qDes;
    obj_value = 0.5 * m_data.qError.transpose() * m_data.qError;
    return true;
}

bool Solver::eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                                  Ipopt::Number* grad_f)
{
    if (new_x) {
        if (!updateState(x)) return false;
    }
    //Gradient of the objective.
    //Simply q - qDes
    Eigen::Map<const Eigen::VectorXd> q(x, n);
    Eigen::Map<Eigen::VectorXd> gradient(grad_f, n);
    gradient = q - m_data.qDes;
    return true;
}

bool Solver::eval_g(Ipopt::Index n, const Ipopt::Number* x,
                    bool new_x, Ipopt::Index m, Ipopt::Number* g)
{
    bool result = true;
    if (new_x) {
        result = result && updateState(x);
    }

    // CoM forward kinematic
    iDynTree::Position com = m_data.dynamics.getCenterOfMass();
    g[0] = com(0);
    g[1] = com(1);
    g[2] = com(2);

    if (m_data.feetInContact == BOTH_FEET_IN_CONTACT) {
        //TODO:
        iDynTree::Transform kinematic = m_data.dynamics.getRelativeTransform("l_sole", "r_sole");
        iDynTree::Position position = kinematic.getPosition();
        iDynTree::Rotation rotation = kinematic.getRotation();
        g[3] = position(0);
        g[4] = position(1);
        g[5] = position(2);

        //TODO:        g_l[3] = g_u[3] = rotation.getQuaterion(0);
        g[6] = rotation(0,0);
        g[7] = rotation(0,0);
        g[8] = rotation(0,0);
        g[9] = rotation(0,0);
    }
    return result;
}

bool Solver::eval_jac_g(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                                 Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index* iRow,
                                                 Ipopt::Index *jCol, Ipopt::Number* values)
{
    bool result = true;
    if (new_x) {
        result = result && updateState(x);
    }
    if (!values) {
        //Sparsity structure of the Jacobian
        for (Index row = 0; row < 3; row++) {
            for (Index col = 0; col < n; col++) {
                iRow[row * n + col] = row;
                jCol[row * n + col] = col;
            }
        }
    } else {
        //Actual Jacobian
        // CoM Jacobian
        result = result && m_data.dynamics.getCenterOfMassJacobian(m_data.comJacobian);
        for (Index row = 0; row < 3; row++) {
            for (Index col = 0; col < n; col++) {
                values[row * n + col] = m_data.comJacobian(row, 6 + m_data.variableToDoFMapping[col]);
            }
        }

        if (m_data.feetInContact == BOTH_FEET_IN_CONTACT) {

        }
    }
    return result;
}

bool Solver::eval_h(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                                             Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number* lambda,
                                             bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index* iRow,
                                             Ipopt::Index* jCol, Ipopt::Number* values)
{
    //this method is not called: I'm using an approximation of the hessian
    //in order to use this method I need the hessian of the constraints
    return true;
//    bool result = true;
//    if (new_x) {
//        result = result && updateState(x);
//    }
//    //We can start with a quasi-Newton method to avoid computing the hessian of the constraints
//    if (!values) {
//        //Sparsity structure of the Hessian
//    } else {
//        //Actual Hessian
//
//    }
}

void Solver::finalize_solution(Ipopt::SolverReturn status, Ipopt::Index n,
                                                        const Ipopt::Number* x, const Ipopt::Number* z_L,
                                                        const Ipopt::Number* z_U, Ipopt::Index m, const Ipopt::Number* g,
                                                        const Ipopt::Number* lambda, Ipopt::Number obj_value,
                                                        const Ipopt::IpoptData* ip_data,
                                                        Ipopt::IpoptCalculatedQuantities* ip_cq)
{
    switch (status) {
        case Ipopt::SUCCESS:
            m_data.finalStatus = SolverFinalStatus::SUCCESS;
            break;
        case Ipopt::MAXITER_EXCEEDED:
        case Ipopt::CPUTIME_EXCEEDED:
            m_data.finalStatus = SolverFinalStatus::MAXITER;
            break;
        case Ipopt::LOCAL_INFEASIBILITY:
            m_data.finalStatus = SolverFinalStatus::INFEASIBLE;
            break;
        default:
            m_data.finalStatus = SolverFinalStatus::ERROR;
            break;
    }

    //Copy solution
    for (Index i = 0; i < n; i++) {
        m_data.primalSolution[i] = x[i];
        m_data.lowerBoundMultipliers[i] = z_L[i];
        m_data.upperBoundMultipliers[i] = z_U[i];
    }
    m_data.optimum = obj_value;

    std::cerr << "Solution (primal) - rad:" << m_data.primalSolution.transpose() << "\nn";
    std::cerr << "Solution (primal) - deg:" << (m_data.primalSolution.transpose() * 180.0/M_PI) << "\n";

    for (Index i = 0; i < m; i++) {
        m_data.constraintsValue[i] = g[i];
        m_data.constraintsMultipliers[i] = lambda[i];
        m_data.lowerBoundMultipliers[n + i] = z_L[n + i];
        m_data.upperBoundMultipliers[n + i] = z_U[n + i];
    }

    std::cerr << "Constraints value (CoM)\n"
    << m_data.constraintsValue << "\n";

}


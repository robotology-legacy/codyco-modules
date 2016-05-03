#include "OptimProblem.h"

#include <yarp/sig/Vector.h>
#include <yarp/os/LogStream.h>
#include <IpIpoptApplication.hpp>

#include "Solver.h"
#include "SolverData.h"

using namespace Ipopt;

OptimProblem::OptimProblem()
: pimpl(0)
{
    pimpl = new SolverData();
}

OptimProblem::~OptimProblem()
{
    if (pimpl) {
        delete pimpl;
        pimpl = 0;
    }
}

bool OptimProblem::initializeModel(const std::string modelFile, const std::vector<std::string> &jointsMapping)
{
    assert(pimpl);
    return pimpl->resetModelInformation(modelFile, jointsMapping);
}


bool OptimProblem::solveOptimization(const yarp::sig::Vector& desiredCoM, const yarp::sig::Vector& desiredJoints, std::string feetInContact)
{
    assert(pimpl);
    pimpl->resetOptimizationData(desiredCoM, desiredJoints, feetInContact);

//    // Create a new instance of your nlp
//    //  (use a SmartPtr, not raw)
    SmartPtr<TNLP> mynlp = new Solver(*pimpl);
//
    // Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();
//
//    // Change some options
//    // Note: The following choices are only examples, they might not be
//    //       suitable for your optimization problem.
//    app->Options()->SetNumericValue("tol", 1e-9);
//    app->Options()->SetStringValue("mu_strategy", "adaptive");
//    app->Options()->SetStringValue("output_file", "ipopt.out");
    app->Options()->SetStringValue("hessian_approximation", "limited-memory");
//
//    // Intialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
        yError("*** Error during initialization of IpOpt!");
        return (int) status;
    }

    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    if (status == Solve_Succeeded) {
        yInfo("*** The problem solved!");
    }
    else {
        yError("*** The problem FAILED!");
    }

    return (int) status;

}

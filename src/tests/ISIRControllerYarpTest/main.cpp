
#include <iostream>

#include <Eigen/Eigen>

#include "Model3T.h"

#include "orcisir/ISIRController.h"
#include "orcisir/Tasks/ISIREasyTaskManager.h"

int main(int argc, char** argv)
{
    VectorXd q      = Eigen::VectorXd(3);
    VectorXd dq     = Eigen::VectorXd(3);
    VectorXd tau    = Eigen::VectorXd(3);
    double dt = 0.01;


    //SET CONTROLLER PARAMETERS
    std::cout<<"SET PARAMETERS\n";
    std::string internalSolver("qld");  //or "quadprog" "qld"
    bool useReducedProblem = false;     //true does not work yet...
    bool useMultiLevel     = false;

    // INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER
    std::cout<<"INITIALIZE ISIR MODEL, CONTROLLER & TASK MANAGER\n";
    Model3T                         model("Model3T");
    orcisir::ISIRController         ctrl("myCtrl", model, internalSolver, useReducedProblem, useMultiLevel);
    orcisir::ISIREasyTaskManager    taskManager(ctrl, model);


    //CREATE SOME TASKS
    std::cout<<"CREATE SOME TASKS\n";
    int refTaskIdx      = taskManager.createFullStateTask("refTask",       0, 0.0001, "INTERNAL");
    taskManager.setTaskStiffnessAndDamping(refTaskIdx,   9, 6);    //set Kp and Kd

    int frameTaskIdx    = taskManager.createPositionFrameTask("frameTask", 0, 1,  "Model3T.segment_3", Displacementd(), "XYZ");
    taskManager.setTaskStiffnessAndDamping(frameTaskIdx, 9, 6);
    taskManager.updateFrameTask(frameTaskIdx, Displacementd(.4, .4, .4), Twistd(0,0,0,0,0,0), Twistd());




    //SIMULATE
    std::cout<<"SIMULATE\n";
    for (int i=0; i<1000; i++)
    {
        std::cout<<"- -  --- - - - -- - - -- - "<<i<<"\n";
        ctrl.computeOutput(tau);    //compute tau
        std::cout<<"tau: "<<tau.transpose()<<"\n";

        VectorXd ddq = model.getAccelerationVariable().getValue();
        std::cout<<"ddq: "<<ddq.transpose()<<"\n";
        //VectorXd ddq = model.getInertiaMatrixInverse() * ( tau - model.getNonLinearTerms() - model.getGravityTerms() );

        dq += ddq * dt;
        q  += dq  * dt;

        model.setJointPositions(q);
        model.setJointVelocities(dq);

        std::cout<<"pos seg3: "<< model.getSegmentPosition(3).getTranslation().transpose()<<"\n";
    }


    return 0;
}

As of January 5,2016 11:51 AM
In order to work with this module as it currently is, please take into account the following considerations about its structure:
1. wholeBodyEstimator has been thought-out to work under the principles of a Factory Design Pattern. This means that the different estimators that this modules can instantiate must be added as independent classes that inherit from a base interface class called IEstimator, and the exact name of the classes that one wants the module to instantiate must be listed in the `.ini` file of the corresponding robot as done [here](https://github.com/robotology/codyco-modules/blob/newModule/wholeBodyEstimator/src/modules/wholeBodyEstimator/app/robots/icubGazeboSim/wholeBodyEstimator.ini)under the group `[ESTIMATORS_LIST]` with their parameters under the groups headed with the corresponding class name. For instance, if the estimator class listed is `QuaternionEKF` its parameters should be listed under a group with the same name, i.e. `[QuaternionEKF]`.
2. The classes `nonLinearAnalyticConditionalGaussian` and `nonLinearMeasurementGaussianPdf` are implementations of base classes belonging to `BFL` and used specifically by `QuaternionEKF`. These might be moved to another directory in the future to tidy up the source directory. 
3. Classes `EstimatorsCreator`, `EstimatorsCreatorImpl` and `EstimatorsFactorty` are helping classes to implement the aforementioned Factory Design pattern.
4. The classes `WholeBodyEstimatorModule` is the main module class, while the instantiated thread is of type `WholeBodyEstimatorThread`. The thread is the one in charge of instantiating the different estimators as specified in the configuration file of this module.
5. Currently it supports two estimators, namely, `LeggedOdometry` and `QuaternionEKF`. 

# Testing QuaternionEKF
This is more of a personal reminder or notes in order to keep track of the standalone debugging procedure I perform on my machine. This means, without having to use the real robot. 
1. Launch the Gazebo simulator with the icub world.
2. Launch the `yarpdataplayer` and open the directoy `dumper01/` in `app/`, then play. This will stream data through the ports `/icubGazeboSim/right_leg/inertialMTB` and `/icub/right_leg/state:o`.
3. Make sure your environmental variable `YARP_ROBOT_NAME` points to `icubGazeboSim`.
4. Run the wholeBodyEstimator with at least the following parameters:
```bash
wbi_conf_file               yarpWholeBodyInterface.ini
joints_list                 ROBOT_DYNAMIC_MODEL_JOINTS

# Module specific parameters
[MODULE_PARAMETERS]
name                        wholeBodyEstimator
period                      10
robot                       icubGazeboSim
verbose                     true
```

These are the default parameters as of January 5 12:37 in the configuration file of the robot `icubGazeboSim`. Next step is to compare the outputs of `/QuaternionEKF/filteredOrientationEuler:o` and `/icub/right_leg/state:o` and check if the estimated orientation matches the actual orientation of the foot. Something else that can be done for better visualization is to launch `iCubGui`and connect `/icub/right_leg/state:o` to `/iCubGui/right_leg/state:i` (or whatever it's called).



#ifndef PARAMS_STRUCTURES_H_
#define PARAMS_STRUCTURES_H_

struct walkingParams{
    double z_c;
    int    n_strides;
    int    T_stride;
    int    T_switch;
    double step_width;
    double step_length;
    double step_height;
    int    n_samples;
    double g;
};

struct odometryParams{
    std::string initial_world_reference_frame;
    std::string initial_fixed_frame;
    std::string floating_base;
    KDL::Vector offset_from_world_reference_frame;
    bool        world_between_feet;
};

struct inverseKinematicsParams{
    double step_tolerance;
    double lambda;
    int    max_iter;
    int    trials_initial_IK;
};

#endif

return rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state DOUBLESUPPORT_GRASPING                                                       --
    -- In this state the robot is standing on double support, and is graspin             --
    ---------------------------------------------------------------------------------------
    ST_GRASPING = rfsm.load("fsm_grasping.lua");

    ---------------------------------------------------------------------------------------
    -- state RIGHT_STEPPING                                                                    --
    -- In this state the robot is standing on double support                             --
    ---------------------------------------------------------------------------------------
    ST_RIGHT_STEPPING = rfsm.load("fsm_right_step.lua"),

    ---------------------------------------------------------------------------------------
    -- state LEFT_STEPPING                                                                    --
    -- In this state the robot is standing on double support                             --
    ---------------------------------------------------------------------------------------
    ST_LEFT_STEPPING = rfsm.load("fsm_left_step.lua"),

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_GRASPING' },

    rfsm.transition { src='ST_GRASPING', tgt='ST_RIGHT_STEPPING',  events={ 'e_right_step_requested',  'e_grasping_disabled' } },
    rfsm.transition { src='ST_RIGHT_STEPPING', tgt='ST_GRASPING',  events={ 'e_right_step_completed' } },

    rfsm.transition { src='ST_GRASPING', tgt='ST_LEFT_STEPPING',  events={ 'e_left_step_requested',  'e_grasping_disabled' } },
    rfsm.transition { src='ST_LEFT_STEPPING', tgt='ST_GRASPING',  events={ 'e_left_step_completed' } },
}
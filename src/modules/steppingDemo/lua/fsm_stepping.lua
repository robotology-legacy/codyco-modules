
return rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state DOUBLESUPPORT_GRASPING                                                       --
    -- In this state the robot is standing on double support, and is graspin             --
    ---------------------------------------------------------------------------------------
    ST_GRASPING = rfsm.load("fsm_grasping.lua");

    ---------------------------------------------------------------------------------------
    -- state STEPPING                                                                    --
    -- In this state the robot is standing on double support                             --
    ---------------------------------------------------------------------------------------
    ST_STEPPING = rfsm.load("fsm_step.lua"),

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_GRASPING' },

    rfsm.transition { src='ST_GRASPING', tgt='ST_STEPPING',  events={ 'e_step_requested',  'e_grasping_disabled' } },
    rfsm.transition { src='ST_STEPPING', tgt='ST_GRASPING',  events={ 'e_step_completed' } },

}

return rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state GRASPING_ACTIVE                                                       --
    -- In this state the robot is standing on double support, and is grasping             --
    ---------------------------------------------------------------------------------------
    ST_GRASPING_ACTIVATION_REQUESTED =  rfsm.state{
        entry=function()
            -- request to activate the grasping
            gas_sendStringToRPC(graspingModule_rpc,"start")
        end,
    },


    ST_GRASPING_ACTIVE =  rfsm.state{
    },

    ------------------------------------------------------------------------------------------
    -- state GRASPING_DISABLING_REQUESTED                                                   --
    -- In this state the robot is standing on double support, and a stop has been requested --
    ------------------------------------------------------------------------------------------
    ST_GRASPING_DISABLING_REQUESTED =  rfsm.state{
        entry=function()
            -- request to stop the grasping
            gas_sendStringToRPC(graspingModule_rpc,"stop")
        end,
    },

    ST_GRASPING_DISABLED =  rfsm.state{
    },


    rfsm.transition { src='initial', tgt='ST_GRASPING_ACTIVATION_REQUESTED' },

    -- Enabling grasping flow
    rfsm.transition { src='ST_GRASPING_DISABLED', tgt='ST_GRASPING_ACTIVATION_REQUESTED',  events={ 'e_grasping_enabling_requested' } },
    rfsm.transition { src='ST_GRASPING_ACTIVATION_REQUESTED', tgt='ST_GRASPING_ACTIVE',  events={ 'e_grasping_enabled' } },

    -- Disabling grasping flow
    rfsm.transition { src='ST_GRASPING_ACTIVE', tgt='ST_GRASPING_DISABLING_REQUESTED',  events={ 'e_grasping_disabling_requested' } },
    rfsm.transition { src='ST_GRASPING_DISABLING_REQUESTED', tgt='ST_GRASPING_DISABLED',  events={ 'e_grasping_disabled' } },

    -- Timeout transitions
    rfsm.transition { src='ST_GRASPING_ACTIVATION_REQUESTED', tgt='ST_GRASPING_DISABLED',  events={ 'e_after(5.0)','e_grasping_disabled' } },
    rfsm.transition { src='ST_GRASPING_DISABLING_REQUESTED', tgt='ST_GRASPING_ACTIVE',  events={ 'e_after(5.0)','e_grasping_enabled' } },

}
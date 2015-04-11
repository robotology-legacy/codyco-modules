
fsm_left_right_sway = rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state ST_WEIGHT_ON_LEFT_FOOT                                                      --
    -- In this state the robot is standing on double support,                            --
    -- but the projection of the desired position of the com is on the left foot.        --
    ---------------------------------------------------------------------------------------
    ST_WEIGHT_ON_LEFT_FOOT = rfsm.state{
        entry=function()
            -- set the com desired position
        gas_setpoints.left_com_in_world:print("[DEBUG] gas_setpoints.left_com_in_world : ")
        gas_setpoints.left_com_in_initial_world:print("[DEBUG] gas_setpoints.left_com_in_initial_world : ")
        gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.left_com_in_initial_world)
        end,
    },

    ---------------------------------------------------------------------------------------
    -- state ST_WEIGHT_ON_RIGHT_FOOT                                                      --
    -- In this state the robot is standing on double support,                            --
    -- but the projection of the desired position of the com is on the right foot.        --
    ---------------------------------------------------------------------------------------
    ST_WEIGHT_ON_RIGHT_FOOT = rfsm.state{
        entry=function()
            -- set the com desired position
            gas_setpoints.right_com_in_world:print("[DEBUG] gas_setpoints.right_com_in_world : ")
            gas_setpoints.right_com_in_initial_world:print("[DEBUG] gas_setpoints.right_com_in_initial_world : ")
            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.right_com_in_initial_world)
        end,
    },

    ---------------------------------------------------------------------------------------
    -- state ST_INITIAL_COM                                                   --
    -- In this state the robot is standing on double support,                            --
    -- but the projection of the desired position of the com is the initial position of the com     --
    ---------------------------------------------------------------------------------------
    ST_INITIAL_COM = rfsm.state{
        entry=function()
            -- set the com desired position
            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.initial_com_in_world)
        end,
    },

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_INITIAL_COM' },

    -- Time  transitions
    rfsm.transition { src='ST_INITIAL_COM', tgt='ST_WEIGHT_ON_LEFT_FOOT', events={'e_after(3)'} },
    rfsm.transition { src='ST_WEIGHT_ON_LEFT_FOOT', tgt='ST_WEIGHT_ON_RIGHT_FOOT', events={ 'e_after('..switching_period..')' } },
    rfsm.transition { src='ST_WEIGHT_ON_RIGHT_FOOT', tgt='ST_WEIGHT_ON_LEFT_FOOT', events={ 'e_after('..switching_period..')' } },

    -- Event transition
    rfsm.transition { src='ST_WEIGHT_ON_LEFT_FOOT', tgt='ST_INITIAL_COM', events={ 'e_reset' } },
    rfsm.transition { src='ST_WEIGHT_ON_RIGHT_FOOT', tgt='ST_INITIAL_COM', events={ 'e_reset' } },

}

return fsm_left_right_sway


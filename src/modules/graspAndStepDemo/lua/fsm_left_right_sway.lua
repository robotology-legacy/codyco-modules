
fsm_left_right_sway = rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state ST_WEIGHT_ON_LEFT_FOOT                                                      --
    -- In this state the robot is standing on double support,                            --
    -- but the projection of the desired position of the com is on the left foot.        --
    ---------------------------------------------------------------------------------------
    ST_WEIGHT_ON_LEFT_FOOT = rfsm.state{
        entry=function()
            -- set the com desired position
            local bot = YarpVectorBottleFromPointCoord(gas_setpoints.left_com_in_world)
            gas_sendSetPointToTrajGen(setpoints_port,"com",bot)
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
            local bot = YarpVectorBottleFromPointCoord(gas_setpoints.right_com_in_world)
            gas_sendSetPointToTrajGen(setpoints_port,"com",bot)
        end,
    },

    ---------------------------------------------------------------------------------------
    -- state ST_WEIGHT_ON_THE_MIDDLE                                                     --
    -- In this state the robot is standing on double support,                            --
    -- but the projection of the desired position of the com is between the two feet     --
    ---------------------------------------------------------------------------------------
    ST_WEIGHT_ON_THE_MIDDLE = rfsm.state{
        entry=function()
            -- set the com desired position
            gas_sendSetPointToTrajGen(setpoints_port,"com",initial_com_in_world_bt)
        end,
    },

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_WEIGHT_ON_THE_MIDDLE' },

    -- Time  transitions
    rfsm.transition { src='ST_WEIGHT_ON_THE_MIDDLE', tgt='ST_WEIGHT_ON_LEFT_FOOT', events={'e_after(3)'} },
    rfsm.transition { src='ST_WEIGHT_ON_LEFT_FOOT', tgt='ST_WEIGHT_ON_RIGHT_FOOT', events={ 'e_after('..switch_period..')' } },
    rfsm.transition { src='ST_WEIGHT_ON_RIGHT_FOOT', tgt='ST_WEIGHT_ON_LEFT_FOOT', events={ 'e_after('..switch_period..')' } },

    -- Event transition
    rfsm.transition { src='ST_WEIGHT_ON_LEFT_FOOT', tgt='ST_WEIGHT_ON_THE_MIDDLE', events={ 'e_reset' } },
    rfsm.transition { src='ST_WEIGHT_ON_RIGHT_FOOT', tgt='ST_WEIGHT_ON_THE_MIDDLE', events={ 'e_reset' } },

}

return fsm_left_right_sway


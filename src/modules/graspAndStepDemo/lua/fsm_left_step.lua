
fsm_left_step = rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state DOUBLESUPPORT_TRANSFER_WEIGHT_TO_RIGHT_FOOT                               --
    -- In this state the robot is standing on double support,                            --
    -- but is shifting its weight to the support foot                                    --
    ---------------------------------------------------------------------------------------
    ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_RIGHT_FOOT = rfsm.state{
        -- set new com setpoint (for single support)
        entry=function()
            -- set the odometry fixed link to the right foot
            gas_sendStringsToPort(fixedLinkOdometry_port,"changeFixedLinkSimpleLeggedOdometry","r_foot");
        end,
    },

    ---------------------------------------------------------------------------------------
    -- state SINGLESUPPORT_SWING                                                         --
    -- In this single support state the robot is standing on double support,             --
    -- but is shifting its weight to the support foot                                    --
    ---------------------------------------------------------------------------------------
     -- Not using nesting for now because there is a bug in rfsm tools rfsm.load("fsm_swing.lua"),
    ST_SINGLESUPPORT_LEFT_SWING = rfsm.state{
        -- set new com setpoint (for single support)
    };


    ---------------------------------------------------------------------------------------
    -- state ST_DOUBLESUPPORT_TRANSFER_WEIGHT_FROM_RIGHT_FOOT                               --
    -- In this state the robot is standing on double support,                            --
    -- but is shifting its weight to the support foot                                    --
    ---------------------------------------------------------------------------------------
    ST_DOUBLESUPPORT_TRANSFER_WEIGHT_FROM_RIGHT_FOOT = rfsm.state{
        -- set new com setpoint
    },

    ----------------------------------
    -- setting the transitions      --
    ----------------------------------

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_RIGHT_FOOT' },

    -- Sensor transitions
    rfsm.transition { src='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_RIGHT_FOOT', tgt='ST_SINGLESUPPORT_LEFT_SWING', events={ 'e_com_motion_done' } },
    rfsm.transition { src='ST_SINGLESUPPORT_LEFT_SWING', tgt='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_FROM_RIGHT_FOOT', events={ 'e_left_leg_swing_motiondone' } },


}

return fsm_left_step


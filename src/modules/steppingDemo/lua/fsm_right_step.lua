
return rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT                               --
    -- In this state the robot is standing on double support,                            --
    -- but is shifting its weight to the support foot                                    --
    ---------------------------------------------------------------------------------------
    ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT = rfsm.state{
    },

    ---------------------------------------------------------------------------------------
    -- state SINGLESUPPORT_RIGHT_SWING                                                         --
    -- In this single support state the robot is standing on double support,             --
    -- but is shifting its weight to the support foot                                    --
    ---------------------------------------------------------------------------------------
     -- Not using nesting for now because there is a bug in rfsm tools rfsm.load("fsm_swing.lua"),
    ST_SINGLESUPPORT_RIGHT_SWING = rfsm.state{
    };


    ---------------------------------------------------------------------------------------
    -- state DOUBLESUPPORT_TRANSFER_WEIGHT_FROM_LEFT_FOOT                                --
    -- In this state the robot is standing on double support,                            --
    -- but is shifting its weight to the support foot                                    --
    ---------------------------------------------------------------------------------------
    ST_DOUBLESUPPORT_TRANSFER_WEIGHT_FROM_LEFT_FOOT = rfsm.state{
    },

    ----------------------------------
    -- setting the transitions      --
    ----------------------------------

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT' },

    -- Sensor transitions
    rfsm.transition { src='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT', tgt='ST_SINGLESUPPORT_SWING', events={ 'e_no_weight_on_right_foot' } },
    rfsm.transition { src='ST_SINGLESUPPORT_SWING', tgt='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_CENTER', events={ 'e_swing_motiondone' } },


}


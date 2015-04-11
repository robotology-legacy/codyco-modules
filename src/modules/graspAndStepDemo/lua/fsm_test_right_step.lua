
fsm_right_step = rfsm.state {
    ST_DOUBLESUPPORT_INITIAL_COM = rfsm.state{
        entry=function()
            -- set the odometry fixed link to the right foot

            gas_activateConstraints(constraints_port,{'r_foot','l_foot'})

            gas_sendStringsToPort(odometry_port,"changeFixedLinkSimpleLeggedOdometry","l_foot");

            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.initial_com_in_world);
        end,
    },

    ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT = rfsm.state{
        entry=function()
            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.weight_on_left_foot_com_in_world);
        end,
    },

    ST_SINGLESUPPORT_RIGHT_SWING = rfsm.state{
        entry=function()
            gas_deactivateConstraints(constraints_port,{'r_foot'})
        end,
    },

    ----------------------------------
    -- setting the transitions      --
    ----------------------------------C

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_DOUBLESUPPORT_INITIAL_COM' },

    -- Sensor transitions
    rfsm.transition { src='ST_DOUBLESUPPORT_INITIAL_COM', tgt='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT', events={ 'e_right_step_requested' } },
    rfsm.transition { src='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT', tgt='ST_SINGLESUPPORT_RIGHT_SWING', events={ 'e_com_motion_done' } },
    rfsm.transition { src='ST_SINGLESUPPORT_RIGHT_SWING', tgt='ST_DOUBLESUPPORT_INITIAL_COM', events={ 'e_reset' } },

}

return fsm_right_step



fsm_right_step = rfsm.state {

    ST_DOUBLESUPPORT_INITIAL_STATE = rfsm.state{},


    ST_DOUBLESUPPORT_INITIAL_COM = rfsm.state{
        entry=function()
            gas_activateConstraints(constraints_port,{'r_foot','l_foot'})

            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.initial_com);


            --gas_sendCOMAndTwoPartsToTrajGen(setpoints_port,gas_setpoints.initial_com_wrt_r_foot_in_world,
            --                                "right_leg",gas_setpoints.initialRightLeg,
            --                                "left_leg",gas_setpoints.initialLeftLeg)
        end,

        doo=function()
            while true do
                gas_activateConstraints(constraints_port,{'r_foot','l_foot'})

                rfsm.yield(true)
            end
        end,
    },

    ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT = rfsm.state{
        entry=function()
            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.com_weight_on_left_foot);
        end,

        doo=function()
            while true do
                gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.com_weight_on_left_foot_single_support);

                rfsm.yield(true)
            end
        end
    },

    ST_SINGLESUPPORT_ON_LEFT_FOOT = rfsm.state{
        entry=function()
            gas_deactivateConstraints(constraints_port,{'r_foot'})
        end,
    },

    ST_SINGLESUPPORT_LIFT_RIGHT_LEG = rfsm.state{
        entry=function()
            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.com_weight_on_left_foot_single_support);
            gas_send_single_support_postural(setpoints_port)

        end,

        doo=function()
            while true do
                gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.com_weight_on_left_foot_single_support);
                gas_send_single_support_postural(setpoints_port)

                rfsm.yield(true)
            end
        end
    },

    ST_SINGLESUPPORT_LOWER_RIGHT_LEG = rfsm.state{
        entry=function()
            gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.com_weight_on_left_foot);
            gas_send_double_support_postural(setpoints_port)
        end,

        doo=function()
            while true do
                gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.com_weight_on_left_foot);
                gas_send_double_support_postural(setpoints_port)

                rfsm.yield(true)
            end
        end
    },

    ST_SINGLESUPPORT_RIGHT_LEG_APPROACHING_GROUND = rfsm.state{
        entry=function()
            end,

        doo=function()
            while true do
                rfsm.yield(true)
            end
        end
    },

    ST_DOUBLESUPPORT_WEIGHT_ON_LEFT_FOOT = rfsm.state{
        entry=function()
            end,

        doo=function()
            while true do
                rfsm.yield(true)
            end
        end
    },

    ----------------------------------
    -- setting the transitions      --
    ----------------------------------

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_DOUBLESUPPORT_INITIAL_STATE' },

    -- Sensor transitions
    rfsm.transition { src='ST_DOUBLESUPPORT_INITIAL_STATE', tgt='ST_DOUBLESUPPORT_INITIAL_COM', events={ 'e_debug' } },
    rfsm.transition { src='ST_DOUBLESUPPORT_INITIAL_COM', tgt='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT', events={ 'e_debug' } },
    rfsm.transition { src='ST_DOUBLESUPPORT_TRANSFER_WEIGHT_TO_LEFT_FOOT', tgt='ST_SINGLESUPPORT_ON_LEFT_FOOT', events={ 'e_debug' } },
    rfsm.transition { src='ST_SINGLESUPPORT_ON_LEFT_FOOT', tgt='ST_SINGLESUPPORT_LIFT_RIGHT_LEG', events={'e_debug'} },
    rfsm.transition { src='ST_SINGLESUPPORT_LIFT_RIGHT_LEG', tgt='ST_SINGLESUPPORT_LOWER_RIGHT_LEG', events={'e_debug'} },
    rfsm.transition { src='ST_SINGLESUPPORT_LOWER_RIGHT_LEG', tgt='ST_SINGLESUPPORT_RIGHT_LEG_APPROACHING_GROUND', events={'e_debug'} },
    rfsm.transition { src='ST_SINGLESUPPORT_RIGHT_LEG_APPROACHING_GROUND', tgt='ST_DOUBLESUPPORT_WEIGHT_ON_LEFT_FOOT', events={'e_debug'} },
    rfsm.transition { src='ST_DOUBLESUPPORT_WEIGHT_ON_LEFT_FOOT', tgt='ST_DOUBLESUPPORT_INITIAL_COM', events={'e_debug'} },

}

return fsm_right_step


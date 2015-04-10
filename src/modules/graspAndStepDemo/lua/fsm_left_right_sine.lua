
fsm_left_right_sine = rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state ST_STREAMING_SINE                                                           --
    -- In this state the robot is following a sine in the desired com position           --
    ---------------------------------------------------------------------------------------
    ST_STREAMING_SINE = rfsm.state{
        doo=function()
            while true do
                gas_setpoints.sine_com_in_world.x = gas_setpoints.initial_com_in_world.x + delta_x*math.sin(trajectory_frequency*yarp_now);
                gas_setpoints.sine_com_in_world.y = gas_setpoints.initial_com_in_world.y + delta_y*math.sin(trajectory_frequency*yarp_now);
                gas_setpoints.sine_com_in_world.z = gas_setpoints.initial_com_in_world.z + delta_z*math.sin(trajectory_frequency*yarp_now);

                gas_sendCOMToTrajGen(setpoints_port,gas_setpoints.sine_com_in_world)
                rfsm.yield(true) 
            end
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
    rfsm.transition { src='ST_INITIAL_COM', tgt='ST_STREAMING_SINE', events={'e_after(3)'} },

    -- Event transition
    rfsm.transition { src='ST_STREAMING_SINE', tgt='ST_INITIAL_COM', events={ 'e_reset' } },


}

return fsm_left_right_sine


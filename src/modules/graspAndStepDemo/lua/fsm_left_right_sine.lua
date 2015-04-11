
fsm_left_right_sine = rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state ST_STREAMING_SINE                                                           --
    -- In this state the robot is following a sine in the desired com position           --
    ---------------------------------------------------------------------------------------
    ST_STREAMING_SINE = rfsm.state{
        doo=function()
            while true do
                local sin_now = math.sin(trajectory_frequency*yarp_now)
                local cos_now = math.cos(trajectory_frequency*yarp_now)
                gas_setpoints.sine_com_in_world.x = gas_setpoints.initial_com_in_world.x + delta_x*sin_now;
                gas_setpoints.sine_com_in_world.y = gas_setpoints.initial_com_in_world.y + delta_y*sin_now;
                gas_setpoints.sine_com_in_world.z = gas_setpoints.initial_com_in_world.z + delta_z*sin_now;

                gas_setpoints.vel_sine_com_in_world.x = delta_x*trajectory_frequency*cos_now
                gas_setpoints.vel_sine_com_in_world.y = delta_y*trajectory_frequency*cos_now
                gas_setpoints.vel_sine_com_in_world.z = delta_z*trajectory_frequency*cos_now

                gas_setpoints.acc_sine_com_in_world.x = -delta_x*trajectory_frequency*trajectory_frequency*sin_now
                gas_setpoints.acc_sine_com_in_world.y = -delta_y*trajectory_frequency*trajectory_frequency*sin_now
                gas_setpoints.acc_sine_com_in_world.z = -delta_z*trajectory_frequency*trajectory_frequency*sin_now

                gas_sendCOMToBalancing(comdes_port,gas_setpoints.sine_com_in_world,
                                                   gas_setpoints.vel_sine_com_in_world
                                                   gas_setpoints.acc_sine_com_in_world)
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


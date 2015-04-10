
fsm_ping_pong = rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state ST_PING                                                      --
    ---------------------------------------------------------------------------------------
    ST_PING = rfsm.state{
        entry=function()
        end,
    },

    ---------------------------------------------------------------------------------------
    -- state ST_PONG                                                    --
    ---------------------------------------------------------------------------------------
    ST_PONG = rfsm.state{
        entry=function()
        end,
    },

    ---------------------------------------------------------------------------------------
    -- state ST_MIDDLE                                                --
    ---------------------------------------------------------------------------------------
    ST_MIDDLE = rfsm.state{
        entry=function()
        end,
    },

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_MIDDLE' },

    -- Time  transitions
    rfsm.transition { src='ST_MIDDLE', tgt='ST_PING', events={'e_after(3)'} },
    rfsm.transition { src='ST_PING', tgt='ST_PONG', events={ 'e_after(10)' } },
    rfsm.transition { src='ST_PONG', tgt='ST_PING', events={ 'e_after(10)' } },

    -- Event transition
    rfsm.transition { src='ST_PING', tgt='ST_MIDDLE', events={ 'e_reset' } },
    rfsm.transition { src='ST_PONG', tgt='ST_MIDDLE', events={ 'e_reset' } },

}

return fsm_left_right_sway


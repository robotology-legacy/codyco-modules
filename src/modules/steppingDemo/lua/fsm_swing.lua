
return rfsm.state {
    ---------------------------------------------------------------------------------------
    -- state SWING                              --
    -- In this state the robot is moding the swing foot                           --
    -- from its initial position to its final position                                    --
    ---------------------------------------------------------------------------------------
    ST_SWING = rfsm.state{},

    ----------------------------------
    -- setting the transitions      --
    ----------------------------------

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_SWING' },
}


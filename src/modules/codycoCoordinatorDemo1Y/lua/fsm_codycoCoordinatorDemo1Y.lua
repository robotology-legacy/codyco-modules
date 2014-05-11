
return rfsm.state {
    -------------------------------------------------------------------
    -- state DOUBLESUPPORT_STABLE                                --
    -- This is the S_1 state from deliverable D5.1                   --
    -- In this state the robot is balancing on its two feet.          --
    -------------------------------------------------------------------
    ST_DOUBLESUPPORT_STABLE = rfsm.state{
        entry=function()
            print("[codycoCoordinatorDemo1Y][rFSM] ST_DOUBLESUPPORT_STABLE entry.")
        end,
        doo=function()
            while true do
                local wb = state_port:prepare()
                wb:clear()
                wb:addInt(st_doublesupport_stable_int)
                state_port:write()
                rfsm.yield(true)
            end
        end
    },

    -----------------------------------------------------------------------------
    -- state DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT                                     --
    -- This is the S_2 state from deliverable D5.1                             --
    -- In this state the robot is balancing on its two feet,                   --
    -- and it's tryng to estabilish contact with the hands on the front table. --
    -----------------------------------------------------------------------------
    ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT = rfsm.state{
        entry=function()
            print("[codycoCoordinatorDemo1Y][rFSM] ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT entry.")
        end,
        doo=function()
            while true do
                local wb = state_port:prepare()
                wb:clear()
                wb:addInt(st_doublesupport_both_hands_seeking_contact_int)
                state_port:write()
                rfsm.yield(true)
            end
         end
    },

    -----------------------------------------------------------------------------
    -- state TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT                           --
    -- This is the S_3 state from deliverable D5.1                             --
    -- In this state the robot is balancing on its two feet and on the right hand,                   --
    -- and it's trying to estabilish contact with left hand on the front table. --
    -----------------------------------------------------------------------------
    ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT = rfsm.state{
        entry=function()
            print("[codycoCoordinatorDemo1Y][rFSM] ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT entry.")
        end,
        doo=function()
            while true do
                local wb = state_port:prepare()
                wb:clear()
                wb:addInt(st_triplesupport_left_hand_seeking_contact_int)
                state_port:write()
                rfsm.yield(true)
            end
         end
    },

    -----------------------------------------------------------------------------
    -- state TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT                           --
    -- This is the S_4 state from deliverable D5.1                             --
    -- In this state the robot is balancing on its two feet and on the left hand,                   --
    -- and it's trying to estabilish contact with right hand on the front table. --
    -----------------------------------------------------------------------------
    ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT = rfsm.state{
        entry=function()
            print("[codycoCoordinatorDemo1Y][rFSM] ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT entry.")
        end,
        doo=function()
            while true do
                local wb = state_port:prepare()
                wb:clear()
                wb:addInt(st_triplesupport_right_hand_seeking_contact_int)
                state_port:write()
                rfsm.yield(true)
            end
         end
    },


    -----------------------------------------------------------------------------
    -- state QUADRUPLESUPPORT_STABLE                     --
    -- This is the S_4 state from deliverable D5.1                             --
    -- In this state the robot is balancing on its two feet and on its two hands.                   --
    -----------------------------------------------------------------------------
    ST_QUADRUPLESUPPORT_STABLE = rfsm.state{
        entry=function()
            print("[codycoCoordinatorDemo1Y][rFSM] ST_QUADRUPLESUPPORT_STABLE entry.")
        end,
        doo=function()
            while true do
                local wb = state_port:prepare()
                wb:clear()
                wb:addInt(st_quadruplesupport_stable_int)
                state_port:write()
                rfsm.yield(true)
            end
         end
    },

    ----------------------------------
    -- setting the transitions      --
    ----------------------------------

    -- Initial transition
    rfsm.transition { src='initial', tgt='ST_DOUBLESUPPORT_STABLE' },

    -- Time transition
    rfsm.transition { src='ST_DOUBLESUPPORT_STABLE', tgt='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT', events={ 'e_after(' .. fsm_simple_balancing_time .. ')' } },

    -- Skin transitions
    rfsm.transition { src='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT', tgt='ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT', events={ 'e_contacts_only_on_right_hand' } },
    rfsm.transition { src='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT', tgt='ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT', events={ 'e_contacts_only_on_left_hand' } },
    rfsm.transition { src='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT', tgt='ST_QUADRUPLESUPPORT_STABLE', events={ 'e_contacts_on_both_hands' } },
    rfsm.transition { src='ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT',  tgt='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT', events={ 'e_no_contacts_on_hands' } },
    rfsm.transition { src='ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT',  tgt='ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT', events={ 'e_contacts_only_on_left_hand' } },
    rfsm.transition { src='ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT',  tgt='ST_QUADRUPLESUPPORT_STABLE', events={ 'e_contacts_on_both_hands' } },
    rfsm.transition { src='ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT',  tgt='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT', events={ 'e_no_contacts_on_hands' } },
    rfsm.transition { src='ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT',  tgt='ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT', events={ 'e_contacts_only_on_right_hand' } },
    rfsm.transition { src='ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT',  tgt='ST_QUADRUPLESUPPORT_STABLE', events={ 'e_contacts_on_both_hands' } },
    rfsm.transition { src='ST_QUADRUPLESUPPORT_STABLE', tgt='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT', events={ 'e_no_contacts_on_hands' } },
    rfsm.transition { src='ST_QUADRUPLESUPPORT_STABLE', tgt='ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT', events={ 'e_contacts_only_on_left_hand' } },
    rfsm.transition { src='ST_QUADRUPLESUPPORT_STABLE', tgt='ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT', events={ 'e_contacts_only_on_right_hand' } },

}


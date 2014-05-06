
return rfsm.state {
    -------------------------------------------------------------------
    -- state DOUBLESUPPORT_STABLE                                --
    -- This is the S_1 state from deliverable D5.1                   --
    -- In this state the robot is balancing on its two feet.          --
    -------------------------------------------------------------------
    ST_DOUBLESUPPORT_STABLE = rfsm.state{
        entry=function()
            print("ST_DOUBLESUPPORT_FEET_STABLE entry.")
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
            print("ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT entry.")
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
            print("ST_TRIPLESUPPORT_LEFT_HAND_SEEKING_CONTACT entry.")
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
            print("ST_TRIPLESUPPORT_RIGHT_HAND_SEEKING_CONTACT entry.")
        end
    },


    -----------------------------------------------------------------------------
    -- state QUADRUPLESUPPORT_STABLE                     --
    -- This is the S_4 state from deliverable D5.1                             --
    -- In this state the robot is balancing on its two feet and on its two hands.                   --
    -----------------------------------------------------------------------------
    ST_QUADRUPLESUPPORT_STABLE = rfsm.state{
        entry=function()
            print("ST_QUADRUPLESUPPORT_STABLE entry.")
        end
    },

    ----------------------------------
    -- setting the transitions      --
    ----------------------------------

    rfsm.transition { src='initial', tgt='ST_DOUBLESUPPORT_BOTH_HANDS_SEEKING_CONTACT' },
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


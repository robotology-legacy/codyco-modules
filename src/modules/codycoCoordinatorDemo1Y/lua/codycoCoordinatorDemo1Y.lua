#!/usr/bin/lua5.1

require("yarp")
require("rfsm")
require("rfsm_timeevent")

print("[codycoCoordinatorDemo1Y] opening yarp")
yarp.Network()

yarpNetworkTimeout = 10
if( not yarp.NetworkBase_checkNetwork(yarpNetworkTimeout) ) then
    print("[codycoCoordinatorDemo1Y] yarp server not found, exiting")
    yarp.Network_fini()
    os.exit()
end

--action_status = 'idle'

-- enabling use rfsm_timeevent with yarp::os::Time::now()
--   in this way if you use gazebo_yarp_plugins the
--   fsm is synchronized with the simulation
function gettime()
    local yarp_time_now = yarp.Time_now()
    local yarp_time_now_sec  = math.floor(yarp.Time_now())
    local yarp_time_now_nsec = math.floor((yarp_time_now-yarp_time_now_sec)*1e9)
    return yarp_time_now_sec, yarp_time_now_nsec
end

rfsm_timeevent.set_gettime_hook(gettime)


----events
event_no_contact             = "e_no_contacts_on_hands"
event_contact_on_left_hand   = "e_contacts_only_on_left_hand"
event_contact_on_right_hand  = "e_contacts_only_on_right_hand"
event_contacts_on_both_hands = "e_contacts_on_both_hands"

----definitions
----copied from skinDynLib common.h
bodyPart_left_arm  = 3
bodyPart_right_arm = 4

----State constants
st_doublesupport_stable_int  = 1
st_doublesupport_both_hands_seeking_contact_int = 2
st_triplesupport_left_hand_seeking_contact_int = 4
st_triplesupport_right_hand_seeking_contact_int = 8
st_quadruplesupport_stable_int = 16

-------
function update_skin_events()
        --Use last received skinContactsList
        skin_contacts = event_port:read(false)
        if skin_contacts ~= nil then
            buffer_skin_contacts = skin_contacts
        end

        --Count contacts on left_arm and right_arm
        contact_left_arm = 0
        contact_right_arm = 0
        last_contact = buffer_skin_contacts:size()-1
        for i = 0,last_contact do
            bp_contact = buffer_skin_contacts:get(i):asList():get(0):asList():get(1):asInt()
            if( bp_contact == bodyPart_left_arm ) then
                contact_left_arm = contact_left_arm + 1
            end
            if( bp_contact == bodyPart_right_arm ) then
                contact_right_arm = contact_right_arm + 1
            end
        end

        -- Produce proper events given the contact state
        if( contact_left_arm > 0 and contact_right_arm > 0 ) then
            event_to_send = event_contacts_on_both_hands
        end
        if( contact_left_arm == 0 and contact_right_arm > 0 ) then
            event_to_send = event_contact_on_right_hand
        end
        if( contact_left_arm > 0 and contact_right_arm == 0 ) then
            event_to_send = event_contact_on_left_hand
        end
        if( contact_left_arm == 0 and contact_right_arm == 0 ) then
            event_to_send = event_no_contact
        end

        rfsm.send_events(fsm, event_to_send)
end

-------
shouldExit = false

-- initialization
print("[codycoCoordinatorDemo1Y] opening resource finder")
rf = yarp.ResourceFinder()
rf:setDefaultConfigFile("default.ini")
rf:setDefaultContext("codycoCoordinatorDemo1Y")
print("[codycoCoordinatorDemo1Y] configuring resource finder")
rf:configure(arg)

-- handling parameters
if( rf:check("fsm_update_period") ) then
    fsm_update_period = rf:find("fsm_update_period"):asDouble()
    print("[codycoCoordinatorDemo1Y] setting fsm_update_period to " .. fsm_update_period)
else
    print("[codycoCoordinatorDemo1Y] fsm_update_period parameter not found, exiting")
    yarp.Network_fini()
    os.exit()
end

if( rf:check("fsm_simple_balancing_time") ) then
    fsm_simple_balancing_time = rf:find("fsm_simple_balancing_time"):asDouble()
    print("[codycoCoordinatorDemo1Y] setting fsm_simple_balancing_time to " .. fsm_simple_balancing_time)
else
    print("[codycoCoordinatorDemo1Y] fsm_simple_balancing_time parameter not found, exiting")
    yarp.Network_fini()
    os.exit()
end

fsm_file = rf:findFile("lua/fsm_codycoCoordinatorDemo1Y.lua")

print("[codycoCoordinatorDemo1Y] opening ports")

-- rpc port, for communicating with C++ module torqueBalancing
cmd_action_rpc = yarp.RpcClient()
cmd_action_rpc:open("/codycoCoordinator1Y/cmd_action:o")

-- Input port for reading skinEvents from skinManager
event_port = yarp.BufferedPortBottle()
event_port:open("/codycoCoordinator1Y/skin_events:i")

-- Streaming port continuously broadcasting the state
state_port = yarp.BufferedPortBottle()
state_port:open("/codycoCoordinator1Y/state:o")

print("[codycoCoordinatorDemo1Y] loading rFSM state machine")
-- load state machine model and initalize it
fsm_model = rfsm.load(fsm_file)
fsm = rfsm.init(fsm_model)

buffer_skin_contacts = yarp.Bottle()
print("[codycoCoordinatorDemo1Y] starting main loop")
repeat
    -- print("[codycoCoordinatorDemo1Y] updating skin events")
    update_skin_events()
    -- print("[codycoCoordinatorDemo1Y] running fsm")
    rfsm.run(fsm)
    -- print("[codycoCoordinatorDemo1Y] waiting for " .. fsm_update_period)
    yarp.Time_delay(fsm_update_period)
until shouldExit ~= false

print("[codycoCoordinatorDemo1Y] finishing")
coroutine.resume(co_updater, true)

event_port:close()
cmd_action_rpc:close()

-- Deinitialize yarp network
yarp.Network_fini()


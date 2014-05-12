#!/usr/bin/lua

require("yarp")
require("rfsm")
require("rfsm_timeevent")

script_name = "codycoCoordinator1Y"
print("[" .. script_name .. "] opening yarp")
yarp.Network()

yarpNetworkTimeout = 10
if( not yarp.NetworkBase_checkNetwork(yarpNetworkTimeout) ) then
    print("[" .. script_name .. "] yarp server not found, exiting")
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
linkId_left_hand   = 6
bodyPart_right_arm = 4
linkId_right_hand  = 6

----State constants
st_doublesupport_stable_int  = 1
st_doublesupport_both_hands_seeking_contact_int = 2
st_triplesupport_left_hand_seeking_contact_int = 4
st_triplesupport_right_hand_seeking_contact_int = 8
st_quadruplesupport_stable_int = 16

states = {
    st_doublesupport_stable_int,
    st_doublesupport_both_hands_seeking_contact_int,
    st_triplesupport_left_hand_seeking_contact_int,
    st_triplesupport_right_hand_seeking_contact_int,
    st_quadruplesupport_stable_int
}

current_state = st_doublesupport_stable_int
current_smooth_state = {}
for _,v in pairs(states) do
    current_smooth_state[v] = 0
end
current_desired_state = {}
for _,v in pairs(states) do
    current_desired_state[v] = 0
end
current_old_state = {}
for _,v in pairs(states) do
    current_old_state[v] = 0
end
last_switch_time = yarp.Time_now();

current_desired_state[st_doublesupport_stable_int] = 1
current_smooth_state[st_doublesupport_stable_int]  = 1
current_old_state[st_doublesupport_stable_int]     = 1

function yarpBottleNorm(bot,val)
    local nrm = 0
    nrOfElems = math.min(val,bot:size()-1)
    for i = 0,nrOfElems do
        local el = bot:get(i):asDouble()
        nrm = nrm + el*el
    end
    nrm = math.sqrt(nrm)
    return nrm
end

-------
function update_buffers()
    skin_contacts = skin_event_port:read(false)
    if skin_contacts ~= nil then
        buffer_skin_contacts = skin_contacts
    end

    new_right_wrench = right_wrench_port:read(false)
    if new_right_wrench ~= nil then
        buffer_right_wrench = new_right_wrench
        buffer_right_force_norm = yarpBottleNorm(bot,3)
    end

    new_left_wrench = left_wrench_port:read(false)
    if new_left_wrench ~= nil then
        buffer_left_wrench = new_right_wrench
        buffer_left_force_norm = yarpBottleNorm(bot,3)
    end
end

-----
function count_contacts()
    skin_contact_left_hand = 0
    skin_contact_right_hand = 0
    last_contact = buffer_skin_contacts:size()-1
    for i = 0,last_contact do
        -- description of skinContact serialization in skinContact::write() method documentation
        bp_contact = buffer_skin_contacts:get(i):asList():get(0):asList():get(1):asInt()
        linkId_contact = buffer_skin_contacts:get(i):asList():get(0):asList():get(2):asInt()
        if( bp_contact == bodyPart_left_arm  and  linkId_contact == linkId_left_hand ) then
            skin_contact_left_hand = skin_contact_left_hand + 1
        end
        if( bp_contact == bodyPart_right_arm and  linkId_contact == linkId_right_hand ) then
            skin_contact_right_hand = skin_contact_right_hand + 1
        end
    end
end

-------
function produce_events()
    print("[codycoCoordinator1Y][debug] buffer_left_force_norm = " .. buffer_left_force_norm )
    print("[codycoCoordinator1Y][debug] buffer_right_force_norm = " .. buffer_right_force_norm )
    if( skin_contact_left_arm > 0 or buffer_left_force_norm > fsm_force_threshold ) then
        contact_left_arm = true
    else
        contact_left_arm = false
    end

    if( skin_contact_right_arm > 0 or buffer_right_force_norm > fsm_force_threshold ) then
        contact_right_arm = true
    else
        contact_right_arm = false
    end

    if( contact_left_arm and contact_right_arm ) then
        event_to_send = event_contacts_on_both_hands
    end
    if( not contact_left_arm and contact_right_arm ) then
        event_to_send = event_contact_on_right_hand
    end
    if( contact_left_arm  and not contact_right_arm ) then
        event_to_send = event_contact_on_left_hand
    end
    if( not contact_left_arm and not contact_right_arm ) then
        event_to_send = event_no_contact
    end
end

function yarp_rf_find_double(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asDouble()
        print("[" .. script_name .. "] setting " .. var_name .. " to " .. var)
        return var
    else
        print("[" .. script_name .. "] " .. var_name .." parameter not found, exiting")
        yarp.Network_fini()
        os.exit()
    end
end

function yarp_rf_find_int(rf,var,var_name)
    if( rf:check(var_name) ) then
        var = rf:find(var_name):asInt()
        print("[" .. script_name .. "] setting " .. var_name .. " to " .. var)
    else
        print("[" .. script_name .. "] " .. var_name .." parameter not found, exiting")
        yarp.Network_fini()
        os.exit()
    end
end

function yarp_rf_find_string(rf,var,var_name)
    if( rf:check(var_name) ) then
        var = rf:find(var_name):asString()
        print("[" .. script_name .. "] setting " .. var_name .. " to " .. var)
    else
        print("[" .. script_name .. "] " .. var_name .." parameter not found, exiting")
        yarp.Network_fini()
        os.exit()
    end
end

-------
function update_skin_events()
        --update buffers
        update_buffers()

        --Count contacts on left_arm and right_arm
        count_contacts()

        -- Produce proper events given the contact state
        rfsm.send_events(fsm, event_to_send)
end

------
function update_smooth_state()
    local current_time = yarp.Time_now()
    local time_from_last_switch = current_time-last_switch_time
    -- print("update_smooth_state - time_from_last_switch : " .. time_from_last_switch )
    -- print("update_smooth_state - fsm_state_smoothing_time_constant : " .. fsm_state_smoothing_time_constant )
    if( time_from_last_switch  > fsm_state_smoothing_time_constant ) then
        for _,st in pairs(states) do
            current_smooth_state[st] = current_desired_state[st]
        end
    else
        assert( time_from_last_switch >= 0.0 )
        assert( time_from_last_switch <= fsm_state_smoothing_time_constant )
        local completing_ratio = (time_from_last_switch/fsm_state_smoothing_time_constant)
        for _,st in pairs(states) do
            current_smooth_state[st] = current_old_state[st] + completing_ratio*(current_desired_state[st]-current_old_state[st])
        end
    end
end

------
function state_entry(state_code)
    print("[" .. script_name .. "] entering in state " .. state_code)
    ---- reset timer
    last_switch_time = yarp.Time_now();

    ---- update current state
    current_state = state_code

    ---- backup old state for smooth state generation
    for _,st in pairs(states) do
        current_old_state[st] = current_smooth_state[st]
    end

    ---- generate new desired smooth state
    for _,st in pairs(states) do
        if( st == state_code ) then
            current_desired_state[st] = 1.0
        else
            current_desired_state[st] = 0.0
        end
    end
end

function broadcast_data()
    ---- broadcast to state port
    local wb = state_port:prepare()
    wb:clear()
    wb:addInt(current_state)
    state_port:write()

    --- broadcast smooth state port
    local wb = smooth_state_port:prepare()
    wb:clear()
    for id,st in pairs(states) do
        wb:addDouble(current_smooth_state[st])
    end
    smooth_state_port:write()
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
yarp_rf_find_string(rf,script_name,"script_name")

if( rf:check("fsm_update_period") ) then
    fsm_update_period = rf:find("fsm_update_period"):asDouble()
    print("[" .. script_name .. "] setting fsm_update_period to " .. fsm_update_period)
else
    print("[" .. script_name .. "] fsm_update_period parameter not found, exiting")
    yarp.Network_fini()
    os.exit()
end


if( rf:check("fsm_simple_balancing_time") ) then
    fsm_simple_balancing_time = rf:find("fsm_simple_balancing_time"):asDouble()
    print("[" .. script_name .. "] setting fsm_simple_balancing_time to " .. fsm_simple_balancing_time)
else
    print("[" .. script_name .. "] fsm_simple_balancing_time parameter not found, exiting")
    yarp.Network_fini()
    os.exit()
end


if( rf:check("fsm_force_threshold") ) then
    fsm_force_threshold = rf:find("fsm_force_threshold"):asDouble()
    print("[" .. script_name .. "] setting fsm_force_threshold to " .. fsm_force_threshold)
else
    print("[" .. script_name .. "] fsm_force_threshold parameter not found, exiting")
    yarp.Network_fini()
    os.exit()
end

--yarp_rf_find_double(rf,fsm_update_period,"fsm_update_period")
--yarp_rf_find_double(rf,fsm_simple_balancing_time,"fsm_simple_balancing_time")
--yarp_rf_find_double(rf,fsm_force_threshold,"fsm_force_threshold")
fsm_state_smoothing_time_constant = yarp_rf_find_double(rf,"fsm_state_smoothing_time_constant")
print("fsm_state_smoothing_time_constant set to " .. fsm_state_smoothing_time_constant)

fsm_file = rf:findFile("lua/fsm_codycoCoordinatorDemo1Y.lua")

print("[" .. script_name .. "] opening ports")

-- rpc port, for communicating with C++ module torqueBalancing
cmd_action_rpc = yarp.RpcClient()
cmd_action_rpc:open("/".. script_name .."/cmd_action:o")

-- Input port for reading skinEvents from skinManager
skin_event_port = yarp.BufferedPortBottle()
skin_event_port:open("/".. script_name .."/skin_events:i")

-- Input port for reading wrenches from wholeBodyDynamicsTree
left_wrench_port = yarp.BufferedPortBottle()
left_wrench_port:open("/".. script_name .."/left_wrench:i")
right_wrench_port = yarp.BufferedPortBottle()
right_wrench_port:open("/".. script_name .."/right_wrench:i")

-- Streaming port continuously broadcasting the state
state_port = yarp.BufferedPortBottle()
state_port:open("/".. script_name .. "/state:o")

-- Streaming port continuously broadcasting the state in a smooth way
smooth_state_port = yarp.BufferedPortBottle()
smooth_state_port:open("/".. script_name .. "/smooth_state:o")

print("[" .. script_name .. "] loading rFSM state machine")
-- load state machine model and initalize it
fsm_model = rfsm.load(fsm_file)
fsm = rfsm.init(fsm_model)

-- Allocate buffers
buffer_skin_contacts = yarp.Bottle()
buffer_left_wrench = yarp.Bottle()
buffer_left_wrench:addDouble(0.0)
buffer_left_wrench:addDouble(0.0)
buffer_left_wrench:addDouble(0.0)
buffer_left_wrench:addDouble(0.0)
buffer_left_wrench:addDouble(0.0)
buffer_left_wrench:addDouble(0.0)
buffer_right_wrench = buffer_left_wrench
buffer_left_force_norm = 0.0
buffer_right_force_norm = 0.0

print("[" .. script_name .. "] starting main loop")
repeat
    -- print("[" .. script_name .. "] updating skin events")
    update_skin_events()
    -- print("[" .. script_name .. "] running fsm")
    rfsm.run(fsm)
    -- print("[" .. script_name .. "] current state is " .. current_state)
    update_smooth_state()
    broadcast_data()
    -- print("[" .. script_name .. "] waiting for " .. fsm_update_period)
    yarp.Time_delay(fsm_update_period)
until shouldExit ~= false

print("[" .. script_name .. "] finishing")

left_wrench_port:close()
right_wrench_port:close()
skin_event_port:close()
cmd_action_rpc:close()
state_port:close()
smooth_state_port:close()

-- Deinitialize yarp network
yarp.Network_fini()


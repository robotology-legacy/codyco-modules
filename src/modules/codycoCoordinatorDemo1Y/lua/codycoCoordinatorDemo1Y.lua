#!/usr/bin/lua

require("yarp")
require("rfsm")
require("rfsm_timeevent")

script_name = "codycoCoordinator1Y"
print("[" .. script_name .. "] opening yarp")
yarp.Network()

verbose = false

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

----events (contact)
event_no_contact             = "e_no_contacts_on_hands"
event_contact_on_left_hand   = "e_contacts_only_on_left_hand"
event_contact_on_right_hand  = "e_contacts_only_on_right_hand"
event_contacts_on_both_hands = "e_contacts_on_both_hands"

----events (input rpc)
event_reset = "e_reset"

events = {
    event_no_contact,
    event_contact_on_left_hand,
    event_contact_on_right_hand,
    event_contacts_on_both_hands,
    event_reset
}

----definitions
----copied from skinDynLib common.h
bodyPart_left_arm  = 3
linkId_left_hand   = 6
bodyPart_right_arm = 4
linkId_right_hand  = 6
linkId_right_foreArm = 4
linkId_left_foreArm = 4

----State constants
st_doublesupport_stable_int  = 1
st_doublesupport_both_hands_seeking_contact_int = 2
st_triplesupport_right_hand_seeking_contact_int = 4
st_triplesupport_left_hand_seeking_contact_int = 8
st_quadruplesupport_stable_int = 16

states = {
    st_doublesupport_stable_int,
    st_doublesupport_both_hands_seeking_contact_int,
    st_triplesupport_right_hand_seeking_contact_int,
    st_triplesupport_left_hand_seeking_contact_int,
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
function close_script()
    --- close ports
    left_wrench_port:close()
    right_wrench_port:close()
    skin_event_port:close()
    cmd_action_rpc:close()
    state_port:close()
    smooth_state_port:close()
    input_events:close()
    monitor_port:close()

    -- Deinitialize yarp network
    yarp.Network_fini()
    os.exit()
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
        buffer_right_force_norm = yarpBottleNorm(buffer_right_wrench,3)
    end

    new_left_wrench = left_wrench_port:read(false)
    if new_left_wrench ~= nil then
        buffer_left_wrench = new_left_wrench
        buffer_left_force_norm = yarpBottleNorm(buffer_left_wrench,3)
    end
end

-----
function count_contacts()
    skin_contact_left_hand = 0
    skin_contact_right_hand = 0
    skin_activated_taxel_left = 0
    skin_activated_taxel_right = 0 
    last_contact = buffer_skin_contacts:size()-1
    for i = 0,last_contact do
        -- description of skinContact serialization in skinContact::write() method documentation
        bp_contact = buffer_skin_contacts:get(i):asList():get(0):asList():get(1):asInt()
        linkId_contact = buffer_skin_contacts:get(i):asList():get(0):asList():get(2):asInt()
        activated_taxels = buffer_skin_contacts:get(i):asList():get(6):asList():size()
        if( bp_contact == bodyPart_left_arm  and  (linkId_contact == linkId_left_hand or linkId_contact == linkId_left_foreArm )) then
            skin_contact_left_hand = skin_contact_left_hand + 1
            skin_activated_taxel_left = skin_activated_taxel_left + activated_taxels
        end
        if( bp_contact == bodyPart_right_arm and  (linkId_contact == linkId_right_hand or linkId_contact == linkId_right_foreArm  ) ) then
            skin_contact_right_hand = skin_contact_right_hand + 1
            skin_activated_taxel_right = skin_activated_taxel_right + activated_taxels
        end
    end
end

-------
function produce_events()
    if( verbose ) then
        print("["..script_name.."][debug] current state = " .. current_state )
        print("["..script_name.."][debug] skin_contact_left_arm = " ..  skin_contact_left_hand )
        print("["..script_name.."][debug] skin_contact_right_arm  = " ..  skin_contact_right_hand  )
        print("["..script_name.."][debug] buffer_left_force_norm = " .. buffer_left_force_norm )
        print("["..script_name.."][debug] buffer_right_force_norm = " .. buffer_right_force_norm )
    end
    if( ( skin_activated_taxel_right >= fsm_taxel_threshold and enable_skin_feedback ) or
        ( buffer_left_force_norm > fsm_force_threshold  and enable_force_feedback ) ) then
        contact_left_hand = true
    else
        contact_left_hand = false
    end

    if( ( skin_activated_taxel_left >= fsm_taxel_threshold and enable_skin_feedback  ) or
        ( buffer_right_force_norm > fsm_force_threshold  and enable_force_feedback ) ) then
        contact_right_hand = true
    else
        contact_right_hand = false
    end

    if( contact_left_hand and contact_right_hand ) then
        event_to_send = event_contacts_on_both_hands
    end
    if( not contact_left_hand and contact_right_hand ) then
        event_to_send = event_contact_on_right_hand
    end
    if( contact_left_hand  and not contact_right_hand ) then
        event_to_send = event_contact_on_left_hand
    end
    if( not contact_left_hand and not contact_right_hand ) then
        event_to_send = event_no_contact
    end
end

-------
function produce_events_input()
    local cmd = yarp.Bottle();
    cmd = input_events:read(false)
    if( cmd ~= nil ) then
        input_event = cmd:get(0):asString()
         for _,ev in pairs(events) do
            if( ev == input_event ) then
                event_to_send = input_event
            end
        end
    end
end

function yarp_rf_find_double(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asDouble()
        print("[" .. script_name .. "] setting " .. var_name .. " to " .. var)
        return var
    else
        print("[" .. script_name .. "] " .. var_name .." parameter not found, exiting")
        close_script()
    end
end

function yarp_rf_find_int(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asInt()
        print("[" .. script_name .. "] setting " .. var_name .. " to " .. var)
        return var
    else
        print("[" .. script_name .. "] " .. var_name .." parameter not found, exiting")
        close_script()
    end
end

function yarp_rf_find_string(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asString()
        print("[" .. script_name .. "] setting " .. var_name .. " to " .. var)
        return var
    else
        print("[" .. script_name .. "] " .. var_name .." parameter not found, exiting")
        close_script()
    end
end

-------
function update_skin_events()
        --update buffers
        update_buffers()

        --Count contacts on left_arm and right_arm
        count_contacts()

        -- Produce proper events given the contact state
        produce_events()

        -- Produce events readed from outside
        -- note: this will override the events produced by produce_events()
        produce_events_input()

        -- Send produced events to the fsm
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
    ---- print("[" .. script_name .. "] entering in state " .. state_code)
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
    wb:addDouble(current_state)
    state_port:write()

    --- broadcast smooth state port
    local wb = smooth_state_port:prepare()
    wb:clear()
    for id,st in pairs(states) do
        wb:addDouble(current_smooth_state[st])
    end
    smooth_state_port:write()

    --- broadcast monitor information
    local wb = monitor_port:prepare()
    wb:clear()
    wb:addDouble(current_state)
    wb:addDouble(buffer_left_force_norm)
    wb:addDouble(buffer_right_force_norm)
    wb:addDouble(skin_activated_taxel_left)
    wb:addDouble(skin_activated_taxel_right)
    monitor_port:write()

end

-------
function print_help()
    ---- list options
    print("["..script_name.."]: --verbose                        : enable verbose output")
    print("["..script_name.."]: --fsm_update_period       period : update period of the FSM (in seconds)")
    print("["..script_name.."]: --fsm_simple_balancing_time time : time (in seconds) to switch from simple balancing to hand seeking phase")
    print("["..script_name.."]: --fsm_state_smoothing_time_constant : time (in seconds) to switch the smooth state")
    print("["..script_name.."]: --fsm_force_threshold threshold : threshold (in Newtons) on external force norm to trigger external contact")
    print("["..script_name.."]: --enable_force_feedback : use external force to detect contacts")
    print("["..script_name.."]: --enable_skin_feedback : use skin to detect contacts")
    print("["..script_name.."]: --disable_force_feedback : NOT use external force to detect contacts (has priority on enable)")
    print("["..script_name.."]: --disable_skin_feedback : NOT use skin to detect contacts (has priority on enable)")
    print("["..script_name.."]: --fsm_taxel_threshold  : minimum number of taxels that a contact should have to be considered a real contact")
    print("["..script_name.."]: --help : print this help")
end

-------
shouldExit = false

-- initialization
print("["..script_name.."] opening resource finder")
rf = yarp.ResourceFinder()
rf:setDefaultConfigFile("default.ini")
rf:setDefaultContext("codycoCoordinatorDemo1Y")
print("["..script_name.."] configuring resource finder")
rf:configure(arg)

-- handling parameters
script_name = yarp_rf_find_string(rf,"script_name")

if( rf:check("verbose") ) then
    print("["..script_name.."]: verbose option found")
    verbose = true
end

if( rf:check("help") ) then
    print_help()
    close_script()
end

if( rf:check("enable_force_feedback") and not rf:check("disable_force_feedback") ) then
    print("["..script_name.."]: using force to detect contacts")
    enable_force_feedback = true
else
    print("["..script_name.."]: NOT using force to detect contacts")
    enable_force_feedback = false
end

if( rf:check("enable_skin_feedback") and not rf:check("disable_skin_feedback") ) then
    print("["..script_name.."]: using skin to detect contacts")
    enable_skin_feedback = true
else
    print("["..script_name.."]: NOT using skin to detect contacts")
    enable_skin_feedback = false
end


fsm_file = rf:findFile("lua/fsm_codycoCoordinatorDemo1Y.lua")

print("[" .. script_name .. "] opening ports")

-- rpc port, for communicating with C++ module torqueBalancing
cmd_action_rpc = yarp.RpcClient()
cmd_action_rpc:open("/".. script_name .."/cmd_action:o")

-- rpc input port, for communicating user
input_events = yarp.BufferedPortBottle()
input_events:open("/".. script_name .."/input_events:i")

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

-- Streaming port continuously broadcasting some monitor information
monitor_port = yarp.BufferedPortBottle()
monitor_port:open("/".. script_name .. "/monitor:o")

fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")
fsm_simple_balancing_time = yarp_rf_find_double(rf,"fsm_simple_balancing_time")
fsm_force_threshold = yarp_rf_find_double(rf,"fsm_force_threshold")
fsm_state_smoothing_time_constant = yarp_rf_find_double(rf,"fsm_state_smoothing_time_constant")
fsm_taxel_threshold = yarp_rf_find_int(rf,"fsm_taxel_threshold")


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

close_script()

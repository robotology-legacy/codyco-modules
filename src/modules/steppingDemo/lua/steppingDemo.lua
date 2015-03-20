#!/usr/bin/lua

require("yarp")
require("rfsm")
require("rfsm_timeevent")

script_name = "steppingDemo"
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
function yarp_gettime()
    local yarp_time_now = yarp.Time_now()
    local yarp_time_now_sec  = math.floor(yarp.Time_now())
    local yarp_time_now_nsec = math.floor((yarp_time_now-yarp_time_now_sec)*1e9)
    return yarp_time_now_sec, yarp_time_now_nsec
end

rfsm_timeevent.set_gettime_hook(yarp_gettime)

----events (contact)
event_no_contact             = "e_no_contacts_on_hands"
event_contact_on_left_hand   = "e_contacts_only_on_left_hand"
event_contact_on_right_hand  = "e_contacts_only_on_right_hand"
event_contacts_on_both_hands = "e_contacts_on_both_hands"

----events (input rpc)
event_reset = "e_reset"
event_start= "e_start"

events = {
    event_no_contact,
    event_contact_on_left_hand,
    event_contact_on_right_hand,
    event_contacts_on_both_hands,
    event_reset,
    event_start
}

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
function print_help()
    ---- list options
    print("["..script_name.."]: --verbose                        : enable verbose output")
    print("["..script_name.."]: --fsm_update_period       period : update period of the FSM (in seconds)")
    print("["..script_name.."]: --help : print this help")
end

-------
shouldExit = false

-- initialization
print("["..script_name.."] opening resource finder")
rf = yarp.ResourceFinder()
rf:setDefaultConfigFile("steppingDemo.ini")
rf:setDefaultContext("steppingDemo")
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

fsm_file = rf:findFile("lua/fsm_stepping.lua")

print("[" .. script_name .. "] opening ports")

-- rpc port, for communicating with C++ module torqueBalancing
cmd_action_rpc = yarp.RpcClient()
cmd_action_rpc:open("/".. script_name .."/cmd_action:o")

-- rpc input port, for communicating user
input_events = yarp.BufferedPortBottle()
input_events:open("/".. script_name .."/input_events:i")


-- Streaming port continuously broadcasting the state
state_port = yarp.BufferedPortBottle()
state_port:open("/".. script_name .. "/state:o")

fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")

print("[" .. script_name .. "] loading rFSM state machine")
-- load state machine model and initalize it
fsm_model = rfsm.load(fsm_file)
fsm = rfsm.init(fsm_model)


print("[" .. script_name .. "] starting main loop")
repeat
    -- print("[" .. script_name .. "] running fsm")
    rfsm.run(fsm)
    -- print("[" .. script_name .. "] waiting for " .. fsm_update_period)
    yarp.Time_delay(fsm_update_period)
until shouldExit ~= false

print("[" .. script_name .. "] finishing")

close_script()

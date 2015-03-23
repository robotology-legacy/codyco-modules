#!/usr/bin/lua

require("yarp")
require("rfsm")
require("rfsm_timeevent")

script_name = "graspAndStepDemo"
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

--- Generate an event reader function optimized for string events.
--
-- When called this function will read all new string events from the given
-- yarp BufferedPort and return them in a table.
--
-- The format supported for getting the events is the following:
--    * a single string is interpreted as an event
--    * it the port element is a Bottle, all the elements of the Bottle
--      that are strings are considered an event.
--
-- @param ... list of BufferedPortBottle to read events from
-- @return getevent function
function yarp_gen_read_str_events(...)
    local function read_events(tgttab, bufferedport)
        while true do
            local cmd = yarp.Bottle();
            cmd = input_events:read(false)
            if( cmd == nil ) then
                break
            end
            if( cmd:isString() ) then
                tgttab[#tgttab+1] = cmd:asString()
            end
            if( cmd.isList() ) then
                for i=0,cmd:size() then
                    if( cmd:get(i):isString() ) then
                        tgttab[#tgttab+1] = cmd:get(i):asString()
                    end
                end
            end
        end
     end

     local ports = {...}
     assert(#ports > 0, "no ports given")
     -- check its all ports
     return function ()
         local res = {}
         for _,port in ipairs(ports) do read_events(res, port) end
         return res
     end
end


rfsm_timeevent.set_gettime_hook(yarp_gettime)

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

fsm_file = rf:findFile("lua/fsm_graspAndStep.lua")

print("[" .. script_name .. "] opening ports")

-- input events port, for getting events from other modules and
--             from the user rpc
input_events = yarp.BufferedPortBottle()
input_events:open("/".. script_name .."/events:i")

-- Streaming port continuously broadcasting the state
state_port = yarp.BufferedPortBottle()
state_port:open("/".. script_name .. "/state:o")

fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")
force_threshold   = yarp_rf_find_double(rf,"force_threshold")

print("[" .. script_name .. "] loading rFSM state machine")
-- load state machine model and initalize it
fsm_model = rfsm.load(fsm_file)
fsm = rfsm.init(fsm_model)

print("[" .. script_name .. "] starting main loop")
repeat
    -- run the monitor to generate events from ports
    -- monitor:run(fsm)
    -- run the finite state machine
    -- the configurator is implicitly executed by
    -- the fsm entry/doo/exit functions
    rfsm.run(fsm)
    yarp.Time_delay(fsm_update_period)
until shouldExit ~= false

print("[" .. script_name .. "] finishing")

close_script()

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

-- use the yarp time for time events
rfsm_timeevent.set_gettime_hook(yarp_gettime)

-------
function gas_close_script()
    --- close ports
    gas_close_ports()

    -- Deinitialize yarp network
    yarp.Network_fini()
    os.exit()
end

-------
function gas_print_help()
    ---- list options
    print("[INFO] --verbose                        : enable verbose output")
    print("[INFO] --fsm_update_period       period : update period of the FSM (in seconds)")
    print("[INFO] --help : print this help")
end

function gas_loadconfiguration()
    -- initialization
    print("[INFO] opening resource finder")
    rf = yarp.ResourceFinder()
    rf:setDefaultConfigFile("graspAndStepDemo.ini")
    rf:setDefaultContext("graspAndStepDemo")
    print("[INFO] configuring resource finder")
    rf:configure(arg)

    -- load helper functions
    dofile(rf:findFile("lua/gas_funcs.lua"))

    -- handling parameters
    script_name = yarp_rf_find_string(rf,"script_name")
    fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")

    if( rf:check("verbose") ) then
        print("[INFO] verbose option found")
        verbose = true
    end

    if( rf:check("help") ) then
        gas_print_help()
        gas_close_script()
    end

    fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")
    --force_threshold   = yarp_rf_find_double(rf,"force_threshold")

end

-------
function gas_open_ports()
    print("[INFO] opening ports")

    -- input events port
    input_events = yarp.BufferedPortBottle()
    input_events:open("/".. script_name .."/events:i")
    -- we don't want to loose any event
    input_events:setStrict()

    -- Streaming port continuously broadcasting the state
    state_port = yarp.BufferedPortBottle()
    state_port:open("/".. script_name .. "/state:o")

    -- Port for publishing trajectory generator setpoints
    setpoints_port = yarp.BufferedPortProperty()
    setpoints_port:open("/".. script_name .. "/setpoints:o")

    -- Port for starting/stopping the graspDemo module
    graspingModule_rpc = yarp.BufferedPortBottle()
    graspingModule_rpc:open("/".. script_name .. "/graspDemo")

    -- Port for activating/deactivating contacts on the controller
    activeContacts_port = yarp.BufferedPortBottle()
    activeContacts_port:open("/".. script_name .. "/constraints");

    -- Port for setting the fixed link in the odometry module
    fixedLinkOdometry_port = yarp.BufferedPortBottle()
    fixedLinkOdometry_port:open("/".. script_name .. "/fixedLink");

    -- Port for sending to iSpeak the current state
    iSpeak_port = yarp.BufferedPortBottle()
    iSpeak_port:open("/".. script_name .. "/speak");

end

function gas_close_ports()
    --close ports
    gas_close_port(input_events)
    gas_close_port(state_port)
    gas_close_port(setpoints_port)
    gas_close_port(graspingModule_rpc)
    gas_close_port(activeContacts_port)
    gas_close_port(fixedLinkOdometry_port)
    gas_close_port(iSpeak_port)
end

-------
shouldExit = false

-- load configuration
gas_loadconfiguration()

-- open ports
gas_open_ports()

-- load main FSM
fsm_file = rf:findFile("lua/fsm_graspAndStep.lua")

print("[INFO] loading rFSM state machine")
-- load state machine model and initalize it
fsm_model = rfsm.load(fsm_file)
fsm = rfsm.init(fsm_model)

-- configure script specific hooks

-- dbg function, callet at each state enter/exit etc etc
fsm.dbg = gas_dbg;

-- getevents function, to read functions from a
fsm.getevents = yarp_gen_read_str_events(input_events);

print("[INFO] starting main loop")
repeat
    -- run the monitor to generate events from ports
    -- monitor:run(fsm)
    -- run the finite state machine
    -- the configurator is implicitly executed by
    -- the fsm entry/doo/exit functions
    yarp_now = yarp.Time_now()
    rfsm.run(fsm)
    yarp.Time_delay(fsm_update_period)
until shouldExit ~= false

print("[INFO] finishing")

gas_close_script()

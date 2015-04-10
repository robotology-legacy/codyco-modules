#!/usr/bin/lua

require("yarp")
require("rfsm")
require("rfsm_timeevent")

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
    print("["..script_name.."]: --verbose                        : enable verbose output")
    print("["..script_name.."]: --help : print this help")
end

function gas_loadconfiguration()
    -- initialization
    print("["..script_name.."] opening resource finder")
    rf = yarp.ResourceFinder()
    rf:setDefaultConfigFile("pingPongDemo.ini")
    rf:setDefaultContext("graspAndStepDemo")
    print("["..script_name.."] configuring resource finder")
    rf:configure(arg)

    -- load yarp_rf helpers
    dofile(rf:findFile("lua/gas_funcs.lua"))

    -- handling parameters
    script_name = yarp_rf_find_string(rf,"script_name")
    fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")

    if( rf:check("verbose") ) then
        print("["..script_name.."]: verbose option found")
        verbose = true
    end

    if( rf:check("help") ) then
        gas_print_help()
        gas_close_script()
    end

end

-------
function gas_open_ports()
    print("[" .. script_name .. "] opening ports")

    -- input events port
    input_events = yarp.BufferedPortBottle()
    input_events:open("/".. script_name .."/events:i")
    -- we don't want to loose any event
    input_events:setStrict()

    -- Streaming port continuously broadcasting the state
    state_port = yarp.BufferedPortBottle()
    state_port:open("/".. script_name .. "/state:o")

    -- Port for sending to iSpeak the current state
    iSpeak_port = yarp.BufferedPortBottle()
    iSpeak_port:open("/".. script_name .. "/speak");

    -- Port for reading the com position in the world
    com_port = yarp.BufferedPortBottle()
    com_port:open("/" .. script_name .. "/com:i");

    -- Port for reading the frame positions w.r.t to the world
    frames_port = yarp.BufferedPortBottle()
    frames_port:open("/" .. script_name .. "/frames:i");

end

function gas_close_ports()
    --close ports
    gas_close_port(input_events)
    gas_close_port(state_port)
    gas_close_port(iSpeak_port)
    gas_close_port(com_port)
    gas_close_port(frames_port)
end


-------
function main()

    shouldExit = false
    script_name = "graspAndStepDemo"

    -- load configuration
    gas_loadconfiguration()

    -- load helper functions
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

    -- open ports
    gas_open_ports()

    -- load main FSM
    fsm_file = rf:findFile("lua/fsm_ping_pong.lua")

    print("[" .. script_name .. "] loading rFSM state machine")
    -- load state machine model and initalize it
    fsm_model = rfsm.load(fsm_file)
    fsm = rfsm.init(fsm_model)

    -- configure script specific hooks

    -- dbg function, callet at each state enter/exit etc etc
    fsm.dbg = gas_dbg;

    -- getevents function, to read functions from a
    fsm.getevents = yarp_gen_read_str_events(input_events);

    -- waiting for reading com data
    print("[" .. script_name .. "] waiting for reading com port")
    initial_com_in_world_bt = com_port:read(true)

    -- waiting for reading frame data
    print("[" .. script_name .. "] waiting for reading frames port")
    gas_frames_bt = frames_port:read(true)

    repeat
        -- run the finite state machine
        -- the configurator is implicitly executed by
        -- the fsm entry/doo/exit functions
        rfsm.run(fsm)
        yarp.Time_delay(fsm_update_period)
    until shouldExit ~= false

    print("[" .. script_name .. "] finishing")

    gas_close_script()

end

main()

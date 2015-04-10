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
    print("[INFO] --verbose                        : enable verbose output")
    print("[INFO] --help : print this help")
end

function gas_loadconfiguration()
    -- initialization
    print("[INFO] opening resource finder")
    rf = yarp.ResourceFinder()
    rf:setDefaultConfigFile("leftRightBalancingDemo.ini")
    rf:setDefaultContext("graspAndStepDemo")
    print("[INFO] configuring resource finder")
    rf:configure(arg)

    -- load helper functions
    dofile(rf:findFile("lua/gas_funcs.lua"))

    -- handling parameters
    script_name = yarp_rf_find_string(rf,"script_name")
    fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")
    switch_period = yarp_rf_find_double(rf,"switch_period")

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
    gas_close_port(setpoints_port)
    gas_close_port(iSpeak_port)
    gas_close_port(com_port)
    gas_close_port(frames_port)
end

function gas_updateframes()
    -- waiting for reading com data
    gas_setponts.initial_com_in_world_bt = com_port:read()
    if( gas_setponts.initial_com_in_world_bt ) then
        PointCoordFromYarpVectorBottle(gas_setponts.initial_com_in_world,
                                       gas_setponts.initial_com_in_world_bt)
    end

    -- waiting for reading frame data
    gas_frames_bt = frames_port:read(true)
    if( gas_frames_bt ) then
        HomTransformTableFromBottle(gas_frames,gas_frames_bt)
    end

    -- get transform
    world_H_l_foot = gas_frames[l_foot_frame]
    world_H_r_foot = gas_frames[r_foot_frame]

    -- update com setpoints
    gas_setponts.left_com_in_world =
        world_H_l_foot.apply(gas_setponts.left_com_in_l_foot)

    gas_setponts.right_com_in_world =
        world_H_r_foot.apply(gas_setponts.right_com_in_r_foot)
end


function main()
    shouldExit = false

    -- load configuration
    gas_loadconfiguration()

    print("[INFO] opening yarp")
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
    fsm_file = rf:findFile("lua/fsm_left_right_sway.lua")

    print("[INFO] loading rFSM state machine")
    -- load state machine model and initalize it
    fsm_model = rfsm.load(fsm_file)
    fsm = rfsm.init(fsm_model)

    -- configure script specific hooks

    -- dbg function, callet at each state enter/exit etc etc
    fsm.dbg = gas_dbg;

    -- getevents function, to read functions from a
    fsm.getevents = yarp_gen_read_str_events(input_events);

    gas_setponts = {
        -- geometric ponts
        initial_com_in_world = PointCoord:new(),
        left_com_in_l_foot = PointCoord:new(),
        right_com_in_r_foot = PointCoord:new(),
        left_com_in_world = PointCoord:new(),
        right_com_in_world = PointCoord:new(),

        -- bottle buffers to load/unload
        initial_com_in_world_bt = yarp.Bottle(),
    }

    gas_frames = {}
    l_foot_frame = "l_sole"
    r_foot_frame = "r_sole"

    -- waiting for reading com data
    gas_setponts.initial_com_in_world_bt = com_port:read(true)
    PointCoordFromYarpVectorBottle(gas_setponts.initial_com_in_world,
                                   gas_setponts.initial_com_in_world_bt)

    -- waiting for reading frame data
    gas_frames_bt = frames_port:read(true)
    HomTransformTableFromBottle(gas_frames,gas_frames_bt)

    -- get transform
    l_foot_H_world = gas_frames[l_foot_frame].inverse()
    r_foot_H_world = gas_frames[r_foot_frame].inverse()

    -- generating left and right desired com
    -- the x and y are the left foot origin
    -- while the z is the one of the initial com
    local initial_com_wrt_left_foot =
        l_foot_H_world.apply(gas_setponts.initial_com_in_world)
    gas_setponts.left_com_in_l_foot.x = initial_com_wrt_left_foot.x
    gas_setponts.left_com_in_l_foot.y = initial_com_wrt_left_foot.y
    gas_setponts.left_com_in_l_foot.z = initial_com_wrt_left_foot.z

    local initial_com_wrt_right_foot =
        r_foot_H_world.apply(gas_setponts.initial_com_in_world)
    gas_setponts.right_com_in_r_foot.x = initial_com_wrt_right_foot.x
    gas_setponts.right_com_in_r_foot.y = initial_com_wrt_right_foot.y
    gas_setponts.right_com_in_r_foot.z = initial_com_wrt_right_foot.z

    print("[INFO] starting main loop")
    repeat
        -- read frames and com information
        gas_updateframes()
        -- run the finite state machine
        -- the configurator is implicitly executed by
        -- the fsm entry/doo/exit functions
        rfsm.run(fsm)
        yarp.Time_delay(fsm_update_period)
    until shouldExit ~= false

    print("[INFO] finishing")

    gas_close_script()
end

main()
#!/usr/bin/lua

require("yarp")
require("rfsm")
require("rfsm_timeevent")
require("math")

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
    swinging_type = yarp_rf_find_string(rf,"swinging_type")

    if( swinging_type ~= "switching" and
        swinging_type ~= "trajectory" ) then
        print("[ERROR] swinging_type should be switching or trajectory")
        gas_close_script()
    end

    if( swinging_type == "switching" ) then
        switching_period = yarp_rf_find_double(rf,"switching_period")
        -- switching_period = yarp_rf_find_double(rf,"switching_period")
        -- switching_period = yarp_rf_find_double(rf,"switching_period")
    end

    if( swinging_type == "trajectory" ) then
  		  trajectory_frequency = yarp_rf_find_double(rf,"trajectory_frequency")
    end

    delta_x = yarp_rf_find_double(rf,"delta_x")
    delta_y = yarp_rf_find_double(rf,"delta_y")
    delta_z = yarp_rf_find_double(rf,"delta_z")

    if( rf:check("verbose") ) then
        print("[INFO]: verbose option found")
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

    -- Port for sending to iSpeak the current state
    iSpeak_port = yarp.BufferedPortBottle()
    iSpeak_port:open("/".. script_name .. "/speak");

    -- Port for reading the com position in the world
    com_port = yarp.BufferedPortBottle()
    com_port:open("/" .. script_name .. "/com:i");

    -- Port for reading the frame positions w.r.t to the world
    frames_port = yarp.BufferedPortBottle()
    frames_port:open("/" .. script_name .. "/frames:i");


    if( swinging_type == "switching" ) then
        -- Port for publishing trajectory generator setpoints
        setpoints_port = yarp.BufferedPortProperty()
        setpoints_port:open("/".. script_name .. "/setpoints:o")
    end

    if( swinging_type == "trajectory" ) then
        -- Port for streaming the com desired trajectory in the world
        comdes_port = yarp.BufferedPortBottle()
        comdes_port:open("/" .. script_name .. "/comDes:o");
    end

end

function gas_close_ports()
    --close ports
    gas_close_port(input_events)
    gas_close_port(state_port)
    gas_close_port(iSpeak_port)
    gas_close_port(com_port)
    gas_close_port(frames_port)
    if( swinging_type == "switching" ) then
        gas_close_port(setpoints_port)
    end

    if( swinging_type == "trajectory" ) then
        gas_close_port(comdes_port)
    end
end

function gas_updateframes()
    -- waiting for reading frame data
    gas_frames_bt = frames_port:read(true)
    if( gas_frames_bt ) then
        HomTransformTableFromBottle(gas_frames,gas_frames_bt)
    end

    -- get transform
    world_H_l_foot = gas_frames[l_foot_frame]
    world_H_r_foot = gas_frames[r_foot_frame]

    -- update com setpoints
    gas_setpoints.left_com_in_world =
        world_H_l_foot:apply(gas_setpoints.left_com_in_l_foot)

    gas_setpoints.right_com_in_world =
        world_H_r_foot:apply(gas_setpoints.right_com_in_r_foot)
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
    if( swinging_type == "switching" ) then
        fsm_file = rf:findFile("lua/fsm_left_right_sway.lua")
    end

    if( swinging_type == "trajectory" ) then
        fsm_file = rf:findFile("lua/fsm_left_right_sine.lua")
    end

    print("[INFO] loading rFSM state machine")
    -- load state machine model and initalize it
    fsm_model = rfsm.load(fsm_file)
    fsm = rfsm.init(fsm_model)

    -- configure script specific hooks

    -- dbg function, callet at each state enter/exit etc etc
    fsm.dbg = gas_dbg;

    -- getevents function, to read functions from a
    fsm.getevents = yarp_gen_read_str_events(input_events);

    gas_setpoints = {
        -- geometric ponts
        initial_com_in_world = PointCoord.create(),

        -- switching mode 
        left_com_in_l_foot = PointCoord.create(),
        right_com_in_r_foot = PointCoord.create(),
        left_com_in_world = PointCoord.create(),
        right_com_in_world = PointCoord.create(),

        -- trajectory mode
        sine_com_in_world  = PointCoord.create(),
        vel_sine_com_in_world  = PointCoord.create(),
        acc_sine_com_in_world  = PointCoord.create(),

        -- bottle buffers to load/unload
        initial_com_in_world_bt = yarp.Bottle(),
        sine_com_in_world_bt    = yarp.Bottle(),
    }

    gas_frames = {}
    l_foot_frame = "l_sole"
    r_foot_frame = "r_sole"

    -- waiting for reading com data
    print("[INFO] waiting for com data")
    gas_setpoints.initial_com_in_world_bt = com_port:read(true)
    PointCoordFromYarpVectorBottle(gas_setpoints.initial_com_in_world,
                                   gas_setpoints.initial_com_in_world_bt)

    -- waiting for reading frame data
    print("[INFO] waiting for frame data")
    gas_frames_bt = frames_port:read(true)
    HomTransformTableFromBottle(gas_frames,gas_frames_bt)

    -- get transform
    --
    l_foot_H_world = gas_frames[l_foot_frame]:inverse()
    r_foot_H_world = gas_frames[r_foot_frame]:inverse()

    -- generating left and right desired com
    local left_com_wrt_world = PointCoord.create()
    left_com_wrt_world.x = gas_setpoints.initial_com_in_world.x + delta_x
    left_com_wrt_world.y = gas_setpoints.initial_com_in_world.y + delta_y
    left_com_wrt_world.z = gas_setpoints.initial_com_in_world.z + delta_z

    gas_setpoints.left_com_in_l_foot =
        l_foot_H_world:apply(left_com_wrt_world)

    print("left_com_wrt_world: ")
    left_com_wrt_world:print()


    local right_com_wrt_world = PointCoord.create()
    right_com_wrt_world.x = gas_setpoints.initial_com_in_world.x - delta_x
    right_com_wrt_world.y = gas_setpoints.initial_com_in_world.y - delta_y
    right_com_wrt_world.z = gas_setpoints.initial_com_in_world.z - delta_z
 
    print("right_com_wrt_world: ")
    right_com_wrt_world:print()

    gas_setpoints.right_com_in_r_foot =
        r_foot_H_world:apply(right_com_wrt_world)


    print("[INFO] starting main loop")
    repeat
        yarp_now = yarp.Time_now()
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

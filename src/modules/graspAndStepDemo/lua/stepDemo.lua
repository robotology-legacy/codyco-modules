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

---
function gas_loadconfiguration()
    -- initialization
    print("[INFO] opening resource finder")
    rf = yarp.ResourceFinder()
    rf:setDefaultConfigFile("stepDemo.ini")
    rf:setDefaultContext("graspAndStepDemo")
    print("[INFO] configuring resource finder")
    rf:configure(arg)

    -- load helper functions
    dofile(rf:findFile("lua/gas_funcs.lua"))
    dofile(rf:findFile("lua/steppingMonitor.lua"))

    -- handling parameters
    if( rf:check("verbose") ) then
        print("[INFO]: verbose option found")
        verbose = true
    end

    if( rf:check("help") ) then
        gas_print_help()
        gas_close_script()
    end

    -- load parameters
    script_name = yarp_rf_find_string(rf,"script_name")
    fsm_update_period = yarp_rf_find_double(rf,"fsm_update_period")
    step_type = yarp_rf_find_string(rf,"step_type")
    gas_setpoints.weight_on_left_foot_com_wrt_l_sole = yarp_rf_find_point(rf,"weight_on_left_foot_com_wrt_l_sole")
    l_foot_frame = "l_sole"
    r_foot_frame = "r_sole"
    root_link = "root_link"
    world     = "world"
    force_threshold = yarp_rf_find_double(rf,"force_threshold");
    com_threshold   = yarp_rf_find_double(rf,"com_threshold")
    step_length     = yarp_rf_find_double(rf,"step_length")
    step_height     = yarp_rf_find_double(rf,"step_height")
    step_penetration = yarp_rf_find_double(rf,"step_penetration")
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

    -- Port for publishing trajectory generator setpoints
    setpoints_port = yarp.BufferedPortProperty()
    setpoints_port:open("/".. script_name .. "/setpoints:o")

    -- Port for activating/deactivating contacts on the controller
    constraints_port = yarp.BufferedPortBottle()
    constraints_port:open("/".. script_name .. "/constraints:o");

    -- Port for configuring the simple odometry
    odometry_port = yarp.BufferedPortBottle()
    odometry_port:open("/".. script_name .. "/odometry:o");

    -- Port for communicating with the cartesianSolver
    root_link_r_sole_solver_port = yarp.Port()
    root_link_r_sole_solver_port:open("/" .. script_name .. "/root_link_r_sole_solver")

    -- Port for reading right leg joint positions
    right_leg_state_port = yarp.BufferedPortBottle()
    right_leg_state_port:open("/" .. script_name .. "/right_leg/state:i")

    -- Port for reading forces are included in the stepperMonitor class
    stepper_monitor = steppingMonitor.create()
end

function gas_close_ports()
    --close ports
    gas_close_port(input_events)
    gas_close_port(state_port)
    gas_close_port(iSpeak_port)
    gas_close_port(com_port)
    gas_close_port(frames_port)
    gas_close_port(setpoints_port)
    gas_close_port(constraints_port)
    gas_close_port(odometry_port)
    gas_close_port(root_link_r_sole_solver_port)
    gas_close_port(right_leg_state_port)

    if( stepper_monitor ~= nil ) then
        stepper_monitor:close()
    end
end

-- initialize setpoints for each state
function gas_initialize_setpoints()
    -- waiting for reading com data
    print("[INFO] waiting for com data")
    comMeas_in_world_bt = com_port:read(true)
    PointCoordFromYarpVectorBottle(comMeas_in_world,
                                   comMeas_in_world_bt)

    comMeas_in_world:print("[INFO] initial com in world frame: ")

    -- waiting for reading frame data
    print("[INFO] waiting for frame data")
    gas_frames_bt = frames_port:read(true)
    gas_frames = {}
    HomTransformTableFromBottle(gas_frames,gas_frames_bt)

    assert(gas_frames[l_foot_frame] ~= nil)
    assert(gas_frames[r_foot_frame] ~= nil)
    assert(gas_frames[root_link]    ~= nil)

    buf = gas_frames[l_foot_frame]

    l_foot_H_world = buf:inverse()

    gas_setpoints.initial_com_in_l_foot = l_foot_H_world:apply(comMeas_in_world)

    -- workaround: weight_on_left_foot_com_wrt_l_sole is used just for the y value
    gas_setpoints.weight_on_left_foot_com_wrt_l_sole.x = gas_setpoints.initial_com_in_l_foot.x
    gas_setpoints.weight_on_left_foot_com_wrt_l_sole.z = gas_setpoints.initial_com_in_l_foot.z
end


function gas_updateframes()
    -- waiting for reading frame data
    gas_frames_bt = frames_port:read(false)
    if( gas_frames_bt ) then
        HomTransformTableFromBottle(gas_frames,gas_frames_bt)
    end

    -- reading com data
    comMeas_in_world_bt = com_port:read()
    if( comMeas_in_world_bt ~= nil ) then
        PointCoordFromYarpVectorBottle(comMeas_in_world,
                                       comMeas_in_world_bt)
    end


    -- get transform
    world_H_l_foot = gas_frames[l_foot_frame]
    world_H_r_foot = gas_frames[r_foot_frame]

    -- update com setpoints
    gas_setpoints.initial_com_in_world =
        world_H_l_foot:apply(gas_setpoints.initial_com_in_l_foot)

    gas_setpoints.weight_on_left_foot_com_in_world =
        world_H_l_foot:apply(gas_setpoints.weight_on_left_foot_com_wrt_l_sole)
end

function main()
    shouldExit = false

    -- create setpoints table (populated by gas_loadconfiguration)
    gas_setpoints = {}

    -- load configuration
    gas_loadconfiguration()

    -- check that yarp network is up
    gas_yarpCheckNetwork()

    -- open ports
    gas_open_ports()

    -- load main FSM
    fsm_file = rf:findFile("lua/fsm_test_right_step.lua")
    print("[INFO] loading rFSM state machine")
    -- load state machine model and initalize it
    fsm_model = rfsm.load(fsm_file)
    fsm = rfsm.init(fsm_model)

    -- configure script specific hooks
    -- use the yarp time for time events
    rfsm_timeevent.set_gettime_hook(yarp_gettime)

    -- dbg function, callet at each state enter/exit etc etc
    fsm.dbg = gas_dbg;

    -- getevents function, to read functions from a
    fsm.getevents = yarp_gen_read_str_events(input_events);

    gas_frames = {}


    print("[INFO] starting main loop")
    comMeas_in_world = PointCoord.create()
    comDes_in_world  = PointCoord.create()
    gas_initialize_setpoints()

    repeat
        yarp_now = yarp.Time_now()
        -- read frames and com information
        gas_updateframes()

        -- raise internal events
        stepper_monitor:run(fsm)
        generate_com_motiondone_events(fsm)

        -- run the finite state machine
        -- the configurator is implicitly executed by
        -- the fsm entry/doo/exit functions
        rfsm.run(fsm)

        -- wait
        yarp.Time_delay(fsm_update_period)
    until shouldExit ~= false

    print("[INFO] finishing")

    gas_close_script()
end

main()

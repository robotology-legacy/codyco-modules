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
    print("[INFO] --verbose : enable verbose output")
    print("[INFO] --help    : print this help")
end

---
function gas_loadconfiguration()
    -- initialization
    print("[INFO] opening resource finder")
    rf = yarp.ResourceFinder()
    rf:setDefaultConfigFile("doubleToSingleDemo.ini")
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
    --gas_setpoints.weight_on_left_foot_com_wrt_l_sole = yarp_rf_find_point(rf,"weight_on_left_foot_com_wrt_l_sole")
    l_foot_frame = "l_sole"
    r_foot_frame = "r_sole"
    root_link = "root_link"
    world     = "world"
    vertical_force_threshold = yarp_rf_find_double(rf,"vertical_force_threshold");
    com_threshold_right_to_left   = yarp_rf_find_double(rf,"com_threshold_right_to_left")
    com_threshold_left_to_right   = yarp_rf_find_double(rf,"com_threshold_left_to_right")
    com_threshold = com_threshold_right_to_left
    q_threshold     = yarp_rf_find_double(rf,"q_threshold")
    step_penetration = yarp_rf_find_double(rf,"step_penetration")
    step_hesitation  = yarp_rf_find_double(rf,"step_hesitation")
    --transfer_delta_com = yarp_rf_find_double(rf,"transfer_delta_com")
    transfer_delta_y_r_sole = yarp_rf_find_double(rf,"transfer_delta_y_r_sole")
    transfer_delta_y_l_sole = yarp_rf_find_double(rf,"transfer_delta_y_l_sole")

    -- single support balancing position
    gas_setpoints.left_leg_single_support = yarp_rf_find_bottle(rf,"left_leg_single_support")
    gas_setpoints.right_leg_single_support = yarp_rf_find_bottle(rf,"right_leg_single_support")
    gas_setpoints.torso_single_support = yarp_rf_find_bottle(rf,"torso_single_support")
    gas_setpoints.left_arm_single_support = yarp_rf_find_bottle(rf,"left_arm_single_support")
    gas_setpoints.right_arm_single_support = yarp_rf_find_bottle(rf,"right_arm_single_support")

    com_single_support = yarp_rf_find_bottle(rf,"com_single_support")


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

    -- Port for reading right leg joint positions
    right_leg_state_port = yarp.BufferedPortBottle()
    right_leg_state_port:open("/" .. script_name .. "/right_leg/state:i")

    -- Port for reading left leg joint positions
    left_leg_state_port = yarp.BufferedPortBottle()
    left_leg_state_port:open("/" .. script_name .. "/left_leg/state:i")

    -- Port for reading right arm joint positions
    right_arm_state_port = yarp.BufferedPortBottle()
    right_arm_state_port:open("/" .. script_name .. "/right_arm/state:i")

    -- Port for reading left arm joint positions
    left_arm_state_port = yarp.BufferedPortBottle()
    left_arm_state_port:open("/" .. script_name .. "/left_arm/state:i")

    -- Port for reading torso joint positions
    torso_state_port = yarp.BufferedPortBottle()
    torso_state_port:open("/" .. script_name .. "/torso/state:i")

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

    -- right_leg specific ports
    gas_close_port(right_leg_state_port)

    -- left_leg specific ports
    gas_close_port(left_leg_state_port)

    gas_close_port(left_arm_state_port)
    gas_close_port(right_arm_state_port)
    gas_close_port(torso_state_port)


    if( stepper_monitor ~= nil ) then
        stepper_monitor:close()
    end
end

-- initialize setpoints for each state
function gas_initialize_setpoints()
    -- waiting for reading com data
    print("[INFO] waiting for com data")
    comMeas_in_world_bt = com_port:read(true)
    PointCoordFromYarpVectorBottle(gas_motion_done_helper.comMeas_in_world,
                                   comMeas_in_world_bt)

    gas_motion_done_helper.comMeas_in_world:print("[INFO] initial com in world frame: ")
    gas_motion_done_helper.comDes_in_world = gas_motion_done_helper.comMeas_in_world

    gas_setpoints.initial_com = gas_motion_done_helper.comMeas_in_world:clone()

    -- The delta is used just for the y value
    gas_setpoints.com_weight_on_left_foot = PointCoord.create()
    gas_setpoints.com_weight_on_left_foot.x = gas_motion_done_helper.comMeas_in_world.x
    --gas_setpoints.weight_on_left_foot_com_wrt_l_sole.y = gas_setpoints.initial_com_wrt_l_foot.y+transfer_delta_com
    gas_setpoints.com_weight_on_left_foot.y = transfer_delta_y_l_sole
    gas_setpoints.com_weight_on_left_foot.z = gas_motion_done_helper.comMeas_in_world.z

    --
    gas_setpoints.com_weight_on_left_foot_single_support = PointCoord.create()
    gas_setpoints.com_weight_on_left_foot_single_support.x = com_single_support:get(0):asDouble()
    gas_setpoints.com_weight_on_left_foot_single_support.y = com_single_support:get(1):asDouble()
    gas_setpoints.com_weight_on_left_foot_single_support.z = com_single_support:get(2):asDouble()


end

function gas_send_double_support_postural(setpoints_port)
    gas_sendAllPartsToTrajGen(setpoints_port,gas_setpoints.initialLeftLeg,gas_setpoints.initialRightLeg,gas_setpoints.initialTorso,gas_setpoints.initialLeftArm,gas_setpoints.initialRightArm);
end

function gas_send_single_support_postural(setpoints_port)
    gas_sendAllPartsToTrajGen(setpoints_port,gas_setpoints.left_leg_single_support,gas_setpoints.right_leg_single_support,gas_setpoints.torso_single_support,gas_setpoints.left_arm_single_support,gas_setpoints.right_arm_single_support)
end

function gas_readcom()
    -- reading com data
    comMeas_in_world_bt = com_port:read(true)
    if( comMeas_in_world_bt ~= nil ) then
        PointCoordFromYarpVectorBottle(gas_motion_done_helper.comMeas_in_world,
                                       comMeas_in_world_bt)
    end

    print("[DEBUG] waiting right leg")
    -- read right leg joint positions
    rightLegMeas_bt = right_leg_state_port:read(true)
    if( rightLegMeas_bt ~= nil ) then
        gas_motion_done_helper.rightLegMeas = yarp.Bottle()
        gas_motion_done_helper.rightLegMeas:copy(rightLegMeas_bt);
        gas_setpoints.initialRightLeg       = yarp.Bottle()
        gas_setpoints.initialRightLeg:copy(rightLegMeas_bt);
    end

    print("[DEBUG] waiting left leg")
    -- read left leg joint positions
    leftLegMeas_bt  = left_leg_state_port:read(true)
    if( leftLegMeas_bt ~= nil ) then
        gas_motion_done_helper.leftLegMeas = yarp.Bottle()
        gas_motion_done_helper.leftLegMeas:copy(leftLegMeas_bt);
        gas_setpoints.initialLeftLeg       = yarp.Bottle()
        gas_setpoints.initialLeftLeg:copy(leftLegMeas_bt);
    end

    print("[DEBUG] waiting torso")
    -- read torso joint positions
    torsoMeas_bt  = torso_state_port:read(true)
    if( torsoMeas_bt ~= nil ) then
        gas_setpoints.initialTorso       = yarp.Bottle()
        gas_setpoints.initialTorso:addDouble(torsoMeas_bt:get(2):asDouble());
        gas_setpoints.initialTorso:addDouble(torsoMeas_bt:get(1):asDouble());
        gas_setpoints.initialTorso:addDouble(torsoMeas_bt:get(0):asDouble());
        gas_motion_done_helper.torsoMeas = yarp.Bottle()
        gas_motion_done_helper.torsoMeas:copy(gas_setpoints.initialTorso);
    end

    print("[DEBUG] waiting left arm")
    -- read left arm joint positions
    leftArmMeas_bt  = left_arm_state_port:read(true)
    if( leftArmMeas_bt ~= nil ) then
        gas_motion_done_helper.leftArmMeas = yarp.Bottle()
        gas_motion_done_helper.leftArmMeas:copy(leftArmMeas_bt,0,5);
        gas_setpoints.initialLeftArm       = yarp.Bottle()
        gas_setpoints.initialLeftArm:copy(leftArmMeas_bt,0,5);
    end

    print("[DEBUG] waiting right arm")
    -- read right arm joint positions
    rightArmMeas_bt  = right_arm_state_port:read(true)
    if( rightArmMeas_bt ~= nil ) then
        gas_motion_done_helper.rightArmMeas = yarp.Bottle()
        gas_motion_done_helper.rightArmMeas:copy(rightArmMeas_bt,0,5);
        gas_setpoints.initialRightArm       = yarp.Bottle()
        gas_setpoints.initialRightArm:copy(rightArmMeas_bt,0,5);
    end


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
    fsm_file = rf:findFile("lua/fsm_doubleToSingle.lua")
    print("[INFO] loading rFSM state machine from " .. fsm_file)
    -- load state machine model and initalize it
    rfsm_timeevent.set_gettime_hook(yarp_gettime)

    fsm_model = rfsm.load(fsm_file)
    fsm = rfsm.init(fsm_model)

    -- configure script specific hooks
    -- use the yarp time for time events


    -- dbg function, callet at each state enter/exit etc etc
    fsm.dbg = gas_dbg;

    -- getevents function, to read functions from a
    fsm.getevents = yarp_gen_read_str_events(input_events);

    -- helper table for keeping world to frame frame
    gas_frames = {}
    -- helper table to keep the desired and commanded joint position
    -- for the various part, useful to generate motion_done events
    gas_motion_done_helper = {}

    gas_motion_done_helper.comMeas_in_world = PointCoord.create()
    gas_motion_done_helper.comDes_in_world  = PointCoord.create()
    -- read com information
    gas_readcom()
    gas_initialize_setpoints()

    print("[INFO] starting main loop")
    repeat
        yarp_now = yarp.Time_now()


        -- raise internal events
        stepper_monitor:run(fsm)
        --generate_motiondone_events(fsm)

        -- run the finite state machine
        -- the configurator is implicitly executed by
        -- the fsm entry/doo/exit functions
        rfsm.run(fsm)

        -- wait (the fsm is doing blocking operations)
        local time_passed = yarp.Time_now()-yarp_now
        if( time_passed >= 0.0 and time_passed < fsm_update_period ) then
            yarp.Time_delay(fsm_update_period-time_passed)
        end

    until shouldExit ~= false

    print("[INFO] finishing")

    gas_close_script()
end

main()

steppingMonitor = {}
steppingMonitor.__index = steppingMonitor

function steppingMonitor.create()
    local mon = {}             -- our new object
    setmetatable(mon,steppingMonitor)  -- make steppingMonitor handle lookup
    -- open external force ports
    mon.right_foot_wrench_port = yarp.BufferedPortBottle()
    mon.right_foot_wrench_port:open("/".. script_name .."/right_foot_wrench:i")
    mon.left_foot_wrench_port = yarp.BufferedPortBottle()
    mon.left_foot_wrench_port:open("/".. script_name .."/left_foot_wrench:i")
    -- open skin events port
    --mon.skin_events_port = yarp.BufferedPortBottle()
    --mon.skin_events_port:open("/".. script_name .."/skin_events:i")
    -- create buffers
    mon.buffer_skin_contacts = yarp.Bottle()
    mon.buffer_left_foot_wrench = yarp.Bottle()
    mon.buffer_left_foot_wrench:addDouble(0.0)
    mon.buffer_left_foot_wrench:addDouble(0.0)
    mon.buffer_left_foot_wrench:addDouble(0.0)
    mon.buffer_left_foot_wrench:addDouble(0.0)
    mon.buffer_left_foot_wrench:addDouble(0.0)
    mon.buffer_left_foot_wrench:addDouble(0.0)
    mon.buffer_right_foot_wrench = buffer_left_foot_wrench
    mon.buffer_left_force_norm = 0.0
    mon.buffer_right_force_norm = 0.0
    return mon
end

function steppingMonitor:update_buffers()
    local new_right_wrench = self.right_foot_wrench_port:read(false)
    if new_right_wrench ~= nil then
        self.buffer_right_foot_wrench = new_right_wrench
        self.buffer_right_foot_force_norm = yarpBottleNorm(self.buffer_right_foot_wrench,3)
    end

    local new_left_wrench = left_wrench_port:read(false)
    if new_left_wrench ~= nil then
        self.buffer_left_foot_wrench = new_left_wrench
        self.buffer_left_foot_force_norm = yarpBottleNorm(self.buffer_left_foot_wrench,3)
    end
end

function steppingMonitor:run(fsm)
    steppingMonitor:update_buffers()

    -- events no weight
    if( self.buffer_left_foot_force_norm < force_threshold )
        rfsm.send_events(fsm,'e_no_weight_on_left_foot')
    else
        rfsm.send_events(fsn,'e_weight_on_left_foot')
    end

    if( self.buffer_right_foot_force_norm < force_threshold )
        rfsm.send_events(fsm,'e_no_weight_on_right_foot')
    else
        rfsm.send_events(fsm,'e_weight_on_right_foot')
    end

end

function steppingMonitor:close()
  -- close ports
  self.right_foot_wrench_port:close()
  self.left_foot_wrench_port:close()
  self.skin_events_port:close()
end




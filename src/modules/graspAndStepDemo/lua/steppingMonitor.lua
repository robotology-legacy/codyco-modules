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

steppingMonitor = {}
steppingMonitor.__index = steppingMonitor

function steppingMonitor.create()
    local mon = {}             -- our new object
    setmetatable(mon,steppingMonitor)  -- make steppingMonitor handle lookup
    -- open external force ports
    print("[DEBUG] creating right_foot_wrench_port")
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
    mon.buffer_right_foot_wrench = yarp.Bottle()
    mon.buffer_right_foot_wrench:addDouble(0.0)
    mon.buffer_right_foot_wrench:addDouble(0.0)
    mon.buffer_right_foot_wrench:addDouble(0.0)
    mon.buffer_right_foot_wrench:addDouble(0.0)
    mon.buffer_right_foot_wrench:addDouble(0.0)
    mon.buffer_right_foot_wrench:addDouble(0.0)
    mon.buffer_left_foot_force_norm = 0.0
    mon.buffer_right_foot_force_norm = 0.0
    return mon
end


function steppingMonitor:update_buffers()
    new_right_wrench = self.right_foot_wrench_port:read(false)
    if new_right_wrench ~= nil then
        self.buffer_right_foot_wrench = new_right_wrench
        self.buffer_right_foot_force_norm = yarpBottleNorm(self.buffer_right_foot_wrench,3)
    end

    new_left_wrench = self.left_foot_wrench_port:read(false)
    if new_left_wrench ~= nil then
        self.buffer_left_foot_wrench = new_left_wrench
        self.buffer_left_foot_force_norm = yarpBottleNorm(self.buffer_left_foot_wrench,3)
    end
end

function steppingMonitor:run(fsm)
    self:update_buffers()

    -- events no weight
    if( math.abs(self.buffer_left_foot_wrench:get(2):asDouble()) < vertical_force_threshold ) then
        rfsm.send_events(fsm,'e_no_weight_on_left_foot')
    else
        rfsm.send_events(fsm,'e_weight_on_left_foot')
    end

    vertical_force = self.buffer_right_foot_wrench:get(2):asDouble();
    --print("Vertical force on right foot : " .. vertical_force)
    if( math.abs(self.buffer_right_foot_wrench:get(2):asDouble()) < vertical_force_threshold ) then
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




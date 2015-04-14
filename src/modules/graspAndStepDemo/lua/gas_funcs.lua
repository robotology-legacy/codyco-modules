
----------------------------------
-- functions                   --
----------------------------------

function YarpVectorBottleForTrajGenFromPointCoord(point)
    local vecBot = yarp.Bottle()

    vecBot:addList()
    vecBot:get(0):asList():addDouble(point.x);
    vecBot:get(0):asList():addDouble(point.y);
    vecBot:get(0):asList():addDouble(point.z);

    return vecBot;
end

function ValueFromBottleAndDeg2Rad(bot)

    local vecBot = yarp.Bottle()
    local deg2rad = math.pi/180.0

    vecBot:addList()

    for i = 0,bot:size()-1 do
        vecBot:get(0):asList():addDouble(deg2rad*bot:get(i):asDouble())
    end

    return vecBot:get(0)
end


--- Send reference point to the trajectory generator.
-- The trajectory generator is the codycoTrajGenY2 module a
-- and it accepts a property where each trajectory setpoint
-- is a Bottle containg a yarp::sig::Vector saved with an appropriate key.
-- @param port The reference point port
-- @param setpoint a PointCoord object with the actual setpoint
--
function gas_sendCOMToTrajGen(port,setpoint)
   gas_motion_done_helper.comDes_in_world = setpoint;
   local botTrajGen = YarpVectorBottleForTrajGenFromPointCoord(setpoint)
   local prop = port:prepare();
   prop:clear()
   prop:put("com",botTrajGen:get(0))
   port:write()
end

function gas_sendPartToTrajGen(port,partName, setpoint_bt)
   assert(setpoint_bt,"gas_sendPartToTrajGen: tryng to send nil value")
   if( partName == "right_leg" ) then
       gas_motion_done_helper.rightLegDes = setpoint_bt
   end
   if( partName == "left_leg" ) then
       gas_motion_done_helper.leftLegDes  = setpoint_bt
   end
   local prop = port:prepare();
   prop:clear()
   prop:put(partName,ValueFromBottleAndDeg2Rad(setpoint_bt))
   port:write()
end

function yarpBottleDiffNorm(bot,bot2,val)
    local nrm = 0
    if( val == nil ) then
        val = 200
    end
    nrOfElems = math.min(val,bot:size()-1,bot2:size()-1)
    for i = 0,nrOfElems do
        local el = bot:get(i):asDouble()
        local el2 = bot2:get(i):asDouble()
        nrm = nrm + (el-el2)*(el-el2)
    end
    nrm = math.sqrt(nrm)
    return nrm
end

function generate_motiondone_events(fsm)
    -- com motion done
    local errX = gas_motion_done_helper.comDes_in_world.x - gas_motion_done_helper.comMeas_in_world.x;
    local errY = gas_motion_done_helper.comDes_in_world.y - gas_motion_done_helper.comMeas_in_world.y;
    local errZ = gas_motion_done_helper.comDes_in_world.z - gas_motion_done_helper.comMeas_in_world.z;
    local comErr = math.sqrt(errX*errX + errY*errY + errZ*errZ);
    --print("comErr: " .. comErr)
    if( comErr < com_threshold ) then
        rfsm.send_events(fsm,'e_com_motion_done')
    end

    -- right leg motion done
    if( gas_motion_done_helper.rightLegDes ~= nil and gas_motion_done_helper.rightLegMeas ~= nil ) then
        qErrRL = yarpBottleDiffNorm(gas_motion_done_helper.rightLegDes,gas_motion_done_helper.rightLegMeas)

        if( qErrRL < q_threshold ) then
            rfsm.send_events(fsm,'e_right_leg_motion_done')
        end
    end

    -- left leg motion done
    if( gas_motion_done_helper.leftLegDes ~= nil and gas_motion_done_helper.leftLegDes ~= nil ) then
        qErrLL = yarpBottleDiffNorm(gas_motion_done_helper.leftLegDes,gas_motion_done_helper.leftLegMeas)

        if( qErrLL < q_threshold ) then
            rfsm.send_events(fsm,'e_left_leg_motion_done')
        end
    end


    if( lastPrintTime == nil ) then
        lastPrintTime = yarp_now
    end

    if( yarp_now-lastPrintTime > 1.0 ) then
        lastPrintTime = yarp_now
        print("comErr : " .. comErr)

        if( qErrRL ~= nill ) then
            print(" qErrRL: " .. qErrRL)
        end

        if( qErrLL ~= nill ) then
            print(" qErrLL: " .. qErrLL)
        end
    end

end

function generate_legs_motiondone_events(fsm)
end

function gas_sendCOMToBalancing(port,pos,vel,acc)
    local bot = port:prepare();
    bot:clear()
    bot:addDouble(pos.x)
    bot:addDouble(pos.y)
    bot:addDouble(pos.z)
    bot:addDouble(vel.x)
    bot:addDouble(vel.y)
    bot:addDouble(vel.z)
    bot:addDouble(acc.x)
    bot:addDouble(acc.y)
    bot:addDouble(acc.z)
    port:write()
end

--- Activate constraints on the controller
-- @param port The constraints port
-- @param constraints a table of names of link
--        that should be considered active constraints
--
function gas_activateConstraints(port,activatedConstraints)
   local bot = port:prepare();
   bot:clear()
   bot:addString("activateConstraints")
   for i = 1,#activatedConstraints do
       bot:addString(activatedConstraints[i])
   end
   port:write()
end

--- Deactivate constraint on the controller
-- @param port The active contacts ports
-- @param contacts a table of names of link
--        that should be considered not active contacts
--
function gas_deactivateConstraints(port,deactivatedConstraints)
   local bot = port:prepare();
   bot:clear()
   bot:addString("deactivateConstraints")
   for i = 1,#deactivatedConstraints do
       bot:addString(deactivatedConstraints[i])
   end
   port:write()
end

--- Send string to a port (useful for non-blocking RPC)
--
--
function gas_sendStringToPort(port,string)
   local bot = port:prepare();
   bot:clear()
   bot:addString(string)
   port:write()
end

--- Send two strings to a port (useful for non-blocking RPC)
--
--
function gas_sendStringsToPort(port,string1,string2)
   local bot = port:prepare();
   bot:clear()
   bot:addString(string1)
   bot:addString(string2)
   port:write()
end

function gas_dbg(event_fqn, state_fqn)
    if( event_fqn == "STATE_ENTER" ) then
        print("[INFO][" .. script_name .. "][" .. event_fqn .. "] entering state " .. state_fqn)
        -- state_fqn : get last part and substitute underscore and dash with spaces
        state_simple_name = string.match(state_fqn,".ST_[%w_]+$")
        if( state_simple_name ) then
            state_speak = state_simple_name:gsub(".ST_",""):gsub("-"," "):gsub("_"," ")
            gas_sendStringToPort(iSpeak_port," entering state " .. state_speak)
        end
    end
end

function gas_close_port(port)
    if(port) then
        port:interrupt()
        port:close()
    end
end

function yarp_rf_find_double(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asDouble()
        print("[INFO] setting " .. var_name .. " to " .. var)
        return var
    else
        print("[ERROR] " .. var_name .." parameter not found, exiting")
        gas_close_script()
    end
end

function yarp_rf_find_int(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asInt()
        print("[INFO] setting " .. var_name .. " to " .. var)
        return var
    else
        print("[ERROR] " .. var_name .." parameter not found, exiting")
        gas_close_script()
    end
end


function yarp_rf_find_string(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asString()
        print("[INFO] setting " .. var_name .. " to " .. var)
        return var
    else
        print("[ERROR] " .. var_name .." parameter not found, exiting")
        gas_close_script()
    end
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
            else
                for i=0,cmd:size() do
                    if( cmd:get(i):isString() ) then
                        tgttab[#tgttab+1] = cmd:get(i):asString()
                    end
                end
            end
        end
     end

     local ports = {...}
     assert(#ports > 0, "[ERROR] no ports given")
     -- check its all ports
     return function ()
         local res = {}
         for _,port in ipairs(ports) do read_events(res, port) end
         return res
     end
end

-- enabling use rfsm_timeevent with yarp::os::Time::now()
--   in this way if you use gazebo_yarp_plugins the
--   fsm is synchronized with the simulation
function yarp_gettime()
    local yarp_time_now = yarp.Time_now()
    local yarp_time_now_sec  = math.floor(yarp.Time_now())
    local yarp_time_now_nsec = math.floor((yarp_time_now-yarp_time_now_sec)*1e9)
    return yarp_time_now_sec, yarp_time_now_nsec
end



--- Basic geometric classes

-- Vector in 3d: difference of points
VectorCoord = {}
VectorCoord.__index = VectorCoord

function VectorCoord.create()
   local vec = {}             -- our new object
   setmetatable(vec,VectorCoord)  -- make RotMatrix handle lookup
   vec.x = 0.0       -- initialize our object
   vec.y = 0.0
   vec.z = 0.0
   return vec
end

function VectorCoord:opposite()
   local oppositeVec = VectorCoord.create()             -- our new object
   oppositeVec.x = -self.x;
   oppositeVec.y = -self.y;
   oppositeVec.z = -self.z;
   return oppositeVec;
end

-- add the current vector to a point, return a point
function VectorCoord:add(point)
    --assert(point.__index == PointCoord)
    local translatedPoint = PointCoord.create()
    translatedPoint.x = point.x + self.x
    translatedPoint.y = point.y + self.y
    translatedPoint.z = point.z + self.z

    return translatedPoint;
end

function VectorCoord:print( prefix )
    if( prefix == nil ) then
       prefix = ""
    end
    print(prefix .. "x: " .. self.x .. " y: " .. self.y .. " z: " .. self.z)
end

-- Point coordinates
PointCoord = {}
PointCoord.__index = PointCoord

function PointCoord.create()
   local point = {}             -- our new object
   setmetatable(point,PointCoord)  -- make RotMatrix handle lookup
   point.x = 0.0       -- initialize our object
   point.y = 0.0
   point.z = 0.0
   return point
end

function PointCoord:opposite()
   local oppositePoint = {}             -- our new object
   oppositePoint.x = -self.x;
   oppositePoint.y = -self.y;
   oppositePoint.z = -self.z;
   return oppositePoint;
end

function PointCoord:add(point)
    -- assert(point.__index == VectorCoord, "PointCoord:add takes a VectorCoord as an input")
    local translatedPoint = PointCoord.create()
    translatedPoint.x = point.x + self.x
    translatedPoint.y = point.y + self.y
    translatedPoint.z = point.z + self.z

    return translatedPoint;
end

function PointCoord:print( prefix )
    if( prefix == nil ) then
       prefix = ""
    end
    print(prefix .. "x: " .. self.x .. " y: " .. self.y .. " z: " .. self.z)
end

function PointCoord:clone()
    clonedPoint = PointCoord.create()
    clonedPoint.x = self.x
    clonedPoint.y = self.y
    clonedPoint.z = self.z

    return clonedPoint
end

-- Rotation (expressed as a rotation matrix)
RotMatrix = {}
RotMatrix.__index = RotMatrix

function RotMatrix.create()
   local rot = {}             -- our new object
   setmetatable(rot,RotMatrix)  -- make RotMatrix handle lookup
   rot.xx = 1.0       -- initialize our object
   rot.yy = 1.0
   rot.zz = 1.0
   rot.xy = 0.0
   rot.xz = 0.0
   rot.yx = 0.0
   rot.yz = 0.0
   rot.zx = 0.0
   rot.zy = 0.0
   return rot
end

function RotMatrix:inverse()
    local invRot = RotMatrix.create()
    invRot.xx = self.xx
    invRot.yy = self.yy
    invRot.zz = self.zz
    invRot.xy = self.yx
    invRot.xz = self.zx
    invRot.yx = self.xy
    invRot.yz = self.zy
    invRot.zx = self.xz
    invRot.zy = self.yz
    return invRot
end

function RotMatrix:apply(point)
    --assert(point.__index == PointCoord or
    --       point.__index == VectorCoord)
    local transformedPoint = PointCoord.create()
    transformedPoint.x = self.xx * point.x + self.xy * point.y + self.xz * point.z;
    transformedPoint.y = self.yx * point.x + self.yy * point.y + self.yz * point.z;
    transformedPoint.z = self.zx * point.x + self.zy * point.y + self.zz * point.z;
    return transformedPoint;
end

function RotMatrix:compose(other)
    local composedRotation = RotMatrix.create()
    composedRotation.xx = self.xx * other.xx + self.xy * other.yx + self.xz * other.zx;
    composedRotation.xy = self.xx * other.xy + self.xy * other.yy + self.xz * other.zy;
    composedRotation.xz = self.xx * other.xz + self.xy * other.yz + self.xz * other.zz;
    composedRotation.yx = self.yx * other.xx + self.yy * other.yx + self.yz * other.zx;
    composedRotation.yy = self.yx * other.xy + self.yy * other.yy + self.yz * other.zy;
    composedRotation.yz = self.yx * other.xz + self.yy * other.yz + self.yz * other.zz;
    composedRotation.zx = self.zx * other.xx + self.zy * other.yx + self.zz * other.zx;
    composedRotation.zy = self.zx * other.xy + self.zy * other.yy + self.zz * other.zy;
    composedRotation.zz = self.zx * other.xz + self.zy * other.yz + self.zz * other.zz;
    return composedRotation;
end

function RotMatrix:clone()
    local clonedRotation = RotMatrix.create()
    clonedRotation.xx = self.xx
    clonedRotation.xy = self.xy
    clonedRotation.xz = self.xz
    clonedRotation.yx = self.yx
    clonedRotation.yy = self.yy
    clonedRotation.yz = self.yz
    clonedRotation.zx = self.zx
    clonedRotation.zy = self.zy
    clonedRotation.zz = self.zz

    return clonedRotation;
end

function RotMatrix:print(prefix)
    if ( prefix == nil ) then
        prefix = ""
    end
    print(prefix .. " : ")
    print(self.xx .. " " .. self.xy .. " " .. self.xz )
    print(self.yx .. " " .. self.yy .. " " .. self.yz )
    print(self.zx .. " " .. self.zy .. " " .. self.zz )
end

function RotMatrixFromAxisAngleTable(tab)
    return RotMatrixFromAxisAngle(tab.ax,tab.ay,tab.az,tab.theta)
end

function RotMatrixFromAxisAngle(ax,ay,az,theta)
    -- ported from Matrix yarp::math::axis2dcm(const Vector &v)
    local rot = RotMatrix.create()

    -- the default rotation is the identity
    if theta==0.0 then
        return rot;
    end

    local costheta=math.cos(theta);
    local sintheta=math.sin(theta);
    local oneMinusCostheta=1.0-costheta;

    local xs =ax*sintheta;
    local ys =ay*sintheta;
    local zs =az*sintheta;
    local xC =ax*oneMinusCostheta;
    local yC =ay*oneMinusCostheta;
    local zC =az*oneMinusCostheta;
    local xyC=ax*yC;
    local yzC=ay*zC;
    local zxC=az*xC;

    rot.xx=ax*xC+costheta;
    rot.xy=xyC-zs;
    rot.xz=zxC+ys;
    rot.yx=xyC+zs;
    rot.yy=ay*yC+costheta;
    rot.yz=yzC-xs;
    rot.zx=zxC-ys;
    rot.zy=yzC+xs;
    rot.zz=az*zC+costheta;

    return rot
end

function AxisAngleTableFromRotMatrix(rot)
    local tolAA = 1e-9
    local tab = {}
    tab.ax = 0
    tab.ay = 0
    tab.az = 0
    tab.theta = 0

    -- ported from Vector yarp::math::dcm2axis(const Matrix &R)
    tab.ax=rot.zy-rot.yz;
    tab.ay=rot.xz-rot.zx;
    tab.az=rot.yx-rot.xy;
    tab.theta=0.0;
    local r = math.sqrt(tab.ax*tab.ax+tab.ay*tab.ay+tab.az*tab.az);

    if (r<tolAA) then
        -- symmetric matrix: theta can be 0 or pi
        local px = rot.xx-1
        local py = rot.yy-1
        local pz = rot.zz-1
        local ax2 = (px-py-pz)/4
        local ay2 = (py-px-pz)/4
        local az2 = (pz-px-py)/4
        if( ax2+ay2+az2 < 0.5 ) then
        -- theta equal to 0
            tab.theta = 0.0
            tab.ax = 0.0
            tab.ay = 0.0
            tab.az = 0.0
        else
            tab.theta = math.pi
            tab.ax = math.sqrt(ax2)
            tab.ay = math.sqrt(ay2)
            tab.az = math.sqrt(az2)
        end

    else
        -- normal case: 0 < theta < pi
        tab.ax = tab.ax/r;
        tab.ay = tab.ay/r;
        tab.az = tab.az/r
        tab.theta=math.atan2(0.5*r,0.5*(rot.xx+rot.yy+rot.zz-1));
    end

    return tab;
end

-- Homogeneous transform
HomTransform = {}
HomTransform.__index = HomTransform

function HomTransform.create()
   local transform = {}             -- our new object
   setmetatable(transform,HomTransform)  -- make HomTransform handle lookup
   transform.rot = RotMatrix.create();
   transform.origin = PointCoord.create();
   return transform
end

function HomTransform:inverse()
    local invTrans = HomTransform.create()
    invTrans.rot = self.rot:inverse();
    invTrans.origin = invTrans.rot:apply(self.origin:opposite());
    return invTrans
end


function HomTransform:apply(point)
    transformedPoint = self.rot:apply(point);

        transformedPoint.x = transformedPoint.x + self.origin.x;
        transformedPoint.y = transformedPoint.y + self.origin.y;
        transformedPoint.z = transformedPoint.z + self.origin.z;

    return transformedPoint
end

function HomTransform:compose(other)
    local composedTransform = HomTransform.create()
    composedTransform.rot = self.rot:compose(other.rot)
    composedTransform.origin = self:apply(other.origin)

    return composedTransform
end

function HomTransform:clone()
    local clonedTransform = HomTransform.create()
    clonedTransform.rot = self.rot:clone()
    clonedTransform.origin = self.origin:clone()

    return clonedTransform
end


--- Functions for loading geom class from yarp data structures
function PointCoordFromYarpVectorBottle(point, vecBot)
    if( point == nil ) then
        point = PointCoord.create()
    end

    point.x = vecBot:get(0):asDouble()
    point.y = vecBot:get(1):asDouble()
    point.z = vecBot:get(2):asDouble()
end


function HomTransformFromYarpMatrixBottle(transform, matBot)
    if( transform == nil ) then
        transform = HomTransform.create()
    end

    local data = matBot:get(2):asList();

    if( matBot:get(0):asInt() ~= 4 or
        matBot:get(1):asInt() ~= 4 or
        data:size() ~= 16 ) then
        print("[ERROR] HomTransformFromYarpMatrix: unexpected matrix size")
        return
    end

    transform.rot.xx = data:get(0):asDouble()
    transform.rot.xy = data:get(1):asDouble()
    transform.rot.xz = data:get(2):asDouble()

    transform.origin.x = data:get(3):asDouble()

    transform.rot.yx = data:get(4):asDouble()
    transform.rot.yy = data:get(5):asDouble()
    transform.rot.yz = data:get(6):asDouble()

    transform.origin.y = data:get(7):asDouble()

    transform.rot.zx = data:get(8):asDouble()
    transform.rot.zy = data:get(9):asDouble()
    transform.rot.zz = data:get(10):asDouble()

    transform.origin.z = data:get(11):asDouble()

    return transform

end

-- This function will load a table of transform of the
-- different frames with respect to the world
-- in a nutshell, tab.r_foot will contain the world_H_r_foot
-- transform
function HomTransformTableFromBottle(transformTable, tabBot)
    if( transformTable == nil ) then
        transformTable = {}
    end

    local botMax = tabBot:size()-1
    for i = 0,botMax do
        local transformName = tabBot:get(i):asList():get(0):asString()
        local matrixBot        = tabBot:get(i):asList():get(1):asList()
        local homTransform = HomTransform.create()
        HomTransformFromYarpMatrixBottle(homTransform, matrixBot)
        transformTable[transformName] = homTransform;
    end
end

--- This function takes a table in the format provided by HomTransformTableFromBottle
--- and it return an HomTransform firstFrame_H_secondFrame
function getTransform(transformTable, firstFrame, secondFrame)
    return transformTable[firstFrame]:inverse():compose(transformTable[secondFrame]);
end


function gas_yarpCheckNetwork()
    print("[INFO] opening yarp")
    yarp.Network_init()

    yarpNetworkTimeout = 10

    if( not yarp.NetworkBase_checkNetwork(yarpNetworkTimeout) ) then
        print("[INFO] yarp server not found, exiting")
        yarp.Network_fini()
        os.exit()
    end
end

function yarp_rf_find_point(rf,var_name)
    if( rf:check(var_name) ) then
        local var = rf:find(var_name):asList()
        local varPoint = PointCoord.create()

        if( var:size() ~= 3 ) then
            print("[ERROR] " .. var_name .." parameter found, but do not have 3 elements as a proper point, exiting")
            gas_close_script()
        end

        varPoint.x = var:get(0):asDouble()
        varPoint.y = var:get(1):asDouble()
        varPoint.z = var:get(2):asDouble()

        print("[INFO] setting " .. var_name .. " to " .. var:toString())
        return varPoint
    else
        print("[ERROR] " .. var_name .." parameter not found, exiting")
        gas_close_script()
    end
end

-- Query (in a blocking way) the cartesianSolver for getting
-- suitable configuration, given a desire HomTransform
-- (conversion to axis angle is done internally)
function query_cartesian_solver(solver_rpc, des_trans, rest_pos_bt)
    local axisAngle =  AxisAngleTableFromRotMatrix(des_trans.rot);
    local req = yarp.Bottle()
    req:addString("ask")
    local poseReq = req:addList()
    poseReq:addString("xd")
    local poseReq2 = poseReq:addList()
    poseReq2:addDouble(des_trans.origin.x)
    poseReq2:addDouble(des_trans.origin.y)
    poseReq2:addDouble(des_trans.origin.z)
    poseReq2:addDouble(axisAngle.ax)
    poseReq2:addDouble(axisAngle.ay)
    poseReq2:addDouble(axisAngle.az)
    poseReq2:addDouble(axisAngle.theta)

    -- adding the current joint position as rest position
    local restPos = req:addList()
    restPos:addString("resp")
    local restPos2 = restPos:addList()
    for i = 0,rest_pos_bt:size()-1 do
        restPos2:add(rest_pos_bt:get(i):asDouble())
    end

    -- adding the current weight of the rest positions
    local restPosWeight = req:addList()
    restPosWeight:addString("resw")
    local restPosWeight2 = restPosWeight:addList()
    for i = 0,rest_pos_bt:size()-1 do
        restPosWeight2:add(0.01)
    end

    -- print("[DEBUG] requested xd" .. poseReq2:toString())
    reply = yarp.Bottle()
    solver_rpc:write(req,reply)
    -- print("[DEBUG] actual cartesianSolver reply: " .. reply:toString())
    xd = reply:get(1):asList():get(1):asList()

    aa_r_sole_root_link_d = {}
    aa_r_sole_root_link_d.ax = xd:get(3):asDouble()
    aa_r_sole_root_link_d.ay = xd:get(4):asDouble()
    aa_r_sole_root_link_d.az = xd:get(5):asDouble()
    aa_r_sole_root_link_d.theta = xd:get(6):asDouble()

    r_sole_R_root_link = RotMatrixFromAxisAngleTable(aa_r_sole_root_link_d)
    r_sole_R_l_sole = r_sole_R_root_link:compose(gas_get_transform(root_link,l_foot_frame).rot)

    r_sole_R_l_sole:print("[DEBUG] Desired r_sole_R_l_sole: ")
    
    qd = reply:get(2):asList():get(1):asList()
    
    -- print("[DEBUG] qd : " .. qd:toString())
    return qd
end

function query_right_leg_cartesian_solver(des_trans)
    -- print("[DEBUG] reading right leg joint positions")
    qMeas = right_leg_state_port:read(true)

    return query_cartesian_solver(root_link_r_sole_solver_port, des_trans, qMeas)

end

function gas_get_transform(final_frame, origin_frame)
    if( final_frame == "world" ) then
        return gas_frames[origin_frame]:clone()
    end
    if( origin_frame == "world" ) then
        return gas_frames[final_frame]:inverse()
    end
    return gas_frames[final_frame]:inverse():compose(gas_frames[origin_frame])
end


function gas_generate_right_foot_setpoints()
    -- first setpoint
    delta_initial_swing_in_r_foot = VectorCoord.create()

    -- go ahead of half a step length
    delta_initial_swing_in_r_foot.x = step_length/2

    -- no lateral change
    delta_initial_swing_in_r_foot.y = 0.0

    -- go up of the step_height parameter
    delta_initial_swing_in_r_foot.z = step_height

     -- rotate the delta in world orientation
    local delta_initial_swing_in_world =  gas_get_transform("world",r_foot_frame).rot:apply(delta_initial_swing_in_r_foot)

    delta_initial_swing_in_r_foot:print("delta_initial_swing_in_r_foot: ")
    delta_initial_swing_in_world:print("delta_initial_swing_in_world : ")


    -- transform the desired orientation of the frame in world
    world_r_foot_cur_pos = gas_get_transform("world",r_foot_frame)

    gas_setpoints.world_r_foot_initial_swing_des_pos = world_r_foot_cur_pos:clone()

    -- save first setpoint in gas_setpoints.world_r_foot_initial_swing_des_pos
    world_r_foot_cur_pos.origin:print(" world_r_foot_cur_pos.origin ")
    delta_initial_swing_in_world:print(" delta_initial_swing_in_world ")
    gas_setpoints.world_r_foot_initial_swing_des_pos.origin = world_r_foot_cur_pos.origin:add(delta_initial_swing_in_world)
    gas_setpoints.world_r_foot_initial_swing_des_pos.origin:print("gas_setpoints.world_r_foot_initial_swing_des_pos.origin")

    gas_get_transform(world,r_foot_frame).origin:print("Initial r_sole pose ")
    gas_setpoints.world_r_foot_initial_swing_des_pos.origin:print("Desired r_sole middle pose ")


    -- second and final setpoint
    delta_final_swing_in_r_foot = VectorCoord.create()

    -- go ahead of half a step length
    delta_final_swing_in_r_foot.x = step_length

    -- no lateral change
    delta_final_swing_in_r_foot.y = 0.0

    -- go up of the step_height parameter
    delta_final_swing_in_r_foot.z = step_penetration

    -- rotate the delta in world orientation
    local delta_final_swing_in_world =  gas_get_transform("world",r_foot_frame).rot:apply(delta_final_swing_in_r_foot)

    -- transform the desired orientation of the frame in world
    world_r_foot_cur_pos = gas_get_transform("world",r_foot_frame)

    gas_setpoints.world_r_foot_final_swing_des_pos = world_r_foot_cur_pos:clone()

    -- save second setpoint in gas_setpoints.world_r_foot_initial_swing_des_pos
    gas_setpoints.world_r_foot_final_swing_des_pos.origin = world_r_foot_cur_pos.origin:add(delta_final_swing_in_world)
end


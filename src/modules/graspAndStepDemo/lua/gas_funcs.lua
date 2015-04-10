
----------------------------------
-- functions                   --
----------------------------------

--- Send reference point to the trajectory generator.
-- The trajectory generator is the codycoTrajGenY2 module a
-- and it accepts a property where each trajectory setpoint
-- is a yarp::sig::Vector saved with an appropriate key.
-- @param port The reference point port
-- @param ref_key the type of reference to send
--        possible values: "com", "right_leg", "left_leg",
--        possible values: "left_arm", "right_arm", "torso"
-- @param setpoint a yarp::sig::Bottle containg the actual setpoints
--
function gas_sendSetPointToTrajGen(port,ref_key,setpoint)
   local prop = port:prepare();
   prop:clear()
   prop:put(ref_name,setpoints)
   port:write()
end

--- Active contacts on the controller
-- @param port The active contacts ports
-- @param contacts a table of names of link
--        that should be considered active contacts
--
function gas_activeContacts(port,activeContacts)
   local bot = port:prepare();
   bot:clear()
   bot:addString("activateContacts")
   for i = 1,#activeContacts do
       bot:addString(activeContacts[i])
   end
   port:write()
end

--- Deactive contacts on the controller
-- @param port The active contacts ports
-- @param contacts a table of names of link
--        that should be considered not active contacts
--
function gas_deactiveContacts(port,deactiveContacts)
   local bot = port:prepare();
   bot:clear()
   bot:addString("deactivateContacts")
   for i = 1,#deactiveContacts do
       bot:addString(deactiveContacts[i])
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
    return invRot
end


function HomTransform:apply(point)
    transformedPoint = rot:apply(point);
    transformedPoint.x = transformedPoint.x + point.x;
    transformedPoint.y = transformedPoint.y + point.y;
    transformedPoint.z = transformedPoint.z + point.z;

    return transformedPoint
end

function HomTransform:compose(other)
    local composedTransform = HomTransform.create()
    composedTransform.rot = self.rot:compose(other.rot)
    composedTransform.origin = self:apply(other.origin)
end


--- Functions for loading geom class from yarp data structures
function PointCoordFromYarpVectorBottle(point, vecBot)
    if( point == nil ) then
        point = PointCoord.create()
    end

    point.x = vecBot.get(0).asDouble()
    point.y = vecBot.get(1).asDouble()
    point.z = vecBot.get(2).adDouble()
end

function YarpVectorBottleFromPointCoord(point)
    local vecBot = yarp.Bottle()

    vecBot.addDouble(point.x);
    vecBot.addDouble(point.y);
    vecBot.addDouble(point.z);

    return vecBot;
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

    return

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
        HomTransformFromYarpMatrix(homTransform, matrixBot)
        transformTable[transformName] = homTransform;
    end
end

--- This function takes a table in the format provided by HomTransformTableFromBottle
--- and it return an HomTransform firstFrame_H_secondFrame
function getTransform(transformTable, firstFrame, secondFrame)
    return transformTable[firstFrame]:inverse():compose(transformTable[secondFrame]);
end

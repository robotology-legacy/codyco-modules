
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
   bot:addString("activeContacts")
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
   bot:addString("deactiveContacts")
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


-- Homogeneous transform
HomTransform = {}
HomTransform.__index = HomTransform

function HomTransform.create()
   local transform = {}             -- our new object
   setmetatable(rot,transform)  -- make HomTransform handle lookup
   rot = RotMatrix.create();
   origin = PointCoord.create();
   return rot
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


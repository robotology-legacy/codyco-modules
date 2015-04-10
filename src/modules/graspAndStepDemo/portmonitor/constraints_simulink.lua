--
-- Copyright (C) 2015 IITRBCS
-- Authors: Silvio Traversaro
-- CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
--

-- loading lua-yarp binding library
require("yarp")

-- create global state of active constraints (for now we support just two constraints on the two feet)
activeConstraints = {}

--
-- PortMonitor table is used by portmonitor_carrier
-- to invoke the corresponding methods.The methods are
-- optional but must satisfy the following format:
--
--  PortMonitor.create = function(options) ... return true end,
--  PortMonitor.destroy = function() ... end,
--  PortMonitor.accept = function(thing) ... return true end,
--  PortMonitor.update = function(thing) ... return thing end,
--  PortMonitor.setparam = function(param) ... end,
--  PortMonitor.getparam = function() ... return param end
--  PortMonitor.trig = function() ... return end
--

--
-- create is called when the port monitor is created
-- @return Boolean
--
PortMonitor.create = function(options)
    activeConstraints = {l_foot=true, r_foot=true}
    return true;
end

--
-- accept is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Boolean
-- if false is returned, the data will be ignored
-- and update() will never be called
PortMonitor.accept = function(thing)
    if thing:asBottle() == nil then
        print("[ERROR] constraints_simulink.lua: got wrong data type (expected type Bottle)")
        return false
    end
    local cmd = thing:asBottle():get(0):asString();
    if cmd == "activateConstraints" or
       cmd == "deactivateConstraints" then
        return true
    else
        print("[ERROR] constraint_simulink.lua: unknown cmd " .. cmd)
        return false
    end
end

--
-- update is called when the port receives new data
-- @param thing The Things abstract data type
-- @return Things
PortMonitor.update = function(thing)
    if thing:asBottle() == nil then
        print("[ERROR] constraint_simulink.lua: got wrong data type (expected a type that can be casted to a Bottle)")
        return thing
    end

    bt = thing:asBottle()

    -- activate contacts
    if( bt:get(0):asString() == "activateConstraints" ) then
        for i = 1,bt:size() do
            if( bt:get(1):asString() == "l_foot" ) then
                activeConstraints.l_foot = true
            end
            if( bt:get(1):asString() == "r_foot" ) then
                activeConstraints.r_foot = true
            end
        end
    end

    -- deactivate contacts
    if( bt:get(0):asString() == "deactivateConstraints" ) then
        for i = 1,bt:size() do
            if( bt:get(1):asString() == "l_foot" ) then
                activeConstraints.l_foot = false
            end
            if( bt:get(1):asString() == "r_foot" ) then
                activeConstraints.r_foot = false
            end
        end
    end

    -- reset input bottle to a vector
    bt:clear()
    if( activeConstraints.l_foot ) then
        bt:addDouble(1.0);
    else
        bt:addDouble(0.0);
    end

    if( activeConstraints.r_foot ) then
        bt:addDouble(1.0);
    else
        bt:addDouble(0.0);
    end

    return thing
end

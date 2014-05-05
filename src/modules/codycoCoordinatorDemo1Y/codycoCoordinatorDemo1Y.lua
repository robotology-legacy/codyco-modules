#!/usr/bin/lua

require("rfsm")
require("yarp")

yarp.Network()

--action_status = 'idle'

----events 
event_no_contact = "e_no_contacts_on_hands"
event_contact_on_left_hand = "e_contacts_only_on_left_hand"
event_contact_on_right_hand = "e_contacts_only_on_right_hand"
event_no_contact = "e_contacts_on_both_hands"

-------
function update_skin_events()
   while true do
        skin_contacts = event_port:read(false)        
        if event ~= nil then 
            print('Received ', event:toString())            
            -- read skin events and produce proper evnets 
            rfsm.send_events(fsm, )
        end            
        if coroutine.yield() == true then break end   
   end
end



-------
shouldExit = false

-- initilization
cmd_action_rpc = yarp.RpcClient()
cmd_action_rpc:open("/codycoCoordinator1Y/cmd_action:o")

event_port = yarp.BufferedPortBottle()
event_port:open("/codycoCoordinator1Y/skin_events:i")

-- load state machine model and initalize it
fsm_model = rfsm.load("./fsm_codyco.lua")
fsm = rfsm.init(fsm_model)

co_updater = coroutine.create(update_events)

repeat   
    coroutine.resume(co_updater) 
    rfsm.run(fsm)
    yarp.Time_delay(0.1)    
until shouldExit ~= false

print("finishing")
coroutine.resume(co_updater, true) 

event_port:close()
cmd_action_rpc:close()

-- Deinitialize yarp network
yarp.Network_fini()


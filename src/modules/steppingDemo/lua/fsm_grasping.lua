
TorqueBalancingSetPoints = {}
TorqueBalancingSetPoints.__index = TorqueBalancingSetPoints

function TorqueBalancingSetPoints.create(balance)
   local acnt = {}             -- our new object
   setmetatable(acnt,Account)  -- make Account handle lookup
   acnt.balance = balance      -- initialize our object
   return acnt
end

function Account:withdraw(amount)
   self.balance = self.balance - amount
end
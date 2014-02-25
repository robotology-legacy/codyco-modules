function [tau,IntStatesD] = fcn(q, qD, M, h, Jc, JcDqD, qDes, qDesD, qdDD, nu, nuDes, nuDesD, m, k, IntStates)
%#codegen

PINV_TOL = 1e-5;
g        = 9.81;

n     =  size(M,1);
Mb    =  M(1:6,1:6);
MbInv = inv(Mb);
Mbj   = M(1:6,7:end); 
Mj    = M(7:end,7:end);
U     = [eye(6) , zeros(6,n)];
Jcb   = Jc*U';
SBar  = [  -MbInv*Mbj 
           eye(n)      ];
NjInv = Mj - Mbj'*MbInv*Mb;

% Gravity affects only linear motions of the CoM
grav = [0;0;m*g;zeros(3,1)];

JcSBar     = Jc*SBar;
JcSBarPinv = pinv(JcSBar, PINV_TOL);

% Definition of reference accelerations for CoM and Joints that ensure
% stabilization of desired trajectories nuDes and qDes 
nu_star = nuDesD - k(1,1)*(nu - nuDes) - k(1,2)*IntStates(n+1:end);
q_star  = qdDD   - k(2,1)*(qD - qDesD) - k(2,2)*(q - qDes)          - k(2,3)*IntStates(1:n) ;

% Updates for integral terms
IntStatesD = [(q - qDes);(nu - nuDes)];

A     = [eye(6) eye(6)];
Apinv = 0.5*A';
NA    = 0.5*[eye(6),-eye(6);-eye(6),eye(6)];


% The following expressions for Ic and IcD hold only if the control 
% objective is the stabilization of the CoM's position
Ic  = [ m*eye(3),zeros(3)
       zeros(3),zeros(3)];
IcD = zeros(6);
       
FeedCoM             = grav + IcD*nu + Ic*nu_star;
EquaLeftRifghFoot   = zeros(size(A,2),1);
fd                  = Apinv*FeedCoM + NA*EquaLeftRifghFoot;

fd(4:6)    = zeros(3,1); 
fd(10:end) = zeros(3,1);

LambdaStar = JcSBarPinv*(Jcb*MbInv*(h(1:6) - Jcb'*fd) - JcDqD);

NPi = eye(n) - JcSBarPinv*JcSBar;

LambdaStar = LambdaStar + NPi * q_star;

tau = NjInv*LambdaStar + SBar'*h - (JcSBar)'*fd;


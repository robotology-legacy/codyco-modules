%%%%INIT Function

%real parameters
m1 = 0.1;
I1 = 1;
l1 = 0.5;
a1 = 1;

m2 = 1;
I2 = 1;
l2 = 0.5;
a2 = 1;

%friction
F = 10.2 * eye(2);

In = [m1;
      I1;
      l1;
      a1;
      
      m2;
      I2;
      l2;
      a2;
      
      F(1,1);
      F(2,2)];

  
q1 = 0;
q2 = 0;
  
a = [m1; m1*(l1 - a1); I1 + m1*(l1 - a1)^2;
    m2; m2*(l2 - a2); I2 + m2*(l2 - a2)^2; 
    F(1,1); F(2,2)];

aH_0 = 2.* [45.3054;
  -22.5593;
   22.5002;
   49.7992;
  -20.9528;
   23.5987;
   51.1767;
   53.5944];

m1H = aH_0(1);
l1H = (aH_0(2) / aH_0(1)) + a1;
I1H = aH_0(3) - (aH_0(2)^2 / aH_0(1));
m2H = aH_0(4);
l2H = (aH_0(5) / aH_0(4)) + a2;
I2H = aH_0(6) - (aH_0(5)^2 / aH_0(4));
F1H = aH_0(7);
F2H = aH_0(8);

m11H = I1H + m1H * l1H^2 + I2H + m2H*(a1^2 + l2H^2 + 2*a1*l2H*cos(q2));
m12H = I2H + m2H*(l2H^2 + a1 *l2H*cos(q2));
m22H = I2H + m2H*l2H^2;

m11H = I1H + m1H * l1H^2 + I2H + m2H*(a1^2 + l2H^2 + 2*a1*l2H*cos(q2));
Mdet = abs(det(m11H));
if (Mdet <= 0.01)
   error('initial det is almost zero'); 
end

% aH_0 = a;

% aH_0 = a + (a .*(-1 + 2 * rand(8,1)));  
% aH_0 = -150 + 300*rand(6,1);
xiu_0 = 0.8;

e1 = [1; 0];
e2 = [0; 1];



dq1 = 0;
dq2 = 0;

w = 2*pi*1;
q2_ref = 0.753;
q2_ref_base = 0.753;

Lambda = 1e+1;
Gamma = 1e+1* eye(8);
Kappa = 1e+2*eye(2);
params = [ 0;  % detSlope
           1]; %epsilon for determinant

x0 = [  q1; 
        q2;
        dq1; 
        dq2; 
        aH_0; 
        xiu_0];
    
initMemory = [0];

%%%%END INIT


function [dx, out, V, regr, xi_out, refs, a_est, q, tau, ya, Mdet, update, trKMinusStuff] = fcn(t, q2_ref,In, Lambda, Kappa, Gamma, params, x)
%#codegen
% x = 
% q1
% q2
% dq1
% dq2
% 
% a (9 params)
% xi_u

%general variables
g = 9.81; %gravity
k = 1; %size of underactuated part
n = 2; %dof

%Reading input parameters
q1 = x(1);
q2 = x(2);
q = [q1; q2];
dq1 = x(3);
dq2 = x(4);
dq = [dq1; dq2];
aH = x(5:12);
xi_1 = x(13);

%real parameters
%read real parameters
m1 = In(1);
I1 = In(2);
l1 = In(3);
a1 = In(4);
m2 = In(5);
I2 = In(6);
l2 = In(7);
a2 = In(8);
F = [In(9), 0;
    0, In(10)];

% setup reference signal
w = q2_ref(3); %frequence 
qd = q2_ref(1) * sin(w * t) + q2_ref(2); %signal
dqd = q2_ref(1) * w * cos(w*t);
ddqd = q2_ref(1) * w^2 * (-sin(w*t));

%define position and velocity errors
q2_tilde = q2 - qd;
dq2_tilde = dq2 - dqd;

%define xi variable. 
xi_2  = dqd -Lambda * q2_tilde;
dxi_2 = ddqd -Lambda * dq2_tilde;

xi = [  xi_1; %to be defined by differential equation
        xi_2]; 

%extract estimated parameters
m1H = aH(1);
l1H = (aH(2) / aH(1)) + a1;
I1H = aH(3) - (aH(2)^2 / aH(1));
m2H = aH(4);
l2H = (aH(5) / aH(4)) + a2;
I2H = aH(6) - (aH(5)^2 / aH(4));
F1H = aH(7);
F2H = aH(8);

%trigonometric variables of the joint positions
c1 = cos(q1);
c2 = cos(q2); s2 = sin(q2);
c12 = cos(q1+q2);

%compute M, C and G with estimated parameters (hat variables)
m11H = I1H + m1H * l1H^2 + I2H + m2H*(a1^2 + l2H^2 + 2*a1*l2H*c2);
m12H = I2H + m2H*(l2H^2 + a1 *l2H*c2);
m22H = I2H + m2H*l2H^2;

hH = -m2H * a1* l2H*s2;
C11H = hH* dq2;
C12H = hH*(dq1+dq2);
C21H = -hH*dq1;
C22H = 0;
 
G1H = (m1H*l1H + m2H * a1)*g*c1 + m2H*l2H*g*c12;
G2H = m2H*l2H*g*c12;

%update dxi: xi_1 is updated through differential equation (sort of
%estimation of the dynamic of the non actuated joints)
dxi =  [1/m11H * (dq1 - (1 + C11H + F1H) * xi(1) - m12H * dxi_2 - C12H * xi(2) - G1H);
        dxi_2];

%define the variable s
s = dq - xi;

%compute regressor
[Y1, Y2] = regressor([q1;q2], [dq1;dq2], xi, dxi, [g; a1;a2]);

%compute real-parameter M, C, G. I do not need it in 
%control, but in the "simulation" part of the code
m11 = I1 + m1 * l1^2 + I2 + m2*(a1^2 + l2^2 + 2*a1*l2*c2);
m12 = I2 + m2*(l2^2 + a1 *l2*c2);
m22 = I2 + m2*l2^2;
M = [m11 m12; m12' m22];

h = -m2 * a1* l2*s2;
C = [h* dq2, h*(dq1+dq2);
     -h*dq1,    0];
 
G = [(m1*l1 + m2 * a1)*g*c1 + m2*l2*g*c12;
      m2*l2*g*c12];

%compute dM/dq. This term is used to render the matrix M11Hat always invertible

dMdqH = zeros(2,2,2);
dMdqH(:,:,1) = ... %dMdq_1
    [0, 0; 0, 0];
dMdqH(:,:,2) = ... %dMdq_2
    [-m2H*2*a1*l2H*s2, - m2H*a1*l2H*s2;
    (-m2H*a1*l2H*s2)',      0];

%compute feedback law for torque
tau = Y2 * aH - Kappa(2,2) * s(2);

%compute real dynamic (needed for simulation)
e1 = [ 1; 0];
e2 = [ 0; 1];

ddq = M \ (tau * e2 - C * dq - G - F*dq);

Y = [ Y1; Y2 ];

delta   = zeros(size(aH,1), 1);   % R^lx1, l = size of parameters
upsilon = zeros(k,k);           % this matrix contains the derivative of hat(M) w.r.t q times q_dot + 
                                % the derivative of hat(M) w.r.t the parameters times the "control" part (Y's)
m11Inv = inv(m11H);             % inverse of the passive (square) part of the mass matrix 

dMdq_p = dMdqH(1:k, 1:k, :);    % select the passive part of dM/dq
Mdet = abs(det(m11H));          % determinant of the passive part of the mass matrix
                                % this IF is for debug: to stop the debugger (simulink do not allow conditional breakpoints)
update = 0;
if (Mdet <= params(2))
    eppps = 1;
end

%compute delta and Upsilon matrix. They are computed "per column" 
for i = 1:k
    %define ei vector
    ei = zeros(k, 1);
    ei(i) = 1;
    %compute regressor for mass matrix Y_M(q, ei) as difference between the
    %regressor with q and qddot and regressor with only q (=> corresponds
    %to gravity term
    [Y_acc, ~] = regressor(q, zeros(2,1), zeros(2,1), [ei; zeros(1,1)], [g; a1; a2]);
    [Yg, ~] = regressor(q, zeros(2,1), zeros(2,1), zeros(2,1), [g; a1; a2]);
    Y_M = Y_acc - Yg;
    
%     dq_da = der(aH, q, [a1, a2]);
%     err = Y_M - dq_da; % <=== CHECKED---- ALWAYS ZERO!!!!
    
    %update delta. See LaTeX
    delta = delta + Y_M' * m11Inv * ei;
    
    %compute derivative of m_i w.r.t. q
    %take the i^th column of the whole mass matrix.
    %I have to reshape it because matlab otherwise will consider it as a
    %3Dimensional vector
%     dmi_dq = reshape(dMdq_p(:, i, :),k, 2) * dq;
    dmi_dq = [0, -2* m2H * a1 * l2H * s2] * dq;
    
    %upsilon is the sum of two terms: 1) the dm_dq times dq  2) the
    %derivative of m w.r.t. its parameters (=> it becomes Y_M) times the
    %(original) update law for the parameters
    upsilon(:,i) = dmi_dq - Y_M * Gamma * Y' * s;
end

% The update law for the parameters is the "old" update law minus a constant
% (eta) times delta
% in order for the determinant to increase, eta must satisfy a condition
lambda_min   = real(min(eig(Gamma))); %Gamma is symmetric >0 so eigenvalues are all reals
delta_norm2  = delta'*delta;          %square norm of delta
zeta         = trace(m11Inv * upsilon); 
etamin       = (params(1)/Mdet - zeta )/ (delta'*Gamma* delta);

%I should choose an eta greater than etamin
eta = etamin ; %0.5 * abs(etamin)

%Old update rule for parameters
daH = - Gamma * Y' * s;

%now.. I should change my update law by some tanh rule..
%waiting to implement this part.
%for now just see if using an if it works. (this does not ensure V_dot < 0)
if (Mdet <= params(2) && zeta < 0)
    daH = daH + eta * Gamma * delta;
    update = 1;
end

%Output part.
%the following variables are defined for output (real and debug/plot)

%this goes to the integrator
dx = [  dq; % => leads to next q
        ddq; % => leads to next dq
        daH; % => update the parameters
        dxi(1)]; % => update xi
       
%Now are all debug variables    
out = (180/pi)*[q1, q2, q2_tilde];%, q2_tilde, qd];

ya = Y1*aH;

a = [m1; m1*(l1 - a1); I1 + m1*(l1 - a1)^2;
    m2; m2*(l2 - a2); I2 + m2*(l2 - a2)^2;
    F(1,1); F(2,2)];


V = [0.5 * s'*M*s + 0.5 * (aH - a)' * inv(Gamma) * (aH - a);
    det(m11H)];

xi_out = [xi; dxi];

Ycheck = M*dxi + C * xi + G + F*xi - Y*a;

regr = [eta; zeta; norm(delta)];
%     eta];
%     det(m11H)^2 * (zeta + eta * delta' * Gamma * delta)];
% regr = [norm(Ycheck(1,:)); norm(Ycheck(2,:))];
refs = [qd];%, dqd, ddqd];

a_est = [I1H;
    m1H;
    l1H;
    I2H;
    m2H;
    l2H];

trKMinusStuff = update*(1 - m11/m11H);


end

function [Y1, Y2] = regressor(q, dq, dq_ext, ddq, params)
g = params(1);
a1 = params(2);
a2 = params(3);

q1 = q(1);
q2 = q(2);

dq1 = dq(1);
dq2 = dq(2);

c1 = cos(q1);
c2 = cos(q2); s2 = sin(q2);
c12 = cos(q1+q2);

dxi = ddq;
xi = dq_ext;

y11 = a1^2 * dxi(1) + a1 * g * c1;
y12 = 2*a1*dxi(1) + g*c1;
y13 = dxi(1);
% y15 = (a1^2 + 2*a1*a2*c2 + a2^2)*dxi(1) + (a1*a2*c2 + a2^2) * dxi(2) - 2*a1*a2*s2*dq2*xi(1) - a1*a2*s2*dq2*xi(2)+ a1*g*c1 + a2*g*c12;
% y16 = 2*(a1*c2 + a2)*dxi(1) + (a1*c2+ 2*a2)*dxi(2) - 2*a1*s2*dq2*xi(1) - a1*s2*dq2*xi(2) + g*c12;
y15 = (a1^2 + 2*a1*a2*c2 + a2^2)*dxi(1) + (a1*a2*c2 + a2^2) * dxi(2) - a2*a1*s2*dq2*xi(1) - a1*a2*s2*(dq1 + dq2)*xi(2)+ a1*g*c1 + a2*g*c12;
y16 = 2*(a1*c2 + a2)*dxi(1) + (a1*c2+ 2*a2)*dxi(2) - a1*s2*dq2*xi(1) - a1*s2*(dq1 + dq2)*xi(2) + g*c12;
y17 = dxi(1) + dxi(2);
y18 = xi(1);
y21 = 0; y22 = 0; y23 = 0;
y25 = (a1*a2*c2 + a2^2)*dxi(1) + a2^2 * dxi(2) + a1*a2*s2*dq1*xi(1) + a2*g*c12;
y26 = (a1*c2 + 2*a2)*dxi(1) + 2*a2*dxi(2) + a1*s2*dq1*xi(1) + g*c12;
y27 = y17;
y28 = xi(2);

Y1 = [y11, y12, y13, y15, y16, y17, y18, 0];
Y2 = [y21, y22, y23, y25, y26, y27, 0, y28];
end

function dm_dp = der(aH, q, params)
    %%
% % m1H = aH(1);
% % l1H = (aH(2) / aH(1)) + a1;
% % I1H = aH(3) - (aH(2)^2 / aH(1));
% % m2H = aH(4);
% % l2H = (aH(5) / aH(4)) + a2;
% % I2H = aH(6) - (aH(5)^2 / aH(4));
% % F1H = aH(7);
% % F2H = aH(8);
    %%
    a1 = params(1);
    a2 = params(2);
    
    c2 = cos(q(2));
    dm_dp = [(aH(2)/aH(1))^2 - (2*aH(2)*(a1 + aH(2)/aH(1)))/aH(1) + (a1 + aH(2)/aH(1))^2;
             -((2 * aH(2))/aH(1)) + 2 *(a1 + aH(2)/aH(1));
             1;
            a1^2 + aH(5)^2/aH(4)^2 + 2* a1* c2* (a2 + aH(5)/aH(4)) + (a2 + aH(5)/aH(4))^2 + ...
 aH(4) *(-((2* a1* c2* aH(5))/aH(4)^2) - (2* aH(5)* (a2 + aH(5)/aH(4)))/aH(4)^2);
            -((2* aH(5))/aH(4)) + aH(4)* ((2 *a1* c2)/aH(4) + (2 *(a2 + aH(5)/aH(4)))/aH(4));
            1;
            0;
            0];
        
    dm_dp = dm_dp';
    
end

function desiredForceComputation(block)
%define quadratic and linear term in eq.
% 1/2 x' * H * x + f' x                                                 (1)

%Our problem is:
%force = argmin_force [ 1/2 H_dot(force) - H_dot^*] ^ 2                 (2)
%H_dot = A(q) * force - mg e3                                           (3)
%
%Rewriting the problem into a quadratic form of 'force' we obtain:
%1/2 force' * A(q)'*A(q) * force - (mg e3 + H_dot^*)' * force           (4)


%%
%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.
%%
setup(block);

%endfunction

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports
% xDDcomStar = block.InputPort(1).Data; %size 3
% m = block.InputPort(2).Data; %size 1
% x_rf = block.InputPort(3).Data; %size 3
% xcom = block.InputPort(4).Data; %size 3
% H = block.InputPort(5).Data; %size 3: momentum
% kw = block.InputPort(6).Data; %size 1: gain
block.NumInputPorts  = 6; %desired com acceleration
block.NumOutputPorts = 1; %desired forces

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% % Override input port properties
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = true;
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = true;
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = true;
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = true;
block.InputPort(5).Dimensions        = 3;
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = true;
block.InputPort(6).Dimensions        = 1;
block.InputPort(6).DatatypeID  = 0;  % double
block.InputPort(6).Complexity  = 'Real';
block.InputPort(6).DirectFeedthrough = true;

% Override output port properties
block.OutputPort(1).Dimensions       = 12;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';

% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [-1 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

% block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
% block.RegBlockMethod('InitializeConditions', @InitializeConditions);
% block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
% block.RegBlockMethod('Update', @Update);
% block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
% function DoPostPropSetup(block)
% block.NumDworks = 1;
%   
%   block.Dwork(1).Name            = 'x1';
%   block.Dwork(1).Dimensions      = 1;
%   block.Dwork(1).DatatypeID      = 0;      % double
%   block.Dwork(1).Complexity      = 'Real'; % real
%   block.Dwork(1).UsedAsDiscState = true;


%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C-MEX counterpart: mdlInitializeConditions
%%
% function InitializeConditions(block)

%end InitializeConditions


%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
% function Start(block)
% 
% block.Dwork(1).Data = 0;

%endfunction

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

%read input port
xDDcomStar = block.InputPort(1).Data; %size 3
m = block.InputPort(2).Data; %size 1
x_rf = block.InputPort(3).Data; %size 3
xcom = block.InputPort(4).Data; %size 3
H = block.InputPort(5).Data; %size 3: momentum
kw = block.InputPort(6).Data; %size 1: gain


%these three rows can be saved and performed only once.
g = 9.81;
grav = [zeros(2,1); -m*g;zeros(3,1)];


% Definition of reference accelerations for CoM and Joints that ensure
% stabilization of desired trajectories xDcomDes and qDes 

% Pr =  x_rf(1:3) - xcom; % Application point of the contact force on the right foot w.r.t. CoM
Pr =  x_rf - xcom; % Application point of the contact force on the right foot w.r.t. CoM
A  = [ eye(3),   zeros(3),eye(3), zeros(3);
      -Sf(xcom),  eye(3), Sf(Pr), eye(3) ];

% The following expressions for Ic and IcD hold only if the control 
% objective is the stabilization of the CoM's position
% kw = 0.1;
hDotDes = [ m*xDDcomStar ;
            -kw*H  ]; %hwDotDes  ]; 
        
linearTerm = A' * (-hDotDes + grav);
quadraticTerm = A' * A;

opts = optimset('Algorithm','active-set','Display','off');

% lb = -Inf * ones(12,1);
% lb(3) = 0;
% lb(9) = 0;
lb = [];

% [optForces, objVal, exitFlag, output, lambda] = ...
optForces = ...
quadprog(quadraticTerm, linearTerm, ...
          [], [], ... %inequalities
          [], [], ... %equalities
          lb, [], ... %bounds
          [],     ... %initial solution
          opts);
        
block.OutputPort(1).Data = optForces;

%end Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
% function Update(block)
% 
% block.Dwork(1).Data = block.InputPort(1).Data;

%end Update

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlDerivatives
%%
% function Derivatives(block)

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate


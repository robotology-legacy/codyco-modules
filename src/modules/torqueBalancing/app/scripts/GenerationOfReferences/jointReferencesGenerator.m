function [ x, normError ] = jointReferencesGenerator(robotModel, qDes, comDes, feetInSupport, limits )
%JOINTREFERENCESGENERATOR Generates joint references as close as possible
%to the input qDes variable satisfying the task constraints
%   

% assert(size(qDes, 1) == robotModel.getNrOfDegreesOfFreedom(), 'DoFs of the model do not match the size of desired joint references');

qMin = [];
qMax = [];
if exist('limits', 'var')
    %check sizes
    dofs = size(qDes);
    limitsSize = size(limits);
    assert(limitsSize(1) == 2 && limitsSize(2) == dofs, 'Please specify limits in a #Dofs x 2 array, i.e. [q(1)_min, q(1)_max; q(2)_min, q(2)_max; ... ; q(Dofs)_min, q(dofs)_max');
    qMin = limits(:, 1);
    qMax = limits(:, 2);
end

%assume the base coincides with either left or right foot
if feetInSupport(1) == 1
    robotModel.setFloatingBase('l_sole');
elseif feetInSupport(2) == 1
    robotModel.setFloatingBase('r_sole');
else
    error('At least one of the two feet must support the robot');
end

%options for the optimization
% see: http://it.mathworks.com/help/optim/ug/optimization-options-reference.html
% and: http://it.mathworks.com/help/optim/ug/optimization-options-reference.html#bu1n1r9-1
% for compatibility in 2015b
options = optimoptions(@fmincon, ...
    'Algorithm', 'interior-point', ...
    'GradObj', 'on', ...
    'GradConstr', 'on', ...
    'DerivativeCheck', 'on');

[x,normError,exitflag,output,lambda,grad,hessian] = fmincon(@(q) objective(q, qDes) , qDes, ...
    [],[],[],[], ...
    qMin,qMax, @(q) nonlinearConstraints(q, robotModel, comDes, feetInSupport), ...
    options);

end

function [fval, gradient, hessian] = objective(q, qDes)
    error = q - qDes;
    fval = 0.5 .* error' * error;
    
    if nargout > 1
        gradient = error;
    end
    
    if nargout > 2
        %note: hessian can be the hessian of the lagrangian. To check this
        %part
        hessian = eye(size(q));
    end
    
end

function [c_ineq, c_eq, J_ineq, J_eq] = nonlinearConstraints(q, robotModel, comDes, feetInSupport, feetDistance)

    c_ineq = [];
    c_eq = [];
    qDot = zeros(size(q));
    gravity = [0;0;-9.81];
    % first constraint: forward kinematics of CoM = desired CoM.
    robotModel.setRobotState(q, qDot, qDot, gravity);
    %call to forward kinematic
    
    % second constraint: relative tranformation between the two feet
    % remains constant (if both feet are in contact with the ground)
    if feetInSupport(1) == 1 && feetInSupport(2) == 1
        % call frame between left and right foot
        left_X_right = robotModel.getRelativeTransform('l_sole', 'r_sole');
        % get position
        left_P_right = left_X_right.getPosition();
         c_eq = [c_eq;
            left_P_right.toMatlab() - feetDistance];
    end
    
    % get orientation
    
    if nargout > 2
        %first constraint: Jacobian of CoM
        J_ineq = [];
        J_eq = [];
        
        if feetInSupport(1) == 1 && feetInSupport(2) == 1
            % get position
            robotModel.getRelativeFrameJacobian('l_sole', 'r_sole');
            
        end
    end

end

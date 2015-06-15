function [ theta ] = computeAngleBetweenVectors( v1,v2 )
%COMPUTEANGLEBETWEENVECTORS arcsin of the cross product
%   Detailed explanation goes here

%    theta = acos(dot(v1',v2')'./...
%        (rowWiseNorm(v1).*rowWiseNorm(v2)));

    
    %theta = asin(rowWiseNorm(cross(v1,v2))./...
     %   (rowWiseNorm(v1).*rowWiseNorm(v2)));

    theta = atan2(norm(cross(v1,v2)),dot(v1',v2')');
end


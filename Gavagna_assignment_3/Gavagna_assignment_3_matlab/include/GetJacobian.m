function J = GetJacobian(bTi, jointType, numberOfLinks)
% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix
J = zeros(6,numberOfLinks);

for i = 1:numberOfLinks
    if (jointType == 0) % revolute
        J(1:3,i) = bTi(1:3,4,i);
        distWrtBase = GetFrameWrtFrame(i, 7, bTi);
        J(4:6,i) = cross(bTi(1:3,4,i), distWrtBase(1:3,4));
    
    elseif (jointType == 1) % prismatic
        J(1:3,i) = [0; 0; 0;];
        distWrtBase = GetFrameWrtFrame(i, 7, bTi);
        J(4:6,i) = distWrtBase(1:3,4);
    end
end
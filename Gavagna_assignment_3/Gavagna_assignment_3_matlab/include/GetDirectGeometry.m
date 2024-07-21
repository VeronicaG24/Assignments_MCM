function [iTj_q] = GetDirectGeometry(q, biTri, jointType, numberOfLinks)
%%% GetDirectGeometryFunction

% Inputs: 
% q : links current position ; 
% iTj : vector of matrices containing the transformation matrices from link
% i to link j
% jointType: vector of size numberOfLinks identiying the joint type, 0 for revolute, 1 for
% prismatic.

% Outputs :
% iTj_q vector of matrices containing the transformation matrices from link i 
% to link j for the input q. 
% The size of iTj_q is equal to (4,4,numberOfLinks)

iTj_q = zeros(4,4,numberOfLinks);

for i = 1:1:numberOfLinks
    iTj_q(:,:,i) = DirectGeometry(q(i), biTri(:,:,i), jointType(i));
end

end
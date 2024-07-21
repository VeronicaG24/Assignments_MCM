function R = ComputeAngleAxis(theta, v)
% Given theta' (angle), 'v' (axis vector), this function
% outputs the rotation matrix 'R' using the Rodrigues formula

% Compute the skew symmetric matrix of the v' (axis vector)
v_skew = skewDim3(v); 

% Compute the rotation matrix 'R' using the Rodrigues formula
R = eye(3) + v_skew*sin(theta) + ((v_skew)^2)*(1-cos(theta));

end
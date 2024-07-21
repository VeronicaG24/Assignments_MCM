function [theta,v] = ComputeInverseAngleAxis(R)
% EULER REPRESENTATION: Given a tensor rotation matrices this function
% outputs the equivalent angle-axis representation values,
% respectively 'theta' (angle), 'v' (axis vector)

% Generate different errors if the input matrix is not 3x3, not orthogonal 
% or the determinant is not 1
    if (size(R,1) ~= 3) || (size(R,2) ~= 3)
        msg = 'Error occurred: WRONG SIZE OF THE INPUT MATRIX';
        error(msg)
    elseif round((R * R'), 5) ~= eye(3) % round at the 5th place
        msg = 'Error occurred: NOT ORTHOGONAL INPUT MATRIX';
        error(msg)
    elseif round(det(R), 5) ~= 1 % round at the 5th place
        msg = 'Error occurred: DETERMINANT OF THE INPUT MATRIX IS 0';
        error(msg)
    else 
        % Compute 'theta' (angle) and 'v' (axis vector)
        theta = acos((trace(R) - 1) / 2);
        v = (1/(sin(theta))) * vex3(R);
        % Compute eigenvectors and eigenvalues (NOT NEEDED)
        [eigenvectors, D] = eig(R); disp("eigenvectors:"); disp(eigenvectors);
        eigenvalues = diag(D); disp("eigenvalues:"); disp(eigenvalues);
    end

end


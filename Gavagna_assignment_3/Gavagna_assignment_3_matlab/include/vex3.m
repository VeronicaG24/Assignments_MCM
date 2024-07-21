function [vexMatrix] = vex3(R)
% vex Compute the vex of a matrix 3x3

% Generate an error if the input matrix is not 3x3
if (size(R,1) ~= 3) || (size(R,2) ~= 3)
        msg = 'Error occurred: WRONG SIZE OF THE INPUT MATRIX';
        error(msg)
else
    % Compute the vex of the input matrix
    vexMatrix = 1/2 * ([(R(3,2) - R(2,3));
                        (R(1,3) - R(3,1));
                        (R(2,1) - R(1,2));]);

end
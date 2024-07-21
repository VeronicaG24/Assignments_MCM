function skewMatrix = skewDim3(x)
% skewDim3 Generates skew symmetric matrix from a vector of dim=3

% Generate an error if the input vector is not 3
if size(x) ~= 3
        msg = 'Error occurred: WRONG SIZE OF THE INPUT MATRIX';
        error(msg)
else 
    % Compute the skew symmetric matrix
    skewMatrix = [    0, -x(3),   x(2); 
                   x(3),     0,  -x(1); 
                  -x(2),  x(1),     0];
end
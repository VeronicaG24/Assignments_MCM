function [theta,v] = ComputeInverseVersorLemma(r1, r2)

% R_1 rotation matrix goal frame wrt base
% R_2 rotation matrix tool frame wrt base
% NB: Enter a square, 3x3 proper-orthogonal matrix to calculate its angle
% and axis of rotation. Error messages must be displayed if the matrix
% does not satisfy the rotation matrix criteria.
% Check matrix R to see if its size is 3x3

% Define a tollerance
tol = 1e-04;

if size(r1,1) ~= 3 || size(r1,2) ~= 3
    msg = 'Error occurred: WRONG SIZE OF THE INPUT MATRIX R1';
    error(msg)
elseif size(r2,1) ~= 3 || size(r2,2) ~= 3
    msg = 'Error occurred: WRONG SIZE OF THE INPUT MATRIX R2';
    error(msg)
elseif round((r1 * r1'), 4) ~= eye(3) 
    % round at the 4th place
    msg = 'Error occurred: NOT ORTHOGONAL INPUT MATRIX R1';
    error(msg)
elseif round((r2 * r2'), 4) ~= eye(3)
    % round at the 4th place
    msg = 'Error occurred: NOT ORTHOGONAL INPUT MATRIX R2';
    error(msg)
elseif round(det(r1), 4) ~= 1  || round(det(r2), 4) ~= 1
    % round at the 4th place
    msg = 'Error occurred: DETERMINANT OF THE INPUT MATRIX IS 0';
    error(msg)

else
    % Compute the angle of rotation
    theta1=acos((trace(r1)-1)/2);
    theta2=acos((trace(r2)-1)/2);
    theta=theta1-theta2;
    
    if cos(theta2) >= 1-tol || cos(theta) >= 1-tol || cos(theta1) >= 1-tol
        % Case when cos(theta) is almost 1; theta is almost 0
        v=[0;0;0];
    elseif abs(cos(theta2)) < 1-tol || abs(cos(theta)) < 1-tol ||  abs(cos(theta1)) < 1-tol
        % Case when abs(cos(theta)) smaller then 1 (and grater than 0)
        
        % Calculate v vector, with axial vector
        R = r2'*r1;
        v=(1/sin(theta))*vex3(R);
    else
        % Case when cos(theta) is almost -1; theta is almost pi (by exclusion)
        
        x_column=r1(:,1)+r2(:,1);
        if (not(x_column - tol == [0;0;0]))
            v=x_column;
        end
    
        y_column=r1(:,2)+r2(:,2);
        if (not(y_column - tol == [0;0;0]))
            v=y_column;
        end
    
        z_column=r1(:,3)+r2(:,3);
        if (not(z_column - tol == [0;0;0]))
            v=z_column;
        end

    end
end
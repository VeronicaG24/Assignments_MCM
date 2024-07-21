function [q] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

% Updating q
q_new = q + q_dot*ts;

if (q_new <= q_min) %check if q_new is less then q_min
    q = q_min;
elseif (q_new >= q_max) %check if q_new is greater then q_min
    q = q_max;
else
    q = q_new;
end 

end
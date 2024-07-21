%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat"); % don't worry about the warnings
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Initial transformation from <base> to <e-e>
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7'); %useful for save initial end-effector orientation w.r.t robot base

% END-EFFECTOR Goal definition 
bOge = [0.6; 0.4; 0.4];
eRge = ComputeAngleAxis(-pi/4, [0,0,1]);
bRe = bTe(1:3,1:3);
bRge = bRe * eRge;
bTge = zeros(4,4);
bTge(1:3,1:3) = bRge;
bTge(1:3,4) = bOge;
bTge(4,4) = 1;

% TOOL Goal definition
bOgt = [0.6; 0.4; 0.4];
eTt = [eye(3) [0,0,0.2]'; 0 0 0 1];
bTt = bTe * eTt;
bRt = bTt(1:3, 1:3);
tRgt = ComputeAngleAxis(-(pi/4), [0,0,1]);
bRgt = bRt * tRgt;

bTgt = zeros(4,4);
bTgt(1:3,1:3) = bRgt;
bTgt(1:3,4) = bOgt;
bTgt(4,4) = 1;

% TOOL Goal 2
bOgt2 = [ 0.6; 0.4; 0.4;];
bRgt2 = [ 0.9986, -0.0412, -0.0335;
          0.0329, -0.0163, 0.9993;
          -0.0417, -0.9990, -0.0149;];
bTgt2 = zeros(4,4);
bTgt2(1:3,1:3) = bRgt2;
bTgt2(1:3,4) = bOgt2;
bTgt2(4,4) = 1;

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
err_linear = zeros(3,1);
err_angular = zeros(3,1);

% Start the inverse kinematic control 
tool = true; % change to true for using the tool
tool2 = true; % change to true for using the tool2
q = q_init;
for i = t
    %% Compute the cartesian error to reach the goal
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        
        if tool2 == false %tool 1
            % update bTt
            bTt = bTe * eTt;
            bRt = bTt(1:3, 1:3);
            
            % compute bJt (named bJe for not modifying the rest of the program)
            ert = eTt(1:3,4);
            bRe = bTe(1:3,1:3);
            skew_eJt = bRe * ert;
            % static jacobian of the tool w.r.t. the EE
            eSt = [eye(3), zeros(3,3);
                   (skewDim3(skew_eJt))', eye(3);];
            bJe = eSt * bJe;

            %error angular
            [theta, h] = ComputeInverseVersorLemma(bRgt, bRt);
            err_angular = bRt * (h * theta);
            %error linear
            err_linear = bTgt(1:3,4) - bTt(1:3,4);
            %Cartesian error
            cart_err = [err_angular; err_linear];
        
        elseif tool2 == true % tool 2
            % update bTt
            bTt = bTe * eTt;
            bRt = bTt(1:3, 1:3);
            
            % compute bJt (named bJe for not modifying the rest of the program)
            ert = eTt(1:3,4);
            bRe = bTe(1:3,1:3);
            skew_eJt = bRe * ert;
            % static jacobian of the tool w.r.t. the EE
            eSt = [eye(3), zeros(3,3);
                   (skewDim3(skew_eJt))', eye(3);];
            bJe = eSt * bJe;

            %error angular
            [theta, h] = ComputeInverseVersorLemma(bRgt2, bRt);
            err_angular = bRt * (h * theta);
            %error linear
            err_linear = bTgt2(1:3,4) - bTt(1:3,4);
            %Cartesian error
            cart_err = [err_angular; err_linear];
        end
        
    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT
        
        %update bRe
        bRe = bTe(1:3,1:3);

        %error angular
        [theta, h] = ComputeInverseVersorLemma(bRge, bRe);
        err_angular = bRe * (h * theta);
        %error linear
        err_linear = bTge(1:3,4) - bTe(1:3,4);
        %Cartesian error
        cart_err = [err_angular; err_linear];
    end
    
       
    %% Compute the reference velocities
    %gain given from the text
    gain = 0.2;

    % angular velocity
    ang_vel = err_angular * gain;
    %linear velocity
    lin_vel = err_linear * gain;
    %compute x_dot
    x_dot = [ang_vel; lin_vel];
    
    %another way to compute x_dot
    lambda_matrix = eye(6) * gain;
    ni_e = lambda_matrix * cart_err;

    %% Compute desired joint velocities
    q_dot = pinv(bJe) * x_dot;
    
    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot, ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    show(model.franka,[q',0,0],'visuals','on');%switch visuals to off for seeing only the frames
    hold on
    if tool == true
        plot3(bTt(1,4),bTt(2,4),bTt(3,4),'go','LineWidth',15);
        plot3(bOgt(1),bOgt(2),bOgt(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOge(1),bOge(2),bOge(3),'ro','LineWidth',5);
    end
    drawnow
    hold off
    if(norm(x_dot) < 0.01)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
end

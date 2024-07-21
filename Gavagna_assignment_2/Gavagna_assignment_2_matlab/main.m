%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');


%% 1.
% You will need to define all the model matrices, and fill them so called iTj 
% matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model 
% you received.
geom_model = BuildTree();

% Useful initizializations
numberOfLinks = 7;             % number of manipulator's links.
jointType =  [0,0,0,0,0,0,0];  % boolean that specifies two possible joint types: Rotational (0), Prismatic (1).
bri= zeros(3,numberOfLinks);   % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base
iTj = zeros(4,4,1);
% Initial joint configuration 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];

% Q1.1 and Q1.2
biTei = GetDirectGeometry(q, geom_model, jointType, numberOfLinks);

%Q1.3 
for i = 1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei, i);
end

for i = 1:(numberOfLinks) % all the (i,j T) = inv(0,i T) * (0,j T)
        for j = 1:(numberOfLinks)
            iTj(:,:,i,j) = GetFrameWrtFrame(i, j, bTi); 
        end
end

for i = 1:numberOfLinks %bri 3x7
    bri(:,i) = GetBasicVectorWrtBase(bTi, i);
end

% Q1.4 and Q1.5
% Hint: use plot3() and line() matlab functions. 

%initialize ibnitial and final configurations
qi_ex = [ 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3; %all
          1.3, 0.0, 1.3, 1.7, 1.3, 0.8, 1.3; %first one
          1.3, 0.1, 0.1, 1.0, 0.2, 0.3, 1.3; %all
          0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5; %second one
          0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5; %fourth one
          0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5; %sixth one
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; %second one
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; %fourth one
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;]; %sixth one

qf_ex = [ 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          2.0, 0.0, 1.3, 1.7, 1.3, 0.8, 1.3;
          2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;
          0.5, 2.5, 0.5, 0.5, 0.5, 0.5, 0.5;
          0.5, 0.5, 0.5, 1.8, 0.5, 0.5, 0.5;
          0.5, 0.5, 0.5, 0.5, 0.5, 3.2, 0.5;
          0.0, 2.5, 0.0, 0.0, 0.0, 0.0, 0.0;
          0.0, 0.0, 0.0, 1.8, 0.0, 0.0, 0.0;
          0.0, 0.0, 0.0, 0.0, 0.0, 3.2, 0.0;]; 

numberOfSteps = 20;
numberOfPositions = size(qi_ex,1);

%initialize varibles
bTi_step = zeros(4,4,numberOfLinks);
bri_step = zeros(3,numberOfLinks);
bTi_start = zeros(4,4,numberOfLinks);
bri_start = zeros(3,numberOfLinks);
step = zeros(6,numberOfLinks);

%plot3 to plot joints
%line to plot links
for c = 1:numberOfPositions %for each configuration initialized before
    qi = qi_ex(c, :); %get qi
    qf = qf_ex(c, :); %get qf
    step(c,:) = (qf-qi)/numberOfSteps; %compute the step
    qs = zeros(1,numberOfLinks);
    figure();
    plot3(0,0,0, "*", 'Color', 'b'); %plot * for the <0> frame
    hold("on");
    %-------------------MOVE----------------------%
    for i = 0:numberOfSteps %for each step
        qs = qi + (step(c,:)*i); %add step to qi
        
        for j = 1:numberOfLinks %for every link
            %initial configuration
            %compute geometric model, transformation matrix wrt base and basic vector
            biTei_start = GetDirectGeometry(qi, geom_model, jointType, numberOfLinks);
            bTi_start(:,:,j)= GetTransformationWrtBase(biTei_start, j);
            bri_start(:,j) = GetBasicVectorWrtBase(bTi_start, j);
            %plot joint of the initial configuration
            plot3(bri_start(1,j),bri_start(2,j),bri_start(3,j), '.');
            
            %step configuration
            %compute geometric model, transformation matrix wrt base and basic vector
            biTei_step = GetDirectGeometry(qs, geom_model, jointType, numberOfLinks);
            bTi_step(:,:,j)= GetTransformationWrtBase(biTei_step, j);
            bri_step(:,j) = GetBasicVectorWrtBase(bTi_step, j);
            %plot joints of the i-th configuration
            plot3(bri_step(1,j),bri_step(2,j),bri_step(3,j), '.');
            hold("on");
    
        end
        
        %plot link of each step configuration
        line([bri_step(1,1), 0], [bri_step(2,1), 0], [bri_step(3,1), 0], 'Color', 'm');
        line(bri_step(1,:), bri_step(2,:), bri_step(3,:), 'Color', 'm');
        
        %plot link of the inizial configuration in blue
        line([bri_start(1,1), 0], [bri_start(2,1), 0], [bri_start(3,1), 0], 'Color', 'b');
        line(bri_start(1,:), bri_start(2,:), bri_start(3,:), 'Color', 'b');

        hold("on");
    end
    %show transformation matrices wrt base of each exercise
    disp("Exercise:");disp(c);
    disp("INITIAL");
    disp(bTi_start);
    disp("FINAL");
    disp(bTi_step);
    pause();
end
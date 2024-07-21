%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix

% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks); % Trasformation matrix i-th link w.r.t. base
jointType = [0,0,0,0,0,0,0]; % type of the joints (RJ = 0, PJ = 1)

% Initial joint configuration 
q = [1.3,1.3,1.3,1.3,1.3,1.3,1.3];
% Compute direct geometry
biTei = GetDirectGeometry(q, geom_model, jointType, numberOfLinks);

% Compute the transformation w.r.t. the base
for i = 1:numberOfLinks
    bTi(:,:,i)= GetTransformationWrtBase(biTei, i);
end

% q given from the text
q_ex = [ 1.3, 1.3, 1.3, 1.3, 1.3, 1.3, 1.3;
         1.3, 0.4, 0.1, 0.0, 0.5, 1.1, 0.0;
         1.3, 0.1, 0.1, 1.0, 0.2, 1.1, 0.0;
         2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0;];

numberOfConfiguration = size(q_ex,1);
% initialize matrices
biTei_ex = zeros(4,4,numberOfLinks,numberOfConfiguration);
bTe_ex = zeros(4,4,numberOfConfiguration);
bTi_ex = zeros(4,4,numberOfLinks,numberOfConfiguration);
J_ex = zeros(6,numberOfLinks,numberOfConfiguration);

for i=1:numberOfConfiguration
    % Compute bTe transformation w.r.t. the base of the end-effector
    bTe_ex(:,:,i) = GetTransformationWrtBase(biTei_ex(:,:,:,i), numberOfLinks);
    
    % compute the transformation matrices of the i-th link w.r.t. the base
    biTei_ex(:,:,:,i) = GetDirectGeometry(q_ex(i,:), geom_model, jointType, numberOfLinks);
    for j = 1:numberOfLinks
        bTi_ex(:,:,j,i)= GetTransformationWrtBase(biTei_ex(:,:,:,i), j);
    end
    % computing jacobian matrix of each configuration
    J_ex(:,:,i) = GetJacobian(bTi_ex(:,:,:,i), jointType, numberOfLinks);
end


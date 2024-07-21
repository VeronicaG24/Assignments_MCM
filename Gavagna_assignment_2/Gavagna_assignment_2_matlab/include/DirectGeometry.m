function iTj_q = DirectGeometry(qi, iTj, linkType)
% DirectGeometry Function 
% inputs: 
% qi : current link position;
% iTj is the constant transformation between the base of the link <i>
% and its end-effector; 
% linkType :0 for revolute, 1 for prismatic

% output :
% iTj_q : transformation between the base of the joint <i> and its end-effector taking 
% into account the actual rotation/traslation of the joint
iTj_q = zeros(4,4);

if linkType == 0 % rotational

    Rz_q = [ cos(qi),    -sin(qi),   0;
             sin(qi),     cos(qi),   0;
                   0,          0,    1;];

    iTj_q(1:3, 1:3) = iTj(1:3, 1:3)*Rz_q;
    iTj_q(:, 4) = iTj(:,4);

elseif linkType == 1 % prismatic
    %k*qi dove k = [0;0;1;]
    r_q = [ 0;
            0;
            qi;];
    
    %first row
    iTj_q(1,1) = iTj(1,1);
    iTj_q(1,2) = iTj(1,2);
    iTj_q(1,3) = iTj(1,3);
    iTj_q(1,4) = iTj(1,4) + r_q(1);
    
    %second row
    iTj_q(2,1) = iTj(2,1); 
    iTj_q(2,2) = iTj(2,2); 
    iTj_q(2,3) = iTj(2,3); 
    iTj_q(2,4) = iTj(2,4) + r_q(2);
    
    %third row
    iTj_q(3,1) = iTj(3,1); 
    iTj_q(3,2) = iTj(3,2); 
    iTj_q(3,3) = iTj(3,3);  
    iTj_q(3,4) = iTj(3,4) + r_q(3);

    %fourth row
    iTj_q(4,1) = iTj(4,1);  
    iTj_q(4,2) = iTj(4,2);  
    iTj_q(4,3) = iTj(4,3);  
    iTj_q(4,4) = iTj(4,4); 
    
end

end
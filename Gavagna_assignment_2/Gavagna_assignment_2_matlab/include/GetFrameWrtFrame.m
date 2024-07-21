function [iTj]=GetFrameWrtFrame(linkNumber_i, linkNumber_j, bTi)
%%% GetFrameWrtFrame function 
% inputs : 
% linkNumber_i : number of ith link 
% linkNumber_j: number of jth link 
% biTei vector of matrices containing the transformation matrices 
% from link i to link i +1 for the current q.
% The size of biTri is equal to (4,4,numberOfLinks)
% outputs:
% iTj : transformationMatrix in between link i and link j for the
% configuration described in biTei

if(linkNumber_i == 0) %if i=0: iTj is equal to bTj
    iTj = bTi(:,:,linkNumber_j);
elseif (linkNumber_j == 0) %if j=0: iTj is equal to the inverse of bTi
    iTj = inv(bTi(:,:,linkNumber_i));
else %if i and j are not 0: iTj = inverse of bTi * bTj
    iTj = (inv(bTi(:,:,linkNumber_i)))*bTi(:,:,linkNumber_j);
end
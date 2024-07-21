function [r]=GetBasicVectorWrtBase(bTi, linkNumber)
%%% GetBasicVectorWrtBase function 
% input :
% iTj trasnformation matrix in between i and j 
% output
% r : basic vector from i to j

r = bTi(1:3, 4, linkNumber);

end
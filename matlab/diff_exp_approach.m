function diff = diff_exp_approach(a,X,Y)
% Finds the error between the exponential approach function with given 
% coefficients and the passed data sets.

diff = a(1)*(1-exp(-a(2)*X)) - Y;

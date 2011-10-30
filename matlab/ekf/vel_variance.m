
% plant uncertainty for velocity.

% Indices in data
TIME = 1;
IN = 2;
OUT = 3;

% Velocity model:
load('vel_coeffs.mat');
% Z transform numerator and denominator coefficients:
zn = (Km*am*ts)/(am*ts + 2)*[1 1];
zd = cat(2, [1 (am*ts-2)/(am*ts+2)], zeros(1, delay));

% Will store error variances here:
vars = zeros(1,5);

% Use frequency response for w = 1 through 5 (model starts to fail
% horribly at high frequency, unsuprisingly - we should avoid abrupt
% changes!
for i = 1:5
  % Read response data:
  infile = sprintf('../velocity/freq_resp/sin_w%d.0.dat', i);
  data = dlmread(infile);
  n = length(data(:,1)); % sample count
  
  % Simulation requires 4 sample delay on input.  So, start
  % sim at sample 5, so we can use 'real' initial conditions.
  to = 6;
  
  vsim = zeros(1,n);
  % Start sim off with true state as initial condition.
  vsim(to-1) = data(to-1,OUT);
  
  % Simulate response:
  for k = to:n
      vsim(k) = zn(1)*data(k-4, IN) + zn(2)*data(k-3, IN) - zd(2)*vsim(k-1);
  end

  e = vsim - reshape(data(:,OUT), size(vsim)); % It's shit like this, Matlab...
  vars(i) = var(e);
end


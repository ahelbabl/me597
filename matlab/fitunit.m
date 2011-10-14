% The velocity step response of the robot should be of the form
%
% v(t) = K(1-exp(-at))
%
% The purpose of this script is to find the best fit of this function to
% the step response data recorded in step_response.dat

clear

data = dlmread('step_response.dat');
len = size(data);
len = len(1);

% helpful index names
TIME = 1;
IN = 2;
OUT = 3;

% Data is a series of positive and negative steps, separated by zeros.
% Find the extents of each of the steps.
stepstarts = zeros(0);
stepends = zeros(0);
for i = 2:len
	% Is this the beginning of a step?
	if (data(i-1,IN)==0 && data(i,IN)~=0)
		stepstarts = cat(1,stepstarts,i);
	end

	% Is this the end of a step?
	if (data(i-1,IN)~=0 && data(i,IN)==0)
		stepends = cat(1,stepends,i-1);
	end
end
% If we don't have an end for the last step, cut it (may not be a full step)
if length(stepends) ~= length(stepstarts)
	stepstarts = stepstarts(1:length(stepends));
end
steps = length(stepstarts);

% Find the shortest step
shortest = min(stepends - stepstarts);


% Split data into individual steps.
split = zeros(steps,shortest,3);
for i = 1:steps
	split(i,:,:) = data(stepstarts(i):stepstarts(i)+shortest-1,:);
	% Flip the negative steps:
	if split(i,1,IN) < 0
		split(i,:,IN) = -split(i,:,IN);
		split(i,:,OUT) = -split(i,:,OUT);
	end
end

% Calculate the average step response delay
delays = zeros(steps,1);
for i = 1:steps
	for j = 1:shortest
		if abs(split(i,j,OUT)) > 0.01
			delays(i) = j;
			break
		end
	end	
end
delay = floor(mean(delays))

% Remove the delays
split = split(:,4:end,:);
% Set the beginning of each step to t=0
for i = 1:steps
	split(i,:,TIME) = split(i,:,TIME) - split(i,1,TIME);
end

% Assemble data for fitting
x = zeros(0);
y = zeros(0);
for i = 1:steps
	x = cat(1, x, squeeze(split(i,:,TIME)));
	y = cat(1, y, squeeze(split(i,:,OUT)));
end
lsqopts = optimset('MaxFunEvals', 1000);
% Fit an exponential approach to our data
coeffs = lsqnonlin(@diff_exp_approach, [0.35, 1.0], [], [], lsqopts, x, y)

% Plot measured data points and the fit function.
clf
hold on
for i = 1:steps
	plot(split(i,:,TIME), split(i,:,OUT),'b*');
end
plot([0:0.02:2], coeffs(1)*(1-exp(-coeffs(2)*[0:0.02:2])), 'r');
hold off


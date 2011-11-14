% Simulation of wavefront path planning for.

clear;

% Initial vehicle state [forward velocity, heading, x, y]
x0 = [0 0 0 0]';
% Friendly index constants:
VEL = 1;
HEAD = 2;
X = 3;
Y = 4;

% (convenience) Start position:
start = x0(X:Y)';
% Goal position:
goal = [5 5]; 

% Region bounds (Define lower left, upper right corners of rectangle) (m)
regionMin = [-2 -2];
regionMax = [8 8];

% Number of obstacles:
numObsts = 4;
% Size bounds on obstacles:
minLen.a = 0.5;
maxLen.a = 3;
minLen.b = 0.5;
maxLen.b = 3;

% Generate a random map:
[obstA, obstB, obstPts] = polygonal_world(regionMin, regionMax, minLen, maxLen, ...
    numObsts, start, goal, 0.5, 10000); 

% Plot the map:
figure(1);
clf();
subplot(2,2,1);
hold on
plotEnvironment(obstPts, regionMin, regionMax, start, goal);
hold off

% Generate an occupancy grid:
occRes = 0.10; % occupancy grid resolution
xaxis = [regionMin(1):occRes:regionMax(1)];
yaxis = [regionMin(2):occRes:regionMax(2)];
for k=1:length(yaxis)
  x(k,:) = xaxis;
end
for k=1:length(xaxis)
  y(:,k) = yaxis;
end

map = zeros(length(x), length(y));
for k=1:numObsts
   map = map + inpolygon(x, y, obstPts(:,2*(k-1)+1), obstPts(:,2*k));
end


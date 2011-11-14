% Simulation of wavefront path planning for.

clear;

% Initial vehicle state [forward velocity, heading, x, y]
x0 = [0 0 0 0]';
% Friendly index constants:
VEL = 1;
HEAD = 2;
X = 3;
Y = 4;

% (convenience) Start positioN:
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


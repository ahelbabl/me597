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
numObsts = 6;
% Size bounds on obstacles:
minLen.a = 0.5;
maxLen.a = 3;
minLen.b = 0.5;
maxLen.b = 3;

% Generate a random map:
[obstA, obstB, obstPts] = polygonal_world(regionMin+[.5 .5], regionMax-[.5 .5], ...
    minLen, maxLen, numObsts, start, goal, 0.5, 10000); 

% Plot the map:
figure(1);
clf();
subplot(2,2,1);
hold on
plotEnvironment(obstPts, regionMin, regionMax, start, goal);
hold off

% Generate an occupancy grid map:
res = 0.10; % occupancy grid resolution
xaxis = [regionMin(1):res:regionMax(1)];
yaxis = [regionMin(2):res:regionMax(2)];
for k=1:length(yaxis)
  x(:,k) = xaxis;
end
for k=1:length(xaxis)
  y(k,:) = yaxis;
end

map = zeros(length(xaxis), length(yaxis));
for k=1:numObsts
   map = map + inpolygon(x, y, obstPts(:,2*(k-1)+1), obstPts(:,2*k));
end

% Generate the wavefront costmap
cost = zeros(size(map));
% Start and goal location indexes within the map:
startInx = floor( (start-regionMin)/res );
goalInx = floor( (goal-regionMin)/res );
% The open set.  (nb: closed set are indexes s.t. cost(i,j) != 0)
open = goalInx;
cost(goalInx(1), goalInx(2)) = 1;
% Index offsets which are 'adjacent' to a given point.
adjacent = [ 1 0; 0 1; -1 0; 0 -1];  % immediate adjacents
% adjacent = [ 1 -1; 1 0; 1 1; 0 -1; 0 0; 0 1; -1 -1; -1 0; -1 1 ];
while (size(open,1) ~= 0)
  % Iterate through cells adjacent to the cell at the top of the open queue:
  for k=1:size(adjacent,1)
    % Calculate index for current adjacent cell:
    adj = open(1,:)+adjacent(k,:);
    % Make sure adjacent cell is in the map
    if min(adj) < 1
      continue
    end
    if adj(1) > length(xaxis)
      continue
    end
    if adj(2) > length(yaxis)
      continue
    end
    % Make sure the adjacent cell is not an obstacle 
    if map(adj(1), adj(2)) == 1
      continue
    end
    % Make sure the adjacent cell is not closed:
    if cost(adj(1), adj(2)) ~= 0
      continue
    end
    % Set the cost and add the adjacent to the open set
    cost(adj(1), adj(2)) = cost(open(1,1), open(1,2)) + 1;
    open(size(open,1)+1,:) = adj;
  end

  % Pop the top open cell from the queue
  open = open(2:end,:);
end

subplot(2,2,2);
imagesc(xaxis, yaxis, cost');
set(gca, 'YDir', 'normal');

% Calculate the gradient of the costmap
costdx = zeros(size(map));
costdy = zeros(size(map));
for i=1:size(map,1)
  for j=1:size(map,2)
    if cost(i,j) == 0
      % no gradient for obstacles
      continue
    end

    % Calculate differential in x
    dx = 0;
    left = cost(i,j);
    right = cost(i,j);
    if (i~=1) && (cost(i-1,j)~=0)
      % Not at left side, and not obstacle to left
      left = cost(i-1,j);
      dx = res;
    end
    if (i~=size(map,1)) && (cost(i+1,j)~=0)
      % Not at right side, and not obstacle to right
      right = cost(i+1,j);
      dx = dx + res;
    end
    if dx ~= 0
      % Is derivative defined?
      costdx(i,j) = (right-left)/dx;
    end

    % Calculate differential in y
    dy = 0;
    up = cost(i,j);
    down = cost(i,j);
    if (j~=1) && (cost(i,j-1)~=0)
      % Not at bottom, and no obstacle below
      down = cost(i,j-1);
      dy = res;
    end
    if (j~=size(map,2)) && (cost(i,j+1)~=0)
      % Not at top, and no obstacle above
      up = cost(i,j+1);
      dy = dy + res;
    end
    if dy ~= 0
      % Is derivative defined?
      costdy(i,j) = (up-down)/dy;
    end
  end
end

% Plot the gradient of the costmap.
n = size(map,1)*size(map,2);
subplot(2,2,3);
quiver(reshape(x,n,1),reshape(y,n,1),reshape(-costdx,n,1),reshape(-costdy,n,1));
axis([regionMin(1) regionMax(1) regionMin(2) regionMax(2)])


% Simulation of wavefront path planning for.

clear;

% Motion parameters
dt = 0.05;   % sampling period
v = 0.2;     % robot velocity
maxU = 0.35; % maximum robot steering angle
L = 0.265;   % robot wheelbase
k = 0.5;     % steering control proportional constant.

% Simulation stop parameters
simMaxIter = 2000;    % how long to run before giving up?
simGoalThresh = 0.5;  % how close to goal before done?

% Initial vehicle state [heading, x, y]
x0 = [0 0 0]';  % n.b. constant velocity model
% Friendly index constants:
HEAD = 1;
X = 2;
Y = 3;

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
startInx = round( (start-regionMin)/res );
goalInx = round( (goal-regionMin)/res );
% The open set.  (nb: closed set are indexes s.t. cost(i,j) != 0)
open = goalInx;
cost(goalInx(1), goalInx(2)) = 1;
% Index offsets which are 'adjacent' to a given point.
% adjacent = [ 1 0; 0 1; -1 0; 0 -1];  % immediate adjacents
adjacent = [ 1 -1; 1 0; 1 1; 0 -1; 0 0; 0 1; -1 -1; -1 0; -1 1 ];
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

% Now, add costs for obstacle cells.  The planned path will not enter these 
% cells, but we want costs to be defined in order to ensure that the robot can find
% its way out of obscles when it enters due to imperfect path tracking.
objBaseCost = max(reshape(cost,1,[])) + 1;
% Scan for occupied cells reachable from unoccupied cells.  Give them base costs,
% and add them to the open set
for i = 1:length(xaxis)
  for j = 1:length(yaxis)
    % screen unoccupied cells, and occupied cells already added to open set
    if cost(i,j) ~= 0
      continue
    end

    reachable = 0;
    for k=1:size(adjacent,1)
      adj = [i j]+adjacent(k,:);
      if map(adj(1),adj(2)) == 0
        reachable = 1;
        break
      end
    end
    if reachable == 1
      cost(i,j) = objBaseCost;
      open(size(open,1)+1,:) = [i j];
    end
  end
end
while size(open, 1) ~= 0
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

figure(1);
clf();
subplot(1,2,1);
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

% Plot a robot trajectory.
x = x0;
u = 0;

k = 1;
while 1
  k = k+1;
  if (k>simMaxIter)
    break
  end

  % Calculate the occupancy grid indices for the current position:
  i = round((x(X,k-1) - regionMin(1)) / res);
  j = round((x(Y,k-1) - regionMin(2)) / res);
  if (i<1) || (i>length(xaxis))
    break
  end
  if (j<1) || (j>length(yaxis))
    break
  end

  d = norm(goal - x(X:Y,k-1)', 2);
  if d < simGoalThresh
    break
  end

  % Reference position is the gradient direction
  gradDir = atan2(-costdy(i,j), -costdx(i,j));
  err =  gradDir - x(HEAD,k-1);
  if (err > pi)
    err = err - 2*pi;
  end
  if (err < -pi)
    err = err + 2*pi;
  end
  % u(k-1) = atan2(k*err, v);
  u(k-1) = k*err;
  u(k-1) = max(-maxU,min(maxU,u(k-1)));

  x(X,k) = x(X,k-1) + dt*v*cos(x(HEAD,k-1));
  x(Y,k) = x(Y,k-1) + dt*v*sin(x(HEAD,k-1));
  x(HEAD,k) = x(HEAD,k-1) + dt*v*sin(u(k-1))/L;
end

subplot(1,2,2);
hold on
plotEnvironment(obstPts, regionMin, regionMax, start, goal);
plot(x(X,:),x(Y,:));
hold off


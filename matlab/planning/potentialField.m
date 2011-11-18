% Potential field function simulation along with the steering controller
clear all; close all; clc; clf;

%% Setting up the environment
% Boundaries
posMinBound = [-2 -2];
posMaxBound = [25 20];

% Start and end(goal) position
startPos = [24 0];
endPos = [6 5];

% Loading up the obstacle positions
load('obsPtsStore.mat');
numObsts = 6;

% Computing the centroids of the obstacles
for i=1:numObsts
    obsCentroid(i,:)...
    = (obsPtsStore(1,2*(i-1)+1:2*i) + obsPtsStore(3,2*(i-1)+1:2*i))/2;
end

% Plotting the environment
figure(1); clf; hold on;
plotEnvironment(obsPtsStore, posMinBound, posMaxBound, startPos, endPos);
hold off

%% Setting up the potential field
Katt = 0.2;     % Scale of the attractive potential
Krep = 1000;    % Scale of the repulsive potential
r0 = 0.5;       % Radius of repulsion from the closest point of obstacles
rc0 = 4;        % Raidus of repulsion from the centre of obstacles
Vmax = 50;      % Upper bound on potential
gVmax = 10;     % 
gVmin = -10;
dx = .4;        % Grid size for x
dy = .4;        % Grid size for y

% Produce the coordinates of a rectangular grid (X,Y)
[X,Y] = meshgrid([posMinBound(1):dx:posMaxBound(1)],...
                 [posMinBound(2):dy:posMaxBound(2)]);

%Calculate potential field at each grid point
V = zeros(size(X));     % Potential field points
[n,m] = size(X);        % Size of the grid points
gV = zeros(2,n,m);      % Potential field gradients
for i=1:length(X(:,1))
    for j=1:length(Y(1,:))
        % Current grid point
        pos = [X(i,j) Y(i,j)];
        % Attractive potential
        V(i,j) = 1/2*Katt*norm(pos-endPos)^2;   % Potential
        gV(:,i,j) = Katt*(pos-endPos);          % Gradient
        
        % Repulsive potentials
        for m=1:numObsts
            curobs = obsPtsStore(:,2*(m-1)+1:2*m);
            if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
                V(i,j) = Vmax;
                gV(:,i,j) = [NaN NaN];
            else
                % Find potential based on minimum distance to obstacle
                curpoly = [curobs curobs([2:end, 1],:)];
                [minD,minPt, d, pt, ind] = minDistToEdges(pos, curpoly);
                if (minD < r0)
                    V(i,j) = V(i,j) + 1/2*Krep*(1/minD-1/r0)^2;
                    gV(:,i,j) = gV(:,i,j) +...
                                Krep*(-1/minD+1/r0)*(pos'-minPt')/minD^(3);
                end
                % Add potential of distance to center, to avoid getting
                % stuck on flat walls
                centD = norm(pos-obsCentroid(m,:));
                if (centD < rc0)
                    V(i,j) =  V(i,j) + 1/2*Krep*(1/centD-1/rc0)^2;
                    gV(:,i,j) = gV(:,i,j) +...
                                Krep*(-1/centD+1/rc0)*...
                                (pos'-obsCentroid(m,:)')/...
                                centD^(3);
                end
            end
        end
        V(i,j) = max(0,min(V(i,j),Vmax));
        gV(1,i,j) = max(gVmin,min(gV(1,i,j),gVmax));
        gV(2,i,j) = max(gVmin,min(gV(2,i,j),gVmax));
    end
end

% Simulate a robot moving through the environment following steepest
% descent, recalculated for current position
Tmax = 10000;
x = zeros(2,Tmax);      % State vectors [x y]
x(:,1) = startPos';     % Initial position
dx = 0.01;              % Position increment
t = 1;                  % Initial time step
gVcur = [1 1];          % Current grid point
while ((norm(gVcur)>0.01) && (t<Tmax))
    t = t+1;
    pos = x(:,t-1)';
    gVcur = Katt*(pos-endPos);
    for m=1:numObsts
        curobs = obsPtsStore(:,2*(m-1)+1:2*m);
        if (inpolygon(pos(1),pos(2),curobs(:,1),curobs(:,2)))
            gVcur = [NaN NaN];
        else
            curpoly = [curobs curobs([2:end, 1],:)];
            [minD,minPt, d, pt, ind] = minDistToEdges(pos, curpoly);
            if (minD < r0)
                gVcur = gVcur + Krep*(-1/minD+1/r0)*(pos-minPt)/minD^(3);
            end
            centD = norm(pos-obsCentroid(m,:));
            if (centD < rc0)
                gVcur = gVcur + Krep*(-1/centD+1/rc0)*(pos-obsCentroid(m,:))/centD^(3);
            end
        end
    end
    x(:,t) = x(:,t-1)-dx.*gVcur';       % Gradient descent implementation
    
end

figure(1); hold on;
plot(x(1,1:t), x(2,1:t), 'g*');
plot(
% figure(4);
% plot(x(:,1:t)');
% Potential field example
clear all; close all; clc

% Vehicle start and end position
startPos = [21.5 18.5];
endPos = [16 5];

% Set up environment

% Region Bounds
posMinBound = [-2 -2];
posMaxBound = [25 20];

% Number of obstacles
numObsts = 6;
% Size bounds on obstacles
minLen.a = 1;
maxLen.a = 3;
minLen.b = 2;
maxLen.b = 6;

% Random environment generation
obstBuffer = 0.5;
maxCount = 10000;
seedNumber = rand('state');
[aObsts,bObsts,obsPtsStore] = polygonal_world(posMinBound, posMaxBound, minLen, maxLen, numObsts, startPos, endPos, obstBuffer, maxCount);
for i=1:numObsts
    obsCentroid(i,:) = (obsPtsStore(1,2*(i-1)+1:2*i)+obsPtsStore(3,2*(i-1)+1:2*i))/2;
end

% Plot random environment
figure(1); clf;
hold on;
plotEnvironment(obsPtsStore,posMinBound, posMaxBound, startPos, endPos);
hold off

% Create potential field
Katt = 0.2; % Attractive
Krep = 1000; % Repulsive
r0 = 0.5; % Radius of Repulsion
rc0 = 4; %repulsion from center of obstacle
Vmax = 50; % Upper bound on potential
gVmax = 10; %max gradient
gVmin = -10; %min gradient

% Grid up the space
dx =.4;
dy = .4;
[X,Y] = meshgrid([posMinBound(1):dx:posMaxBound(1)],[posMinBound(2):dy:posMaxBound(2)]);

Tmax = 10000;
x = zeros(2,Tmax);
x(:,1) = startPos';
dx = 0.01;
t = 1;
gVcur = [1 1];
while ((norm(gVcur)>0.01) && (t<Tmax))
    t=t+1;
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
    x(:,t) = x(:,t-1) -dx.*gVcur';

end
figure(1); hold on;
plot(x(1,1:t), x(2,1:t));
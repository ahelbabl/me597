% Simulates the motion of the robot under the controller implemented in
% steering_control.py

dt = 0.05;   % Sampling period (s)
v = 0.2;     % Robot speed (m/s)
t = 30.0;    % Length of simulation (s)
k = 0.5;     % steering control proportional constant
maxU = 0.35; % maximum steering angle (rad)
L = 0.265;   % robot wheelbase lenght (m)

x0 = [-2 1 2.0]'; % Initial State
xr = [2 0 0]';    % Target Waypoint

n = t/dt;    % Samples

x = zeros(length(x0),n); % [x, y, heading]
u = zeros(1,n); % [steering angle]
e = zeros(2,n); % [eHead, eCrosstrack]

% intermeiate xtrack values, for inspection:
ex = zeros(1,n); 
ey = zeros(1,n); 

% Simulate!
x(:,1) = x0;
for i=2:n
    % State update equation.
    x(:,i) = x(:,i-1) + [dt*v*cos(x(3,i-1)); dt*v*sin(x(3,i-1)); dt*v*sin(u(i-1))/L];

    % Calculate cross-track error
    ex(i) = (xr(1) - x(1,i)) * (1 - (cos(xr(3)))^2);
    ey(i) = (xr(2) - x(2,i)) * (1 - (sin(xr(3)))^2);
    e(2,i) = sqrt(ex(i)^2 + ey(i)^2);
    e(2,i) = e(2,i) * sign(cos(xr(3))*ey(i) - sin(xr(3))*ex(i));
   
    % Heading error
    e(1,i) = xr(3) - x(3,i);

    % Control law
    u(i) = e(1,i) + atan2(k*e(2,i), v);
    % Saturate control output:
    u(i) = max(-maxU,min(maxU,u(i)));
end
% Disturbance free dynamics

% Plot
figure(1); 
clf(); 
hold on
% Plot the targe point, with a heading line
plot([xr(1)-cos(xr(3)) xr(1)+cos(xr(3))], [xr(2)-sin(xr(3)) xr(2)+sin(xr(3))], 'r');
plot(xr(1),xr(2),'r*');
% Plot the simulated trajectory
plot(x(1,:), x(2,:), 'b');
axis equal
hold off

% Simulates robot motion, producing ground truth against which
% the extended kalman filter may be tested against.

% Simulation parameters:
time = 10;   % simulation length [s]
ts = 0.05;  % sim timestep [s]
len = 0.265; % robot length [m]

% Process covariance.  (assume independence):
Q = diag([0.01 0.01 0.001 0.001]);
% Measurement covariance:
R = diag([0.05 0.1 0.1 0.1]);

% Sample count.  Because the discrete velocity model needs to look back
% 5 samples, we add 5 samples and set t=0 at sample 5
n = time / ts + 5;  
to = 5;

t = [(-to+1)*ts:ts:time];

% System state.  States are [velocity, heading, x pos, y pos]
xi = zeros(4,n);  % ideal state
% Initial state:  
xi(:,to) = [0 0 0 0];
x = xi;  % true state, with process noise

% Measurements.  Direct, in same order as state vector.
m = zeros(4,n);

% Input signal.  Inputs are [motor %, steering angle (rad)]
u = [ones(1,n)*100; ones(1,n)*0.3];
u(:,1:to) = zeros(2,to);

% Velocity model:
load('vel_coeffs.mat');
zn = (Km*am*ts)/(am*ts + 2)*[1 1];
zd = cat(2, [1 (am*ts-2)/(am*ts+2)], zeros(1, delay));

% Simulate!
for k = to+1:n
    % Ideal model:
    % Velocity:
    xi(1,k) = zn(1)*u(1,k-5) + zn(2)*u(1,k-4) - zd(2)*xi(1,k-1);
    % Heading:
    xi(2,k) = xi(2,k-1) + xi(1,k-1)*sin(u(2,k-1))*ts/len;
    % x pos:
    xi(3,k) = xi(3,k-1) + xi(1,k-1)*cos(xi(2,k-1))*ts;
    % y pos:
    xi(4,k) = xi(4,k-1) + xi(1,k-1)*sin(xi(2,k-1))*ts;

    % Noisy model:
    x(1,k) = zn(1)*u(1,k-5) + zn(2)*u(1,k-4) - zd(2)*x(1,k-1);
    x(2,k) = x(2,k-1) + x(1,k-1)*sin(u(2,k-1))*ts/len;
    x(3,k) = x(3,k-1) + x(1,k-1)*cos(x(2,k-1))*ts;
    x(4,k) = x(4,k-1) + x(1,k-1)*sin(x(2,k-1))*ts;
    x(:,k) = x(:,k) + Q*randn(4,1);

    % Measurements:
    m(:,k) = x(:,k) + R*randn(4,1);


end

% Plot real state vs. ideal state vs. measurements:
figure(1);
clf();

% Velocity
subplot(3,1,1);
hold on
plot(t,m(1,:),'c');
plot(t,xi(1,:));
plot(t,x(1,:),'r');
hold off

% Heading
subplot(3,1,2);
hold on
plot(t,m(2,:),'c');
plot(t,xi(2,:));
plot(t,x(2,:),'r');
hold off

% Position
subplot(3,1,3);
hold on
plot(m(3,:),m(4,:),'c');
plot(xi(3,:),xi(4,:));
plot(x(3,:),x(4,:),'r');
axis square
axis equal
hold off


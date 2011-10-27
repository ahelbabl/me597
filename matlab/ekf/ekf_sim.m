% Simulates robot motion, producing ground truth against which
% the extended kalman filter may be tested against.
%
% See http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf
% for background on the EKF.

% =========== Simulation Parameters =============
time = 10;   % simulation length [s]
ts = 0.05;  % sim timestep [s]
len = 0.265; % robot length [m]

% States and measurements are [velocity, heading, x pos, y pos]
% Process covariance.  (assume independence):
Q = diag([sqrt(0.0005) 0.02 0.005 0.005].^2);
% Measurement covariance:
R = diag([sqrt(0.0035) 0.5 0.5 0.5].^2);

% =========== Simulation ========================

% Sample count.  Because the discrete velocity model needs to look back
% 5 samples, we add 5 samples and set t=0 at sample 5
n = time / ts + 5;  
to = 5;
t = [(-to+1)*ts:ts:time];

% Input signal.  Inputs are [motor %, steering angle (rad)]
u = [ones(1,n)*100; ones(1,n)*0.3];
u(:,1:to) = zeros(2,to);

% System state.
xi = zeros(4,n);  % ideal state
% Initial state:  
xi(:,to) = [0 0 0 0];
x = xi;  % true state, with process noise

% Measurements.  Direct, in same order as state vector.
m = zeros(4,n);

% Kalman estimates:
xe = xi;  % Estimate means, start with correct i.c.
Pe = zeros(4,4,n); % Estimate covariances
Pe(:,:,to) = Q;  % Initial condition covariance

% Velocity model:
load('vel_coeffs.mat');
% Z transform numerator and denominator coefficients:
zn = (Km*am*ts)/(am*ts + 2)*[1 1];
zd = cat(2, [1 (am*ts-2)/(am*ts+2)], zeros(1, delay));

% Simulate!
for k = to+1:n
    % Ideal model:
    % Velocity:
    xi(1,k) = zn(1)*u(1,k-4) + zn(2)*u(1,k-3) - zd(2)*xi(1,k-1);
    % Heading:
    xi(2,k) = xi(2,k-1) + xi(1,k-1)*sin(u(2,k-1))*ts/len;
    % x pos:
    xi(3,k) = xi(3,k-1) + xi(1,k-1)*cos(xi(2,k-1))*ts;
    % y pos:
    xi(4,k) = xi(4,k-1) + xi(1,k-1)*sin(xi(2,k-1))*ts;

    % Noisy model:
    x(1,k) = zn(1)*u(1,k-4) + zn(2)*u(1,k-3) - zd(2)*x(1,k-1);
    x(2,k) = x(2,k-1) + x(1,k-1)*sin(u(2,k-1))*ts/len;
    x(3,k) = x(3,k-1) + x(1,k-1)*cos(x(2,k-1))*ts;
    x(4,k) = x(4,k-1) + x(1,k-1)*sin(x(2,k-1))*ts;
    x(:,k) = x(:,k) + sqrt(Q)*randn(4,1);  % Only valid if noise is independent!

    % Measurements:
    m(:,k) = x(:,k) + sqrt(R)*randn(4,1);

    % A priori state estimate:
    xp = zeros(4,1);
    xp(1) = zn(1)*u(1,k-4) + zn(2)*u(1,k-3) - zd(2)*xe(1,k-1);
    xp(2) = xe(2,k-1) + xe(1,k-1)*sin(u(2,k-1))*ts/len;
    xp(3) = xe(3,k-1) + xe(1,k-1)*cos(xe(2,k-1))*ts;
    xp(4) = xe(4,k-1) + xe(1,k-1)*sin(xe(2,k-1))*ts;

    % Jacobian of current state w.r.t previous state:
    A = [-zd(2)                0                        0 0;
         ts*sin(u(2,k-1))/len  1                        0 0;
         ts*cos(xe(2,k-1)) -xe(1,k-1)*ts*sin(xe(2,k-1)) 1 0;
         ts*sin(xe(2,k-1)) xe(1,k-1)*ts*cos(xe(2,k-1))  0 1; ];

    % Jacobian of state w.r.t process noise:
    W = eye(4);
    % Jacobian of measurements w.r.t state:
    H = eye(4);
    % Jacobian of measurements w.r.t measurement noise:
    V = eye(4);

    % A priori covariance:
    Pp = A*Pe(:,:,k-1)*A' + W*Q*W';

    % Kalman gain:
    K = Pp*H'*inv(H*Pp*H' + V*R*V');

    % EKF state estimate:
    xe(:,k) = xp + K*(m(:,k) - xp);
    Pe(:,:,k) = (eye(4) - K*H)*Pp;
end

% Plot true state vs. ideal state vs. measurements:
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

% Plot EKF estimate 1 sigma s.d bounds vs. true state
figure(2);
clf();

% Velocity
subplot(2,2,1);
hold on
title('Velocity (m/s)');
plot(t,m(1,:),'g*');
plot(t,xe(1,:)+sqrt(reshape(Pe(1,1,:),size(t))));
plot(t,xe(1,:)-sqrt(reshape(Pe(1,1,:),size(t))));
plot(t,x(1,:),'r');
hold off

% Heading
subplot(2,2,2);
hold on
title('Heading (radian)');
plot(t,m(2,:),'g*');
plot(t,xe(2,:)+sqrt(reshape(Pe(2,2,:),size(t))));
plot(t,xe(2,:)-sqrt(reshape(Pe(2,2,:),size(t))));
plot(t,x(2,:),'r');
hold off

% X Position
subplot(2,2,3);
hold on
title('X Position (m)');
plot(t,m(3,:),'g*');
plot(t,xe(3,:)+sqrt(reshape(Pe(3,3,:),size(t))));
plot(t,xe(3,:)-sqrt(reshape(Pe(3,3,:),size(t))));
plot(t,x(3,:),'r');
hold off

% Y Position
subplot(2,2,4);
hold on
title('Y Position (m)');
plot(t,m(4,:),'g*');
plot(t,xe(4,:)+sqrt(reshape(Pe(4,4,:),size(t))),'b');
plot(t,xe(4,:)-sqrt(reshape(Pe(4,4,:),size(t))),'b');
plot(t,x(4,:),'r');
hold off

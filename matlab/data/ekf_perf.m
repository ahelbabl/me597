x = dlmread('msmt.dat');  % 'true' data
x = x';
u = dlmread('output.dat');
u = u';
ts = 0.05;
len = 0.265;

% Size of dataset
datasize = size(m);
n = datasize(2);

% estimates:
m = zeros(datasize);
xe = zeros(datasize);
xe(:,1) = x(:,1);
Pe = zeros(4,4,n);

% Process covariance.  (assume independence):
Q = diag([sqrt(0.0005) 0.02 0.005 0.005].^2);
% Measurement covariance:
R = diag([sqrt(0.0035) 0.5 0.5 0.5].^2);

% Velocity model:
load('../ekf/vel_coeffs.mat');
Km = 0.0022;

% Simulate!
for k = 2:n
    % Add some noise to the LPS data
    m(:,k) = x(:,k) + randn(4,1).*[0; R(2,2); R(3,3); R(4,4)];

    % A priori state estimate:
    xp = zeros(4,1);
    xp(1) = xe(1,k-1) - am*ts*xe(1,k-1) + am*Km*ts*u(1,k);
    xp(2) = xe(2,k-1) + xe(1,k-1)*sin(0.0035*u(2,k-1))*ts/len;
    xp(3) = xe(3,k-1) + xe(1,k-1)*cos(xe(2,k-1))*ts;
    xp(4) = xe(4,k-1) + xe(1,k-1)*sin(xe(2,k-1))*ts;

    % Jacobian of current state w.r.t previous state:
    A = [-am*ts                0                        0 0;
         ts*sin(u(2,k-1)*0.0035)/len  1                        0 0;
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

    % Hack hack hack angle wrap around correction
    if abs(xp(2) - m(2,k)) > abs(xp(2) - 2*pi - m(2,k))
      xp(2) = xp(2) - 2*pi
    end

    % EKF state estimate:
    xe(:,k) = xp + K*(m(:,k) - xp);
    Pe(:,:,k) = (eye(4) - K*H)*Pp;
end

figure(1);
clf();

x(1,2:n) = sqrt( (x(3,2:n)-x(3,1:n-1)).^2 + (x(4,2:n)-x(4,1:n-1)).^2 ) / ts;

subplot(2,1,1);
hold on
ylabel('Forward Velocity (m/s)');
plot(m(1,:),'g');
plot(xe(1,:),'r');
hold off

subplot(2,1,2);
hold on
ylabel('Heading (rad)');
plot(m(2,:),'g');
plot(x(2,:),'b');
plot(xe(2,:),'r');
hold off

figure(2);
clf();

subplot(2,1,1);
hold on
ylabel('X Position (m)');
plot(m(3,:),'g');
plot(x(3,:),'b');
plot(xe(3,:),'r');
hold off

subplot(2,1,2);
hold on
ylabel('Y Position (m)');
plot(m(4,:),'g');
plot(x(4,:),'b');
plot(xe(4,:),'r');
hold off


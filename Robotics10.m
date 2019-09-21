% Author:  JingXuan Yang
% E-mail:  yangjingxuan@stu.hit.edu.cn
% Date:    2019.04.05
% Project: Robotics HW 10
% Purpose: fifth order polynomial plan of two DOF manipulator
% Note   : all angles in this script are in degree

clear;
clc;

% initial data
% arm 1
theta10 = 10;
theta1f = 60;
dtheta1f = theta1f - theta10;

% arm 2
theta20 = 20;
theta2f = 100;
dtheta2f = theta2f - theta20;

% time
tf = 31;

% coefficients of theta1
a10 = theta10;
a11 = 0;
a12 = 0;
a13 = 10*dtheta1f/tf^3;
a14 = -15*dtheta1f/tf^4;
a15 = 6*dtheta1f/tf^5;

% coefficients of theta2
a20 = theta20;
a21 = 0;
a22 = 0;
a23 = 10*dtheta2f/tf^3;
a24 = -15*dtheta2f/tf^4;
a25 = 6*dtheta2f/tf^5;

% time series
t = 0:1:tf;
% length of time series
nt = length(t);

% initialize position matrices
x1 = zeros(1,nt);
y1 = zeros(1,nt);
x2 = zeros(1,nt);
y2 = zeros(1,nt);

% calculate theta1, omega1 and alpha1
theta1 = a10 + a11*t + a12*t.^2 + a13*t.^3 + a14*t.^4 + a15*t.^5;
dtheta1 = a11 + 2*a12*t + 3*a13*t.^2 + 4*a14*t.^3 + 5*a15*t.^4;
ddtheta1 = 2*a12 + 6*a13*t + 12*a14*t.^2 + 20*a15*t.^3;

% calculate theta2, omega2 and alpha2
theta2 = a20 + a21*t + a22*t.^2 + a23*t.^3 + a24*t.^4 + a25*t.^5;
dtheta2 = a21 + 2*a22*t + 3*a23*t.^2 + 4*a24*t.^3 + 5*a25*t.^4;
ddtheta2 = 2*a22 + 6*a23*t + 12*a24*t.^2 + 20*a25*t.^3;

% find max omega, max alpha and corresponding theta
[dtheta1max, theta1dmax] = max(dtheta1);
[ddtheta1max, theta1ddmax] = max(ddtheta1);

[dtheta2max, theta2dmax] = max(dtheta2);
[ddtheta2max, theta2ddmax] = max(ddtheta2);

% output results
fprintf('omega1 max is %.4f, at theta1 = %.4f\n',dtheta1max, 0.01*theta1dmax);
fprintf('alpha1 max is %.4f, at theta1 = %.4f\n',ddtheta1max, 0.01*theta1ddmax);

fprintf('omega2 max is %.4f, at theta2 = %.4f\n',dtheta2max, 0.01*theta2dmax);
fprintf('alpha2 max is %.4f, at theta2 = %.4f\n',ddtheta2max, 0.01*theta2ddmax);

% draw figures
% arm 1
figure(1)
subplot(1,3,1);
plot(t,theta1);
xlabel('t/s');
ylabel('\theta_1/\circ');
title('\theta_1');

subplot(1,3,2);
plot(t,dtheta1);
xlabel('t/s');
ylabel('\omega_1/(\circ/s)');
title('\omega_1');

subplot(1,3,3);
plot(t,ddtheta1);
xlabel('t/s');
ylabel('\alpha_1/(\circ/s^2)');
title('\alpha_1');

% arm 2
figure(2)
subplot(1,3,1);
plot(t,theta2);
xlabel('t/s');
ylabel('\theta_2/\circ');
title('\theta_2');

subplot(1,3,2);
plot(t,dtheta2);
xlabel('t/s');
ylabel('\omega_2/(\circ/s)');
title('\omega_2');

subplot(1,3,3);
plot(t,ddtheta2);
xlabel('t/s');
ylabel('\alpha_2/(\circ/s^2)');
title('\alpha_2');

% motion state of two arms
figure(3)
for i = 1:tf+1
    x1(i) = cosd(theta1(i));
    y1(i) = sind(theta1(i));
    x2(i) = cosd(theta1(i)) + cosd(theta1(i)+theta2(i));
    y2(i) = sind(theta1(i)) + sind(theta1(i)+theta2(i));
    line([0 x1(i) x2(i)],[0 y1(i) y2(i)],'LineWidth',2,'Marker','d');
    hold on;
end
xlabel('x/m');
ylabel('y/m');
title('Motion State');

% Author:  JingXuan Yang
% E-mail:  yangjingxuan@stu.hit.edu.cn
% Date:    2019.04.09
% Project: Robotics HW 11
% Purpose: third order polynomial plan of 3R elbow arm
% Note   : all angles in this script are in degree

clear;
clc;

% initial data
l1 = 1;
l2 = 1;
l3 = 1;

theta1 = 30;
theta2 = 40;
theta3 = 50;
theta12 = theta1 + theta2;
theta123 = theta1 + theta2 + theta3;

fex = 10;
fey = 15;
Tez = 50;

J11 = - l1*sind(theta1) - l2*sind(theta12) - l3*sind(theta123);
J21 = - l2*sind(theta12) - l3*sind(theta123);
J31 = - l3*sind(theta123);

J12 = l1*cosd(theta1) + l2*cosd(theta12) + l3*cosd(theta123);
J22 = l2*cosd(theta12) + l3*cosd(theta123);
J32 = l3*cosd(theta123);

J13 = 1;
J23 = 1;
J33 = 1;

Jforce = [J11 J12 J13;
          J21 J22 J23;
          J31 J32 J33];

Force = [fex fey Tez]';

Tao = Jforce*Force;





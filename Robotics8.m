%Author:  JingXuan Yang
%E-mail:  yangjingxuan@stu.hit.edu.cn
%Date:    2019.03.28
%Project: Robotics HW 8
%Purpose: inverse kinematics for SRS - 6 DOF manipulator
%Note   : all angles in this script are in degree

clear;
clc;

syms theta1 theta2 theta3 a2 a3 d1 pi d3 d2 a1

T01 = TDH(theta1,-pi/2,0,d1);
T12 = TDH(theta2,0,a2,0);
T23 = TDH(theta3,0,a3,0);

T13 = simplify(T12*T23);
T03 = simplify(T01*T13);

T32 = simplify(inv(T23));
T31 = simplify(inv(T13));
T30 = simplify(inv(T03));

T01s = TDH(theta1,-pi/2,0,d1);
T12s = TDH(theta2,-pi/2,0,0);
T23s = TDH(theta3,0,0,d3);

T02s = simplify(T01s*T12s);
T03s = simplify(T02s*T23s);
T13s = simplify(T12s*T23s);

T32s = simplify(inv(T23s));
T31s = simplify(inv(T13s));
T30s = simplify(inv(T03s));

%cross(a,b)

%--------------------------------------------------------------------------
%-----functions------------------------------------------------------------
%--------------------------------------------------------------------------
%TDH, transform matrix for D-H method
function [T] = TDH(theta,alpha,a,d)

T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0            sin(alpha)             cos(alpha)         d    ;
         0                0                      0              1   ];
     
end

syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 pi...
     d1 d2 d3 d4 d5 d6 d7 a1 a2 a3 a4 a5 a6 a7

T01 = TDH(theta1,pi/2,0,d1);
T12 = TDH(theta2,0,a2,d2);
T23 = TDH(theta3,0,a3,d3);
T34 = TDH(theta4,pi/2,0,d4);
T45 = TDH(theta5,pi/2,0,0);
T56 = TDH(theta6,0,0,d6);

T67 = TDH(theta7,0,0,d7);

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
% nx3 = sin(theta1)*sin(theta3)+cos(theta1)*cos(theta2)*cos(theta3);
% ny3 = sin(theta1)*cos(theta2)*cos(theta3)- cos(theta1)*sin(theta3);
% 
% nx4 = sin(theta1)*sin(theta3+theta4)+cos(theta1)*cos(theta2)*cos(theta3+theta4);
% ny4 = sin(theta1)*cos(theta2)*cos(theta3+theta4)- cos(theta1)*sin(theta3+theta4);
% nz4 = sin(theta2)*cos(theta3+theta4);
% ox4 = sin(theta1)*cos(theta3+theta4)-cos(theta1)*cos(theta2)*sin(theta3+theta4);
% oy4 = -sin(theta1)*cos(theta2)*sin(theta3+theta4)- cos(theta1)*cos(theta3+theta4);
% oz4 = -sin(theta2)*sin(theta3+theta4);
% ax4 = cos(theta1)*sin(theta2);
% ay4 = sin(theta1)*sin(theta2);
% az4 = -cos(theta2);
% px4 = d2*sin(theta1)+(d3+d4)*cos(theta1)*sin(theta2)+a3*nx3+a4*nx4;
% py4 = (d3+d4)*sin(theta1)*sin(theta2)-d2*cos(theta1)+a3*ny3+a4*ny4;
% pz4 = d1-(d3+d4)*cos(theta2)+a3*sin(theta2)*cos(theta3)...
%       + a4*sin(theta2)*cos(theta3+theta4);
% 
% T04m = [nx4 ox4 ax4 px4;
%         ny4 oy4 ay4 py4;
%         nz4 oz4 az4 pz4;
%          0   0   0   1];

T05 = T04*T45;

% nx5 = sin(theta1)*sin(theta3+theta4+theta5)+...
%     cos(theta1)*cos(theta2)*cos(theta3+theta4+theta5);
% ny5 = sin(theta1)*cos(theta2)*cos(theta3+theta4+theta5)-...
%     cos(theta1)*sin(theta3+theta4+theta5);
% nz5 = sin(theta2)*cos(theta3+theta4+theta5);
% 
% ax5 = -sin(theta1)*cos(theta3+theta4+theta5)+...
%       cos(theta1)*cos(theta2)*sin(theta3+theta4+theta5);
% ay5 = sin(theta1)*cos(theta2)*sin(theta3+theta4+theta5)+...
%       cos(theta1)*cos(theta3+theta4+theta5);
% az5 = sin(theta2)*sin(theta3+theta4+theta5);
% 
% ox5 = cos(theta1)*sin(theta2);
% oy5 = sin(theta1)*sin(theta2);
% oz5 = -cos(theta2);
% 
% px5 = d2*sin(theta1)+(d3+d4)*cos(theta1)*sin(theta2)+a3*nx3+a4*nx4...
%     +d5*cos(theta1)*sin(theta2);
% py5 = (d3+d4)*sin(theta1)*sin(theta2)-d2*cos(theta1)+a3*ny3+a4*ny4...
%     +d5*sin(theta1)*sin(theta2);
% pz5 = d1-(d3+d4)*cos(theta2)+a3*sin(theta2)*cos(theta3)...
%       + a4*sin(theta2)*cos(theta3+theta4)-d5*cos(theta2);
%   
% T05m = [nx5 ox5 ax5 px5;
%         ny5 oy5 ay5 py5;
%         nz5 oz5 az5 pz5;
%          0   0   0   1];
  
T06 = T05*T56;

T36 = T34*T45*T56;

T07 = T01*T12*T23*T34*T45*T56*T67;
T17 = T12*T23*T34*T45*T56*T67;

%test
% T01t = TDH(theta1,-pi/2,0,d1);
% T12t = TDH(theta2,0,a2,0);
% T23t = TDH(theta3,0,a3,0);
% 
% T03t = T01t*T12t*T23t;
%--------------------------------------------------------------------------

dr = pi/180;

%calculate
%T01c = TDH(0,pi/2,0,1.6);
T12c = TDH(20*dr,pi/2,0,0.5);
T23c = TDH(30*dr,0,2,0.5);
T34c = TDH(15*dr,0,2,0.5);
T45c = TDH(-10*dr,pi/2,0,0.5);
T56c = TDH(50*dr,pi/2,0,0);
T67c = TDH(45*dr,0,0,0.5);

T17c = T12c*T23c*T34c*T45c*T56c*T67c;

%--------------------------------------------------------------------------
function [T] = TDH(theta,alpha,a,d)

T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0            sin(alpha)             cos(alpha)            d     ;
         0               0                      0                  1    ];

end
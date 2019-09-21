%Author:  JingXuan Yang
%E-mail:  yangjingxuan@stu.hit.edu.cn
%Date:    2019.03.24
%Project: Robotics HW 7
%Purpose: inverse kinematics for SRS - 6 DOF manipulator
%Note   : all angles in this script are in degree

clear;
clc;

%initial data
d1 = 60;
a2 = 103;
d4 = 155;
d6 = 104;

alpha1 = -pi/2;
alpha2 = 0;
alpha3 = pi/2;
alpha4 = -pi/2;
alpha5 = pi/2;
alpha6 = 0;

% initial transform matrix
T06 = round(Trt(0,-pi/2,pi,0,0,0));

% line planning
% initial point
P0 = [d4+d6;0;50];
% final point
Pf = [d4+d6;0;200];

q0 = [0,-pi/2,pi,0,0,0];
pd = Pf;

Theta = inv6RArm(q0,pd);
rd = 180/pi;
Thetad = round(Theta*rd);
T06m = Trt(Theta(1),Theta(2),Theta(3),Theta(4),Theta(5),Theta(6));

%--------------------------------------------------------------------------
%-----functions------------------------------------------------------------
%--------------------------------------------------------------------------
%TDH, transform matrix for D-H method
function [T] = TDH(theta,alpha,a,d)

T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0             sin(alpha)              cos(alpha)               d    ;
         0                 0                       0                      1   ];
     
end

%RDH, rotational matrix for D-H method
function [R] = RDH(theta,alpha)

R = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) ;
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) ;
         0            sin(alpha)             cos(alpha)      ];

end

%Trt, r-reference, t-tool
%calculating forward kinematics with 6 DOFs
function [T06] = Trt(theta1,theta2,theta3,theta4,theta5,theta6)
    
    %initial data
    d1 = 60;
    a2 = 103;
    d4 = 155;
    d6 = 104;

    alpha1 = -pi/2;
    alpha2 = 0;
    alpha3 = pi/2;
    alpha4 = -pi/2;
    alpha5 = pi/2;
    alpha6 = 0;
    
    %transform matrix for each joint
    T01 = TDH(theta1,alpha1,0,d1);
    T12 = TDH(theta2,alpha2,a2,0);
    T23 = TDH(theta3,alpha3,0,0);
    T34 = TDH(theta4,alpha4,0,d4);
    T45 = TDH(theta5,alpha5,0,0);
    T56 = TDH(theta6,alpha6,0,d6);
    
    %total transform matrix
    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
    T06 = T05*T56;

end

function Theta = inv6RArm(q0,pd)
% initial data
% initial joint angle, q0, row vector
% demand position, pd, column vector
% Theta = [theta1 theta2 theta3 theta4 theta5 theta6], in degree

% D-H parameters
% d1 = 60;
a2 = 103;
d4 = 155;
d6 = 104;

% demand position and orientation
% pd = [0.3;0.1732;-0.3];
nd = [0;0;-1];
od = [0;1;0];
ad = [1;0;0];

% set allowed error
% position error
ep = 10^(-10);
%orientation error
eo = 10^(-10);

% assign initial joint angles
qc(1,:) = q0;

% set initial loop number
k = 1;

% begin while loop
while 1 && k < 1000000
    
    % calculate three joint angles
    theta1 = qc(k,1);
    theta2 = qc(k,2);
    theta3 = qc(k,3);
    theta4 = qc(k,4);
    theta5 = qc(k,5);
    theta6 = qc(k,6);
    
    % calculate transform matrix from {0} to {6}
    T06 = Trt(theta1,theta2,theta3,theta4,theta5,theta6);
    
    % elements of velocity Jacobian matrix for 6R manipulator
    
    s1 = sin(theta1);
    c1 = cos(theta1);
    s2 = sin(theta2);
    c2 = cos(theta2);
%     s3 = sin(theta3);
%     c3 = cos(theta3);
    s4 = sin(theta4);
    c4 = cos(theta4);
    s5 = sin(theta5);
    c5 = cos(theta5);
%     s6 = sin(theta6);
%     c6 = cos(theta6);
    s23 = sin(theta2 + theta3);
    c23 = cos(theta2 + theta3);
    
    nx = -s1*(a2*c2+d4*s23) - d6*(s1*(s23*c5 + c23*c4*s5) + c1*s4*s5);
    ny = c1*(a2*c2+d4*s23) + d6*(c1*(s23*c5 + c23*c4*s5) - s1*s4*s5);
    nz = 0;
    na = 0;
    nb = 0;
    nc = 1;
    
    ox = c1*(-a2*s2 + d4*c23 + d6*(c23*c5 - s23*c4*s5));
    oy = s1*(-a2*s2 + d4*c23 + d6*(c23*c5 - s23*c4*s5));
    oz = -a2*c2 - d4*s23 - d6*(s23*c5 + c23*c4*s5);
    oa = -s1;
    ob = c1;
    oc = 0;
    
    ax = c1*(d4*c23 + d6*(c23*c5 - s23*c4*s5));
    ay = s1*(d4*c23 + d6*(c23*c5 - s23*c4*s5));
    az = - d4*s23 - d6*(s23*c5 + c23*c4*s5);
    aa = -s1;
    ab = c1;
    ac = 0;
    
    rx = -d6*(s1*c4 + c1*c23*s4)*s5;
    ry = d6*(c1*c4 - s1*c23*s4)*s5;
    rz = d6*s23*s4*s5;
    ra = c1*s23;
    rb = s1*s23;
    rc = c23;
    
    sx = -d6*(c1*(s23*s5 - c23*c4*c5) + s1*s4*c5);
    sy = -d6*(s1*(s23*s5 - c23*c4*c5) - c1*s4*c5);
    sz = -d6*(c23*s5 + s23*c4*c5);
    sa = -s1*c4 - c1*c23*s4;
    sb = c1*c4 - s1*c23*s4;
    sc = s23*s4;
    
    tx = 0;
    ty = 0;
    tz = 0;
    ta = c1*(s23*c5 + c23*c4*s5) - s1*s4*s5;
    tb = s1*(s23*c5 + c23*c4*s5) + c1*s4*s5;
    tc = c23*c5 - s23*c4*s5;
    
    % velocity Jacobian matrix
    J06 = [nx ox ax rx sx tx;
           ny oy ay ry sy ty;
           nz oz az rz sz tz;
           na oa aa ra sa ta;
           nb ob ab rb sb tb;
           nc oc ac rc sc tc];    
    
    % vectors of transform matrix calculated in each iteration
    pc = T06(1:3,4);
    nc = T06(1:3,1);
    oc = T06(1:3,2);
    ac = T06(1:3,3);
    
    % position error
    dp = pd - pc;
    
    %oriention error
    do = 0.5*(cross(nc,nd) + cross(oc,od) + cross(ac,ad));
    
    % norm of position error
    ndp = norm(dp);    
    
    %norm of orientation error
    ndo = norm(do);
    
    % decide if exiting the loop
    if ndp <= ep && ndo <= eo
        % errors are allowable
        % exit loop
        break;
    else
        % errors are not allowable
        % increments of joint angles
        dq = pinv(J06)*[dp;do];
        
        % take transposition
        dqc = dq';
        
        % update joint angles
        qc(k+1,:) = qc(k,:) + dqc;
        
        % increase the loop number
        k = k + 1;
    end % w.r.t if
    
end % w.r.t while

% output results
% theta1 = wrapTo180(theta1);
% theta2 = wrapTo180(theta2);
% theta3 = wrapTo180(theta3);
% theta4 = wrapTo180(theta4);
% theta5 = wrapTo180(theta5);
% theta6 = wrapTo180(theta6);

Theta = [theta1 theta2 theta3 theta4 theta5 theta6];

end
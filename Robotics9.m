%Author:  JingXuan Yang
%E-mail:  yangjingxuan@stu.hit.edu.cn
%Date:    2019.03.30
%Project: Robotics HW 9
%Purpose: numeric solutions for a demand position and orientation
%Note   : all angles in this script are in degree

clear;
clc;

%initial data
%initial joint angle
q0 = [10,10,10];

%D-H parameters
a2 = 0.4;
a3 = 0.6;
d1 = 0.5;

%demand position and orientation
pd = [0.3;0.1732;-0.3];
nd = [0;0;-1];
od = [-0.866;-0.5;0];
ad = [-0.5;0.866;0];

%set allowed error
%position error
ep = 5*10^(-15);
%orientation error
eo = 7.4*10^(-6);

%assign initial joint angles
qc(1,:) = q0;

%set initial loop number
k = 1;

%begin while loop
while 1
    
    %calculate three joint angles
    theta1 = qc(k,1);
    theta2 = qc(k,2);
    theta3 = qc(k,3);
    
    %calculate three transform matrix
    T01 = TDH(theta1,-90,0,d1);
    T12 = TDH(theta2,0,a2,0);
    T23 = TDH(theta3,0,a3,0);
    
    %transform matrix from {0} to {3}
    T03 = T01*T12*T23;
    
    %elements of velocity Jacobian matrix for elbow manipulator
    nxv = -sind(theta1)*(a2*cosd(theta2)+a3*cosd(theta2+theta3));
    nyv = cosd(theta1)*(a2*cosd(theta2)+a3*cosd(theta2+theta3));
    nzv = 0;
    oxv = -cosd(theta1)*(a2*sind(theta2)+a3*sind(theta2+theta3));
    oyv = -sind(theta1)*(a2*sind(theta2)+a3*sind(theta2+theta3));
    ozv = -(a2*cosd(theta2)+a3*cosd(theta2+theta3));
    axv = -a3*cosd(theta1)*sind(theta2+theta3);
    ayv = -a3*sind(theta1)*sind(theta2+theta3);
    azv = -a3*cosd(theta2+theta3);
    
    %velocity Jacobian matrix
    Jv03 = [nxv oxv axv;
            nyv oyv ayv;
            nzv ozv azv];
    
    %elements of angular velocity Jacobian matrix for elbow manipulator
    nxw = 0;
    nyw = 0;
    nzw = 1;
    oxw = -sind(theta1);
    oyw = cosd(theta1);
    ozw = 0;
    axw = -sind(theta1);
    ayw = cosd(theta1);
    azw = 0;
    
    %angular velocity Jacobian matrix
    Jw03 = [nxw oxw axw;
            nyw oyw ayw;
            nzw ozw azw];
    
    %total Jacobian matrix
    J03 = [Jv03;Jw03];
    
    %vectors of transform matrix calculated in each iteration
    pc = T03(1:3,4);
    nc = T03(1:3,1);
    oc = T03(1:3,2);
    ac = T03(1:3,3);
    
    %position error
    dp = pd - pc;
    
    %oriention error
    do = 0.5*(cross(nc,nd) + cross(oc,od) + cross(ac,ad));
    
    %norm of position error
    ndp = norm(dp);
    
    %norm of orientation error
    ndo = norm(do);
    
    %decide if exiting the loop
    %if ndp <= ep && ndo <= eo
    if ndp <= ep
        %errors are allowable
        %exit loop
        break;
    else
        %errors are not allowable
        %increments of joint angles
        %dq = pinv(J03)*[dp;do];
        dq = Jv03\dp;
        
        %take transposition
        dqc = dq';
        
        %update joint angles
        qc(k+1,:) = qc(k,:) + dqc;
        
        %increase the loop number
        k = k + 1;
    end %w.r.t if
    
end %w.r.t while

%output results
theta1 = wrapTo180(theta1);
theta2 = wrapTo180(theta2);
theta3 = wrapTo180(theta3);

fprintf('The angles found are:\n');
fprintf('theta1 = %.4f, theta2 = %.4f, theta3 = %.4f\n',theta1,theta2,theta3);
T03 %#ok<NOPTS>
    
%draw figures
figure(1)
t = 1:1:k;
plot(t,qc(:,1),t,qc(:,2),t,qc(:,3))
xlabel('loop time');
ylabel('\theta/\circ');
title('Iteration Diagram of Three Joint Angles');
legend('\theta_1','\theta_2','\theta_3');

%--------------------------------------------------------------------------
%-----functions------------------------------------------------------------
%--------------------------------------------------------------------------
%TDH, transform matrix for D-H method
function [T] = TDH(theta,alpha,a,d)

T = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
     sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
         0            sind(alpha)             cosd(alpha)         d    ;
         0                0                      0              1   ];
     
end
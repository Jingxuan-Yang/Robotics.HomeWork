% Author:  JingXuan Yang
% E-mail:  yangjingxuan@stu.hit.edu.cn
% Date:    2019.04.09
% Project: Robotics HW 11
% Purpose: third order polynomial plan of 3R elbow arm
% Note   : all angles in this script are in degree

clear;
clc;

% initial data
% D-H parameters
a2 = 0.4;
a3 = 0.6;
d1 = 0.5;

% two points in {0}
P0 = [0.2 0.1 1.2];
Pf = [0.1 -0.2 0.8];

% two vectors on the arc
OcP0 = [0.2 0.1 0.2];
OcPf = [0.1 -0.2 -0.2];

% norm vector of the arc
nc = cross(OcP0,OcPf);
% normalization
n = -nc/norm(nc);

% radius of the arc
r = 0.3; 

% center of the arc
c = [0 0 1]; 

% angle parameter
theta = (0:2*pi/100:2*pi)'; 
tf = 100;
t = 0:0.1:tf;
nt = length(t);

% two orthonormal basis of n
o = null(n);
a = o(:,1);
b = o(:,2);

% parameters of center
c1 = c(1)*ones(size(theta,1),1);
c2 = c(2)*ones(size(theta,1),1);
c3 = c(3)*ones(size(theta,1),1);

% parameters of the arc
x = c1 + r*a(1)*cos(theta) + r*b(1)*sin(theta);
y = c2 + r*a(2)*cos(theta) + r*b(2)*sin(theta);
z = c3 + r*a(3)*cos(theta) + r*b(3)*sin(theta);

% constrains for theta
% initial angle
sinthi = (a(2)*P0(1) - a(1)*P0(2))/n(3);
costhi = -(b(2)*P0(1) - b(1)*P0(2))/n(3);
thetai = atan2(sinthi,costhi);
thetaid = atan2d(sinthi,costhi);

% final angle
sinthf = (a(2)*Pf(1) - a(1)*Pf(2))/n(3);
costhf = -(b(2)*Pf(1) - b(1)*Pf(2))/n(3);
thetaf = atan2(sinthf,costhf);
thetafd = atan2d(sinthf,costhf);

% let angle change continuously
if thetaf < 0
    thetaf = thetaf + 2*pi;
    thetafd = thetafd + 360;
end

% 3rd polynomial planning
% coefficients
k0 = thetaid;
k1 = 0;
k2 = 3*(thetafd-thetaid)/tf^2;
k3 = -2*(thetafd-thetaid)/tf^3;

% parameter theta under cubic polynomial planning
ctheta = k0 + k1*t + k2*t.^2 + k3*t.^3;

% initialize the Theta matrix, Theta = [theta1 theta2 theta3]
Theta = zeros(nt,3);

% initial joint angles
Theta(1,:) = [26.5651 -126.9498 87.6120];

% call invElbowArm subfunction to calculate joint angles corresponding to
% demanding end effector's position
for i = 2:nt
    
    % demand position of end effector
    xd = c(1) + r*a(1)*cosd(ctheta(i)) + r*b(1)*sind(ctheta(i));
    yd = c(2) + r*a(2)*cosd(ctheta(i)) + r*b(2)*sind(ctheta(i));
    zd = c(3) + r*a(3)*cosd(ctheta(i)) + r*b(3)*sind(ctheta(i));
    pd = [xd;yd;zd];
    
    % call invElbowArm subfunction
    Theta(i,:) = invElbowArm(Theta(i-1,:),pd);
    
end

% three joint angles
theta1 = Theta(:,1);
theta2 = Theta(:,2);
theta3 = Theta(:,3);

% forward kinematics calculation
p01 = zeros(3,nt);
p02 = zeros(3,nt);
p03 = zeros(3,nt);
for i = 1:nt
    % calculate three transform matrix
    T01 = TDH(theta1(i),-90,0,d1);
    T12 = TDH(theta2(i),0,a2,0);
    T23 = TDH(theta3(i),0,a3,0);

    % transform matrix from {0} to {2,3}
    T02 = T01*T12;
    T03 = T01*T12*T23;

    % position
    p01(:,i) = T01(1:3,4);
    p02(:,i) = T02(1:3,4);
    p03(:,i) = T03(1:3,4);
end

% take transposition
p01 = p01';
p02 = p02';
p03 = p03';

% draw joint angle
% first joint angle
figure(1)
plot(t,Theta(:,1));
xlabel('t/s');
ylabel('\theta_1/\circ');
title('Angle of Joint 1');

% second joint angle
figure(2)
plot(t,Theta(:,2));
xlabel('t/s');
ylabel('\theta_2/\circ');
title('Angle of Joint 2');

% third joint angle
figure(3)
plot(t,Theta(:,3));
xlabel('t/s');
ylabel('\theta_3/\circ');
title('Angle of Joint 3');

% draw arc
figure(4)
plot3(x,y,z,'Color','r','LineWidth',2)
xlabel('x/m')
ylabel('y/m')
zlabel('z/m')
title('The Demand Arc Path')
axis equal
axis([-0.6 0.6 -0.6 0.6 0 1.4]);
grid on
hold on
line([-0.5 0.5],[0 0 ],[0 0],'Color','b');
line([0 0],[-0.5 0.5],[0 0],'Color','b');
line([0 0],[0 0],[0 1.3],'Color','b');
line([-0.4 0.4],[0 0 ],[1 1],'Color','r');
line([0 0],[-0.2 0.2],[1 1],'Color','r');
text(0.2,0.1,1.3,'P_0');
text(0.1,-0.2,0.9,'P_f');
hold on
plot3(0.2,0.1,1.2,'Marker','*','MarkerSize',16);
hold on
plot3(0.1,-0.2,0.8,'Marker','*','MarkerSize',16);
hold on
line([0 0.2],[0 0.1],[1 1.2],'Color','#7E2F8E','LineWidth',2);
line([0 0.1],[0 -0.2],[1 0.8],'Color','#7E2F8E','LineWidth',2);
hold on
% draw plane containing the arc
[X,Y]= meshgrid(-0.5:0.01:0.5);
Z = (-n(1)*X - n(2)*Y)/n(3) + 1;
mesh(X,Y,Z);
hold on
% draw normal vector
line([0 n(1)],[0 n(2)],[1 n(3)+1],'Color','b','LineWidth',2);

% draw motion state of elbow arm
figure(5)
xdata = [zeros(nt,1) p01(:,1) p02(:,1) p03(:,1)];
ydata = [zeros(nt,1) p01(:,2) p02(:,2) p03(:,2)];
zdata = [zeros(nt,1) p01(:,3) p02(:,3) p03(:,3)];
plot3(x,y,z,'Color','g','LineWidth',2);
xlabel('x/m');
ylabel('y/m');
zlabel('z/m');
title('Motion State of 3R Elbow Arm');
axis equal
axis([-0.6 0.6 -0.6 0.6 0 1.4]);
hold on
for i = 1:nt
    if mod(i,20) == 0
        line(xdata(i,:),ydata(i,:),zdata(i,:),'Marker','*','MarkerEdgeColor','r')
    end
end
grid on
% view in 3D
view(3)

%--------------------------------------------------------------------------
%-----subfunctions---------------------------------------------------------
%--------------------------------------------------------------------------
%TDH, transform matrix for D-H method
function [T] = TDH(theta,alpha,a,d)

T = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) a*cosd(theta);
     sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) a*sind(theta);
         0            sind(alpha)             cosd(alpha)         d    ;
         0                0                      0              1   ];
     
end

% invElbowArm, 
% calculate joint angles for arbitrary end effector's position
function Theta = invElbowArm(q0,pd)
% initial data
% initial joint angle, q0, row vector
% demand position, pd, column vector
% Theta = [theta1 theta2 theta3], in degree

% D-H parameters
a2 = 0.4;
a3 = 0.6;
d1 = 0.5;

% demand position and orientation
% pd = [0.3;0.1732;-0.3];

% set allowed error
% position error
ep = 10^(-5);

% assign initial joint angles
qc(1,:) = q0;

% set initial loop number
k = 1;

% begin while loop
while 1 && k < 10000
    
    % calculate three joint angles
    theta1 = qc(k,1);
    theta2 = qc(k,2);
    theta3 = qc(k,3);
    
    % calculate three transform matrix
    T01 = TDH(theta1,-90,0,d1);
    T12 = TDH(theta2,0,a2,0);
    T23 = TDH(theta3,0,a3,0);
    
    % transform matrix from {0} to {3}
    T03 = T01*T12*T23;
    
    % elements of velocity Jacobian matrix for elbow manipulator
    nxv = -sind(theta1)*(a2*cosd(theta2)+a3*cosd(theta2+theta3));
    nyv = cosd(theta1)*(a2*cosd(theta2)+a3*cosd(theta2+theta3));
    nzv = 0;
    oxv = -cosd(theta1)*(a2*sind(theta2)+a3*sind(theta2+theta3));
    oyv = -sind(theta1)*(a2*sind(theta2)+a3*sind(theta2+theta3));
    ozv = -(a2*cosd(theta2)+a3*cosd(theta2+theta3));
    axv = -a3*cosd(theta1)*sind(theta2+theta3);
    ayv = -a3*sind(theta1)*sind(theta2+theta3);
    azv = -a3*cosd(theta2+theta3);
    
    % velocity Jacobian matrix
    Jv03 = [nxv oxv axv;
            nyv oyv ayv;
            nzv ozv azv];    
    
    % vectors of transform matrix calculated in each iteration
    pc = T03(1:3,4);
    
    % position error
    dp = pd - pc;
        
    % norm of position error
    ndp = norm(dp);    
    
    % decide if exiting the loop
    % if ndp <= ep && ndo <= eo
    if ndp <= ep
        % errors are allowable
        % exit loop
        break;
    else
        % errors are not allowable
        % increments of joint angles
        % dq = pinv(J03)*[dp;do];
        dq = Jv03\dp;
        
        % take transposition
        dqc = dq';
        
        % update joint angles
        qc(k+1,:) = qc(k,:) + dqc;
        
        % increase the loop number
        k = k + 1;
    end % w.r.t if
    
end % w.r.t while

% output results
theta1 = wrapTo180(theta1);
theta2 = wrapTo180(theta2);
theta3 = wrapTo180(theta3);

Theta = [theta1 theta2 theta3];
end



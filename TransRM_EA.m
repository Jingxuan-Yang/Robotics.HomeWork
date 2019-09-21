%Author: JingXuan Yang
%Date: 2019.03.07
%transform between rotation matrix and Euler angles
%Many sub functions are provided in this script
%Rx, Ry, Rz ----input rotation angle
%           ----output basic rotation matrix
%Rxyz, Rxzy, Ryzx, Ryxz, Rzxy, Rzyx ----first kind rotation matrix
%                                   ----input three rotation angles
%                                   ----ouput total rotation matrix
%Rxyx, Rxzx, Ryxy, Ryzy, Rzxz, Rzyz ----second kind rotation matrix
%                                   ----input three rotation angles
%                                   ----ouput total rotation matrix
%invRxyz, invRxzy, invRyzx, invRyxz, invRzxy, invRzyx 
%                            ----rotation matrix to first kind Euler angles
%                            ----input total rotation matrix
%                            ----ouput three rotation angles
%invRxyx, invRxzx, invRyxy, invRyzy, invRzxz, invRzyz 
%                           ----rotation matrix to second kind Euler angles
%                           ----input total rotation matrix
%                           ----ouput three rotation angles
%JExyz, JExzy, JEyzx, JEyxz, JEzxy, JEzyx 
%                     ----omega transform matrix of first kind Euler angles
%                     ----input first two rotation angles
%                     ----ouput omega transform matrix
%JExyx, JExzx, JEyxy, JEyzy, JEzxz, JEzyz 
%                    ----omega transform matrix of second kind Euler angles
%                    ----input first two rotation angles
%                    ----ouput omega transform matrix
%All the angles in this script are in degree,
%use dr or rd to transform if needed

dr = pi/180; %degree-radical ratio
rd = 180/pi; %radical-degree ratio

%Calculate Euler Angle for ZXY
%begin main script
%initial data

alpha = 20;
beta = -30;
gamma = 40;

aPob = [-100 400 150]';
bPpb = [-20 30 -30]';
bomegab = [5 10 -8]';

%calculating

aRb = Ryxy(alpha,beta,gamma);

%homegenerious transform matrix
aTb = [aRb aPob; 0 0 0 1];

%Pb in b
bppb = [bPpb; 1];

%Pb in a
appb = aTb*bppb;

%omega in a
aomegab = aRb*bomegab;

%transfrom matrix from Euler angular velocity to rigid angular velocity
JEulerYXY = JEyxy(alpha,beta);

inv = inv(JEulerYXY);

dPsiYXY = JEulerYXY\aomegab;

%end main script

%---------basic rotation matrix-------------------------------------------- 
function [RX] = Rx(angle)

RX = [1       0           0     ;
      0 cosd(angle) -sind(angle);
      0 sind(angle) cosd(angle)];

end

function [RY] = Ry(angle)

RY = [cosd(angle)  0 sind(angle) ;
           0       1      0      ;
      -sind(angle) 0 cosd(angle)];

end

function [RZ] = Rz(angle)

RZ = [cosd(angle) -sind(angle) 0 ;
      sind(angle) cosd(angle)  0 ;
          0           0        1];

end

%---------end--------------------------------------------------------------

%---------first kind rotation matrix---------------------------------------
% XYZ, return rotation matrix of XYX Euler angle
function [RXYZ] = Rxyz(angle1,angle2,angle3)

RXYZ = Rx(angle1)*Ry(angle2)*Rz(angle3);

end

% XZY, return rotation matrix of XZY Euler angle
function [RXZY] = Rxzy(angle1,angle2,angle3)

RXZY = Rx(angle1)*Rz(angle2)*Ry(angle3);

end

% YZX, return rotation matrix of YZX Euler angle
function [RYZX] = Ryzx(angle1,angle2,angle3)

RYZX = Ry(angle1)*Rz(angle2)*Rx(angle3);

end

% YXZ, return rotation matrix of YXZ Euler angle
function [RYXZ] = Ryxz(angle1,angle2,angle3)

RYXZ = Ry(angle1)*Rx(angle2)*Rz(angle3);

end

% ZXY, return rotation matrix of ZXY Euler angle
function [RZXY] = Rzxy(angle1,angle2,angle3)

RZXY = Rz(angle1)*Rx(angle2)*Ry(angle3);

end

% ZYX, return rotation matrix of ZYX Euler angle
function [RZYX] = Rzyx(angle1,angle2,angle3)

RZYX = Rz(angle1)*Ry(angle2)*Rx(angle3);

end

%------------end-----------------------------------------------------------

%------------second kind rotation matrix-----------------------------------
% XYX, return rotation matrix of XYX Euler angle
function [RXYX] = Rxyx(angle1,angle2,angle3)

RXYX = Rx(angle1)*Ry(angle2)*Rx(angle3);

end

% XZX, return rotation matrix of XZX Euler angle
function [RXZX] = Rxzx(angle1,angle2,angle3)

RXZX = Rx(angle1)*Rz(angle2)*Rx(angle3);

end

% YXY, return rotation matrix of YXY Euler angle
function [RYXY] = Ryxy(angle1,angle2,angle3)

RYXY = Ry(angle1)*Rx(angle2)*Ry(angle3);

end

% YZY, return rotation matrix of YZY Euler angle
function [RYZY] = Ryzy(angle1,angle2,angle3)

RYZY = Ry(angle1)*Rz(angle2)*Ry(angle3);

end

% ZXZ, return rotation matrix of ZXZ Euler angle
function [RZXZ] = Rzxz(angle1,angle2,angle3)

RZXZ = Rz(angle1)*Rx(angle2)*Rz(angle3);

end

% ZYZ, return rotation matrix of ZYZ Euler angle
function [RZYZ] = Rzyz(angle1,angle2,angle3)

RZYZ = Rz(angle1)*Ry(angle2)*Rz(angle3);

end

%-------------end----------------------------------------------------------

%-------------first kind Euler angles--------------------------------------
% xyz, output angle in degree
function [Angle] = invRxyz(Rxyz)

rd = 180/pi;

% - pi to pi
beta1 = asin(Rxyz(1,3));

    if beta1 > 0
        beta2 = pi - beta1;
    else
        beta2 = - pi - beta1;
    end
    
alpha1 = atan2(-Rxyz(2,3)/cos(beta1),Rxyz(3,3)/cos(beta1));
gamma1 = atan2(-Rxyz(1,2)/cos(beta1),Rxyz(1,1)/cos(beta1));

alpha2 = atan2(-Rxyz(2,3)/cos(beta2),Rxyz(3,3)/cos(beta2));
gamma2 = atan2(-Rxyz(1,2)/cos(beta2),Rxyz(1,1)/cos(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% xzy, output angle in degree
function [Angle] = invRxzy(Rxzy)

rd = 180/pi;

% - pi to pi
beta1 = asin(-Rxzy(1,2));

    if beta1 > 0
        beta2 = pi - beta1;
    else
        beta2 = - pi - beta1;
    end
    
alpha1 = atan2(Rxzy(3,2)/cos(beta1),Rxzy(2,2)/cos(beta1));
gamma1 = atan2(Rxzy(1,3)/cos(beta1),Rxzy(1,1)/cos(beta1));

alpha2 = atan2(Rxzy(3,2)/cos(beta2),Rxzy(2,2)/cos(beta2));
gamma2 = atan2(Rxzy(1,3)/cos(beta2),Rxzy(1,1)/cos(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% yzx, output angle in degree
function [Angle] = invRyzx(Ryzx)

rd = 180/pi;

% - pi to pi
beta1 = asin(Ryzx(2,1));

    if beta1 > 0
        beta2 = pi - beta1;
    else
        beta2 = - pi - beta1;
    end
    
alpha1 = atan2(-Ryzx(3,1)/cos(beta1),Ryzx(1,1)/cos(beta1));
gamma1 = atan2(-Ryzx(2,3)/cos(beta1),Ryzx(2,2)/cos(beta1));

alpha2 = atan2(-Ryzx(3,1)/cos(beta2),Ryzx(1,1)/cos(beta2));
gamma2 = atan2(-Ryzx(2,3)/cos(beta2),Ryzx(2,2)/cos(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% yxz, output angle in degree
function [Angle] = invRyxz(Ryxz)

rd = 180/pi;

% - pi to pi
beta1 = asin(-Ryxz(2,3));

    if beta1 > 0
        beta2 = pi - beta1;
    else
        beta2 = - pi - beta1;
    end
    
alpha1 = atan2(Ryxz(1,3)/cos(beta1),Ryxz(3,3)/cos(beta1));
gamma1 = atan2(Ryxz(2,1)/cos(beta1),Ryxz(2,2)/cos(beta1));

alpha2 = atan2(Ryxz(1,3)/cos(beta2),Ryxz(3,3)/cos(beta2));
gamma2 = atan2(Ryxz(2,1)/cos(beta2),Ryxz(2,2)/cos(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% zxy, output angle in degree
function [Angle] = invRzxy(Rzxy)

rd = 180/pi;

% - pi to pi
beta1 = asin(Rzxy(3,2));

    if beta1 > 0
        beta2 = pi - beta1;
    else
        beta2 = - pi - beta1;
    end
    
alpha1 = atan2(-Rzxy(1,2)/cos(beta1),Rzxy(2,2)/cos(beta1));
gamma1 = atan2(-Rzxy(3,1)/cos(beta1),Rzxy(3,3)/cos(beta1));

alpha2 = atan2(-Rzxy(1,2)/cos(beta2),Rzxy(2,2)/cos(beta2));
gamma2 = atan2(-Rzxy(3,1)/cos(beta2),Rzxy(3,3)/cos(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% zyx, output angle in degree
function [Angle] = invRzyx(Rzyx)

rd = 180/pi;

% - pi to pi
beta1 = asin(-Rzyx(3,1));

    if beta1 > 0
        beta2 = pi - beta1;
    else
        beta2 = - pi - beta1;
    end
    
alpha1 = atan2(Rzyx(2,1)/cos(beta1),Rzyx(1,1)/cos(beta1));
gamma1 = atan2(Rzyx(3,2)/cos(beta1),Rzyx(3,3)/cos(beta1));

alpha2 = atan2(Rzyx(2,1)/cos(beta2),Rzyx(1,1)/cos(beta2));
gamma2 = atan2(Rzyx(3,2)/cos(beta2),Rzyx(3,3)/cos(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

%-------------end----------------------------------------------------------

%-------------second kind Euler angles-------------------------------------
% xyx, output angle in degree
function [Angle] = invRxyx(Rxyx)

rd = 180/pi;

% - pi to pi
beta1 = acos(Rxyx(1,1));
beta2 = -acos(Rxyx(1,1));
    
alpha1 = atan2(Rxyx(2,1)/sin(beta1),-Rxyx(3,1)/sin(beta1));
gamma1 = atan2(Rxyx(1,2)/sin(beta1),Rxyx(1,3)/sin(beta1));

alpha2 = atan2(Rxyx(2,1)/sin(beta2),-Rxyx(3,1)/sin(beta2));
gamma2 = atan2(Rxyx(1,2)/sin(beta2),Rxyx(1,3)/sin(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% xzx, output angle in degree
function [Angle] = invRxzx(Rxzx)

rd = 180/pi;

% - pi to pi
beta1 = acos(Rxzx(1,1));
beta2 = -acos(Rxzx(1,1));
    
alpha1 = atan2(Rxzx(3,1)/sin(beta1),Rxzx(2,1)/sin(beta1));
gamma1 = atan2(Rxzx(1,3)/sin(beta1),-Rxzx(1,2)/sin(beta1));

alpha2 = atan2(Rxzx(3,1)/sin(beta2),Rxzx(2,1)/sin(beta2));
gamma2 = atan2(Rxzx(1,3)/sin(beta2),-Rxzx(1,2)/sin(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% yxy, output angle in degree
function [Angle] = invRyxy(Ryxy)

rd = 180/pi;

% - pi to pi
beta1 = acos(Ryxy(2,2));
beta2 = -acos(Ryxy(2,2));
    
alpha1 = atan2(Ryxy(1,2)/sin(beta1),Ryxy(3,2)/sin(beta1));
gamma1 = atan2(Ryxy(2,1)/sin(beta1),-Ryxy(2,3)/sin(beta1));

alpha2 = atan2(Ryxy(1,2)/sin(beta2),Ryxy(3,2)/sin(beta2));
gamma2 = atan2(Ryxy(2,1)/sin(beta2),-Ryxy(2,3)/sin(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% yzy, output angle in degree
function [Angle] = invRyzy(Ryzy)

rd = 180/pi;

% - pi to pi
beta1 = acos(Ryzy(2,2));
beta2 = -acos(Ryzy(2,2));
    
alpha1 = atan2(Ryzy(3,2)/sin(beta1),-Ryzy(1,2)/sin(beta1));
gamma1 = atan2(Ryzy(2,3)/sin(beta1),Ryzy(2,1)/sin(beta1));

alpha2 = atan2(Ryzy(3,2)/sin(beta2),-Ryzy(1,2)/sin(beta2));
gamma2 = atan2(Ryzy(2,3)/sin(beta2),Ryzy(2,1)/sin(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% zxz, output angle in degree
function [Angle] = invRzxz(Rzxz)

rd = 180/pi;

% - pi to pi
beta1 = acos(Rzxz(3,3));
beta2 = -acos(Rzxz(3,3));
    
alpha1 = atan2(Rzxz(1,3)/sin(beta1),-Rzxz(2,3)/sin(beta1));
gamma1 = atan2(Rzxz(3,1)/sin(beta1),Rzxz(3,2)/sin(beta1));

alpha2 = atan2(Rzxz(1,3)/sin(beta2),-Rzxz(2,3)/sin(beta2));
gamma2 = atan2(Rzxz(3,1)/sin(beta2),Rzxz(3,2)/sin(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

% zyz, output angle in degree
function [Angle] = invRzyz(Rzyz)

rd = 180/pi;

% - pi to pi
beta1 = acos(Rzyz(3,3));
beta2 = -acos(Rzyz(3,3));
    
alpha1 = atan2(Rzyz(2,3)/sin(beta1),Rzyz(1,3)/sin(beta1));
gamma1 = atan2(Rzyz(3,2)/sin(beta1),-Rzyz(3,1)/sin(beta1));

alpha2 = atan2(Rzyz(2,3)/sin(beta2),Rzyz(1,3)/sin(beta2));
gamma2 = atan2(Rzyz(3,2)/sin(beta2),-Rzyz(3,1)/sin(beta2));

cases = ['case1'; 'case2'];
alpha = [alpha1*rd; alpha2*rd];
beta = [beta1*rd; beta2*rd];
gamma = [gamma1*rd; gamma2*rd];

Angle = table(cases,alpha,beta,gamma);

end

%----------------end-------------------------------------------------------

%---------first kind omega transform matrix--------------------------------

function [JEuler] = JExyz(angle1,angle2)

aR2 = Rx(angle1);
aR3 = Rx(angle1)*Ry(angle2);

JEuler = [aR2(:,1) aR2(:,2) aR3(:,3)];

end

function [JEuler] = JExzy(angle1,angle2)

aR2 = Rx(angle1);
aR3 = Rx(angle1)*Rz(angle2);

JEuler = [aR2(:,1) aR2(:,3) aR3(:,2)];

end

function [JEuler] = JEyzx(angle1,angle2)

aR2 = Ry(angle1);
aR3 = Ry(angle1)*Rz(angle2);

JEuler = [aR2(:,2) aR2(:,3) aR3(:,1)];

end

function [JEuler] = JEyxz(angle1,angle2)

aR2 = Ry(angle1);
aR3 = Ry(angle1)*Rx(angle2);

JEuler = [aR2(:,2) aR2(:,1) aR3(:,3)];

end

function [JEuler] = JEzxy(angle1,angle2)

aR2 = Rz(angle1);
aR3 = Rz(angle1)*Rx(angle2);

JEuler = [aR2(:,3) aR2(:,1) aR3(:,2)];

end

function [JEuler] = JEzyx(angle1,angle2)

aR2 = Rz(angle1);
aR3 = Rz(angle1)*Ry(angle2);

JEuler = [aR2(:,3) aR2(:,2) aR3(:,1)];

end

%---------end--------------------------------------------------------------

%---------second kind omega transform matrix-------------------------------

function [JEuler] = JExyx(angle1,angle2)

aR2 = Rx(angle1);
aR3 = Rx(angle1)*Ry(angle2);

JEuler = [aR2(:,1) aR2(:,2) aR3(:,1)];

end

function [JEuler] = JExzx(angle1,angle2)

aR2 = Rx(angle1);
aR3 = Rx(angle1)*Rz(angle2);

JEuler = [aR2(:,1) aR2(:,3) aR3(:,1)];

end

function [JEuler] = JEyxy(angle1,angle2)

aR2 = Ry(angle1);
aR3 = Ry(angle1)*Rx(angle2);

JEuler = [aR2(:,2) aR2(:,1) aR3(:,2)];

end

function [JEuler] = JEyzy(angle1,angle2)

aR2 = Ry(angle1);
aR3 = Ry(angle1)*Rz(angle2);

JEuler = [aR2(:,2) aR2(:,3) aR3(:,2)];

end

function [JEuler] = JEzxz(angle1,angle2)

aR2 = Rz(angle1);
aR3 = Rz(angle1)*Rx(angle2);

JEuler = [aR2(:,3) aR2(:,1) aR3(:,3)];

end

function [JEuler] = JEzyz(angle1,angle2)

aR2 = Rz(angle1);
aR3 = Rz(angle1)*Ry(angle2);

JEuler = [aR2(:,3) aR2(:,2) aR3(:,3)];

end

%---------end--------------------------------------------------------------

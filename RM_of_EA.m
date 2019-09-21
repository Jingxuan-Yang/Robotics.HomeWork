%Author: JingXuan Yang
%Date: 2019.03.07
%Symbol calculation for correcting the derivation of fomulas
%RXYZ,RXZY,..., RZYZ 
%                        ---- rotation matrix for each type of Euler angles
%JEulerXYZ, JEulerXZY,...,JEulerZYZ
%                  ----omega transform matrix for each type of Euler angles
%invJEulerXYZ, invJEulerXZY,...,invJEulerZYZ
%          ----inverse omega transform matrix for each type of Euler angles

syms alpha beta gamma

%first kind Euler angles
RXYZ = Rx(alpha)*Ry(beta)*Rz(gamma);
RXZY = Rx(alpha)*Rz(beta)*Ry(gamma);

RYZX = Ry(alpha)*Rz(beta)*Rx(gamma);
RYXZ = Ry(alpha)*Rx(beta)*Rz(gamma);

RZXY = Rz(alpha)*Rx(beta)*Ry(gamma);
RZYX = Rz(alpha)*Ry(beta)*Rx(gamma);

%second kind Euler angles
RXYX = Rx(alpha)*Ry(beta)*Rx(gamma);
RXZX = Rx(alpha)*Rz(beta)*Rx(gamma);

RYXY = Ry(alpha)*Rx(beta)*Ry(gamma);
RYZY = Ry(alpha)*Rz(beta)*Ry(gamma);

RZXZ = Rz(alpha)*Rx(beta)*Rz(gamma);
RZYZ = Rz(alpha)*Ry(beta)*Rz(gamma);

%first kind omega transfrom matrix
JEulerXYZ = JExyz(alpha, beta);
invJEulerXYZ = inv(JEulerXYZ);

JEulerXZY = JExzy(alpha, beta);
invJEulerXZY = inv(JEulerXZY);

JEulerYZX = JEyzx(alpha, beta);
invJEulerYZX = inv(JEulerYZX);

JEulerYXZ = JEyxz(alpha, beta);
invJEulerYXZ = inv(JEulerYXZ);

JEulerZXY = JEzxy(alpha, beta);
invJEulerZXY = inv(JEulerZXY);

JEulerZYX = JEzyx(alpha, beta);
invJEulerZYX = inv(JEulerZYX);

%second kind omega transform matrix
JEulerXYX = JExyx(alpha, beta);
invJEulerXYX = inv(JEulerXYX);

JEulerXZX = JExzx(alpha, beta);
invJEulerXZX = inv(JEulerXZX);

JEulerYXY = JEyxy(alpha, beta);
invJEulerYXY = inv(JEulerYXY);

JEulerYZY = JEyzy(alpha, beta);
invJEulerYZY = inv(JEulerYZY);

JEulerZXZ = JEzxz(alpha, beta);
invJEulerZXZ = inv(JEulerZXZ);

JEulerZYZ = JEzyz(alpha, beta);
invJEulerZYZ = inv(JEulerZYZ);

%---------basic rotation matrix--------------------------------------------
function [RX] = Rx(angle)

RX = [1      0          0     ;
      0 cos(angle) -sin(angle);
      0 sin(angle) cos(angle)];

end

function [RY] = Ry(angle)

RY = [cos(angle)  0 sin(angle) ;
           0      1      0     ;
      -sin(angle) 0 cos(angle)];

end

function [RZ] = Rz(angle)

RZ = [cos(angle) -sin(angle) 0 ;
      sin(angle) cos(angle)  0 ;
         0          0        1];

end

%---------end--------------------------------------------------------------

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

%Author: JingXuan Yang
%Date: 2019.03.12
%Test for two formation of inverse matrix for 3DOF

% syms l1 l2 l3 t1 t2 t3
l1 = 1;
l2 = 3;
l3 = 5;
t1 = 10;
t2 = 11;
t3 = 12;

s1 = sind(t1);
c1 = cosd(t1);
s2 = sind(t2);
c2 = cosd(t2);
s3 = sind(t3);
c3 = cosd(t3);
s12 = sind(t1-t2);
c12 = cosd(t1-t2);
s123 = sind(t3+t1-t2);
c123 = cosd(t3+t1-t2);
s23 = sind(t3-t2);
c23 = cosd(t3-t2);

J3 = [-l1*s1-l2*s12-l3*s123   l2*s12+l3*s123   -l3*s123;
      l1*c1+l2*c12+l3*c123    -l2*c12-l3*c123  l3*c123 ;
               1                    -1            1   ];

invJ3 = inv(J3);
detJ3 = det(J3);

invJ3s = [  c12*l2         s12*l2          s3*l2*l3      ;
          c1*l1+c12*l2  s1*l1+s12*l2  s23*l1*l3+s3*l2*l3 ;
             c1*l1         s1*l1     -s2*l1*l2+s23*l1*l3];
      
invsJ3 = -invJ3s/detJ3;
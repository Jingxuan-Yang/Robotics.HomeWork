%Author: JingXuan Yang
%Date: 2019.03.12
%Symbol calculation for inverse kinematics of 2 and 3 DOF

syms l1 l2 l3 s1 c1 s12 c12 s123 c123

J3 = [-l1*s1-l2*s12-l3*s123   l2*s12+l3*s123   -l3*s123;
      l1*c1+l2*c12+l3*c123    -l2*c12-l3*c123  l3*c123 ;
               1                    -1            1   ];

J2 = [-l1*s1-l2*s12  -l2*s12;
      l1*c1+l2*c12   l2*c12];

invJ3 = inv(J3);
detJ3 = det(J3);

invJ2 = inv(J2);
detJ2 = det(J2);




syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 

a2 = 2;
a3 = 2;

d1 = 0.5;
d2 = 0.5;
d3 = 0.5;
d4 = 0.5;
d5 = 0;
d6 = 0.5;
d234 = d2+d3+d4;

alpha1 = 90;
alpha2 = 0;
alpha3 = 0;
alpha4 = 90;
alpha5 = 90;
alpha6 = 0;

dr = pi/180;

nx = -0.2213;
ny = -0.8710;
nz = -0.4386;
ox = -0.1789;
oy = -0.4058;
oz = 0.8963;
ax = -0.9586;
ay = 0.2768;
az = -0.0660;
px = 3.0;
py = 1.2;
pz = 0.6;

Tk = [-0.2213 -0.1789 -0.9586 3.0;
     -0.8710 -0.4058 0.2768  1.2;
     -0.4386 0.8963 -0.0660  0.6;
         0      0      0      1];

R06 = [-0.2213 -0.1789 -0.9586 ;
       -0.8710 -0.4058  0.2768 ;
       -0.4386  0.8963 -0.0660];

T02 = T01*T12;
T03 = T02*T23;
T04 = T03*T34;
T05 = T04*T45;
T06 = T05*T56;
T36 = T34*T45*T56;

th31 = acosd(((px - d6*ax)^2+(py-d6*ay)^2+(pz-d6*az-d1)^2-d234^2-a2^2-a3^2) ...
            /(2*a2*a3));
th32 = -th31;

C1 = pz-d6*az-d1;
A1 = a2+a3*cosd(th31);
B1 = a3*sind(th31);
sB1 = sqrt((a2+a3*cosd(th31))^2+(a3*sind(th31))^2);
phi1 = atan2d(B1,A1);

th21 = asind(C1/sB1) - phi1;
th22 = 180 - asind(C1/sB1) - phi1;

C2 = pz-d6*az-d1;
A2 = a2+a3*cosd(th32);
B2 = a3*sind(th32);
sB2 = sqrt((a2+a3*cosd(th32))^2+(a3*sind(th32))^2);
phi2 = atan2d(B2,A2);

th23 = asind(C2/sB2) - phi2;
th24 = 180 - asind(C2/sB2) - phi2;

%1
c21 = cosd(th21);
c2311 = cosd(th21+th31);

c11 = (px-d6*ax)*(a2*c21+a3*c2311)-(py-d6*ay)*d234;
s11 = (px-d6*ax)*d234+(py-d6*ay)*(a2*c21+a3*c2311);
th11 = atan2d(s11,c11);

%2
c22 = cosd(th22);
c2312 = cosd(th22+th31);

c12 = (px-d6*ax)*(a2*c22+a3*c2312)-(py-d6*ay)*d234;
s12 = (px-d6*ax)*d234+(py-d6*ay)*(a2*c22+a3*c2312);
th12 = atan2d(s12,c12);

%3
c23 = cosd(th23);
c2321 = cosd(th23+th32);

c13 = (px-d6*ax)*(a2*c23+a3*c2321)-(py-d6*ay)*d234;
s13 = (px-d6*ax)*d234+(py-d6*ay)*(a2*c23+a3*c2321);
th13 = atan2d(s13,c13);

%4
c24 = cosd(th24);
c2322 = cosd(th24+th32);

c14 = (px-d6*ax)*(a2*c24+a3*c2322)-(py-d6*ay)*d234;
s14 = (px-d6*ax)*d234+(py-d6*ay)*(a2*c24+a3*c2322);
th14 = atan2d(s14,c14);

%456-1
R011 = RDH(th11,alpha1);
R121 = RDH(th21,alpha2);
R231 = RDH(th31,alpha3);
R031 = R011*R121*R231;
R361 = R031\R06;
nz361 = R361(3,1);
oz361 = R361(3,2);
ax361 = R361(1,3);
ay361 = R361(2,3);
az361 = R361(3,3);

th511 = acosd(az361);
th512 = -th511;
th411 = atan2d(ay361*sind(th511),ax361*sind(th511));
th412 = atan2d(ay361*sind(th512),ax361*sind(th512));
th611 = atan2d(-oz361*sind(th511),nz361*sind(th511));
th612 = atan2d(-oz361*sind(th512),nz361*sind(th512));

%456-2
R012 = RDH(th12,alpha1);
R122 = RDH(th22,alpha2);
R232 = RDH(th31,alpha3);
R032 = R012*R122*R232;
R362 = R032\R06;
nz362 = R362(3,1);
oz362 = R362(3,2);
ax362 = R362(1,3);
ay362 = R362(2,3);
az362 = R362(3,3);

th521 = acosd(az362);
th522 = -th521;
th421 = atan2d(ay362*sind(th521),ax362*sind(th521));
th422 = atan2d(ay362*sind(th522),ax362*sind(th522));
th621 = atan2d(-oz362*sind(th521),nz362*sind(th521));
th622 = atan2d(-oz362*sind(th522),nz362*sind(th522));

%456-3
R013 = RDH(th13,alpha1);
R123 = RDH(th23,alpha2);
R233 = RDH(th32,alpha3);
R033 = R013*R123*R233;
R363 = R033\R06;
nz363 = R363(3,1);
oz363 = R363(3,2);
ax363 = R363(1,3);
ay363 = R363(2,3);
az363 = R363(3,3);

th531 = acosd(az363);
th532 = -th531;
th431 = atan2d(ay363*sind(th531),ax363*sind(th531));
th432 = atan2d(ay363*sind(th532),ax363*sind(th532));
th631 = atan2d(-oz363*sind(th531),nz363*sind(th531));
th632 = atan2d(-oz363*sind(th532),nz363*sind(th532));

%456-4
R014 = RDH(th14,alpha1);
R124 = RDH(th24,alpha2);
R234 = RDH(th32,alpha3);
R034 = R013*R123*R233;
R364 = R034\R06;
nz364 = R364(3,1);
oz364 = R364(3,2);
ax364 = R364(1,3);
ay364 = R364(2,3);
az364 = R364(3,3);

th541 = acosd(az364);
th542 = -th541;
th441 = atan2d(ay364*sind(th541),ax364*sind(th541));
th442 = atan2d(ay364*sind(th542),ax364*sind(th542));
th641 = atan2d(-oz364*sind(th541),nz364*sind(th541));
th642 = atan2d(-oz364*sind(th542),nz364*sind(th542));

%----------
Tc1 = Trt(th11,th21,th31,th411,th511,th611);

%--------------------------------------------------------------------------
function [T] = TDH(theta,alpha,a,d)

T = [cos(theta) -sin(theta)*cos(alpha) sin(theta)*sin(alpha) a*cos(theta);
     sin(theta) cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
         0            sin(alpha)             cos(alpha)            d     ;
         0               0                      0                  1    ];

end

%in degree
%--------------------------------------------------------------------------
function [R] = RDH(theta,alpha)

R = [cosd(theta) -sind(theta)*cosd(alpha) sind(theta)*sind(alpha) ;
     sind(theta) cosd(theta)*cosd(alpha) -cosd(theta)*sind(alpha) ;
         0            sind(alpha)             cosd(alpha)      ];

end

%--------------------------------------------------------------------------
function [T06] = Trt(theta1,theta2,theta3,theta4,theta5,theta6)

    a2 = 2;
    a3 = 2;

    d1 = 0.5;
    d2 = 0.5;
    d3 = 0.5;
    d4 = 0.5;
    d6 = 0.5;
    T01 = TDH(theta1,pi/2,0,d1);
    T12 = TDH(theta2,0,a2,d2);
    T23 = TDH(theta3,0,a3,d3);
    T34 = TDH(theta4,pi/2,0,d4);
    T45 = TDH(theta5,pi/2,0,0);
    T56 = TDH(theta6,0,0,d6);
    
    T02 = T01*T12;
    T03 = T02*T23;
    T04 = T03*T34;
    T05 = T04*T45;
    T06 = T05*T56;

end

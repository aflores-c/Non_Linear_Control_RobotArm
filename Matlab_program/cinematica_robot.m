function [x,Ja]=cinematica_robot(q)

%Cálculo del vector x y la matriz jacobiana analítica
%del robot
%x=[posicion; angulos_de_euler]
%qi en radianes

% t0=q(1);
% d0=0.792;
% a0=0.0;
% al0=0;

t1=q(1);
d1=0.078+0.792;
a1=0.1967;
al1=-pi/2;

t2=q(2)-pi/2;
d2=0.0;
a2=0.3481 ;
al2=0.0;

t3=q(3)+pi/2;
d3=0.0;
a3=0.4533;
al3=0;

t4=q(4);
d4=0.0;
a4=0.0;
al4=-pi/2;

t5=q(5);
d5=0.177;
a5=0;
al5=0.0;

A1=matrizAi(t1,d1,a1,al1);
A2=matrizAi(t2,d2,a2,al2);
A3=matrizAi(t3,d3,a3,al3);
A4=matrizAi(t4,d4,a4,al4);
A5=matrizAi(t5,d5,a5,al5);

T5=A1*A2*A3*A4*A5;

posicion=T5(1:3,4);
Ro = T5(1:3,1:3);

if ((Ro(1,3)==0)&(Ro(2,3)==0)),
    disp('configuración singular')
end
    
% te=atan2(sqrt(1-Ro(3,3)^2),Ro(3,3));
% fi=atan2(Ro(2,3)/sin(te),Ro(1,3)/sin(te));
% psi=atan2(Ro(3,2)/sin(te),-Ro(3,1)/sin(te));
euler= rotm2eul(Ro,'ZYZ');
fi = euler(1);
te = euler(2);
psi = euler(3);

ang_euler = [fi;te;psi];

x=[posicion; ang_euler];

O0 = [0;0;0];
O1 = A1*[0;0;0;1];  %coordenadas de O1
O1 = O1(1:3,1);
O2 = A1*A2*[0;0;0;1];
O2 = O2(1:3,1);
O3 = A1*A2*A3*[0;0;0;1];
O3 = O3(1:3,1);
O4 = A1*A2*A3*A4*[0;0;0;1];
O4 = O4(1:3,1);
O5 = A1*A2*A3*A4*A5*[0;0;0;1];
O5 = O5(1:3,1);


z0 = [0;0;1];
z1 = A1*[0;0;1;0];  %se extrae la tercera columna de R1
z1 = z1(1:3,1);
z2 = A1*A2*[0;0;1;0];  %se extrae la tercera columna de R1
z2 = z2(1:3,1);
z3 = A1*A2*A3*[0;0;1;0];  %se extrae la tercera columna de R1
z3 = z3(1:3,1);
z4 = A1*A2*A3*A4*[0;0;1;0];  %se extrae la tercera columna de R1
z4 = z4(1:3,1);


Jv =[cross(z0,(O5-O0))  cross(z1,(O5-O1))  cross(z2,(O5-O2))  cross(z3,(O5-O3))   cross(z4,(O5-O4))];
Jw =[z0  z1  z2  z3  z4];

J = [Jv;Jw];

B= [0  -sin(fi)  cos(fi)*sin(te)
    0  cos(fi)   sin(fi)*sin(te)
    1    0        cos(te)       ];

BBi = [eye(3)   zeros(3)
      zeros(3) inv(B)  ];
  
Ja = BBi*J;

% Make sure to have the server side running in V-REP: 
% in a child script of a V-REP scene, add following command
% to be executed just once, at simulation start:
%
% simRemoteApi.start(19999)
%
% then start simulation, and run this program.
%
% IMPORTANT: for each successful call to simxStart, there
% should be a corresponding call to simxFinish at the end!
clc
close all;
clear all;

%Parámetros de Denavit-Hartenberg
%t1=q1;  %variable
d1=0.078+0.792;
a1=0.1967;
al1=-pi/2;

%t2=q2-pi/2; %variable
d2=0.0;
a2=0.3481 ;
al2=0.0;

%t3=q3 + pi/2;   %variable
d3=0.0;
a3=0.4533;
al3=0;

%t4=q4;  %variable
d4=0.0;
a4=0.0;
al4=-pi/2;

%t5=q5;   %variable
d5=0.177;
a5=0;
al5=0.0;

%Orientación a alcanzar en el plano XY
gama = 30*pi/180;

%Posición a alcanzar
O =[0.6093;0.3518;1.1145];

%Cálculo de Oc
Oc = [O(1),O(2),O(3) + d5]

%Inversa de posición
t1 = atan2(Oc(2),Oc(1));
t1g=t1*180/pi;

r=sqrt(Oc(1)^2+Oc(2)^2)-a1;
s=Oc(3)-d1;
D = (r^2+s^2-a2^2-a3^2)/(2*a2*a3)
beta=atan2(-sqrt(1-D^2) , D);
alpha=(atan2(s,r)-atan2(a3*sin(beta) , a2+a3*cos(beta)));

alphag=alpha*180/pi;
betag=beta*180/pi;

%Inversa de orientación

% A1=matrizAi(t1,d1,a1,al1);
% A2=matrizAi(t2,d2,a2,al2);
% A3=matrizAi(t3,d3,a3,al3);

% T0_3 = A1*A2*A3
% R0_3 = T0_3(1:3,1:3);
% Ro = R0_3'*R;
% 
% if ((Ro(1,3)==0)&(Ro(2,3)==0)),
%     disp('configuración singular')
% end


t5 = t1 - gama;
t5g=t5*180/pi;

%Comprobando
% A4=matrizAi(t4,d4,a4,al4);
% A5=matrizAi(t5,d5,a5,al5);
 
% H=A1*A2*A3*A4*A5
% 
% O5=[0;0;0];
% P=[O5;1];
% P5_0=H*P;

q1=t1g
q2=90-alphag
q3=-betag-90
q4=-q2-q3
q5=t5g


disp('Program started');
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connected to remote API server');
    
    
    [returnCode,handle1]=vrep.simxGetObjectHandle(clientID,'Revolute_joint',vrep.simx_opmode_blocking );
    [returnCode,handle2]=vrep.simxGetObjectHandle(clientID,'Revolute_joint0',vrep.simx_opmode_blocking );
    [returnCode,handle3]=vrep.simxGetObjectHandle(clientID,'Revolute_joint1',vrep.simx_opmode_blocking );
    [returnCode,handle4]=vrep.simxGetObjectHandle(clientID,'Revolute_joint2',vrep.simx_opmode_blocking );
    [returnCode,handle5]=vrep.simxGetObjectHandle(clientID,'Revolute_joint3',vrep.simx_opmode_blocking );
    
   
     %Enviamos la posicion a una articulacion
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,q1*pi/180,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,q2*pi/180,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,q3*pi/180,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,q4*pi/180,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,q5*pi/180,vrep.simx_opmode_blocking);

    pause(0.1)
    % Now send some data to V-REP in a non-blocking fashion:
    vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);
    
    % Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    vrep.simxGetPingTime(clientID);
    
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

disp('Program ended');
%end
%Control Cinemático de un robot ABB
clc
close all;
clear all;

%Orientación a alcanzar
fi=pi/2;
te=pi;
psi=-pi/2;

z_base = 0.792;
%Posición a alcanzar
%O =[0.7;-0.3;0.09];
O =[0.7;0.0;0.7-z_base];

q = [30*pi/180, 10*pi/180, -20*pi/180, 10*pi/180, 0];
q_m = [0,0,0,0,0]
disp('Iniciar conexión');
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
    
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,q(1),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,q(2),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,q(3),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,q(4),vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,q(5),vrep.simx_opmode_blocking);
     pause(4);
    [returnCode,q_m(1)]=vrep.simxGetJointPosition(clientID,handle1,vrep.simx_opmode_blocking);
    [returnCode,q_m(2)]=vrep.simxGetJointPosition(clientID,handle2,vrep.simx_opmode_blocking);
    [returnCode,q_m(3)]=vrep.simxGetJointPosition(clientID,handle3,vrep.simx_opmode_blocking);
    [returnCode,q_m(4)]=vrep.simxGetJointPosition(clientID,handle4,vrep.simx_opmode_blocking);
    [returnCode,q_m(5)]=vrep.simxGetJointPosition(clientID,handle5,vrep.simx_opmode_blocking);
    q_m*180/pi
    [x,Ja]=cinematica_robot(q);
    x
    
    vrep.simxGetPingTime(clientID);
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end;
vrep.delete(); % call the destructor!

disp('Program ended'); 
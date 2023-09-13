%Control Cinemático de un robot ABB
clc
close all;
clear all;

%Orientación a alcanzar
 fi=0;
 te=-pi;
 psi=150*pi/180;

%Posición a alcanzar
%O =[0.7;-0.3;0.09];
O =[0.6093;0.3518;1.1145];

xd=[O;fi;te;psi];
xd_p=zeros(6,1);

%Periodo de muestreo
T=0.5;

%Parámetros del controlador
Kp=0.01

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
    
    for t=0:T:10
        [returnCode,q(1)]=vrep.simxGetJointPosition(clientID,handle1,vrep.simx_opmode_blocking);
        [returnCode,q(2)]=vrep.simxGetJointPosition(clientID,handle2,vrep.simx_opmode_blocking);
        [returnCode,q(3)]=vrep.simxGetJointPosition(clientID,handle3,vrep.simx_opmode_blocking);
        [returnCode,q(4)]=vrep.simxGetJointPosition(clientID,handle4,vrep.simx_opmode_blocking);
        [returnCode,q(5)]=vrep.simxGetJointPosition(clientID,handle5,vrep.simx_opmode_blocking);
        q
        [x,Ja]=cinematica_robot(q);
        x
        JaT = Ja'*inv(Ja*Ja');
        
        q_p = JaT*(xd_p+Kp*(xd-x))
       
        for i=1:5
            if q_p(i)>pi/2
                q_p(i)=pi/2;
            end;
            if q_p(i)<-pi/2
                q_p(i)=-pi/2;
            end;
        end;
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle1,q_p(1),vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle2,q_p(2),vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle3,q_p(3),vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle4,q_p(4),vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle5,q_p(5),vrep.simx_opmode_blocking);
        %pause(0.050)
    end;
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle1,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle2,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle3,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle4,0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle5,0,vrep.simx_opmode_blocking);
    
    vrep.simxFinish(clientID);
end;
vrep.delete(); % call the destructor!

disp('Program ended');
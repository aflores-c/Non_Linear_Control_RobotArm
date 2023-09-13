%Control Cinemático de un robot ABB
clc
close all;
clear all;



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
    
    t0=clock;
    %Posición a alcanzar
    Oi =[0.65;0.0;1.0411];
    Of =[0.7;0.0;1.0411];
    gama = 0*pi/180;
    xd=[Oi;gama];
    xd_p=zeros(6,1);
    %Parámetros del controlador
    Kp=0.05
    %Periodo de muestreo
    T=0.6;

   
    vd=0.005;
    am=0.02;
    s=0;
    s_p=0;
    
    for t=0:T:50
        
        %Trayectoria deseada
        sd=norm(Of-Oi);
        
        T0 = sqrt(sd/am);
        if (am*T0)<=vd,
            if t<=T0,
                s_p0=0;
                s0=0;
                s_2p=am;
                s_p=s_p0+am*t;
                s = s0+s_p0*t+1/2*am*t^2;
            end;
            if (T0<t)&&(t<=2*T0),
                s_p0=am*T0;
                s0=1/2*am*T0^2;
                s_2p=-am;
                s_p= s_p0-am*(t-T0);
                s = s0 +s_p0*(t-T0)-1/2*am*(t-T0)^2;
            end;
            if t>2*T0,
                s_2p=0; s_p=0; s=sd;
            end;
        end;
        if (am*T0)>vd,
            T1=vd/am;
            T2=sd/vd-T1;
            if t<=T1,
                s_p0=0;
                s0=0;
                s_2p=am;
                s_p=s_p0+am*t;
                s = s0+s_p0*t+1/2*am*t^2;
            end;
            if (T1<t)&&(t<=(T1+T2)),
                s0=T1*vd/2;
                s_2p=0; 
                s_p=vd;
                s =s0+ vd*(t-T1);
            end;
            if ((T1+T2)<t)&&(t<=(2*T1+T2)),
                s_p0=vd;
                s0=T1*vd/2+T2*vd;
                s_2p=-am;
                s_p=s_p0-am*(t-T1-T2);
                s = s0+s_p0*(t-T1-T2)-1/2*am*(t-T1-T2)^2;
            end;
            if t>(2*T1+T2),
                s_2p=0; s_p=0; s=sd;
            end;
        end;
%         s=s+T*s_p;
%         s_p=s_p+T*s_2p;
        
        xd(1:3,1)=Oi+s*(Of-Oi)/norm(Of-Oi);
        xd_p(1:3,1)=s_p*(Of-Oi)/norm(Of-Oi);

    
        [returnCode,q(1)]=vrep.simxGetJointPosition(clientID,handle1,vrep.simx_opmode_blocking);
        [returnCode,q(2)]=vrep.simxGetJointPosition(clientID,handle2,vrep.simx_opmode_blocking);
        [returnCode,q(3)]=vrep.simxGetJointPosition(clientID,handle3,vrep.simx_opmode_blocking);
        [returnCode,q(4)]=vrep.simxGetJointPosition(clientID,handle4,vrep.simx_opmode_blocking);
        [returnCode,q(5)]=vrep.simxGetJointPosition(clientID,handle5,vrep.simx_opmode_blocking);
        
        [qd,Ja]=inversa_robot(xd);
        
        JaT = Ja'*inv(Ja*Ja');
        
        qd_p = JaT*(xd_p);
       
        q_p = Kp*(qd-q) + qd_p'
%        q_p = Kp*(qd-q)
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
%         [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle1,0,vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle2,0,vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle3,0,vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle4,0,vrep.simx_opmode_blocking);
%         [returnCode]=vrep.simxSetJointTargetVelocity(clientID,handle5,0,vrep.simx_opmode_blocking);
    
    vrep.simxFinish(clientID);
end;
vrep.delete(); % call the destructor!

disp('Program ended');
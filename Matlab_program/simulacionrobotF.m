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
clear all
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
    [returnCode,handle6]=vrep.simxGetObjectHandle(clientID,'Prismatic_joint',vrep.simx_opmode_blocking );
    [returnCode,handle7]=vrep.simxGetObjectHandle(clientID,'Prismatic_joint0',vrep.simx_opmode_blocking );
    [returnCode,handle8]=vrep.simxGetObjectHandle(clientID,'Prismatic_joint1',vrep.simx_opmode_blocking );
    
    
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-100*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,50*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,30*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,-80*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-99*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle6,1.11,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle7,1.89,vrep.simx_opmode_blocking);
     pause(4);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-125*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,60*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,1*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,-60*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-125*pi/180,vrep.simx_opmode_blocking);
     pause(5);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle8,-0.6,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-60*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-60*pi/180,vrep.simx_opmode_blocking);
     %aqui
     pause(4);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-125*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,30*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,1*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,-30*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-125*pi/180,vrep.simx_opmode_blocking);
pause(4);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-125*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,10*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,1*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,-10*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-125*pi/180,vrep.simx_opmode_blocking);
pause(2);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-60*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,10*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,1*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,-10*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-125*pi/180,vrep.simx_opmode_blocking);

     pause(2);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-60*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,60*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,1*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,-60*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-60*pi/180,vrep.simx_opmode_blocking);
    
%aqui
     pause(4);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle8,0,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,-125*pi/180,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,-125*pi/180,vrep.simx_opmode_blocking);
     pause(4);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle1,0,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle2,0,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle3,0,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle4,0,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle5,0,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle6,0,vrep.simx_opmode_blocking);
     [returnCode]=vrep.simxSetJointTargetPosition(clientID,handle7,0,vrep.simx_opmode_blocking);
     pause(6);
         
    % Now send some data to V-REP in a non-blocking fashion:
   % vrep.simxAddStatusbarMessage(clientID,'Hello V-REP!',vrep.simx_opmode_oneshot);
    
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
clear all
close all
clc
vrep=remApi('remoteApi');
vrep.simxFinish(-1);

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
joint_pos1=[pi/2,pi/5.5,-pi/3,-pi/3.5];
joint_pos2=[0,-pi/3,0,pi/4];


h=[0,0,0,0]

if (clientID>-1)
disp('Connected');

[returnCode,h(1)]=vrep.simxGetObjectHandle(clientID,'rotary',vrep.simx_opmode_blocking);
[returnCode,h(2)]=vrep.simxGetObjectHandle(clientID,'lower_arm',vrep.simx_opmode_blocking);
[returnCode,h(3)]=vrep.simxGetObjectHandle(clientID,'upper_arm',vrep.simx_opmode_blocking);
[returnCode,h(4)]=vrep.simxGetObjectHandle(clientID,'ee',vrep.simx_opmode_blocking);
        %Other Code
       while true

         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(1),joint_pos1(1,1),vrep.simx_opmode_streaming); 
         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(2),joint_pos1(1,2),vrep.simx_opmode_streaming); 
         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(3),joint_pos1(1,3),vrep.simx_opmode_streaming);
         pause(2);

         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(2),joint_pos2(1,2),vrep.simx_opmode_streaming);
         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(4),joint_pos2(1,4),vrep.simx_opmode_streaming); 
         pause(2);     

         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(4),joint_pos1(1,4),vrep.simx_opmode_streaming);
         pause(2);

         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(1),0,vrep.simx_opmode_streaming);
         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(2),0,vrep.simx_opmode_streaming);
         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(3),0,vrep.simx_opmode_streaming);
         pause(5);
         
         [returnCode]=vrep.simxSetJointTargetPosition(clientID,h(4),0,vrep.simx_opmode_streaming);
         pause(5);
       end 
end
vrep.delete();
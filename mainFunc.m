clf;        
clc;        
clear;     


axis equal; 
map=[0,0;
    66,0;
    66,44;
   
    44,44;
    44,67;
    110,67;
    110,111;
    0,111];  %map based on lab arena measurements


% initialise the robot
COM_CloseNXT all;  
h = COM_OpenNXT(); 
COM_SetDefaultNXT(h); %set default handle


% botSim = BotSim(map,[0.01,0.005,0]);  %sets up a botSim object a map, and debug mode on.
botSim = BotSim(map,[0,0,0]);  %sets up a botSim object a map, and debug mode on.
botSim.drawMap();
drawnow;


returnedPos = localiseNXT_test(botSim,map); %Where the magic happens
 NXT_PlayTone(300,700, h); % play tone to specify that 

COM_CloseNXT all;



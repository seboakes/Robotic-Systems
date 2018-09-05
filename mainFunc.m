% Sebastian Oakes
% University of Bristol
% August 2018
% Base program for particle filter (for NXT mindstorms)
%
%

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


% initialise NXT
COM_CloseNXT all;  
h = COM_OpenNXT(); 
COM_SetDefaultNXT(h); %set default handle


botSim = BotSim(map,[0,0,0]);  %set up a botSim object and map
botSim.drawMap();
drawnow;


returnedPos = localiseNXT_cont(botSim,map); %Where the magic happens
 NXT_PlayTone(300,700, h); % play tone to specify convergence or 1 min elapsed

COM_CloseNXT all;   %close ports



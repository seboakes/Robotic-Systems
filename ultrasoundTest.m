%% For ultrasound testing

%%Testing of robot functions%%

clc;
clear;

% initialise the robot
COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle


OpenUltrasonic(SENSOR_3);

%% Lets measure distance
for i = 1:1000
    GetUltrasonic(SENSOR_3)
    pause(0.1);
end





%% ONLY after you wont use the sensor again (e.g. exit program), clear with:

CloseSensor(SENSOR_3);
COM_CloseNXT all;

%%Testing of robot functions%%

clc;
clear;

% initialise the robot
COM_CloseNXT all;  %prepares workspace
h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
COM_SetDefaultNXT(h); %sets default handle

%% lets initialize two instances of motor objects for drive wheels
mB = NXTMotor('B');  %right wheel motor
mC = NXTMotor('C');  %left wheel motor
mA = NXTMotor('A');  %motor for driving ultrasound sensor movement
OpenUltrasonic(SENSOR_3);

%% Lets measure distance
OpenUltrasonic(SENSOR_3); % open ultrasonic sensor on port 4
GetUltrasonic(SENSOR_3) % get readings in "cm"
ultraScan(mA,100,6)



%% ONLY after you wont use the sensor again (e.g. exit program), clear with:

CloseSensor(SENSOR_3);


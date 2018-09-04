clf;        %clears figures
clc;        %clears console
clear;      %clears workspace
axis equal; %keeps the x and y scale the same



map=[0,0;
    66,0;
    66,44;
    44,44;
    44,67;
    110,67;
    110,111;
    0,111];

COM_CloseNXT all;  
h = COM_OpenNXT(); 
COM_SetDefaultNXT(h); %set default handle
OpenUltrasonic(SENSOR_1);

botSim = BotSim(map);
scans= 95;
sgolayWindow = 15;
Snoise = 1;

botSim.setBotPos([42 22])
botSim.setBotAng(pi/4);
botSim.setSensorNoise(Snoise);

botSim.setScanConfig(botSim.generateScanConfig(scans));



particleDist  = botSim.ultraScan(); %perform simulated scan
botDist = ultraScanNXT_cont(scans,20,5);

particleDist = smoothdata(particleDist, 'sgolay', 5);


z = sqrt(sum((particleDist-botDist').^2))

botSim.drawMap();
botSim.drawBot(3);

plot(botDist)
plot(particleDist)

COM_CloseNXT all;


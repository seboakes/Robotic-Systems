function [radii] = ultraScanNXT_cont(scans, scanSpeed, sgolayWindow)

% COM_CloseNXT all;  
% h = COM_OpenNXT(); 
% COM_SetDefaultNXT(h); %set default handle
% OpenUltrasonic(SENSOR_1);
correct_fac = 3;
mot_a = NXTMotor('A');
maxVal=40;



radii=zeros(1,scans);
%motor.stop('Brake'); %cancels any previous movement that may be happening
mot_a.ActionAtTachoLimit = 'Brake'; % Want precise movment
mot_a.Power = -scanSpeed; % so it turns counterclockwise
mot_a.SmoothStart = false; %we want the scan to be as linear as possible


mot_a.SendToNXT(); %move motor


    
for i = 1:scans
    radii(i) = GetUltrasonic(SENSOR_1);
end
mot_a.Stop('off');
mot_a.WaitFor();


%move the motor back to it's original position
%you could probably get a second scan at this point if you wanted and merge
%the data. Or just stay at this position until you want to scan again and
%turn the other way to stop the cable getting tangled.

mot_a.Power = 90;
mot_a.TachoLimit = 363;
%mot_a.TachoLimit = 300;
mot_a.SendToNXT(); %move motor
mot_a.WaitFor();

% 
% for j = 1:scans
%     if radii(j)>maxVal
%         radii(j)=maxVal;
%     end
% end

% 
% clf
% figure();
% hold on
% plot(radii);

radii = smoothdata(radii, 'sgolay',sgolayWindow);




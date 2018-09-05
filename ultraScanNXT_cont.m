% Sebastian Oakes
% University of Bristol
% August 2018
% Function to produce continuous scan using ultrasound (for NXT mindstorms)
%
%



function [radii] = ultraScanNXT_cont(scans, scanSpeed, sgolayWindow)
%function takes values for number of scans, scan power (speed), and a
%window size for the savitsky-golay filtering used to smooth distribution 

mot_a = NXTMotor('A');

radii=zeros(1,scans);
%motor.stop('Brake'); %cancels any previous movement that may be happening
mot_a.ActionAtTachoLimit = 'Brake'; 
mot_a.Power = -scanSpeed; 
mot_a.SmoothStart = false; 




%% Carry out rotation, ending when total scan number = 'scans' 
mot_a.SendToNXT(); 
for i = 1:scans
    radii(i) = GetUltrasonic(SENSOR_1);
end
mot_a.Stop('off');
mot_a.WaitFor();


%% Return sensor to original position
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

radii = smoothdata(radii, 'sgolay',sgolayWindow);   %savitsky-golay filter for smoothing of distribution




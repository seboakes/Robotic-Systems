function [radii, angles] = ultraScan(mot_a,scanSpeed,samples)


mot_a = NXTMotor('A');
ultraScan
%motor.stop('Brake'); %cancels any previous movement that may be happening
mot_a.ActionAtTachoLimit = 'Brake'; % Want precise movment
mot_a.Power = -scanSpeed; % so it turns counterclockwise
mot_a.SmoothStart = false; %we want the scan to be as linear as possible
disp('scanning...');
angleIt = 360/samples;
angles = zeros(samples,1); %preallocate
radii = zeros(samples,1); %preallocate
for i = 0:samples-1
    radii(i+1) = GetUltrasonic(SENSOR_3);
    angles(i+1) = i*angleIt;
    mot_a.TachoLimit =angleIt;
    mot_a.SendToNXT(); %move motor
    mot_a.WaitFor();    
end

%move the motor back to it's original position
%you could probably get a second scan at this point if you wanted and merge
%the data. Or just stay at this position until you want to scan again and
%turn the other way to stop the cable getting tangled.
disp('returning to original position');
mot_a.Power = scanSpeed;
mot_a.TachoLimit = 360;
mot_a.SendToNXT(); %move motor
mot_a.WaitFor();

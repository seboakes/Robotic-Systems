

%% Function to move robot
%Takes in values for distance and turn, as
%circumference of wheel = 4.25cm

function move_complete = botMove(moveVal)
% 
% COM_CloseNXT all;  %standard setup - 
% h = COM_OpenNXT(); 
% COM_SetDefaultNXT(h); 

if moveVal ~= 0
    
    if moveVal>0 
        power = 100;
    else
        power = -100;
    end

    moveVal=abs(moveVal);
    
    wheel_circ = 4.4*pi;
    rot_num = moveVal/wheel_circ;
    rot_degrees = round(rot_num*360);
    
    mot_b = NXTMotor('B');  %right wheel motor
    mot_c = NXTMotor('C');  %left wheel motor
    
    %motor.stop('Brake'); %cancels any previous movement that may be happening
    mot_b.ActionAtTachoLimit = 'Brake'; % Want precise movment
    mot_c.ActionAtTachoLimit = 'Brake'; % Want precise movment
    
    mot_b.Power = power; % so it turns counterclockwise
    mot_c.Power = power; % so it turns counterclockwise
    
    mot_b.SmoothStart = false; %we want the scan to be as linear as possible
    mot_c.SmoothStart = false; %we want the scan to be as linear as possible
    
    mot_b.TachoLimit =rot_degrees;
    mot_c.TachoLimit =rot_degrees;
    
    mot_b.SendToNXT(); %move motor
    mot_c.SendToNXT(); %move motor
    
    mot_b.WaitFor();
    mot_c.WaitFor();
    
    move_complete = 1;

else
    move_complete = 0;


end

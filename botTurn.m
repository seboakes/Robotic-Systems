%% Function to turn robot


function turn_complete = botTurn(turnVal)

% initialise the robot
% COM_CloseNXT all;  %prepares workspace
% h = COM_OpenNXT(); %prepares workspace,  if this fails, there is an issue with your robot (e.g. connectoin, driver, motorControl22 not running)
% COM_SetDefaultNXT(h); %sets default handle

turnVal = turnVal*(180/pi);  %convert to degrees for NXT device

if turnVal ~= 0  %skips over if value is zero
    
    power = 100;
    
   
    wheel_circ = 4.2*pi;
    wheelbase_circ = 12*pi;
    
    circ_fraction = turnVal/360;
    req_wheeldist = wheelbase_circ*circ_fraction;
    req_turns_deg = round(abs((req_wheeldist/wheel_circ)*360));   % find required degree value to send to wheels - also rounds to nearest integer, and makes positive
   
    
    
    mot_b = NXTMotor('B');  %left wheel
    mot_c = NXTMotor('C');  %right wheel
%     
    %motor.stop('Brake'); %cancels any previous movement that may be happening
    
    if turnVal>0   % for left turn
        
         mot_b.Power = -power; 
         mot_c.Power = power; 
         
    else    % for right turn
        
         mot_b.Power = power;
         mot_c.Power = -power; 
         
    end

        
    
    mot_b.ActionAtTachoLimit = 'Brake'; % Want precise movment
    mot_c.ActionAtTachoLimit = 'Brake'; % Want precise movment
    

    
    mot_b.SmoothStart = false; %we want the scan to be as linear as possible
    mot_c.SmoothStart = false; %we want the scan to be as linear as possible
    
    mot_b.TachoLimit = req_turns_deg;
    mot_c.TachoLimit =req_turns_deg;
    
    mot_b.SendToNXT(); %move motor
    mot_c.SendToNXT(); %move motor
    
    mot_b.WaitFor();
    mot_c.WaitFor();

    turn_complete = 1;


else
    turn_complete = 0;
end



end
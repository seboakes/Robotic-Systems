function [botEst] = localiseNXT_cont_v2(botSim,map)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function


%% debug set to one if viewing of particle positions required
% particle correction set to 1 if want to use particle orientation
% correction
debug=1;



%% Set scan number and filtering window
scans=95;
sgolayWindow=15;

%%  SET UP UNIVERSAL CONST. + STARTER VALUES
  %set initial parameters and certain variables
%SD = 10;
SD = 100;
SD_POS = 8;  % standard deviation position - for convergence check
SD_ANG = 10; 

 
% Mnoise = 0.05;  %noise variable
% Tnoise = 0.05;
% Snoise = 0.3;

Mnoise = 0.07;  %noise variables
Tnoise = 0.07;
Snoise = 0.05;

num=300;   %set number of particles
moveDistFrac = 0.4;
randFrac = 0.5;
turnCorrFrac = 1;
topCandidates = round(num/15);

damp = 0.0005;




allWeights = zeros(1,num);  %array for storing weights before normalisation.
finParticleWeights = zeros(2,num);
resultsArray = zeros(4,num);

convergeCheck = 0;
iterations=0;


botEst = BotSim(map);


%% SET NXT MOTORS/SENSOR
mot_a = NXTMotor('A');  %motor for ultrasound
mot_b = NXTMotor('B');  %right wheel motor
mot_c = NXTMotor('C');  %left wheel motor
OpenUltrasonic(SENSOR_1);


%% INITIALISE PARTICLES

%initialise particles randomly.
% if debug==1
% 
% disp('Initialising particles.');
% end
particles(num,1) = BotSim;      %how to set up a vector of objects
for i =1:num
    
    particles(i) = BotSim(map);
    particles(i).randomPose(10);  % note - number specifies distance from boundary
  
  
    particles(i).setScanConfig(particles(i).generateScanConfig(scans))
    particles(i).setMotionNoise(Mnoise);       %movement noise
    particles(i).setTurningNoise(Tnoise);      %turning noise
    particles(i).setSensorNoise(Snoise);    %sensor noise
end



%% DRAW INITIAL DISTRIB.
if debug==1
botSim.drawMap();


for k=1:num
    
    particles(k).drawBot(3, 'black');
end


hold off;
disp('initial distrib.');

end

clf;



%% 

while convergeCheck==0  %outer loop for iterations
%% 
randVariable=rand();
%





%% MAKE BOT MEASUREMENT

    upperLim=150;
    botDist = ultraScanNXT_cont(scans,20,sgolayWindow);   %take real scan

    
%     % Place upper limit on readings
%     for i = 1:scans        
%         if botDist(i)>upperLim
%             botDist(i)=upperLim;
%         end    
%     end



%% PARTICLE MEASUREMENTS (only if particleCorrection=!0)
collectedZvalues = zeros(1,num);
for i = 1:num
    
    partInsideMap = particles(i).insideMap();
    
    if  partInsideMap ~=1
        particles(i).randomPose(10);
    end
      
        particleDist  = particles(i).ultraScan(); %perform simulated scan
        particleDist = smoothdata(particleDist, 'sgolay', sgolayWindow);
        
         % Place upper limit on readings
%         for j = 1:scans       
%              if particleDist(j)>upperLim
%                  particleDist(j)=upperLim;
%              end
%         end

        resultsArray(1,i) = i;  %send indice to results array
       
        particlePosition = particles(i).getBotPos(1); %for virtual bot position
        resultsArray(2,i) = particlePosition(1);  % insert x position into results array
        resultsArray(3,i) = particlePosition(2);   % insert y position into results array
        resultsArray(4,i) = particles(i).getBotAng();  %insert angle into results array
        
        z = sqrt(sum((particleDist-botDist').^2));
        collectedZvalues(i)= z;
       
        
       
           
end


%% WEIGHT CALCULATION


clf;

SD = std(collectedZvalues);
 SD
    for i = 1:num
        
        z = collectedZvalues(i);
        
        LHS = (2*pi*(SD^2))^(-0.5);
        expon = -(z)^2/(2*(SD^2));
        
        weight = LHS*exp(expon) + damp;   %find weight (un-normalised)
        allWeights(1,i) = weight;    %add to collected weight array

    end
    weightSum = sum(allWeights);    %sum needed for normalisation
    weightNorm = allWeights/weightSum;    %normalisation
    weightNormSum = sum(weightNorm);
        
    resultsFull = [resultsArray;weightNorm];
    resultsFullSorted = sortrows(resultsFull',-5);







%% CHECK FOR CONVERGENCE



standardDev_X = std(resultsFullSorted( 1:(topCandidates), 2));
standardDev_y = std(resultsFullSorted( 1:(topCandidates), 3));
standardDev_ang = std(resultsFullSorted( 1:(topCandidates), 4));


if standardDev_X<SD_POS && standardDev_y<SD_POS &&  standardDev_ang<SD_ANG
    convergeCheck = 1;
else
    convergeCheck=0;
end

%% DRAW AFTER CONV CHECK
if debug==1
botSim.drawMap();


for k=1:num
    
    particles(k).drawBot(3, 'black');
end

disp('After conv check.');
hold off;
end

 clf;


%% REALLOCATE PARTICLES ACCORDING TO WEIGHTS

if convergeCheck ==0
    
%     if debug==1
%     disp('Reallocating according to weights');
%     end
j=1;
P_allocated=0;
P_leftover=0;
P_random=0;

for n =1:num
    
  % particleNum(1,i) = resultsFullSorted(i,5)*num;
  
    particleAllocated = floor(resultsFullSorted(n,5)*num);
 
    particleTotal(n) = particleAllocated;
    
    if particleAllocated >1   %if statement to break when allocation no longer necessary
        
        for p=1:particleAllocated
            
    
        %pause(0.001);
        particles(j).setBotPos([resultsFullSorted(n,2), resultsFullSorted(n,3)]);  %allocates new particle to same position as previously weighted particles
        particles(j).setBotAng(resultsFullSorted(n,4));   %sets angle the same
        
        j=j+1;
        P_allocated = P_allocated+1;
        end
        %alreadyDone = alreadyDone+particleAllocated;
        
   
    
    else
        break;
    end
   
end



%allocate 15 random particles to ensure diversity
if j<=num-15
    for x=1:15
        %pause(0.001);
        particles(j).randomPose(10);
       
        j=j+1;
        P_random=P_random+1;
    end
else
end

% if debug==1
% disp('Allocating leftover particles.');
% end

P_total = P_allocated+P_random;

particleSum = sum(particleTotal);

if P_total<num         %if statement to sweep up unallocated particles, and evenly distribute them to top candidates.
    
    leftoverParticleNum = (200-2) - particleSum;
    
    
    for a=1:num-P_total
        %pause(0.001);
        particles(j).setBotPos([resultsFullSorted(a,2), resultsFullSorted(a,3)]);
        particles(j).setBotAng(resultsFullSorted(a,4));
        
        j=j+1;
        P_leftover = P_leftover+1;
     
    end
else
end


%% DRAW AFTER REALLOC
if debug==1
botSim.drawMap();


for k=1:num
    
    particles(k).drawBot(3, 'black');
end

disp('after reallocation');
hold off;
end

clf;

%% MOVE BOT
% if debug==1
% 
% disp('Moving bot.');
% 
% end

turn_inc = (2*pi)/scans;
prox_dist = 20;

halfway = scans/2;

[maxDist, dirIndice1] = max(botDist);   %returns maximum distance measured, and an indice to denote the direction.
[minDist, dirIndice2] = min(botDist); %returns min distance, and indice

    %if minDist<prox_dist
        
%         if dirIndice2<=halfway
%         
%             botTurn((dirIndice2-1)*(turn_inc));   %turn left towards shortest distance
%         else
%             botTurn((scans-dirIndice2)*(-turn_inc));   %turn right towards shortest distance
%         end
%         
%         BOT_TURN = (dirIndice2)*(turn_inc);
%        
%         botMove(-10);   %move bot backwards away from wall
%         BOT_MOVE = -10;
   % else
        if randVariable<randFrac
            
         if dirIndice1<=halfway
        
            botTurn((dirIndice1)*(turn_inc));   %turn left towards longest distance
        else
            botTurn((scans-dirIndice1)*(-turn_inc));   %turn right towards longest dist.
        end
            BOT_TURN = (dirIndice1)*(turn_inc);
            
            
            
            botMove(moveDistFrac*maxDist);   %move bot a fraction of the previously measured max distance
            BOT_MOVE = moveDistFrac*maxDist;
            
        else
            X=rand;
            
            angle =pi/4;
            safeMin = min(botDist);   %random movement to avoid getting stuck
            
            if X>=0.5
                botTurn(angle);
                BOT_TURN = angle;
            else
                botTurn(-angle);
                BOT_TURN = -angle;
            end
            
            botMove(0.5*safeMin);
            BOT_MOVE = 0.5*safeMin;
        end
        
    %end

   

    %%  MOVE ALL PARTICLES

    
   for j=1:num
       
       %copy all movements made by bot
       particles(j).turn(BOT_TURN);
       particles(j).move(BOT_MOVE);
  
   end  
   
   
 

else
    break
end
iterations=iterations+1;


%% DRAW AFTER MOVE PARTICLES
if debug==1
botSim.drawMap();
for k=1:num
    
    particles(k).drawBot(3, 'black');
end

disp('after particle move');
hold off;
end

clf;

end

cf=1;

if convergeCheck==1
botSim.drawMap();


for i=1:num
    
    particles(i).drawBot(3, 'black');
end

disp('converged');
hold off;
    end


clf;

%% EXTRACT ESTIMATED BOT POSITION




topX=zeros(1,topCandidates);
topY=zeros(1,topCandidates);
topAng=zeros(1,topCandidates);



for i=1:(topCandidates)
    k = resultsFullSorted(i,1);
    position = particles(k).getBotPos();
    angle = particles(k).getBotAng();
    topX(i) = position(1,1);
    topY(i) = position(1,2);
    topAng(i) = angle;
end



    
estimateX = mean(topX);
estimateY = mean(topY);
estimateAng = mean(topAng);



botEst.setBotPos([estimateX estimateY]);
botEst.setBotAng(estimateAng);

clf
 
botSim.drawMap()
botSim.drawBot(10)
botEst.drawBot(5)

fprintf('X position is %d\n',estimateX);
fprintf('Y position is %d\n',estimateY);

CloseSensor(SENSOR_1);



end
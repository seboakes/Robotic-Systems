function [botEst] = localiseNXT(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

debug=1;

%%  SET UP UNIVERSAL CONST. + STARTER VALUES
  %set initial parameters and certain variables
SD = 10;
SD_POS = 10;  % standard deviation position - for convergence check
SD_ANG = 5; 

 
Mnoise = 0.05;  %noise variable
Tnoise = 0.05;
Snoise = 0.3;

num=300;   %set number of particles
moveDistFrac = 0.3;
randFrac = 0.7;
turnCorrFrac = 1;
topCandidates = num/10;




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
OpenUltrasonic(SENSOR_3);


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
  
    
    particles(i).setMotionNoise(Mnoise);       %movement noise
    particles(i).setTurningNoise(Tnoise);      %turning noise
    particles(i).setSensorNoise(Snoise);    %sensor noise
end



%% DRAW INITIAL DISTRIB.
if debug==1
botSim.drawMap();


for i=1:num
    
    particles(i).drawBot(3, 'black');
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

    botDist = ultraScanNXT(mot_a,100,6);   %take real scan
    
    botVecMag = norm(botDist);   %vector mag for bot distance data


%% PARTICLE MEASUREMENTS
for i = 1:num
    
    partInsideMap = particles(i).insideMap();
    
    if  partInsideMap ==1
      
        particleDist  = particles(i).ultraScan(); %perform simulated scan

        resultsArray(1,i) = i;  %send indice to results array
       
        particlePosition = particles(i).getBotPos(1); %for virtual bot position
        resultsArray(2,i) = particlePosition(1);  % insert x position into results array
        resultsArray(3,i) = particlePosition(2);   % insert y position into results array
   
        vecMag = zeros(1,6);
        
        for m=1:6    %loop for cycling through possible orientations with respect to actual bot
            vec = circshift(particleDist,m-1);
           vecMag(m) = sqrt(sum((vec-botDist).^2));   %finds best orientation fit
             
        end
        
        
        [z, dirInd] = min(vecMag);    %select smallest vectoral difference to use for weighting calc
        randForTurn = rand();
        
        if randForTurn<=turnCorrFrac
            
            particles(i).turn((dirInd-1)*(-pi/3));    %turn towards best fitting orientation
        else
        end
     
        resultsArray(4,i) = particles(i).getBotAng();  %insert new angle into results array
        
        collectedZvalues(i)= z;
        
        LHS = (2*pi*(SD^2))^(-0.5);
        expon = -(z)^2/(2*(SD^2));
        
        weight = LHS*exp(expon) + damp;   %find weight (un-normalised)
        allWeights(1,i) = weight;    %add to collected weight array
       
   
    else
        allWeights(1,i) = 0;
    end
    
  
    
end


%% DRAW AFTER CORRECTION
if debug==1
botSim.drawMap();


for i=1:num
    
    particles(i).drawBot(3, 'black');
end

disp('After Correction for optimal angle.');
hold off;
end

clf;


%%
% if debug==1
% disp('determining weights.');
% end

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


for i=1:num
    
    particles(i).drawBot(3, 'black');
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

[maxDist, dirIndice] = max(botDist);   %returns maximum distance measured, and an indice to denote the direction.
    
    if randVariable<randFrac
         
        botTurn((dirIndice-1)*(-pi/3));   %turn towards longest distance
        BOT_TURN = (dirIndice-1)*(-pi/3);
       
        botMove(moveDistFrac*maxDist);   %move bot a fraction of the previously measured max distance
        BOT_MOVE = moveDistFrac*maxDist;
       
    else
        safeMin = min(botDist);   %random movement to avoid getting stuck
        botTurn(pi/2);
        BOT_TURN = pi/2;
        
        botMove(0.9*safeMin);
        BOT_MOVE = 0.9*safeMin;
    end
    
 

  
 %% DRAW AFTER MOVE BOT
if debug==1
botSim.drawMap();


for i=1:num
    
    particles(i).drawBot(3, 'black');
end

disp('After bot move.');
hold off;
end

clf;
  

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


for i=1:num
    
    particles(i).drawBot(3, 'black');
end

disp('after particle move');
hold off;
end





end

cf=1;

if convergeCheck==1
botSim.drawMap();


for i=1:num
    
    particles(i).drawBot(3, 'black');
end

disp('after particle move');
hold off;
end


clf;

%% EXTRACT ESTIMATED BOT POSITION
disp('converged');



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

CloseSensor(SENSOR_3);



end

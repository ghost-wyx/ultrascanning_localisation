function [botSim] = localise(botSim,map,target)
%This function returns botSim, and accepts, botSim, a map and a target.
%LOCALISE Template localisation function

%% setup code
%you can modify dthe map to take account of your robots configuration space
modifiedMap = map; %you need to do this modification yourself
botSim.setMap(modifiedMap);
sotSim = BotSim(modifiedMap);
virSim = BotSim(modifiedMap);
%generate some random particles inside the map
num =300; % number of particles
particles(num,1) = BotSim; %how to set up a vector of objects
for i = 1:num
    particles(i) = BotSim(modifiedMap);  %each particle should use the same map as the botSim object
    particles(i).randomPose(0); %spawn the particles in random locations
end
%% Localisation code
sigma = 5;
parBot_dist  = zeros(36,1);
particles_Dist = zeros(6,36);
particle_Para = zeros(36,2);
particle_List = zeros(36,2);
particles_angle= zeros(36,1);
particle_Newdist = zeros(6,num);
weight = zeros(num,1); %weight of each particle
maxNumOfIterations = 30;
n = 0;
converged =0;    %The filter has not converged yet
while(converged == 0 && n < maxNumOfIterations) %%particle filter loop
    n = n+1;                          %increment the current number of iterations
    botScan = botSim.ultraScan();     %get a scan from the real robot.
    %% Write code for updating your particles scans  
    for i=1:num
        if  particles(i).insideMap() == 0
            particles(i).randomPose(0);
        end
        for j = 1:36
            particles_Dist(:,j) = particles(i).ultraScan();
            parBot_dist(j) = norm(botScan-particles_Dist(:,j));
            particles_angle(j)=particles(i).getBotAng();
            particles(i).turn(pi/18);
        end
        particle_Para = [parBot_dist,particles_angle];
        particle_List=sortrows( particle_Para);
        %scan the paticle again
        particles(i).setBotAng(particle_List(1,2));
        particle_Newdist(:,i) = particles(i).ultraScan();
        difference=norm(botScan-particle_Newdist(:,i));
        weight(i)=normpdf(difference,0,sigma);
    end
 %% Write code for scoring your particles
        weight_sum = sum(weight);                              % calculate sum 
        w = weight./weight_sum;                                % normalization 
    %% Write code for resampling your particles
        index = randi(num);
        w_max = max(w);
        beta = 0;
       for i = 1 : num
           beta = beta+rand()*2*w_max; 
         while(beta > w(index))
            beta = beta-w(index);
            index = index + 1;
            if index > num
                index = 1;
            end     
         end   
          particles(i).setBotPos(particles(index).getBotPos());
       end
       
    %% Write code to check for convergence   
    P_Pos = zeros(num,2);
    P_Ang = zeros(num,1);
    P_norAng = zeros(num,1);
    for i = 1 : num
        P_Pos(i,:) = particles(i).getBotPos();
        P_Ang(i,:) = particles(i).getBotAng();
    end    
    P_norAng = mod(P_Ang,2*pi);
    dist_std = std(P_Pos,1);
    ang_std = std(P_norAng,1);
    if(dist_std(1,1)<1.1 && dist_std(1,2)<1.1 && ang_std<0.08)
        converged = 1;
    else
        converged = 0;
    end    
    %% Write code to take a percentage of your particles and respawn in randomised locations (important for robustness)	
    
    
    %% Write code to decide how to move next
    Botdist = botSim.ultraScan();
     if Botdist(1,1)<10 || Botdist(2,1)<10 || Botdist(6,1)<10
         turnToOut = pi/3 *(find(Botdist==max(Botdist))-1);
         moveToOut = max(Botdist(1,1))*rand;
     else  
         turnToOut = pi/180*rand;
         moveToOut = 5*rand;
     end
         botSim.turn(turnToOut);
         botSim.move(moveToOut);
%     turn = 0.5;
%     move = 3;
%     botSim.turn(turn); %turn the real robot.  
%     botSim.move(move); %move the real robot. These movements are recorded for marking
    P_Pose = zeros(num,2);
    P_NewAng = zeros(num,1);
    for i =1:num %for all the particles. 
        particles(i).turn(turnToOut); %turn the particle in the same way as the real robot
        particles(i).move(moveToOut); %move the particle in the same way as the real robot
        P_Pose(i,:) = particles(i).getBotPos();
        P_NewAng(i,:) = particles(i).getBotAng();
    end
     PAng_normal = mod(P_NewAng,2*pi); %angle(0,2*pi)
     parAng_aver = sum(PAng_normal)/num; 
     Pcenter = mean(P_Pose);
    %create a new robot
     sotSim.setBotPos(Pcenter);
     sotSim.setBotAng(parAng_aver);
    
    %% Drawing
    %only draw if you are in debug mode or it will be slow during marking
    if botSim.debug()
        hold off; %the drawMap() function will clear the drawing when hold is off
        botSim.drawMap(); %drawMap() turns hold back on again, so you can draw the bots
        botSim.drawBot(30,'g'); %draw robot with line length 30 and green
         for i =1:num
             particles(i).drawBot(3); %draw particle with line length 3 and default color
         end
        sotSim.drawBot(20,'b');
        drawnow;
    end
end
%% pathplan by A*
starpoint=ceil(Pcenter);
target=round(target);%makes target to be integers
coor=findcoordinates(modifiedMap,starpoint,target);%coordinates of path
plot(coor(:,1),coor(:,2),'LineWidth',2,'color','r')

S = size(coor);
tAngle = zeros(S(1,1),1); %angles of the lines
tA = zeros(S(1,1),1);
tNewAng =  zeros(S(1,1)+1,1);
moving = zeros(S(1,1),1); %movement of the robot

for i=1:S(1,1)-1
    ty = coor(i+1,2)-coor(i,2);
    tx = coor(i+1,1)-coor(i,1);
    moving(i) = sqrt(ty^2+tx^2);
    if tx==1
        if ty==0
            tAngle(i)=0;
        elseif ty==1
                 tAngle(i)=pi/4;
        else
                 tAngle(i)=7*pi/4;
        end
    elseif tx==0
        if ty==1
            tAngle(i)=pi/2;
        else
            tAngle(i)=3*pi/2;
        end
       elseif tx==-1
            if ty==1
            tAngle(i)=3*pi/4;
            elseif ty==0
                 tAngle(i)=pi;
            else
                 tAngle(i)=5*pi/4;
            end
        end
end
tNewAng = [0;tAngle];
tA = diff(tNewAng);  %turning angle of path
firTurning=zeros(S(1,1),1);
firTurning(1) = parAng_aver; %creates a matrix which contains the angle of the robot
Turning = tA-firTurning; %turning angle the robot
for i=1:S(1,1)-1
    botSim.turn(Turning(i)); %turn the real robot.  
    botSim.move(moving(i));
    botSim.drawBot(3,'y');    
    hold on
end
end

 
                
            
        
        
     


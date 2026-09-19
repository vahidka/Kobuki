        
        clc
        clear
        close all

        map = zeros(200,200);
        map(1,:) = 1; % Top wall
        map(end,:) = 1; % Bottom wall
        map(:,1) = 1; % Left wall
        map(:,end) = 1; % Right wall
        
        % -------------- % (Single Obstacle)
        % map(70:135,90:115) = 1; % Map No.1
        % map(59:139,90:110) = 1; % Map No.2
        % map(90:110,60:141) = 1; % Map No.2
        map(50:114,90:113) = 1; % Map No.3
        map(70:94,90:132) = 1; % Map No.3
        % map(120:130,90:110) = 1; % Map No.4  
        % map(110:119,95:115) = 1; % Map No.4 
        % map(99:110,100:120) = 1; % Map No.4 
        % map(90:100,105:125) = 1; % Map No.4 
        % map(79:90,110:130) = 1; % Map No.4 
        % map(69:80,115:135) = 1; % Map No.4 

        % -------------- % (Multi Obstacles)
        % map(136:180,25:59) = 1; % Obstacle No.1
        % map(101:149,70:80) = 1; % Obstacle No.2
        % map(83:88,88:115) = 1; % Obstacle No.3
        % map(65:77,124:143) = 1; % Obstacle No.4
        % map(58:83,131:136) = 1; % Obstacle No.4
        % map(40:56,151:190) = 1; % Obstacle No.5
        % map(18:56,180:190) = 1; % Obstacle No.5
        % map(20:60,20:30) = 1; % Obstacle No.6
        % map(35:45,20:60) = 1; % Obstacle No.6
        % map(140:160,160:180) = 1; % Obstacle No.7


        tic
        figure
        % bug = Bug0(map,'inflate',1);      % create navigation object
        % bug = Bug1(map,'inflate',1);      % create navigation object 
        % bug = Bug2(map,'inflate',1);      % create navigation object 
        % bug = Bug0_HCM(map,'inflate',1);      % create navigation object -3
        % bug = Bug1_HCM(map,'inflate',1);      % create navigation object
        % bug = Bug2_HCM(map,'inflate',1);      % create navigation object 
        bug = Bug0_SCC(map,'inflate',1);      % create navigation object -3
        % bug = Bug1_SCC(map,'inflate',1);      % create navigation object 
        % bug = Bug2_SCC(map,'inflate',1);      % create navigation object 
        % (Single Obstacles)
        start = [10,100]; %bu
        goal = [190,100]; %bu
        % (Multi Obstacles)
        % start = [10,190]; %bu
        % goal = [190,10]; %bu

        radius = 5;
        path = bug.query(start, goal, radius, 'animate');   % animate path
        toc
        disp(['path Length = ', num2str(length(path))])



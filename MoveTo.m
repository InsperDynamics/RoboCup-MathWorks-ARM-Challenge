function MoveTo(coordinator, taskConfig)
    isAway = checkTargetAchieved();

    if isAway % se o robô não estiver na pose desejada, planeje um movimento
        %% Inicializar planejador RRT
        planner = manipulatorRRT(coordinator.Robot, coordinator.World);
        %% Calcular uma configuração de aperto sem colisões
        Tw_0 = taskConfig;
        Tw_0(1:3,1:3) = eye(3);
        Te_w = taskConfig;
        Te_w(1:3,4) = [0;0;0];
        bounds = [0, 0.0;
        0, 0;
        -0.0, 0.0;
        0, 0;
        0, 0;
        -0.001, 0.001;];
        Te_0 = taskConfig;
        searchiter=1;
        while(any(checkCollision(coordinator.Robot, goalConfig, coordinator.World)))
            Te_0 = exampleHelperSampleWGRROSGazeboScene(Te_w, Tw_0, bounds);
            [goalConfig, ~] = ik(coordinator.RobotEndEffector, ...
                Te_0, ones(1,6), ...
                coordinator.Robot.homeConfiguration);
            disp("Searching for other config...",num2str(searchiter));
            searchiter = searchiter + 1;
        end
        warning('on','robotics:robotmanip:rigidbodytree:ConfigJointLimitsViolationAutoAdjusted');
        %% Planejar um caminho livre de colisões
        startConfig = coordinator.CurrentRobotJConfig;
        disp("Now planning...");
        planner.MaxConnectionDistance=0.2;
        planner.ValidationDistance=0.2;
        planner.EnableConnectHeuristic=true;
        path = planner.plan(startConfig, goalConfig);
        path=planner.shorten(path,40);
        %% Interpolar caminho
        planner.ValidationDistance=0.02;
        positions = interpolate(planner, path);
        positions = [coordinator.CurrentRobotJConfig;positions];
        robotPos = positions';
        %% Gerar uma trajetória trapezoidal a partir do caminho
        h = 0.03;
        timeInterval = [0;h*size(positions,1)-h];
        [~,sd,~,~,~] = trapveltraj(timeInterval',size(positions,1));
        timeSteps = ((-sd+max(sd))/max(sd))*3*h+h;
        trajTimes = [0,cumsum(timeSteps(1:end-1))];
        %  Calcular as velocidades e acelerações das articulações na taxa necessária para execução pelo robô

        % Velocidades interpoladas de juntas
        robotVelTemp = (diff(robotPos'))./diff(trajTimes)';
        robotVel= [zeros(1,coordinator.NumJoints);robotVelTemp];

        % Acelerações interpoladas de juntas
        robotAccTemp = diff(robotVel)./diff(trajTimes)';
        robotAcc = [zeros(2,coordinator.NumJoints);robotAccTemp];

        q = robotPos;
        qd = robotVel';
        qdd = robotAcc'; 
        
        %% Visualizar trajetória
        hold on
        visualizePath(coordinator,q);        
      
        %% Empacotar e enviar a trajetória desejada ao robô para execução pelo JointTrajectoryController
        [trajAct,trajGoal] = rosactionclient('/my_gen3/gen3_joint_trajectory_controller/follow_joint_trajectory');
        trajAct.FeedbackFcn = [];
        JointTrajectory(trajGoal,coordinator.ROSinfo.gazeboJointNames,q,qd,qdd,trajTimes);  
        waitForServer(trajAct);

        sendGoal(trajAct,trajGoal)

        % Aguardar até que o robô chegue ao destino
        pause(1.0);        % esta pausa é necessária para começar a perguntar sobre o status do movimento
        isAway = true;
        disp('Waiting until robot reaches the desired configuration');
        while isAway
            isAway = checkTargetAchieved();
            pause(1);
        end   

        % Esperar até que o robô pare de se mover novamente
        isMoving = true;
        while isMoving
            [isMoving] = getMovementStatus(coordinator);
            pause(1);
        end

    end

    % Atualizar a configuração atual do robô
    coordinator.CurrentRobotJConfig = getCurrentRobotJConfig(coordinator);
    coordinator.CurrentRobotTaskConfig = getTransform(coordinator.Robot, (coordinator.CurrentRobotJConfig), coordinator.RobotEndEffector); 
    
    % Excluir o caminho na plotagem após a conclusão do movimento
    hold on
    show(coordinator.Robot, coordinator.CurrentRobotJConfig,'PreservePlot', false, 'Frames', 'off');
    try
        coordinator.PathHandle.Visible = 'off';
    catch
        warning('Pathhandle was not found.');
    end

    function isAway = checkTargetAchieved()
        isAway = true;
        jointCurrent=(getCurrentRobotJConfig(coordinator));
        if all(abs((jointCurrent-(goalConfig)))<0.10)     
            isAway=false; % Objetivo alcançado       
        end           
    end
end

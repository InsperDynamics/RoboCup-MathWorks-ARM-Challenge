function msg = JointTrajectory(msg,jointNames,q,qd,qdd,trajTimes)
    % Inicializar valores
    N = numel(trajTimes);
    numJoints = size(q,1);
    zeroVals = zeros(numJoints,1);
    
    % Atribuir nomes de juntas à mensagem ROS
    msg.Trajectory.JointNames = jointNames;
    
    % Atribuir restrições
    for idx = 1:numJoints
        msg.GoalTolerance(idx) = rosmessage('control_msgs/JointTolerance');
        msg.GoalTolerance(idx).Name = jointNames{idx};
        msg.GoalTolerance(idx).Position = 0;
        msg.GoalTolerance(idx).Velocity = 0.1;
        msg.GoalTolerance(idx).Acceleration = 0.1;
    end
    
    % Loopar pelos pontos de referência e preencher seus dados
    for idx = 1:N
        m = rosmessage('trajectory_msgs/JointTrajectoryPoint');
        m.TimeFromStart = rosduration(trajTimes(idx));
        m.Positions = q(:,idx);
        m.Velocities = qd(:,idx);
        m.Accelerations = qdd(:,idx);
        m.Effort = zeroVals;
        trajPts(idx) = m;
    end  
    msg.Trajectory.Points = trajPts;    
end

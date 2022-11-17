function BuildWorld(coordinator)
    %% Mover para posições de digitalização e capturar dados de nuvem de pontos e poses de câmera
    tftree = rostf;
    camera_quaternions = cell(numel(coordinator.ScanPoses),1); 
    camera_translations = cell(numel(coordinator.ScanPoses),1); 
    pointclouds = cell(numel(coordinator.ScanPoses),1);
    for scanIter = 1: numel(coordinator.ScanPoses)
        % Mover para a próxima pose de varredura
        disp(['Moving to scanning pose ' num2str(scanIter)]);
        MoveTo(coordinator, coordinator.ScanPoses{scanIter});
        pause(2);
        % Capturar nuvem de pontos
        disp(['Capturing point cloud ' num2str(scanIter)]);
        pointCloudSub = rossubscriber('/camera/depth/points');
        pointclouds{scanIter} = receive(pointCloudSub);
        % Obter pose da câmera
        disp(['Getting camera pose ' num2str(scanIter)]);
        camera_transf = getTransform(tftree, 'world', 'camera_link');        
        camera_transl = camera_transf.Transform.Translation;
        camera_rotation = camera_transf.Transform.Rotation;
        camera_quaternions{scanIter} = [camera_rotation.W, camera_rotation.X,...
            camera_rotation.Y,camera_rotation.Z];
        camera_translations{scanIter} = [camera_transl.X,...
            camera_transl.Y,camera_transl.Z];
    end
    %% Transformar nuvens de pontos em estrutura mundial e mesclar em nuvem de ponto único
    pointclouds = pointclouds';
    for i=1:size(pointclouds,2)
        % Extraia localizações xyz da nuvem de pontos
        xyz = readXYZ(pointclouds{1,i});
        
        % Remover NaNs
        xyzLess = double(rmmissing(xyz));

        % Criar objeto de nuvem de pontos
        ptCloud = pointCloud(xyzLess);

        % Criar transformações afins a partir de quaternions e translações de câmera
        quat = camera_quaternions{i};
        rotm = quat2rotm(quat);
        fixedRotation = eul2rotm([0 pi 0],"XYZ"); % rotação fixa entre câmera gazebo e link de câmera urdf
        rotm = rotm*fixedRotation' ;
        translVect = camera_translations{i};
        tform = rigid3d(rotm,[translVect(1),translVect(2),translVect(3)]);
     
        % Transformar a nuvem de pontos em quadro mundial
        ptCloudWorld = pctransform(ptCloud,tform);
        
        % Mesclar com outras nuvens de pontos
        if i == 1
            pcMerged = ptCloudWorld;
        else
            mergeSize = 0.015;
            pcMerged = pcmerge(pcMerged, ptCloudWorld, mergeSize);
        end
    end
    pause(3);
    
    % Remover pontos abaixo da altura da mesa
    indxPlane = find(pcMerged.Location(:,3) > -0.09 & pcMerged.Location(:,3) < -0.07);
    plane = select(pcMerged,indxPlane); % mesa   
    indx = find(pcMerged.Location(:,3) > -0.08);
    pcMergedTop = select(pcMerged,indx);
    coordinator.MergedPointCloud = pcMergedTop;

    % Inicializar lista de obstáculos com tabela
    tableMesh = collisionMesh(plane.Location);
    tablePointCloud = plane;
    hold on
    [~,patchObj] = show(tableMesh);
    patchObj.LineStyle = 'none';
    patches = {patchObj}; 
    obstacles = {tableMesh};
    segments = {tablePointCloud};
    drawnow;
    pause(1);

    %% Segmentar nuvem de pontos
    [labels, numClusters] = pcsegdist(pcMergedTop,0.05);
    
    %% Criar malhas de colisão para nuvens de pontos segmentadas
    for i=1:numClusters
        labelIdx = find(labels==i);
        obstacle = select(pcMergedTop,labelIdx);
        if obstacle.Count > 100
            obstacleMesh = collisionMesh(obstacle.Location);
            hold on
            [~,patchObj] =  show(obstacleMesh);
            patchObj.FaceColor = rand(1,3);
            patchObj.LineStyle = 'none';
            obstacles{end+1} = copy(obstacleMesh);
            segments{end+1} = copy(obstacle);
            patches{end+1} =  patchObj; 
            drawnow;
            pause(1);
        end        
    end
    %% Adicione malhas de colisão ao "mundo" de planejamento
    coordinator.World = obstacles;
    % Armazenar segmentos de nuvem de pontos
    coordinator.PointCloudSegments = segments;
    % Armazenar alças de patch de malhas do mundo para remover quando uma peça for pegada
    coordinator.WorldPatchHandles = patches;
end


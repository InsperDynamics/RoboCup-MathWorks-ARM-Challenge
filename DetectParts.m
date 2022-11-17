function DetectParts(coordinator)
    % Matriz de células vazias de peças para detectar novas peças
    coordinator.Parts = {};
    
    % Carregar arquivos STL de latas e garrafas
    bottle = stlread('Bottle.stl');
    can =  stlread('Can.stl');
    
    % Gerar nuvens de pontos de STL
    pcBottle = pointCloud(bottle.Points);
    pcCan = pointCloud(can.Points);

    % Registrar nuvens de pontos segmentados capturados em nuvens de pontos conhecidos de peças a serem selecionadas
    for label=1:numel(coordinator.PointCloudSegments)         
        object = coordinator.PointCloudSegments{label};       
        [tformsBottle(label),objectTransformed,rmseBottle(label)] = pcregistericp(object,pcBottle, 'Metric','pointToPoint');
        [tformsCan(label),objectTransformed,rmseCan(label)] = pcregistericp(object,pcCan,'Metric','pointToPoint');       
    end
    
    % Identificar garrafa
    [~, bottleLabel] = min(rmseBottle(rmseBottle>0));
    part.index = bottleLabel;
    part.type = 1;
    part.pointcloud = coordinator.PointCloudSegments{bottleLabel};
    part.mesh = coordinator.World{bottleLabel};
    disp("Bottle detected...");
    % Calcular centróide de garrafa
    tformBottle = invert(tformsBottle(bottleLabel));
    pcBottleCadWorld = pctransform(pcBottle,tformBottle);
    centroidBottleCad = mean(pcBottleCadWorld.Location);
    part.centerPoint = centroidBottleCad ;
    % Adicionar garrafa à lista de peças
    coordinator.Parts{1} = part;
    
    % Identificar lata
    [~, canLabel] = min(rmseCan(rmseCan>0));
    part.index = canLabel;
    part.type = 2;
    part.pointcloud = coordinator.PointCloudSegments{canLabel};
    part.mesh = coordinator.World{canLabel};
    disp("Can detected..."); 
    % Calcular centróide de lata
    tformCan = invert(tformsCan(canLabel));
    pcCanCadWorld = pctransform(pcCan,tformCan);
    centroidCanCad = mean(pcCanCadWorld.Location);
    part.centerPoint = centroidCanCad;
    % Adicionar lata à lista de peças
    coordinator.Parts{2} = part;
    
    coordinator.NextPart = 0;
    if ~isempty(coordinator.Parts) 
        coordinator.DetectedParts = coordinator.Parts;
        return;
    end
    coordinator.NumDetectionRuns = coordinator.NumDetectionRuns +1;
end
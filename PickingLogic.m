function remainingParts = PickingLogic(coordinator)
       % As peças serão selecionadas de acordo com a ordem na lista Coordinator.detectedParts
        coordinator.NextPart = coordinator.NextPart + 1; 
        if coordinator.NextPart<=length(coordinator.Parts)               
            % Os objetos são colocados em bin1 ou bin2 de acordo com seu tipo
            if coordinator.DetectedParts{coordinator.NextPart}.type == 1
                coordinator.DetectedParts{coordinator.NextPart}.placingBelt = 1;                    
            else
                coordinator.DetectedParts{coordinator.NextPart}.placingBelt = 2;
            end
            disp(coordinator.DetectedParts{coordinator.NextPart}.type)
            remainingParts = true; 
            return;
        end
        remainingParts = false; 
end